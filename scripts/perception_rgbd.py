#!/usr/bin/env python3
"""Read registered RGB-D and publish observed geometry in the arm reference frame."""

import json
from pathlib import Path
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_backend.perception import Settings, backproject, detect, transform_points
from g1_backend.perception_tracking import Tracker, TrackingSettings, transform_item


def decode(image):
    """Decode ROS Image including byte order and padded rows."""
    types = {'rgb8': ('u1', 3), '32FC1': ('f4', 1), '16UC1': ('u2', 1)}
    if image.encoding not in types:
        raise ValueError('Supported encodings: rgb8, 32FC1 and 16UC1')
    dtype, channels = types[image.encoding]
    dtype = np.dtype(('>' if image.is_bigendian else '<') + dtype)
    width_bytes = image.width * channels * dtype.itemsize
    if image.step < width_bytes or len(image.data) != image.step * image.height:
        raise ValueError('Invalid image buffer or row stride')
    rows = np.frombuffer(bytes(image.data), dtype=np.uint8).reshape(image.height, image.step)
    array = np.ascontiguousarray(rows[:, :width_bytes]).view(dtype)
    if channels == 3:
        return array.reshape(image.height, image.width, channels)
    array = array.reshape(image.height, image.width).astype(float)
    return array / 1000 if image.encoding == '16UC1' else array


def matrix(transform):
    t, q = transform.transform.translation, transform.transform.rotation
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    result[:3, 3] = [t.x, t.y, t.z]
    return result


def ros_pose(item):
    result = Pose()
    result.position.x, result.position.y, result.position.z = item['center']
    (result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w) = (
        item['orientation_xyzw']
    )
    return result


def visualization(tables, header, now, lifetime):
    """Stable IDs, observed bounds, text and local XYZ axes for RViz."""
    markers = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    markers.markers.append(clear)
    for table in tables:
        for item in [table, *table['objects']]:
            base_id = item['track_id'] * 10
            is_table = item['kind'] == 'table_surface'
            marker = Marker(header=header, ns='rgbd_perception', id=base_id)
            marker.type, marker.action = Marker.CUBE, Marker.ADD
            marker.pose = ros_pose(item)
            marker.scale.x, marker.scale.y = item['dimensions'][:2]
            marker.scale.z = max(0.005, item['dimensions'][2])
            marker.color.a = 0.55
            if is_table:
                marker.color.g = 0.8
            else:
                marker.color.r = 1.0
            marker.lifetime = Duration(seconds=lifetime).to_msg()
            markers.markers.append(marker)
            age = max(0.0, now - item['last_seen'])
            label = Marker(header=header, ns='rgbd_labels', id=base_id + 1)
            label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose = ros_pose(item)
            up = Rotation.from_quat(item['orientation_xyzw']).as_matrix()[:, 2]
            position = np.array(item['center']) + up * (item['dimensions'][2] / 2 + 0.08)
            label.pose.position.x, label.pose.position.y, label.pose.position.z = position.tolist()
            label.scale.z = 0.035
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            dimensions = ' x '.join(
                f'{size:.3f}' for size in item['dimensions'][: 2 if is_table else 3]
            )
            label.text = (
                f'{"Mesa" if is_table else "Caja"} {item["id"]}\n'
                f'{dimensions} m | edad {age:.2f} s\n'
                f'{"confirmada" if item["confirmed"] else "provisional"}'
            )
            if item.get('yaw_ambiguous'):
                label.text += ' | orientacion ambigua'
            label.lifetime = marker.lifetime
            markers.markers.append(label)
            for axis in range(3):
                arrow = Marker(header=header, ns='rgbd_axes', id=base_id + 2 + axis)
                arrow.type, arrow.action = Marker.ARROW, Marker.ADD
                arrow.pose = ros_pose(item)
                tip = [0.0, 0.0, 0.0]
                tip[axis] = 0.12
                arrow.points = [Point(), Point(x=tip[0], y=tip[1], z=tip[2])]
                arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.004, 0.009, 0.018
                setattr(arrow.color, ('r', 'g', 'b')[axis], 1.0)
                arrow.color.a = 1.0
                arrow.lifetime = marker.lifetime
                markers.markers.append(arrow)
    return markers


class Perception(Node):
    def __init__(self):
        super().__init__('g1_perception')
        self.declare_parameter('processing_frame', 'base_link')
        self.declare_parameter('target_frame', 'pelvis')
        self.declare_parameter('tracking_frame', 'odom')
        self.declare_parameter('max_age', 0.5)
        self.declare_parameter('update_period', 0.5)
        self.settings = Settings()
        for key, value in vars(self.settings).items():
            self.declare_parameter(key, value)
            setattr(self.settings, key, self.get_parameter(key).value)
        self.settings.validate()
        tracking = TrackingSettings()
        for key, value in vars(tracking).items():
            self.declare_parameter('tracking.' + key, value)
            setattr(tracking, key, self.get_parameter('tracking.' + key).value)
        self.tracker = Tracker(tracking)
        if (
            min(self.get_parameter('max_age').value, self.get_parameter('update_period').value)
            <= 0
        ):
            raise ValueError('max_age and update_period must be positive')
        self.info = None
        self.pending = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.result_pub = self.create_publisher(String, '/perception/detections', 10)
        self.table_pub = self.create_publisher(PoseStamped, '/perception/table_pose', 10)
        self.box_pub = self.create_publisher(PoseArray, '/perception/box_poses', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/perception/markers', 10)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.calibration, 10)
        self.rgb = Subscriber(
            self, Image, '/camera/color/image_raw', qos_profile=qos_profile_sensor_data
        )
        self.depth = Subscriber(
            self, Image, '/camera/depth/image_rect_raw', qos_profile=qos_profile_sensor_data
        )
        self.sync = ApproximateTimeSynchronizer([self.rgb, self.depth], 5, 0.02)
        self.sync.registerCallback(self.capture)
        self.create_timer(self.get_parameter('update_period').value, self.process)

    def calibration(self, info):
        self.info = info

    def capture(self, rgb, depth):
        self.pending = rgb, depth

    def clear(self, status, stamp=None):
        header = PoseArray().header
        header.frame_id = self.get_parameter('target_frame').value
        header.stamp = stamp or self.get_clock().now().to_msg()
        self.publish([], header, status)

    def process(self):
        if self.pending is None:
            self.clear('waiting_for_rgbd')
            return
        rgb, depth = self.pending
        self.pending = None
        if self.info is None:
            self.clear('waiting_for_calibration', depth.header.stamp)
            return
        try:
            capture_time = Time.from_msg(depth.header.stamp)
            age = (self.get_clock().now() - capture_time).nanoseconds / 1e9
            if age < -0.02 or age > self.get_parameter('max_age').value:
                raise ValueError('RGB-D capture is stale or in the future')
            info = self.info
            if (
                rgb.header.frame_id != depth.header.frame_id
                or info.header.frame_id != depth.header.frame_id
                or (info.width, info.height) != (depth.width, depth.height)
            ):
                raise ValueError('Registered RGB-D calibration/frame mismatch')
            if info.distortion_model not in ('', 'plumb_bob') or any(
                abs(d) > 1e-8 for d in info.d
            ):
                raise ValueError('Input must be rectified with zero distortion')
            processing = self.get_parameter('processing_frame').value
            target = self.get_parameter('target_frame').value
            tracking_frame = self.get_parameter('tracking_frame').value
            camera_to_processing = matrix(
                self.tf.lookup_transform(
                    processing, depth.header.frame_id, capture_time, timeout=Duration(seconds=0.05)
                )
            )
            processing_to_tracking = matrix(
                self.tf.lookup_transform(
                    tracking_frame, processing, capture_time, timeout=Duration(seconds=0.05)
                )
            )
            tracking_to_target = matrix(
                self.tf.lookup_transform(
                    target, tracking_frame, capture_time, timeout=Duration(seconds=0.05)
                )
            )
            points, colors = backproject(decode(depth), decode(rgb), info.k, self.settings)
            tables = detect(
                transform_points(points, camera_to_processing),
                colors,
                self.settings,
                processing_to_tracking,
            )
            tables = self.tracker.update(tables, capture_time.nanoseconds / 1e9)
            tables = [transform_item(table, tracking_to_target) for table in tables]
            header = PoseArray().header
            header.stamp, header.frame_id = depth.header.stamp, target
            self.publish(tables, header, 'ok' if tables else 'no_table')
        except (ValueError, TransformException) as error:
            self.clear('unavailable: ' + str(error), depth.header.stamp)
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)

    def publish(self, tables, header, status):
        now = self.get_clock().now().nanoseconds / 1e9
        for table in tables:
            for item in [table, *table['objects']]:
                item['age_sec'] = max(0.0, now - item['last_seen'])
        payload = {
            'status': status,
            'frame_id': header.frame_id,
            'stamp': {'sec': header.stamp.sec, 'nanosec': header.stamp.nanosec},
            'units': 'meters',
            'tracking_frame': self.get_parameter('tracking_frame').value,
            'tables': tables,
        }
        self.result_pub.publish(String(data=json.dumps(payload, allow_nan=False)))
        poses = PoseArray(header=header)
        poses.poses = [ros_pose(obj) for table in tables for obj in table['objects']]
        markers = visualization(tables, header, now, self.get_parameter('max_age').value)
        if tables:
            self.table_pub.publish(PoseStamped(header=header, pose=ros_pose(tables[0])))
        self.box_pub.publish(poses)
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = Perception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
