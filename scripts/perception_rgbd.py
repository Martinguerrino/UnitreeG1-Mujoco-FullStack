#!/usr/bin/env python3
"""Read registered RGB-D and publish observed geometry in the arm reference frame."""

import json
from pathlib import Path
import sys
import time

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
from g1_backend.perception_free_space import FreeSpaceSettings, attach_free_space
from g1_backend.perception_sync import CaptureQueue
from g1_backend.perception_tracking import Tracker, TrackingSettings, transform_item
from g1_backend.perception_rgbd import RefinementSettings, detect_instances, validated_instances
from g1_backend.perception_views import ViewAccumulator, ViewSettings
from g1_backend.perception_yolo import YoloSegmenter, overlay


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
                semantic = item.get('semantic_class', 'box')
                if semantic == 'cylinder':
                    marker.color.r, marker.color.g = 1.0, 0.65
                elif semantic == 'sphere':
                    marker.color.b = 1.0
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
            name = (
                'Mesa'
                if is_table
                else {'box': 'Caja', 'cylinder': 'Cilindro', 'sphere': 'Esfera'}.get(
                    item.get('semantic_class', 'box'), 'Objeto'
                )
            )
            label.text = (
                f'{name} {item["id"]}\n'
                f'{dimensions} m | edad {age:.2f} s\n'
                f'{"confirmada" if item["confirmed"] else "provisional"}'
                f' | {"estable" if item.get("stable", False) else "inestable"}'
            )
            if item.get('yaw_ambiguous'):
                label.text += ' | orientacion ambigua'
            if 'quality' in item:
                q = item['quality']
                segmentation = (
                    f'YOLO {q["segmentation_confidence"]:.2f}'
                    if q['segmentation_confidence'] is not None
                    else 'plano geometrico'
                )
                label.text += (
                    f'\n{segmentation} | '
                    f'profundidad {q["valid_depth_fraction"]:.0%} | '
                    f'vistas {q.get("view_count", 1)} | {q["status"]}'
                )
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
        for index, region in enumerate(table.get('free_regions', [])):
            patch = Marker(
                header=header, ns='rgbd_free_surface', id=table['track_id'] * 100 + index
            )
            patch.type, patch.action = Marker.CUBE, Marker.ADD
            patch.pose = ros_pose(region)
            up = Rotation.from_quat(region['orientation_xyzw']).as_matrix()[:, 2]
            position = np.array(region['center']) + up * 0.008
            patch.pose.position.x, patch.pose.position.y, patch.pose.position.z = position.tolist()
            patch.scale.x, patch.scale.y = region['dimensions'][:2]
            patch.scale.z = 0.006
            patch.color.g, patch.color.b, patch.color.a = 1.0, 1.0, 0.75
            patch.lifetime = Duration(seconds=lifetime).to_msg()
            markers.markers.append(patch)
    return markers


class Perception(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('g1_perception', parameter_overrides=parameter_overrides)
        self.declare_parameter('processing_frame', 'base_link')
        self.declare_parameter('target_frame', 'pelvis')
        self.declare_parameter('tracking_frame', 'odom')
        self.declare_parameter('max_age', 0.5)
        self.declare_parameter('update_period', 0.5)
        self.declare_parameter('segmentation_backend', 'yolo')
        self.declare_parameter('yolo.model', 'artifacts/perception/g1_objects_yolo11n_seg.onnx')
        self.declare_parameter('yolo.confidence', 0.3)
        self.declare_parameter('yolo.iou', 0.5)
        self.declare_parameter('yolo.threads', 2)
        self.declare_parameter('multiview.enabled', True)
        self.declare_parameter('free_space.enabled', True)
        self.declare_parameter('sync.queue_size', 8)
        self.declare_parameter('sync.tf_wait', 0.3)
        self.declare_parameter('sync.poll_period', 0.02)
        backend = self.get_parameter('segmentation_backend').value
        if backend not in ('yolo', 'geometry'):
            raise ValueError('segmentation_backend must be yolo or geometry')
        self.segmenter = (
            YoloSegmenter(
                self.get_parameter('yolo.model').value,
                self.get_parameter('yolo.confidence').value,
                self.get_parameter('yolo.iou').value,
                self.get_parameter('yolo.threads').value,
            )
            if backend == 'yolo'
            else None
        )
        self.refinement = RefinementSettings()
        for key, value in vars(self.refinement).items():
            self.declare_parameter('refinement.' + key, value)
            setattr(self.refinement, key, self.get_parameter('refinement.' + key).value)
        self.refinement.validate()
        views = ViewSettings()
        for key, value in vars(views).items():
            self.declare_parameter('multiview.' + key, value)
            setattr(views, key, self.get_parameter('multiview.' + key).value)
        self.views = ViewAccumulator(views)
        self.free_settings = FreeSpaceSettings()
        for key, value in vars(self.free_settings).items():
            self.declare_parameter('free_space.' + key, value)
            setattr(self.free_settings, key, self.get_parameter('free_space.' + key).value)
        self.free_settings.validate()
        self.inference_ms = 0.0
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
        self.captures = CaptureQueue(
            self.get_parameter('sync.queue_size').value,
            self.get_parameter('max_age').value,
            self.get_parameter('sync.tf_wait').value,
        )
        self.last_inference = None
        self.last_publish_wall = 0.0
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.result_pub = self.create_publisher(String, '/perception/detections', 10)
        self.table_pub = self.create_publisher(PoseStamped, '/perception/table_pose', 10)
        self.box_pub = self.create_publisher(PoseArray, '/perception/box_poses', 10)
        self.object_pub = self.create_publisher(PoseArray, '/perception/object_poses', 10)
        self.overlay_pub = self.create_publisher(
            Image, '/perception/segmentation/image', qos_profile_sensor_data
        )
        self.raw_overlay_pub = self.create_publisher(
            Image, '/perception/segmentation/raw_image', qos_profile_sensor_data
        )
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
        poll_period = self.get_parameter('sync.poll_period').value
        if not np.isfinite(poll_period) or poll_period <= 0:
            raise ValueError('sync.poll_period must be positive and finite')
        self.create_timer(poll_period, self.process)

    def calibration(self, info):
        self.info = info

    def capture(self, rgb, depth):
        self.captures.push(
            (rgb, depth), Time.from_msg(depth.header.stamp).nanoseconds / 1e9, time.monotonic()
        )

    def clear(self, status, stamp=None):
        header = PoseArray().header
        header.frame_id = self.get_parameter('target_frame').value
        header.stamp = stamp or self.get_clock().now().to_msg()
        self.publish([], header, status)

    def transforms_ready(self, pair, stamp):
        capture_time = Time.from_msg(pair[1].header.stamp)
        processing = self.get_parameter('processing_frame').value
        tracking = self.get_parameter('tracking_frame').value
        target = self.get_parameter('target_frame').value
        return all(
            self.tf.can_transform(to_frame, from_frame, capture_time)
            for to_frame, from_frame in [
                (processing, pair[1].header.frame_id),
                (tracking, processing),
                (target, tracking),
            ]
        )

    def process(self):
        now = self.get_clock().now().nanoseconds / 1e9
        wall = time.monotonic()
        period = self.get_parameter('update_period').value
        if self.last_inference is not None and now < self.last_inference:
            self.last_inference = None
            self.tracker.expire(now)
        due = self.last_inference is None or now - self.last_inference >= period
        capture = self.captures.select(now, wall, self.transforms_ready, consume=due)
        if not due:
            return
        if capture is None:
            if wall - self.last_publish_wall >= self.get_parameter('max_age').value:
                self.clear('waiting_for_tf' if self.captures.queue else 'waiting_for_rgbd')
            return
        rgb, depth = capture.pair
        self.last_inference = now
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
                    processing, depth.header.frame_id, capture_time, timeout=Duration(seconds=0.0)
                )
            )
            processing_to_tracking = matrix(
                self.tf.lookup_transform(
                    tracking_frame, processing, capture_time, timeout=Duration(seconds=0.0)
                )
            )
            tracking_to_target = matrix(
                self.tf.lookup_transform(
                    target, tracking_frame, capture_time, timeout=Duration(seconds=0.0)
                )
            )
            color_image, depth_image = decode(rgb), decode(depth)
            started = time.monotonic()
            if self.segmenter is None:
                points, colors = backproject(depth_image, color_image, info.k, self.settings)
                tables = detect(
                    transform_points(points, camera_to_processing),
                    colors,
                    self.settings,
                    processing_to_tracking,
                )
            else:
                instances = self.segmenter.predict(color_image)
                tables = detect_instances(
                    depth_image,
                    color_image,
                    info.k,
                    instances,
                    camera_to_processing,
                    processing_to_tracking,
                    self.settings,
                    self.refinement,
                )
                validated = validated_instances(
                    depth_image,
                    color_image,
                    info.k,
                    instances,
                    tables,
                    processing_to_tracking @ camera_to_processing,
                    self.settings,
                )
                image = Image(
                    header=rgb.header,
                    height=rgb.height,
                    width=rgb.width,
                    encoding='rgb8',
                    step=rgb.width * 3,
                    data=np.ascontiguousarray(overlay(color_image, validated)).tobytes(),
                )
                self.overlay_pub.publish(image)
                if self.raw_overlay_pub.get_subscription_count():
                    raw = Image(
                        header=rgb.header,
                        height=rgb.height,
                        width=rgb.width,
                        encoding='rgb8',
                        step=rgb.width * 3,
                        data=np.ascontiguousarray(overlay(color_image, instances)).tobytes(),
                    )
                    self.raw_overlay_pub.publish(raw)
            if self.get_parameter('free_space.enabled').value:
                scene, _ = backproject(depth_image, color_image, info.k, self.settings)
                attach_free_space(
                    tables,
                    transform_points(scene, processing_to_tracking @ camera_to_processing),
                    self.free_settings,
                    self.settings.plane_distance,
                    self.settings.min_object_height,
                )
            self.inference_ms = (time.monotonic() - started) * 1000
            tables = self.tracker.update(tables, capture_time.nanoseconds / 1e9)
            if self.get_parameter('multiview.enabled').value:
                tables = self.views.update(
                    tables,
                    processing_to_tracking @ camera_to_processing,
                    capture_time.nanoseconds / 1e9,
                )
            else:
                for table in tables:
                    for item in [table, *table['objects']]:
                        for key in list(item):
                            if key.startswith('_'):
                                item.pop(key)
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
                if item['age_sec'] > self.get_parameter('max_age').value and 'stability' in item:
                    item['stable'] = False
                    item['stability']['status'] = 'stale'
                    item['stability']['reasons'].append('stale_observation')
            if table['age_sec'] > self.get_parameter('max_age').value and 'free_space' in table:
                table['free_regions'] = []
                table['free_space']['area_m2'] = 0.0
                table['free_space']['status'] = 'stale_observation'
        self.last_publish_wall = time.monotonic()
        payload = {
            'status': status,
            'frame_id': header.frame_id,
            'stamp': {'sec': header.stamp.sec, 'nanosec': header.stamp.nanosec},
            'units': 'meters',
            'tracking_frame': self.get_parameter('tracking_frame').value,
            'segmentation_backend': self.get_parameter('segmentation_backend').value,
            'inference_ms': self.inference_ms,
            'synchronization': {
                'queued': len(self.captures.queue),
                'dropped': self.captures.dropped.copy(),
            },
            'tables': tables,
        }
        self.result_pub.publish(String(data=json.dumps(payload, allow_nan=False)))
        poses = PoseArray(header=header)
        poses.poses = [
            ros_pose(obj)
            for table in tables
            for obj in table['objects']
            if obj['kind'] == 'box_candidate'
        ]
        objects = PoseArray(
            header=header, poses=[ros_pose(obj) for table in tables for obj in table['objects']]
        )
        markers = visualization(tables, header, now, self.get_parameter('max_age').value)
        if tables:
            self.table_pub.publish(PoseStamped(header=header, pose=ros_pose(tables[0])))
        self.box_pub.publish(poses)
        self.object_pub.publish(objects)
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
