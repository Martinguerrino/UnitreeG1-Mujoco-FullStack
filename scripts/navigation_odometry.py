"""LiDAR and RGB-D SE(3) odometry fused by Kalman; no IMU or ground-truth input."""

from collections import deque
from pathlib import Path
import json
import math
import os
import sys

# Keep small numerical solves from spawning BLAS pools that starve DDS/control.
for variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[variable] = '1'

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from scipy.spatial.transform import Rotation
from g1_backend.sensor_odometry import (
    CloudReference,
    PoseKalman,
    matrix_pose,
    pose_matrix,
    transform_points,
)
from g1_backend.rgbd_odometry import RgbdReference


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def matrix_tf(transform):
    q, t = transform.transform.rotation, transform.transform.translation
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


class Tracker:
    """Each sensor maintains its own keyframe trajectory and uncertainty."""

    def __init__(self):
        self.reference = None
        self.anchor = None
        self.mount = None
        self.covariance = np.zeros((6, 6))
        self.timestamp = None
        self.failures = 0

    def initialize(self, reference, mount, pose, timestamp, covariance=None):
        self.reference = reference
        self.mount = mount
        self.anchor = pose
        self.timestamp = timestamp
        self.failures = 0
        if covariance is not None:
            self.covariance = covariance.copy()

    def measurement(self, result, mount, timestamp):
        pose = self.anchor @ self.mount @ result.transform @ np.linalg.inv(mount)
        dt = timestamp - self.timestamp
        delta = np.linalg.inv(self.anchor) @ pose
        if (
            dt <= 0
            or np.linalg.norm(delta[:3, 3]) > min(0.7, 2 * dt + 0.05)
            or Rotation.from_matrix(delta[:3, :3]).magnitude() > min(0.6, 4 * dt + 0.05)
        ):
            return None
        # Convert camera/local translation uncertainty into the odometry axes.
        axes = np.zeros((6, 6))
        axes[:3, :3] = axes[3:, 3:] = (self.anchor @ self.mount)[:3, :3]
        covariance = self.covariance + axes @ result.covariance @ axes.T
        covariance += np.diag([0.0004] * 3 + [0.0001] * 3)
        return pose, covariance, delta


class SensorOdometry(Node):
    def __init__(self):
        super().__init__(
            'g1_sensor_odometry',
            parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)],
        )
        cv2.setNumThreads(1)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tf = TransformBroadcaster(self)
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.scan = self.create_publisher(LaserScan, '/scan', 10)
        self.sources = {
            name: self.create_publisher(Odometry, '/odometry/' + name, 10)
            for name in ('lidar', 'rgbd', 'fused_3d')
        }
        self.status = self.create_publisher(String, '/odometry/status', 10)
        self.create_subscription(
            PointCloud2, '/mid360/points', self.cloud, qos_profile_sensor_data
        )
        for name, topic, kind, qos in (
            ('rgb', '/camera/color/image_raw', Image, qos_profile_sensor_data),
            ('depth', '/camera/depth/image_rect_raw', Image, qos_profile_sensor_data),
            ('info', '/camera/color/camera_info', CameraInfo, 10),
        ):
            self.create_subscription(
                kind, topic, lambda msg, name=name: self.image(name, msg), qos
            )
        self.images = {}
        self.pending = deque(maxlen=12)
        self.filter = PoseKalman()
        self.trackers = {name: Tracker() for name in ('lidar', 'rgbd')}
        self.accepted = {'lidar': 0, 'rgbd': 0}
        self.rejected = {'lidar': 0, 'rgbd': 0}
        self.last_source = {'lidar': None, 'rgbd': None}
        self.last_published = None
        self.last_warning = -math.inf
        self.create_timer(0.01, self.process)

    def cloud(self, message):
        self.pending.append(('lidar', message.header, message))

    def image(self, name, message):
        timestamp = seconds(message.header.stamp)
        frame = self.images.setdefault(timestamp, {})
        frame[name] = message
        if len(frame) == 3:
            self.pending.append(('rgbd', message.header, self.images.pop(timestamp)))
        while len(self.images) > 12:
            self.images.pop(min(self.images))

    def warn(self, message):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_warning > 2:
            self.get_logger().warning(message)
            self.last_warning = now

    def process(self):
        if not self.pending:
            return
        self.pending = deque(
            sorted(self.pending, key=lambda item: seconds(item[1].stamp)), maxlen=12
        )
        name, header, data = self.pending[0]
        timestamp = seconds(header.stamp)
        now = self.get_clock().now().nanoseconds * 1e-9
        # Brief reorder window for independent sensor processes and UDP chunks.
        if now - timestamp < 0.06:
            return
        try:
            mount = matrix_tf(
                self.buffer.lookup_transform(
                    'pelvis', header.frame_id, Time.from_msg(header.stamp)
                )
            )
        except TransformException:
            if now - timestamp > 0.5:
                self.pending.popleft()
                self.warn('No local sensor TF at capture time; discarding frame')
            return
        self.pending.popleft()
        if self.filter.timestamp is not None and timestamp < self.filter.timestamp - 1e-6:
            self.rejected[name] += 1
            return
        if name == 'lidar':
            points = read_points_numpy(data, field_names=['x', 'y', 'z'], skip_nans=True).astype(
                float
            )
            # Keep floor returns: they constrain height, roll and pitch without IMU.
            points = points[np.linalg.norm(points, axis=1) < 18]
            if len(points) < 60:
                self.rejected[name] += 1
                return
            reference_factory = lambda: CloudReference(points)
        else:
            rgb_message, depth_message, info = data['rgb'], data['depth'], data['info']
            if (
                rgb_message.encoding != 'rgb8'
                or depth_message.encoding != '32FC1'
                or rgb_message.header.frame_id != depth_message.header.frame_id
                or rgb_message.header.frame_id != info.header.frame_id
                or (info.height, info.width) != (rgb_message.height, rgb_message.width)
                or (rgb_message.height, rgb_message.width)
                != (depth_message.height, depth_message.width)
            ):
                self.rejected[name] += 1
                return
            rgb = np.ndarray(
                (rgb_message.height, rgb_message.width, 3),
                dtype=np.uint8,
                buffer=rgb_message.data,
                strides=(rgb_message.step, 3, 1),
            ).copy()
            dtype = '>f4' if depth_message.is_bigendian else '<f4'
            depth = np.ndarray(
                (depth_message.height, depth_message.width),
                dtype=dtype,
                buffer=depth_message.data,
                strides=(depth_message.step, 4),
            ).astype(np.float32)
            intrinsic = np.asarray(info.k).reshape(3, 3)
            if not np.isfinite(intrinsic).all() or min(intrinsic[0, 0], intrinsic[1, 1]) <= 0:
                self.rejected[name] += 1
                return
            reference_factory = lambda: RgbdReference(rgb, depth, intrinsic)
        self.filter.predict(timestamp)
        tracker = self.trackers[name]
        if tracker.reference is None:
            tracker.initialize(
                reference_factory(),
                mount,
                pose_matrix(self.filter.state[:6]),
                timestamp,
                self.filter.covariance[:6, :6] * 4
                if self.filter.last_measurement is not None
                else None,
            )
            # A keyframe initializes a coordinate system; it is not a movement measurement.
            return
        predicted = (
            np.linalg.inv(tracker.anchor @ tracker.mount)
            @ pose_matrix(self.filter.state[:6])
            @ mount
        )
        if name == 'lidar':
            result = tracker.reference.register(points, predicted)
        else:
            result = tracker.reference.register(rgb, depth, intrinsic, predicted)
        measurement = None if result is None else tracker.measurement(result, mount, timestamp)
        accepted = False
        if measurement is not None:
            pose, covariance, delta = measurement
            accepted = self.filter.update(matrix_pose(pose), covariance)
            if accepted:
                self.accepted[name] += 1
                tracker.failures = 0
                self.last_source[name] = timestamp
                self.sources[name].publish(
                    self.message(header.stamp, matrix_pose(pose), covariance)
                )
                if (
                    np.linalg.norm(delta[:3, 3]) > 0.25
                    or Rotation.from_matrix(delta[:3, :3]).magnitude() > 0.15
                ):
                    tracker.initialize(reference_factory(), mount, pose, timestamp, covariance)
        if not accepted:
            self.rejected[name] += 1
            tracker.failures += 1
            other = 'rgbd' if name == 'lidar' else 'lidar'
            other_time = self.last_source[other]
            if tracker.failures >= 5 and other_time is not None and timestamp - other_time <= 0.2:
                # Reacquire a local reference after losing overlap/features.
                # Its origin comes from the other sensor, with inflated uncertainty.
                tracker.initialize(
                    reference_factory(),
                    mount,
                    pose_matrix(self.filter.state[:6]),
                    timestamp,
                    self.filter.covariance[:6, :6] * 4,
                )
            self.warn(f'{name} registration/update rejected; using other sensor if available')
        status = {
            'accepted': self.accepted,
            'rejected': self.rejected,
            'last_source': self.last_source,
            'source': name,
            'valid': accepted,
            'time': timestamp,
            'method': None if result is None else result.method,
            'rmse': None if result is None else result.rmse,
        }
        self.status.publish(String(data=json.dumps(status)))
        if accepted:
            self.publish(header.stamp)
        if (
            name == 'lidar'
            and self.filter.last_measurement is not None
            and timestamp - self.filter.last_measurement <= 0.2
        ):
            self.publish_scan(points, mount, header.stamp)

    def message(self, stamp, pose, covariance):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = 'odom'
        message.child_frame_id = 'pelvis'
        p = message.pose.pose.position
        p.x, p.y, p.z = map(float, pose[:3])
        q = message.pose.pose.orientation
        q.x, q.y, q.z, q.w = map(float, Rotation.from_euler('xyz', pose[3:]).as_quat())
        message.pose.covariance = covariance.ravel().tolist()
        return message

    def publish(self, stamp):
        pose, covariance = self.filter.state[:6], self.filter.covariance[:6, :6]
        full = self.message(stamp, pose, covariance)
        self.sources['fused_3d'].publish(full)
        planar = self.message(stamp, np.r_[pose[:2], 0, 0, 0, pose[5]], covariance)
        planar.child_frame_id = 'base_link'
        planar_covariance = np.zeros((6, 6))
        for i in (0, 1, 5):
            for j in (0, 1, 5):
                planar_covariance[i, j] = covariance[i, j]
        planar_covariance[2, 2] = planar_covariance[3, 3] = planar_covariance[4, 4] = 1e6
        planar.pose.covariance = planar_covariance.ravel().tolist()
        c, s = math.cos(pose[5]), math.sin(pose[5])
        axes = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
        velocity = axes @ self.filter.state[6:9]
        planar.twist.twist.linear.x, planar.twist.twist.linear.y = map(float, velocity[:2])
        planar.twist.twist.angular.z = float(self.filter.state[11])
        velocity_axes = np.eye(6)
        velocity_axes[:3, :3] = axes
        planar.twist.covariance = (
            (velocity_axes @ self.filter.covariance[6:, 6:] @ velocity_axes.T).ravel().tolist()
        )
        self.odom.publish(planar)
        base = TransformStamped()
        base.header = planar.header
        base.child_frame_id = 'base_link'
        base.transform.translation.x, base.transform.translation.y = map(float, pose[:2])
        base.transform.rotation = planar.pose.pose.orientation
        pelvis = TransformStamped()
        pelvis.header.stamp = stamp
        pelvis.header.frame_id = 'base_link'
        pelvis.child_frame_id = 'pelvis'
        pelvis.transform.translation.z = float(pose[2])
        # Remove yaw from the full estimated rotation, keeping estimated roll/pitch.
        residual = Rotation.from_euler('z', -pose[5]) * Rotation.from_euler('xyz', pose[3:])
        q = pelvis.transform.rotation
        q.x, q.y, q.z, q.w = map(float, residual.as_quat())
        self.tf.sendTransform([base, pelvis])
        self.last_published = seconds(stamp)

    def publish_scan(self, points, mount, stamp):
        # Project measured returns using only the fused sensor pose.
        world = transform_points(points, pose_matrix(self.filter.state[:6]) @ mount)
        pose = self.filter.state[:6]
        delta = world[:, :2] - pose[:2]
        distances = np.linalg.norm(delta, axis=1)
        angles = (np.arctan2(delta[:, 1], delta[:, 0]) - pose[5] + math.pi) % (2 * math.pi)
        bins = np.floor(angles * 360 / (2 * math.pi) + 0.5).astype(int) % 360
        valid = (
            (world[:, 2] >= 0.15) & (world[:, 2] <= 1.5) & (distances >= 0.05) & (distances <= 20)
        )
        ranges = np.full(360, np.inf)
        np.minimum.at(ranges, bins[valid], distances[valid])
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'laser'
        scan.angle_min = -math.pi
        scan.angle_increment = 2 * math.pi / 360
        scan.angle_max = scan.angle_min + 359 * scan.angle_increment
        scan.range_min, scan.range_max = 0.05, 20.0
        scan.scan_time = 0.1
        scan.ranges = ranges.tolist()
        self.scan.publish(scan)


def main():
    rclpy.init()
    node = SensorOdometry()
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
