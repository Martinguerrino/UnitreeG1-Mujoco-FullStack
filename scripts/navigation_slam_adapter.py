"""Adapt native FAST-LIO/RTAB-Map estimates to Nav2 and persist 3-D maps."""
from collections import deque
from pathlib import Path
import copy
import math
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points, read_points_numpy
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformListener, StaticTransformBroadcaster, TransformException
from g1_backend.slam_mapping import (
    OccupancyProjection, VoxelMap, navigation_pose, pose_matrix, write_pcd,
)


def matrix_transform(message):
    p, q = message.transform.translation, message.transform.rotation
    return pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def transform_message(matrix, parent, child, stamp):
    message = TransformStamped()
    message.header.frame_id, message.child_frame_id = parent, child
    message.header.stamp = stamp
    p, q = message.transform.translation, message.transform.rotation
    p.x, p.y, p.z = map(float, matrix[:3, 3])
    q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(matrix[:3, :3]).as_quat())
    return message


class SlamAdapter(Node):
    def __init__(self):
        super().__init__('g1_slam_adapter')
        self.backend = self.declare_parameter('backend', 'lidar3d').value
        self.output = Path(self.declare_parameter('output_dir', 'results/navigation/slam').value)
        self.output.mkdir(parents=True, exist_ok=True)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.offset = np.array([-0.0435535, -0.00224, 0.96592]) if self.backend == 'lidar3d' else np.array([0., 0., 0.793])
        self.source_frame = 'torso_imu' if self.backend == 'lidar3d' else 'pelvis'
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.full_odom = self.create_publisher(Odometry, '/odometry/slam_3d', 10)
        self.scan = self.create_publisher(LaserScan, '/scan', 10)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_points = self.create_publisher(PointCloud2, '/slam/map_points', map_qos)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/map', map_qos) if self.backend == 'lidar3d' else None
        self.voxels = VoxelMap()
        self.projection = OccupancyProjection()
        self.rgb_cloud = None
        self.pending = deque(maxlen=20)
        self.pending_scans = deque(maxlen=10)
        self.previous = None
        self.current_pose = None
        self.latest_stamp = None
        self.last_projection = -math.inf
        self.create_subscription(Odometry, '/slam/odometry', lambda msg: self.pending.append(msg), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/mid360/points',
                                 lambda msg: self.pending_scans.append(msg), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/slam/registered_points' if self.backend == 'lidar3d' else '/slam/map_points',
                                 self.cloud, qos_profile_sensor_data)
        self.create_timer(0.01, self.process)
        self.create_timer(0.01, self.process_scans)
        self.create_timer(1.0, self.publish_map)
        self.create_timer(30.0, self.save_map)
        self.create_service(Trigger, '/slam/save_map', self.save_service)
        if self.backend == 'lidar3d':
            # FAST-LIO initializes at the IMU origin. Anchor it at nominal standing
            # height, not ground-truth telemetry. Both public maps use floor z=0.
            initial = np.eye(4)
            initial[:3, 3] = self.offset
            self.static_tf.sendTransform([
                transform_message(np.eye(4), 'map', 'odom', self.get_clock().now().to_msg()),
                transform_message(initial, 'odom', 'camera_init', self.get_clock().now().to_msg()),
            ])

    def process(self):
        if not self.pending:
            return
        raw = self.pending[0]
        try:
            mount = matrix_transform(self.buffer.lookup_transform(
                'pelvis', self.source_frame, Time.from_msg(raw.header.stamp)))
        except TransformException:
            if self.get_clock().now().nanoseconds * 1e-9 - self.seconds(raw.header.stamp) > 0.5:
                self.pending.popleft()
            return
        self.pending.popleft()
        if raw.pose.covariance[0] < 0:
            return  # RTAB-Map publishes null odometry when tracking is lost.
        p, q = raw.pose.pose.position, raw.pose.pose.orientation
        try:
            sensor = pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        except ValueError:
            return
        if not np.isfinite(sensor).all():
            return
        timestamp = self.seconds(raw.header.stamp)
        if self.previous is not None and timestamp <= self.previous[0]:
            return
        base, pelvis = navigation_pose(sensor, mount, self.offset)
        full = copy.deepcopy(raw)
        full.header.frame_id, full.child_frame_id = 'odom', 'pelvis'
        estimated_pelvis = base @ pelvis
        full.pose.pose.position.x, full.pose.pose.position.y, full.pose.pose.position.z = map(float, estimated_pelvis[:3, 3])
        full.pose.pose.orientation = transform_message(estimated_pelvis, '', '', raw.header.stamp).transform.rotation
        self.full_odom.publish(full)
        planar = copy.deepcopy(full)
        planar.child_frame_id = 'base_link'
        planar.pose.pose.position.z = 0.0
        planar.pose.pose.orientation = transform_message(base, '', '', raw.header.stamp).transform.rotation
        planar.twist.twist.linear.x = planar.twist.twist.linear.y = planar.twist.twist.linear.z = 0.0
        planar.twist.twist.angular.x = planar.twist.twist.angular.y = planar.twist.twist.angular.z = 0.0
        if self.previous is not None:
            old_time, old_base = self.previous
            dt = timestamp - old_time
            velocity = base[:3, :3].T @ (base[:3, 3] - old_base[:3, 3]) / dt
            planar.twist.twist.linear.x, planar.twist.twist.linear.y = map(float, velocity[:2])
            planar.twist.twist.angular.z = float(Rotation.from_matrix(old_base[:3, :3].T @ base[:3, :3]).as_rotvec()[2] / dt)
        self.tf.sendTransform([
            transform_message(base, 'odom', 'base_link', raw.header.stamp),
            transform_message(pelvis, 'base_link', 'pelvis', raw.header.stamp),
        ])
        self.odom.publish(planar)
        self.previous = (timestamp, base)
        self.current_pose, self.latest_stamp = estimated_pelvis, raw.header.stamp

    @staticmethod
    def seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def cloud(self, message):
        if self.backend == 'rgb-d':
            # This is RTAB-Map's complete optimized cloud, not accumulated raw
            # frames: retaining it respects corrections after loop closure.
            self.rgb_cloud = message
            return
        xyz = read_points_numpy(message, field_names=['x', 'y', 'z'], skip_nans=True).astype(float)
        xyz += self.offset
        self.voxels.add(xyz)
        timestamp = self.seconds(message.header.stamp)
        if self.current_pose is not None and timestamp - self.last_projection >= 0.5:
            self.projection.add(xyz, self.current_pose[:3, 3])
            self.last_projection = timestamp
        self.latest_stamp = message.header.stamp

    def process_scans(self):
        # Sensor clouds arrive before the estimated base -> pelvis transform.
        # Retain them until odometry catches up, rather than losing every scan.
        now = self.get_clock().now().nanoseconds * 1e-9
        while self.pending_scans:
            message = self.pending_scans[0]
            if now - self.seconds(message.header.stamp) > 0.5:
                self.pending_scans.popleft()
                continue
            if not self.scan_cloud(message):
                break
            self.pending_scans.popleft()

    def scan_cloud(self, message):
        if self.current_pose is None:
            return False
        try:
            local = matrix_transform(self.buffer.lookup_transform('base_link', message.header.frame_id, Time.from_msg(message.header.stamp)))
        except TransformException:
            return False
        points = read_points_numpy(message, field_names=['x', 'y', 'z'], skip_nans=True).astype(float)
        points = points @ local[:3, :3].T + local[:3, 3]
        ranges = np.linalg.norm(points[:, :2], axis=1)
        valid = (points[:, 2] >= 0.15) & (points[:, 2] <= 1.5) & (ranges >= 0.05) & (ranges <= 20)
        angles = (np.arctan2(points[:, 1], points[:, 0]) + math.pi) % (2 * math.pi)
        bins = np.floor(angles * 360 / (2 * math.pi) + 0.5).astype(int) % 360
        scan = LaserScan()
        scan.header = Header(stamp=message.header.stamp, frame_id='laser')
        scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / 360
        scan.angle_max = scan.angle_min + 359 * scan.angle_increment
        scan.range_min, scan.range_max, scan.scan_time = 0.05, 20.0, 0.1
        nearest = np.full(360, np.inf)
        np.minimum.at(nearest, bins[valid], ranges[valid])
        scan.ranges = nearest.tolist()
        self.scan.publish(scan)
        return True

    def publish_map(self):
        if self.backend != 'lidar3d' or self.latest_stamp is None:
            return
        xyz = self.voxels.points()
        if len(xyz):
            self.map_points.publish(create_cloud_xyz32(Header(stamp=self.latest_stamp, frame_id='map'), xyz))
        projected = self.projection.grid()
        if projected is not None:
            grid, origin = projected
            message = OccupancyGrid()
            message.header = Header(stamp=self.latest_stamp, frame_id='map')
            message.info.resolution = self.projection.resolution
            message.info.height, message.info.width = grid.shape
            message.info.origin.position.x, message.info.origin.position.y = map(float, origin)
            message.info.origin.orientation.w = 1.0
            message.data = grid.ravel().tolist()
            self.grid_pub.publish(message)

    def save_map(self):
        if self.backend == 'lidar3d':
            xyz, rgb = self.voxels.points(), None
        elif self.rgb_cloud is not None:
            cloud = read_points(self.rgb_cloud, skip_nans=False)
            xyz = np.column_stack([cloud[name].ravel() for name in ('x', 'y', 'z')])
            rgb = cloud['rgb'].ravel() if 'rgb' in cloud.dtype.names else None
        else:
            return 0
        if not len(xyz):
            return 0
        return write_pcd(self.output / 'map.pcd', xyz, rgb)

    def save_service(self, request, response):
        try:
            count = self.save_map()
            response.success = count > 0
            response.message = f'{count} points: {self.output / "map.pcd"}'
        except (OSError, ValueError) as error:
            response.success, response.message = False, str(error)
        return response


def main():
    rclpy.init()
    node = SlamAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        count = node.save_map()
        print(f'SLAM map saved: {node.output / "map.pcd"} ({count} points)', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
