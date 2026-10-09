"""ROS-only process: no SDK2/CycloneDDS libraries loaded into the ROS runtime."""

import json
import io
import math
import socket
import struct
import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64, String
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2, PointField
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def stamp(seconds):
    from builtin_interfaces.msg import Time

    nanoseconds = round(seconds * 1e9)
    return Time(sec=nanoseconds // 10**9, nanosec=nanoseconds % 10**9)


class Bridge(Node):
    def __init__(self):
        super().__init__('g1_navigation_bridge')
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', 19420))
        self.socket.setblocking(False)
        self.sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sensor_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        self.sensor_socket.bind(('127.0.0.1', 19422))
        self.sensor_socket.setblocking(False)
        self.real_odom = self.declare_parameter('real_odom', False).value
        self.last_estimated_time = -math.inf
        self.last_sim_time = 0.0
        if self.real_odom:
            self.create_subscription(Odometry, '/odom', self.estimated_odometry, 10)
        self.feet = self.create_publisher(String, '/feet_estimation/state', 100)
        self.imu = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        # Reliable delivery also matches FAST-LIO2's upstream IMU subscription.
        self.torso_imu = self.create_publisher(Imu, '/imu/torso', 100)
        self.truth = self.create_publisher(Odometry, '/ground_truth/odom', 10)
        self.clock = self.create_publisher(Clock, '/clock', 10)
        if not self.real_odom:
            self.odom = self.create_publisher(Odometry, '/odom', 10)
            self.scan = self.create_publisher(LaserScan, '/scan', 10)
        self.points = self.create_publisher(PointCloud2, '/mid360/points', qos_profile_sensor_data)
        self.color = self.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
        self.depth_preview_pub = self.create_publisher(Image, '/camera/depth/preview', qos_profile_sensor_data)
        self.depth = self.create_publisher(Image, '/camera/depth/image_rect_raw', qos_profile_sensor_data)
        self.camera_pitch_state = self.create_publisher(Float64, '/camera/pitch_state', 10)
        self.camera_info = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.joint_states = self.create_publisher(JointState, '/joint_states', 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        transforms = []
        for parent, child, xyz, quaternion in (
            ('base_link', 'laser', (0.0, 0.0, 0.78), (1.0, 0.0, 0.0, 0.0)),
            ('torso_link', 'torso_imu', (-0.03959, -0.00224, 0.11892),
             (1.0, 0.0, 0.0, 0.0)),
        ):
            sensor = TransformStamped()
            sensor.header.frame_id = parent
            sensor.child_frame_id = child
            sensor.transform.translation.x = xyz[0]
            sensor.transform.translation.y = xyz[1]
            sensor.transform.translation.z = xyz[2]
            (sensor.transform.rotation.w, sensor.transform.rotation.x,
             sensor.transform.rotation.y, sensor.transform.rotation.z) = quaternion
            transforms.append(sensor)
        self.static_tf.sendTransform(transforms)
        from navigation_camera_controls import CameraControls
        self.camera_controls = CameraControls(self)
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.create_timer(0.005, self.poll)
        self.last_scan = None
        self.last_received = -math.inf
        self.previous_pose = None
        self.sensor_chunks = {}

    def estimated_odometry(self, message):
        self.last_estimated_time = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def command(self, message):
        values = [message.linear.x, message.linear.y, message.angular.z]
        if not all(math.isfinite(value) for value in values):
            return
        if time.monotonic() - self.last_received > 0.5:
            values = [0.0, 0.0, 0.0]
        if self.real_odom and self.last_sim_time - self.last_estimated_time > 0.3:
            values = [0.0, 0.0, 0.0]
        self.socket.sendto(json.dumps({'velocity': values}).encode(), ('127.0.0.1', 19421))

    def poll(self):
        for _ in range(100):
            try:
                payload, _ = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            row = json.loads(payload)
            self.last_received = time.monotonic()
            self.last_sim_time = row['time']
            timestamp = stamp(row['time'])
            self.clock.publish(Clock(clock=timestamp))
            for sample in row.get('feet_estimation', []):
                self.feet.publish(String(data=json.dumps(sample)))
            for sample in row.get('torso_imu', []):
                inertial = Imu()
                inertial.header.stamp = stamp(sample['time'])
                inertial.header.frame_id = 'torso_imu'
                # FAST-LIO estimates orientation; no simulator quaternion input.
                inertial.orientation_covariance[0] = -1.0
                (inertial.angular_velocity.x, inertial.angular_velocity.y,
                 inertial.angular_velocity.z) = map(float, sample['gyro'])
                (inertial.linear_acceleration.x, inertial.linear_acceleration.y,
                 inertial.linear_acceleration.z) = map(float, sample['acc'])
                inertial.angular_velocity_covariance = [2.5e-7, 0., 0., 0., 2.5e-7, 0., 0., 0., 2.5e-7]
                inertial.linear_acceleration_covariance = [1e-4, 0., 0., 0., 1e-4, 0., 0., 0., 1e-4]
                self.torso_imu.publish(inertial)
            # JSON numbers may be integers; ROS C serialization requires floats.
            w, x, y, z = map(float, row['quaternion'])
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            odom = Odometry()
            odom.header.stamp = timestamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = map(float, row['position'][:2])
            odom.pose.pose.orientation.z = math.sin(yaw / 2)
            odom.pose.pose.orientation.w = math.cos(yaw / 2)
            vx, vy = map(float, row['linear_velocity'][:2])
            odom.twist.twist.linear.x = math.cos(yaw) * vx + math.sin(yaw) * vy
            odom.twist.twist.linear.y = -math.sin(yaw) * vx + math.cos(yaw) * vy
            if self.previous_pose is not None:
                previous_time, previous_yaw = self.previous_pose
                dt = row['time'] - previous_time
                if dt > 0:
                    delta = yaw - previous_yaw
                    odom.twist.twist.angular.z = math.atan2(math.sin(delta), math.cos(delta)) / dt
            self.previous_pose = (row['time'], yaw)
            real = getattr(self, 'real_odom', False)
            if real:
                truth = Odometry()
                truth.header.stamp = timestamp
                truth.header.frame_id = 'ground_truth'
                truth.child_frame_id = 'ground_truth_base'
                truth.pose = odom.pose
                truth.twist = odom.twist
                self.truth.publish(truth)
            else:
                self.odom.publish(odom)
            if 'imu' in row:
                imu = Imu()
                imu.header.stamp = timestamp
                imu.header.frame_id = 'pelvis'
                (imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z) = map(float, row['imu']['quat'])
                (imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z) = map(float, row['imu']['gyro'])
                (imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z) = map(float, row['imu']['acc'])
                imu.orientation_covariance = [0.0001, 0., 0., 0., 0.0001, 0., 0., 0., 0.0001]
                imu.angular_velocity_covariance = [0.0001, 0., 0., 0., 0.0001, 0., 0., 0., 0.0001]
                imu.linear_acceleration_covariance = [0.01, 0., 0., 0., 0.01, 0., 0., 0., 0.01]
                self.imu.publish(imu)
            joints = JointState()
            joints.header.stamp = timestamp
            joints.name = row['joint_names']
            joints.position = list(map(float, row['joint_positions']))
            self.joint_states.publish(joints)
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = 'base_link'
            transform.transform.translation.x = odom.pose.pose.position.x
            transform.transform.translation.y = odom.pose.pose.position.y
            transform.transform.rotation = odom.pose.pose.orientation
            pelvis = TransformStamped()
            pelvis.header.stamp = timestamp
            pelvis.header.frame_id = 'base_link'
            pelvis.child_frame_id = 'pelvis'
            pelvis.transform.translation.z = 0.793 if real else float(row['position'][2])
            # Remove only yaw; retain the physical pelvis roll and pitch.
            c, s = math.cos(yaw / 2), math.sin(yaw / 2)
            (pelvis.transform.rotation.w, pelvis.transform.rotation.x,
             pelvis.transform.rotation.y, pelvis.transform.rotation.z) = (
                c*w + s*z, c*x + s*y, c*y - s*x, c*z - s*w)
            if not real:
                self.tf.sendTransform([transform, pelvis])
            if not real and row['scan'] is not None and row['scan_time'] != self.last_scan:
                self.last_scan = row['scan_time']
                scan = LaserScan()
                scan.header.stamp = stamp(row['scan_time'])
                scan.header.frame_id = 'laser'
                scan.angle_min = -math.pi
                scan.angle_increment = 2 * math.pi / len(row['scan'])
                scan.angle_max = scan.angle_min + (len(row['scan']) - 1) * scan.angle_increment
                scan.scan_time = 0.1
                scan.range_min, scan.range_max = 0.05, 20.0
                scan.ranges = [float('inf') if v is None else float(v) for v in row['scan']]
                self.scan.publish(scan)

        self.poll_sensors()

    def poll_sensors(self):
        for _ in range(100):
            try:
                packet, _ = self.sensor_socket.recvfrom(65535)
            except BlockingIOError:
                return
            if len(packet) < 8:
                continue
            sequence, index, count = struct.unpack('!IHH', packet[:8])
            if not 0 < count <= 128 or index >= count:
                continue
            now = time.monotonic()
            self.sensor_chunks = {key: value for key, value in self.sensor_chunks.items()
                                  if now - value[0] < 0.5}
            if len(self.sensor_chunks) >= 32 and sequence not in self.sensor_chunks:
                self.sensor_chunks.pop(next(iter(self.sensor_chunks)))
            _, chunks = self.sensor_chunks.setdefault(sequence, (now, [None] * count))
            if len(chunks) != count or index >= count:
                self.sensor_chunks.pop(sequence, None)
                continue
            chunks[index] = packet[8:]
            if all(chunk is not None for chunk in chunks):
                payload = b''.join(chunks)
                self.sensor_chunks.pop(sequence, None)
                try:
                    with np.load(io.BytesIO(payload), allow_pickle=False) as values:
                        self.publish_sensors(values)
                except (ValueError, OSError):
                    self.get_logger().warning('Discarding malformed simulated sensor frame')

    def publish_sensors(self, values):
        timestamp = stamp(float(values['time']))
        if 'd435i_rgb' in values or 'd435i_depth' in values:
            # Capture-specific angle travels in the same packet as each image.
            angle = math.radians(float(values.get('camera_pitch_deg', 0.0))) / 2
            c, s = math.cos(angle), -math.sin(angle)
            # Original optical transform wxyz=(.5,-.5,.5,-.5).
            optical = TransformStamped()
            optical.header.stamp = timestamp
            optical.header.frame_id = 'd435_link'
            optical.child_frame_id = 'd435i_color_optical_frame'
            optical.transform.rotation.w = .5*c + .5*s
            optical.transform.rotation.x = .5*s - .5*c
            optical.transform.rotation.y = .5*c - .5*s
            optical.transform.rotation.z = -.5*c - .5*s
            self.tf.sendTransform(optical)
        if 'mid360_points' in values:
            xyz = np.ascontiguousarray(values['mid360_points'], dtype=np.float32)
            cloud = PointCloud2()
            cloud.header.stamp = timestamp
            cloud.header.frame_id = 'mid360_link'
            cloud.height, cloud.width = 1, len(xyz)
            cloud.fields = [
                PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                for name, offset in (('x', 0), ('y', 4), ('z', 8))
            ]
            # The ray sensor is an instantaneous XYZ snapshot, not a timed scan.
            # Supply intensity for FAST-LIO's generic PointXYZI input path.
            xyzi = np.zeros((len(xyz), 4), dtype=np.float32)
            xyzi[:, :3] = xyz
            cloud.fields.append(PointField(name='intensity', offset=12,
                                           datatype=PointField.FLOAT32, count=1))
            cloud.is_bigendian = False
            cloud.point_step = 16
            cloud.row_step = cloud.point_step * cloud.width
            cloud.is_dense = True
            cloud.data = xyzi.tobytes()
            self.points.publish(cloud)
        if 'd435i_rgb' in values:
            rgb = np.ascontiguousarray(values['d435i_rgb'], dtype=np.uint8)
            image = Image()
            image.header.stamp = timestamp
            image.header.frame_id = 'd435i_color_optical_frame'
            image.height, image.width = rgb.shape[:2]
            image.encoding = 'rgb8'
            image.is_bigendian = False
            image.step = image.width * 3
            image.data = rgb.tobytes()
            self.color.publish(image)
            self.publish_camera_info(timestamp, image.width, image.height)
            self.camera_pitch_state.publish(Float64(data=float(values.get('camera_pitch_deg', 0.0))))
        if 'd435i_depth' in values:
            depth = np.ascontiguousarray(values['d435i_depth'], dtype=np.float32)
            image = Image()
            image.header.stamp = timestamp
            image.header.frame_id = 'd435i_color_optical_frame'
            image.height, image.width = depth.shape
            image.encoding = '32FC1'
            image.is_bigendian = False
            image.step = image.width * 4
            image.data = depth.tobytes()
            self.depth.publish(image)
            # Fixed color scale avoids far-clip/background dominating RViz normalization.
            from g1_backend.depth_preview import depth_preview
            preview = depth_preview(depth)
            shown = Image()
            shown.header = image.header
            shown.height, shown.width = depth.shape
            shown.encoding = 'rgb8'
            shown.step = shown.width * 3
            shown.data = preview.tobytes()
            self.depth_preview_pub.publish(shown)

    def publish_camera_info(self, timestamp, width, height):
        fy = height / (2 * math.tan(math.radians(58) / 2))
        fx = fy
        cx, cy = (width - 1) / 2, (height - 1) / 2
        info = CameraInfo()
        info.header.stamp = timestamp
        info.header.frame_id = 'd435i_color_optical_frame'
        info.width, info.height = width, height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info.publish(info)


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # A signal can invalidate the ROS context between executor checks.
        if rclpy.ok():
            raise
    finally:
        try:
            node.socket.sendto(json.dumps({'velocity': [0, 0, 0]}).encode(), ('127.0.0.1', 19421))
            node.socket.close()
            node.sensor_socket.close()
            node.camera_controls.socket.close()
        finally:
            if rclpy.ok():
                node.destroy_node()
                rclpy.try_shutdown()


if __name__ == '__main__':
    main()
