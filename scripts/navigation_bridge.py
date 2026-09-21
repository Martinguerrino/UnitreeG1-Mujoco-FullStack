"""ROS-only process: no SDK2/CycloneDDS libraries loaded into the ROS runtime."""

import json
import io
import math
import socket
import struct
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2, PointField
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
        self.clock = self.create_publisher(Clock, '/clock', 10)
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.scan = self.create_publisher(LaserScan, '/scan', 10)
        self.points = self.create_publisher(PointCloud2, '/mid360/points', 10)
        self.color = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth = self.create_publisher(Image, '/camera/depth/image_rect_raw', 10)
        self.camera_info = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        transforms = []
        for child, xyz, quaternion in (
            ('mid360_link', (0.0002835, 0.00003, 0.428434),
             (0.0, 0.9996733, 0.0, -0.0255576)),
            ('laser', (0.0002835, 0.00003, 0.428434),
             (0.0, 0.9996733, 0.0, -0.0255576)),
            ('d435i_color_optical_frame', (0.0576235, 0.01753, 0.42987),
             (0.5, 0.5, -0.5, -0.5)),
        ):
            sensor = TransformStamped()
            sensor.header.frame_id = 'torso_link'
            sensor.child_frame_id = child
            sensor.transform.translation.x = xyz[0]
            sensor.transform.translation.y = xyz[1]
            sensor.transform.translation.z = xyz[2]
            (sensor.transform.rotation.w, sensor.transform.rotation.x,
             sensor.transform.rotation.y, sensor.transform.rotation.z) = quaternion
            transforms.append(sensor)
        self.static_tf.sendTransform(transforms)
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.create_timer(0.005, self.poll)
        self.last_scan = None
        self.last_received = -math.inf
        self.previous_pose = None
        self.sensor_chunks = {}

    def command(self, message):
        values = [message.linear.x, message.linear.y, message.angular.z]
        if not all(math.isfinite(value) for value in values):
            return
        if time.monotonic() - self.last_received > 0.5:
            values = [0.0, 0.0, 0.0]
        self.socket.sendto(json.dumps({'velocity': values}).encode(), ('127.0.0.1', 19421))

    def poll(self):
        self.poll_sensors()
        for _ in range(100):
            try:
                payload, _ = self.socket.recvfrom(65535)
            except BlockingIOError:
                return
            row = json.loads(payload)
            self.last_received = time.monotonic()
            timestamp = stamp(row['time'])
            self.clock.publish(Clock(clock=timestamp))
            w, x, y, z = row['quaternion']
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            odom = Odometry()
            odom.header.stamp = timestamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = row['position'][:2]
            odom.pose.pose.orientation.z = math.sin(yaw / 2)
            odom.pose.pose.orientation.w = math.cos(yaw / 2)
            vx, vy = row['linear_velocity'][:2]
            odom.twist.twist.linear.x = math.cos(yaw) * vx + math.sin(yaw) * vy
            odom.twist.twist.linear.y = -math.sin(yaw) * vx + math.cos(yaw) * vy
            if self.previous_pose is not None:
                previous_time, previous_yaw = self.previous_pose
                dt = row['time'] - previous_time
                if dt > 0:
                    delta = yaw - previous_yaw
                    odom.twist.twist.angular.z = math.atan2(math.sin(delta), math.cos(delta)) / dt
            self.previous_pose = (row['time'], yaw)
            self.odom.publish(odom)
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = 'base_link'
            transform.transform.translation.x = odom.pose.pose.position.x
            transform.transform.translation.y = odom.pose.pose.position.y
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(transform)
            torso = TransformStamped()
            torso.header = odom.header
            torso.header.frame_id = 'base_link'
            torso.child_frame_id = 'torso_link'
            (torso.transform.translation.x, torso.transform.translation.y,
             torso.transform.translation.z) = row['torso_position_base']
            qw, qx, qy, qz = row['torso_quaternion_base']
            torso.transform.rotation.w = qw
            torso.transform.rotation.x = qx
            torso.transform.rotation.y = qy
            torso.transform.rotation.z = qz
            self.tf.sendTransform(torso)
            if row['scan'] is not None and row['scan_time'] != self.last_scan:
                self.last_scan = row['scan_time']
                scan = LaserScan()
                scan.header.stamp = stamp(row['scan_time'])
                scan.header.frame_id = 'laser'
                scan.angle_min = -math.pi
                scan.angle_increment = 2 * math.pi / len(row['scan'])
                scan.angle_max = scan.angle_min + (len(row['scan']) - 1) * scan.angle_increment
                scan.scan_time = 0.1
                scan.range_min, scan.range_max = 0.05, 20.0
                scan.ranges = [float('inf') if v is None else v for v in row['scan']]
                self.scan.publish(scan)

    def poll_sensors(self):
        for _ in range(100):
            try:
                packet, _ = self.sensor_socket.recvfrom(65535)
            except BlockingIOError:
                return
            if len(packet) < 8:
                continue
            sequence, index, count = struct.unpack('!IHH', packet[:8])
            chunks = self.sensor_chunks.setdefault(sequence, [None] * count)
            if len(chunks) != count or index >= count:
                self.sensor_chunks.pop(sequence, None)
                continue
            chunks[index] = packet[8:]
            if all(chunk is not None for chunk in chunks):
                payload = b''.join(chunks)
                self.sensor_chunks = {
                    key: value for key, value in self.sensor_chunks.items() if key > sequence
                }
                try:
                    with np.load(io.BytesIO(payload), allow_pickle=False) as values:
                        self.publish_sensors(values)
                except (ValueError, OSError):
                    self.get_logger().warning('Discarding malformed simulated sensor frame')

    def publish_sensors(self, values):
        timestamp = stamp(float(values['time']))
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
            cloud.is_bigendian = False
            cloud.point_step = 12
            cloud.row_step = cloud.point_step * cloud.width
            cloud.is_dense = True
            cloud.data = xyz.tobytes()
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
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.socket.sendto(json.dumps({'velocity': [0, 0, 0]}).encode(), ('127.0.0.1', 19421))
            node.socket.close()
            node.sensor_socket.close()
        finally:
            if rclpy.ok():
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    main()
