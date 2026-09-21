"""ROS-only process: no SDK2/CycloneDDS libraries loaded into the ROS runtime."""

import json
import math
import socket
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
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
        self.clock = self.create_publisher(Clock, '/clock', 10)
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.scan = self.create_publisher(LaserScan, '/scan', 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        laser = TransformStamped()
        laser.header.frame_id = 'base_link'
        laser.child_frame_id = 'laser'
        laser.transform.translation.z = 0.78
        laser.transform.rotation.w = 1.0
        self.static_tf.sendTransform(laser)
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.create_timer(0.005, self.poll)
        self.last_scan = None
        self.last_received = -math.inf
        self.previous_pose = None

    def command(self, message):
        values = [message.linear.x, message.linear.y, message.angular.z]
        if not all(math.isfinite(value) for value in values):
            return
        if time.monotonic() - self.last_received > 0.5:
            values = [0.0, 0.0, 0.0]
        self.socket.sendto(json.dumps({'velocity': values}).encode(), ('127.0.0.1', 19421))

    def poll(self):
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
        finally:
            if rclpy.ok():
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    main()
