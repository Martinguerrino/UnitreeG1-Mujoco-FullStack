"""Wait for a working sensor SLAM map and odometry before starting Nav2."""
import time

import rclpy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init()
    node = rclpy.create_node('g1_slam_ready')
    node.declare_parameter('startup_timeout', 180.0)
    ready = {'map': False, 'odom': False, 'scan': False}
    def odom(message):
        timestamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        now = node.get_clock().now().nanoseconds * 1e-9
        ready['odom'] = timestamp >= 6 and 0 <= now - timestamp < .3
    def grid(message):
        ready['map'] = message.info.width > 0 and message.info.height > 0
    def scan(message):
        timestamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        now = node.get_clock().now().nanoseconds * 1e-9
        ready['scan'] = bool(message.ranges) and 0 <= now - timestamp < .3
    node.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, '/map', grid, qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)
    started = time.monotonic()
    result = 1
    try:
        while rclpy.ok() and time.monotonic() - started < node.get_parameter('startup_timeout').value:
            rclpy.spin_once(node, timeout_sec=.1)
            if all(ready.values()):
                node.get_logger().info('SLAM map, sensor odometry and scan ready; starting Nav2')
                result = 0
                break
        if result:
            node.get_logger().error('Timed out waiting for a SLAM map and recent odometry/scan')
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
