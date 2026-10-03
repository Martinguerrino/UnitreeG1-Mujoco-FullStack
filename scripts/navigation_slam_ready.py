"""Wait for a working sensor SLAM map and odometry before starting Nav2."""
import time
import json
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import Info


def main():
    rclpy.init()
    node = rclpy.create_node('g1_slam_ready')
    node.declare_parameter('startup_timeout', 180.0)
    node.declare_parameter('require_localization', False)
    node.declare_parameter('report_path', '')
    matches = []
    ready = {'map': False, 'odom': False, 'scan': False,
             'localized': not node.get_parameter('require_localization').value}
    def localization(message):
        if message.loop_closure_id > 0 or message.proximity_detection_id > 0:
            ready['localized'] = True
            matches.append({'ref_id': message.ref_id,
                            'matched_id': message.loop_closure_id or message.proximity_detection_id})
            node.get_logger().info('Sensor match accepted against saved map')
    node.create_subscription(Info, '/slam/info', localization, 10)
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
    report_path = node.get_parameter('report_path').value
    if report_path:
        Path(report_path).write_text(json.dumps({
            'ready': ready, 'sensor_matches': matches, 'passed': result == 0,
            'elapsed_seconds': time.monotonic() - started,
        }, indent=2))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
