"""Bounded live SLAM check: maps, sensor odometry and optional slow motion."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Imu


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=25)
    parser.add_argument('--startup-timeout', type=float, default=120)
    parser.add_argument('--drive', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('g1_slam_check')
    counts = {'odom': 0, 'map': 0, 'cloud': 0, 'imu': 0, 'truth': 0}
    values = {}
    samples = []
    baseline = {}
    def record(name, message):
        counts[name] += 1
        values[name] = message
        if name in ('odom', 'truth'):
            p = message.pose.pose.position
            baseline.setdefault(name, np.array([p.x, p.y]))
        if name == 'odom' and 'truth' in values:
            truth = values['truth']
            t = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            tt = truth.header.stamp.sec + truth.header.stamp.nanosec * 1e-9
            if abs(t - tt) <= .15:
                p, gt = message.pose.pose.position, truth.pose.pose.position
                delta = np.array([p.x, p.y]) - baseline['odom']
                actual = np.array([gt.x, gt.y]) - baseline['truth']
                samples.append({'time': t, 'odom': delta.tolist(), 'truth': actual.tolist(), 'error_xy': float(np.linalg.norm(delta - actual))})
    for name, topic, kind in (
        ('odom', '/odom', Odometry), ('truth', '/ground_truth/odom', Odometry),
        ('map', '/map', OccupancyGrid), ('cloud', '/slam/map_points', PointCloud2),
        ('imu', '/imu/torso', Imu),
    ):
        node.create_subscription(kind, topic, lambda msg, name=name: record(name, msg), qos_profile_sensor_data)
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    started = time.monotonic()
    measurement_started = None
    drive_started = None
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=.03)
        now = time.monotonic()
        if measurement_started is None:
            if counts['odom'] > 10 and counts['cloud'] > 0 and counts['truth'] > 0:
                measurement_started = now
            elif now - started >= args.startup_timeout:
                break
        elif now - measurement_started >= args.duration:
            break
        odom_time = 0 if 'odom' not in values else values['odom'].header.stamp.sec
        if args.drive and counts['odom'] > 10 and counts['cloud'] > 0 and odom_time >= 5:
            drive_started = time.monotonic() if drive_started is None else drive_started
            command = Twist()
            elapsed = time.monotonic() - drive_started
            if elapsed < 6:
                command.linear.x = .35
            elif elapsed < 10:
                command.linear.x = .25
                command.angular.z = .15
            publisher.publish(command)
    publisher.publish(Twist())
    cloud = values.get('cloud')
    grid = values.get('map')
    maximum_error = max((row['error_xy'] for row in samples), default=None)
    displacement = max((float(np.linalg.norm(row['truth'])) for row in samples), default=0.)
    result = {
        'counts': counts, 'cloud_points': 0 if cloud is None else cloud.width * cloud.height,
        'cloud_fields': [] if cloud is None else [field.name for field in cloud.fields],
        'map_cells': 0 if grid is None else grid.info.width * grid.info.height,
        'samples': samples,
        'max_error_xy': maximum_error, 'max_displacement_xy': displacement,
        'passed': (counts['odom'] > 10 and counts['map'] > 0 and cloud is not None
                   and cloud.width > 0 and maximum_error is not None and maximum_error < .25
                   and (not args.drive or displacement > .25)),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2))
    print(json.dumps({key: value for key, value in result.items() if key != 'samples'}, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
