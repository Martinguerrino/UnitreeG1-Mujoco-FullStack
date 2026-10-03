"""Bounded live check of supported leg estimates and their odometry contribution."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=15.)
    parser.add_argument('--startup-timeout', type=float, default=60.)
    parser.add_argument('--drive', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('g1_feet_check')
    result = {'samples': 0, 'valid': 0, 'odom': 0, 'fusion': {},
              'max_supported_speed': 0., 'errors_xy': []}
    latest, origin = {}, {}

    def feet(message):
        sample = json.loads(message.data)
        result['samples'] += 1
        result['valid'] += int(sample['valid'])
        if sample['valid']:
            result['max_supported_speed'] = max(result['max_supported_speed'],
                                                 float(np.linalg.norm(sample['velocity_body'])))

    def fusion(message):
        result['fusion'] = json.loads(message.data)

    def pose(name, message):
        p = message.pose.pose.position
        latest[name] = (message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                        np.array([p.x, p.y]))
        if name == 'odom':
            result['odom'] += 1
        if len(latest) == 2 and abs(latest['odom'][0] - latest['truth'][0]) < .1:
            for key in latest:
                origin.setdefault(key, latest[key][1].copy())
            error = (latest['odom'][1] - origin['odom']) - (latest['truth'][1] - origin['truth'])
            result['errors_xy'].append(float(np.linalg.norm(error)))

    node.create_subscription(String, '/feet_estimation/state', feet, 100)
    node.create_subscription(String, '/feet_estimation/fusion_status', fusion, 10)
    for name, topic in [('odom', '/odom'), ('truth', '/ground_truth/odom')]:
        node.create_subscription(Odometry, topic, lambda msg, name=name: pose(name, msg),
                                 qos_profile_sensor_data)
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    started, ready = time.monotonic(), None
    drive_start = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            if ready is None:
                if result['odom'] > 10 and result['samples'] > 10 and 'truth' in latest:
                    ready = now
                elif now - started > args.startup_timeout:
                    break
            elif now - ready >= args.duration:
                break
            command = Twist()
            if args.drive and ready is not None and latest['odom'][0] >= 5:
                if drive_start is None:
                    drive_start = latest['odom'][0]
                if latest['odom'][0] - drive_start < 6:
                    command.linear.x = .3
            publisher.publish(command)
    finally:
        publisher.publish(Twist())
        if origin:
            result['distance_truth'] = float(np.linalg.norm(latest['truth'][1] - origin['truth']))
            result['distance_odom'] = float(np.linalg.norm(latest['odom'][1] - origin['odom']))
        errors = result.pop('errors_xy')
        result['max_error_xy'] = max(errors, default=None)
        result['passed'] = (result['valid'] > 10 and result['fusion'].get('used', 0) > 5
                            and result['max_supported_speed'] <= 2.
                            and (not args.drive or result.get('distance_truth', 0.) > .2))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2))
        print(json.dumps(result))
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if result['passed'] else 2)


if __name__ == '__main__':
    main()
