"""Smoke test ROS topics from real MuJoCo physics, without Nav2/SLAM packages.

Run with /usr/bin/python3 after sourcing ROS Jazzy, ROS_DOMAIN_ID=43.
"""

import json
import argparse
import math
from pathlib import Path
import subprocess
import sys
import time
from itertools import pairwise

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan

from navigation_bridge import Bridge
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_backend.python_runtime import resolve_python


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--turn-rate', type=float, default=0.0)
    parser.add_argument('--turn-assist-speed', type=float, default=0.0,
                        help='Forward gait command used while benchmarking a turn')
    parser.add_argument('--turn-assist-period', type=float, default=0.0,
                        help='Alternate forward/reverse every N simulated seconds')
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--minimum-yaw', type=float, default=0.0,
                        help='Fail if absolute measured turn is below this many radians')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    rclpy.init()
    bridge = Bridge()
    observer = rclpy.create_node('navigation_smoke_observer')
    counts = {'scan': 0, 'odom': 0, 'clock': 0}
    scans = []
    simulation_time = [0.0]
    positions = []
    yaws = []

    def receive(key, message):
        counts[key] += 1
        if key == 'scan':
            scans.append(message)
        elif key == 'clock':
            simulation_time[0] = message.clock.sec + message.clock.nanosec / 1e9
        elif key == 'odom':
            positions.append(message.pose.pose.position.x)
            q = message.pose.pose.orientation
            yaws.append(math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z))

    subscriptions = [
        observer.create_subscription(kind, '/' + name,
                                     lambda msg, key=name: receive(key, msg), 10)
        for kind, name in ((LaserScan, 'scan'), (Odometry, 'odom'), (Clock, 'clock'))
    ]
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(observer)
    publisher = observer.create_publisher(Twist, '/cmd_vel', 10)
    process = subprocess.Popen([
        resolve_python(),
        str(root / 'scripts/navigation_sim.py'), '--viewer', 'false',
        '--duration', str(args.duration),
    ])
    try:
        deadline = time.monotonic() + 45
        while process.poll() is None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
            command = Twist()
            active = 4.2 <= simulation_time[0] < args.duration - 0.2
            if args.turn_rate:
                command.angular.z = args.turn_rate if active else 0.0
                direction = 1.0
                if args.turn_assist_period:
                    direction = 1.0 if int(
                        max(0.0, simulation_time[0] - 4.2) / args.turn_assist_period
                    ) % 2 == 0 else -1.0
                command.linear.x = args.turn_assist_speed * direction if active else 0.0
            else:
                command.linear.x = 0.3 if active else 0.0
            publisher.publish(command)
        assert process.poll() == 0, 'Simulation did not terminate normally'
        assert all(value > 5 for value in counts.values()), counts
        assert scans and all(len(scan.ranges) == 360 for scan in scans)
        assert all(scan.header.frame_id == 'laser' for scan in scans)
        result = {'ros_topics_received': counts, 'rays_per_scan': 360}
        if args.turn_rate:
            unwrapped_yaw = sum(
                math.atan2(math.sin(current - previous), math.cos(current - previous))
                for previous, current in pairwise(yaws)
            )
            assert abs(unwrapped_yaw) >= args.minimum_yaw, (
                f'Angular command produced only {unwrapped_yaw:.3f} rad'
            )
            result['commanded_turn_rate_rad_s'] = args.turn_rate
            result['turn_assist_speed_m_s'] = args.turn_assist_speed
            result['turn_assist_period_s'] = args.turn_assist_period
            result['unwrapped_yaw_rad'] = unwrapped_yaw
        else:
            assert max(positions) - positions[0] > 0.1, 'ROS command did not move robot'
            result['forward_displacement_m'] = positions[-1] - positions[0]
        print(json.dumps(result))
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=10)
        for subscription in subscriptions:
            observer.destroy_subscription(subscription)
        executor.shutdown()
        bridge.socket.close()
        bridge.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
