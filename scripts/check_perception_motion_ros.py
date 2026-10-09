"""Headless locomotion + ROS perception smoke test in the real Maxwell simulation."""

import argparse
import ast
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='results/perception/motion_ros')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('perception_motion_observer')
    captures, positions, clock = [], [], [0.0]
    subscriptions = [
        node.create_subscription(
            String, '/perception/detections', lambda msg: captures.append(json.loads(msg.data)), 10
        ),
        node.create_subscription(
            Odometry,
            '/ground_truth/odom',
            lambda msg: positions.append(
                [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
            ),
            10,
        ),
        node.create_subscription(
            Clock,
            '/clock',
            lambda msg: clock.__setitem__(0, msg.clock.sec + msg.clock.nanosec * 1e-9),
            10,
        ),
    ]
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    output = ROOT / args.output
    output.mkdir(parents=True, exist_ok=True)
    log = (output / 'launch.log').open('w')
    process = subprocess.Popen(
        [
            'bash',
            str(ROOT / 'scripts/navigation.sh'),
            'nav2:=false',
            'rviz:=false',
            'viewer:=false',
            'perception:=true',
            'duration:=16',
            'real_sensors:=true',
            'initial_x:=4.5',
            'initial_y:=-5.9',
            'initial_yaw:=1.57079632679',
        ],
        cwd=ROOT,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    timed_out = False
    try:
        deadline = time.monotonic() + 100
        while process.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            command = Twist()
            stamp = clock[0]
            if 4.5 <= stamp < 7:
                command.linear.x = 0.12
            elif 8 <= stamp < 10:
                command.angular.z = 0.2
            elif 11 <= stamp < 13:
                command.linear.x = -0.08
            publisher.publish(command)
        timed_out = process.poll() is None
    finally:
        publisher.publish(Twist())
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
        log.close()
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    valid = [capture for capture in captures if capture['status'] == 'ok']
    boxes = [
        obj
        for capture in valid
        for table in capture['tables']
        for obj in table['objects']
        if obj['kind'] == 'box_candidate'
    ]
    displacement = (
        max(
            ((p[0] - positions[0][0]) ** 2 + (p[1] - positions[0][1]) ** 2) ** 0.5
            for p in positions
        )
        if positions
        else 0
    )
    physics = {}
    for line in (output / 'launch.log').read_text().splitlines():
        if "{'results_dir':" in line:
            try:
                result = ast.literal_eval(line[line.index("{'results_dir':") :])
                physics = {
                    key: result.get(key)
                    for key in ['status', 'falls', 'collisions', 'watchdog_events', 'render_fps']
                }
            except (ValueError, SyntaxError):
                pass
    report = {
        'timed_out': timed_out,
        'physics': physics,
        'exit_code': process.returncode,
        'sim_seconds': clock[0],
        'displacement_m': displacement,
        'captures': len(captures),
        'valid_captures': len(valid),
        'box_observations': len(boxes),
        'stable_observations': sum(obj.get('stable', False) for obj in boxes),
        'ids': sorted({obj['id'] for obj in boxes}),
        'statuses': {
            status: sum(c['status'] == status for c in captures)
            for status in {c['status'] for c in captures}
        },
        'limits': [
            'Short headless commanded walk; not Nav2 goal completion.',
            'IDs are unassociated candidates; multiple objects can appear.',
        ],
        'records': captures,
    }
    (output / 'evaluation.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != 'records'}, indent=2))
    assert not timed_out and process.returncode == 0, (
        'Simulation did not complete; inspect launch.log'
    )
    assert displacement > 0.05 and len(boxes) >= 2, 'Insufficient movement or observed boxes'


if __name__ == '__main__':
    main()
