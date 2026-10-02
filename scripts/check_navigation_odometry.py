"""Compare sensor-fused odometry to MuJoCo reference and count accepted sources."""

import argparse
from collections import Counter, deque
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('navigation_odometry_audit')
    truth = deque(maxlen=10000)
    pending = []
    matched = []
    counts = Counter()
    methods = Counter()
    status = {}
    subscriptions_seen = []

    def pose(message):
        q = message.pose.pose.orientation
        return (
            message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
            np.array([message.pose.pose.position.x, message.pose.pose.position.y]),
            Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2],
        )

    def estimated(message):
        counts['fused'] += 1
        pending.append(pose(message))

    def diagnostic(message):
        nonlocal status
        status = json.loads(message.data)
        if status['valid']:
            methods[status['source'] + ':' + status['method']] += 1

    subscriptions = [
        node.create_subscription(
            Odometry, '/ground_truth/odom', lambda msg: truth.append(pose(msg)), 10
        ),
        node.create_subscription(Odometry, '/odom', estimated, 10),
        node.create_subscription(String, '/odometry/status', diagnostic, 10),
    ]
    started = time.monotonic()
    while time.monotonic() - started < args.duration:
        rclpy.spin_once(node, timeout_sec=0.02)
        if not subscriptions_seen and status:
            try:
                subscriptions_seen = [
                    topic
                    for topic, _ in node.get_subscriber_names_and_types_by_node(
                        'g1_sensor_odometry', '/'
                    )
                ]
            except RuntimeError:
                pass
        if not truth:
            continue
        remaining = []
        for row in pending:
            nearest = min(truth, key=lambda item: abs(item[0] - row[0]))
            if abs(nearest[0] - row[0]) < 0.025:
                matched.append((row, nearest))
            elif row[0] >= truth[-1][0]:
                remaining.append(row)
        pending = remaining[-1000:]
    result = {
        'counts': dict(counts),
        'methods': dict(methods),
        'last_status': status,
        'matched': len(matched),
    }
    if matched:
        first_est, first_truth = matched[0]
        yaw_offset = first_truth[2] - first_est[2]
        c, s = np.cos(yaw_offset), np.sin(yaw_offset)
        axes = np.array([[c, -s], [s, c]])
        offset = first_truth[1] - axes @ first_est[1]
        errors = [
            np.linalg.norm(axes @ estimate[1] + offset - reference[1])
            for estimate, reference in matched
        ]
        yaw_errors = [
            abs((estimate[2] + yaw_offset - reference[2] + np.pi) % (2 * np.pi) - np.pi)
            for estimate, reference in matched
        ]
        result.update(
            simulated_duration=matched[-1][0][0] - first_est[0],
            final_error_m=float(errors[-1]),
            maximum_error_m=float(max(errors)),
            rms_error_m=float(np.sqrt(np.mean(np.square(errors)))),
            maximum_yaw_error_rad=float(max(yaw_errors)),
            truth_displacement_m=float(np.linalg.norm(matched[-1][1][1] - first_truth[1])),
        )
    try:
        result['estimator_subscriptions'] = [
            topic
            for topic, _ in node.get_subscriber_names_and_types_by_node('g1_sensor_odometry', '/')
        ]
    except RuntimeError:
        result['estimator_subscriptions'] = subscriptions_seen
        result['estimator_running_at_end'] = False
    print(json.dumps(result, indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n')
    del subscriptions
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
