"""Audit the running navigation stack, including TF, RGB-D, map and Nav2.

Source ROS/install first. Start navigation.sh viewer:=false rviz:=false separately.
Run with system Python and the same ROS_DOMAIN_ID (normally 43).
"""

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=45, help='Wall seconds')
    parser.add_argument('--ideal', action='store_true')
    parser.add_argument('--goal', nargs=2, type=float, help='Optional map XY goal')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node(
        'navigation_sensor_audit',
        parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)],
    )
    buffer = Buffer(cache_time=Duration(seconds=30))
    listener = TransformListener(buffer, node)
    counts = Counter()
    stamps = {}
    pending = []
    matched = Counter()
    errors = []
    images = {}
    tf_missing = Counter()
    tf_startup = Counter()
    obstacles = Counter()
    started = time.monotonic()
    lifecycle = node.create_client(GetState, '/bt_navigator/get_state')
    state_future = None
    navigator_active = False
    goal_future = None
    result_future = None
    goal_handle = None
    action = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    def receive(name, message):
        counts[name] += 1
        if name in ('map', 'costmap', 'global_costmap'):
            obstacles[name] += sum(value > 0 for value in message.data)
            return
        key = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        stamps.setdefault(name, []).append(key)
        pending.append((name, message, time.monotonic()))
        if name in ('rgb', 'depth'):
            images.setdefault(key, {})[name] = (
                message.width,
                message.height,
                message.header.frame_id,
            )
        if name == 'depth':
            depth = np.frombuffer(message.data, dtype=np.float32)
            if not np.isfinite(depth).all() or np.min(depth) <= 0:
                errors.append('invalid rendered depth')
        if name == 'scan':
            if (
                abs(
                    message.angle_max
                    - (message.angle_min + (len(message.ranges) - 1) * message.angle_increment)
                )
                > 1e-5
            ):
                errors.append('inconsistent scan angle limits')
            if not any(math.isfinite(value) for value in message.ranges):
                errors.append('empty scan')

    subscriptions = []
    for name, topic, kind, qos in (
        ('scan', '/scan', LaserScan, 10),
        ('cloud', '/mid360/points', PointCloud2, qos_profile_sensor_data),
        ('rgb', '/camera/color/image_raw', Image, qos_profile_sensor_data),
        ('depth', '/camera/depth/image_rect_raw', Image, qos_profile_sensor_data),
        ('info', '/camera/color/camera_info', CameraInfo, 10),
        ('map', '/map', OccupancyGrid, 10),
        ('costmap', '/local_costmap/costmap', OccupancyGrid, 10),
        ('global_costmap', '/global_costmap/costmap', OccupancyGrid, 10),
    ):
        subscriptions.append(
            node.create_subscription(
                kind, topic, lambda message, name=name: receive(name, message), qos
            )
        )
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            remaining = []
            for name, message, received in pending:
                try:
                    transform = buffer.lookup_transform(
                        'odom', message.header.frame_id, Time.from_msg(message.header.stamp)
                    )
                    matched[name] += 1
                    if name == 'scan':
                        q = transform.transform.rotation
                        if abs(q.x) > 1e-6 or abs(q.y) > 1e-6:
                            errors.append('scan frame is tilted')
                        if abs(transform.transform.translation.z - 0.78) > 1e-6:
                            errors.append('scan height mismatch')
                except TransformException:
                    if time.monotonic() - received < 2:
                        remaining.append((name, message, received))
                    else:
                        if received - started < 3:
                            tf_startup[name] += 1
                        else:
                            tf_missing[name] += 1
            pending[:] = remaining
            if state_future is None and lifecycle.service_is_ready():
                state_future = lifecycle.call_async(GetState.Request())
            if state_future is not None and state_future.done():
                navigator_active = state_future.result().current_state.id == 3
                state_future = None
            if (
                args.goal
                and navigator_active
                and goal_future is None
                and counts['map']
                and counts['costmap'] > 1
                and action.server_is_ready()
            ):
                goal = NavigateToPose.Goal()
                goal.pose.header.frame_id = 'map'
                goal.pose.header.stamp = node.get_clock().now().to_msg()
                goal.pose.pose.position.x, goal.pose.pose.position.y = args.goal
                goal.pose.pose.orientation.w = 1.0
                goal_future = action.send_goal_async(goal)
            if goal_future is not None and goal_future.done() and result_future is None:
                goal_handle = goal_future.result()
                if goal_handle.accepted:
                    result_future = goal_handle.get_result_async()
                else:
                    errors.append('Nav2 rejected goal')
                    break
        if goal_handle is not None and (result_future is None or not result_future.done()):
            future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        pairs = [value for value in images.values() if 'rgb' in value and 'depth' in value]
        if any(value['rgb'] != value['depth'] for value in pairs):
            errors.append('RGB/depth dimensions or optical frame differ')
        required = ['scan', 'map', 'costmap', 'global_costmap'] + (
            [] if args.ideal else ['cloud', 'rgb', 'depth', 'info']
        )
        for name in required:
            if counts[name] < 2:
                errors.append(f'insufficient {name}: {counts[name]}')
        if not obstacles['map'] or not obstacles['global_costmap']:
            errors.append('no obstacles observed in map/global costmap')
        if tf_missing:
            errors.append(f'TF unavailable at acquisition time: {dict(tf_missing)}')
        if not args.ideal and not pairs:
            errors.append('no synchronized RGB/depth pairs')
        if args.goal and (
            result_future is None or not result_future.done() or result_future.result().status != 4
        ):
            errors.append('Nav2 goal did not succeed within audit duration')
        rates = {}
        for name, values in stamps.items():
            unique = sorted(set(values))
            if len(unique) > 1:
                rates[name] = (len(unique) - 1) * 1e9 / (unique[-1] - unique[0])
        report = {
            'counts': dict(counts),
            'tf_verified': dict(matched),
            'tf_missing': dict(tf_missing),
            'tf_startup': dict(tf_startup),
            'obstacle_cells_observed': dict(obstacles),
            'synchronized_rgb_depth_pairs': len(pairs),
            'simulated_hz': rates,
            'goal_status': result_future.result().status
            if result_future and result_future.done()
            else None,
            'errors': sorted(set(errors)),
        }
        text = json.dumps(report, indent=2)
        print(text)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(text + '\n')
        return bool(errors)
    finally:
        listener.unregister()
        action.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
