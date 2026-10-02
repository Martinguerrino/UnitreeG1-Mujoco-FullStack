"""Send a real Nav2 goal and check that scans and velocity reach the robot."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.msg import CollisionMonitorState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, default=1.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--timeout', type=float, default=45.0)
    parser.add_argument('--startup-timeout', type=float, default=120.0)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--require-inflated-start', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('g1_navigation_goal_check')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    observed = {'scans': 0, 'moving_commands': 0, 'max_displacement': 0.0}
    positions = {}
    diagnostics = {'controller_commands': 0, 'assisted_commands': 0,
                   'monitor_stops': 0, 'initial_cost': None, 'monitor_states': {}}
    costmaps = {}

    def command(message):
        if abs(message.linear.x) + abs(message.linear.y) + abs(message.angular.z) > .01:
            observed['moving_commands'] += 1

    def odometry(message):
        position = (message.pose.pose.position.x, message.pose.pose.position.y)
        positions['latest'] = position
        if 'start' in positions:
            distance = math.dist(position, positions['start'])
            observed['max_displacement'] = max(observed['max_displacement'], distance)

    def scan(message):
        observed['scans'] += 1

    node.create_subscription(Twist, '/cmd_vel', command, 10)
    node.create_subscription(Odometry, '/odom', odometry, qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)
    def stage(message, name):
        if abs(message.linear.x) + abs(message.angular.z) > .01:
            diagnostics[name] += 1
    node.create_subscription(Twist, '/cmd_vel_smoothed', lambda msg: stage(msg, 'controller_commands'), 10)
    node.create_subscription(Twist, '/cmd_vel_assisted', lambda msg: stage(msg, 'assisted_commands'), 10)
    def monitor(message):
        key = str(message.action_type)
        diagnostics['monitor_states'][key] = diagnostics['monitor_states'].get(key, 0) + 1
        if message.action_type == 1:
            diagnostics['monitor_stops'] += 1
    node.create_subscription(CollisionMonitorState, '/collision_monitor_state', monitor, 10)
    node.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda msg: costmaps.update(local=msg), 10)
    action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    lifecycle = node.create_client(GetState, '/bt_navigator/get_state')
    started = time.monotonic()
    active = False
    while rclpy.ok() and time.monotonic() - started < args.startup_timeout:
        rclpy.spin_once(node, timeout_sec=.1)
        if lifecycle.service_is_ready() and action.server_is_ready() and 'latest' in positions:
            future = lifecycle.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=1)
            if (future.done() and future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
                    and 'local' in costmaps):
                active = True
                break
    status = None
    accepted = False
    if active:
        positions['start'] = positions['latest']
        grid = costmaps['local']
        x = int((positions['start'][0] - grid.info.origin.position.x) / grid.info.resolution)
        y = int((positions['start'][1] - grid.info.origin.position.y) / grid.info.resolution)
        if 0 <= x < grid.info.width and 0 <= y < grid.info.height:
            diagnostics['initial_cost'] = int(grid.data[y * grid.info.width + x])
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = args.x, args.y
        goal.pose.pose.orientation.z = math.sin(args.yaw / 2)
        goal.pose.pose.orientation.w = math.cos(args.yaw / 2)
        future = action.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if future.done() and future.result().accepted:
            accepted = True
            handle = future.result()
            result = handle.get_result_async()
            rclpy.spin_until_future_complete(node, result, timeout_sec=args.timeout)
            if result.done():
                status = result.result().status
            else:
                cancellation = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancellation, timeout_sec=3)
    passed = (status == GoalStatus.STATUS_SUCCEEDED and observed['scans'] > 0
              and observed['moving_commands'] > 0 and observed['max_displacement'] > .25)
    if args.require_inflated_start:
        passed = passed and diagnostics['initial_cost'] is not None and 0 < diagnostics['initial_cost'] < 99
    report = {**observed, **diagnostics, 'nav2_active': active, 'goal_accepted': accepted,
              'goal_status': status, 'start': positions.get('start'),
              'end': positions.get('latest'), 'passed': passed}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
