"""Validate turn assistance, obstacle stopping and recovery through native Nav2."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import rclpy
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped, Twist
from lifecycle_msgs.srv import ChangeState
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster
import yaml
from g1_backend.navigation_control import merge_parameters

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--backend', choices=['lidar3d', 'rgb-d'], default='lidar3d')
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    defaults = yaml.safe_load(Path('/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml').read_text())
    tuning = yaml.safe_load((ROOT / 'configs/navigation/nav2_tuning.yaml').read_text())
    merge_parameters(defaults, tuning['nav2'])
    if args.backend == 'lidar3d':
        merge_parameters(defaults, yaml.safe_load(
            (ROOT / 'configs/navigation/nav2_lidar.yaml').read_text())['nav2'])
    monitor = defaults['collision_monitor']['ros__parameters']
    monitor.update(use_sim_time=False, base_frame_id='base_link', enable_stamped_cmd_vel=False)
    config = output / 'monitor.yaml'
    config.write_text(yaml.safe_dump({'collision_monitor': {'ros__parameters': monitor}}))
    rclpy.init()
    node = rclpy.create_node('collision_pipeline_validation')
    velocity = node.create_publisher(Twist, '/cmd_vel_smoothed', 10)
    scan_pub = node.create_publisher(LaserScan, '/scan', 10)
    footprint_pub = node.create_publisher(PolygonStamped, '/local_costmap/published_footprint', 10)
    static = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_link', 'laser'
    transform.transform.rotation.w = 1.
    static.sendTransform(transform)
    samples = {'output': [], 'assisted': []}
    node.create_subscription(Twist, '/cmd_vel', lambda msg: samples['output'].append(
        (msg.linear.x, msg.angular.z, msg.linear.y)), 10)
    node.create_subscription(Twist, '/cmd_vel_assisted', lambda msg: samples['assisted'].append(
        (msg.linear.x, msg.angular.z, msg.linear.y)), 10)
    processes, logs = [], []
    report = {}
    try:
        commands = [
            ['/opt/ros/jazzy/lib/nav2_collision_monitor/collision_monitor', '--ros-args',
             '--params-file', str(config)],
            [sys.executable, str(ROOT / 'scripts/navigation_command_filter.py')],
        ]
        for index, command in enumerate(commands):
            log = (output / f'process_{index}.log').open('w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))
        client = node.create_client(ChangeState, '/collision_monitor/change_state')
        if not client.wait_for_service(timeout_sec=15):
            raise RuntimeError('Collision monitor service unavailable')
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            if not future.done() or not future.result().success:
                raise RuntimeError('Cannot activate collision monitor')

        def phase(linear, angular, obstacle=False, publish_scan=True, seconds=2., wall=False, lateral=0.):
            samples['output'].clear()
            samples['assisted'].clear()
            start = time.monotonic()
            next_publish = 0.
            while time.monotonic() - start < seconds:
                if time.monotonic() >= next_publish:
                    next_publish = time.monotonic() + .05
                    stamp = node.get_clock().now().to_msg()
                    command = Twist()
                    command.linear.x, command.angular.z = linear, angular
                    command.linear.y = lateral
                    velocity.publish(command)
                    if publish_scan:
                        scan = LaserScan()
                        scan.header.stamp, scan.header.frame_id = stamp, 'laser'
                        scan.angle_min, scan.angle_increment = -np.pi, 2 * np.pi / 360
                        scan.angle_max = scan.angle_min + 359 * scan.angle_increment
                        scan.range_min, scan.range_max = .05, 20.
                        ranges = [float('inf')] * 360
                        if obstacle:
                            ranges[170:191] = [.3] * 21
                        if wall:
                            for index in range(195, 346):
                                sine = np.sin(-np.pi + index * 2 * np.pi / 360)
                                ranges[index] = .335 / sine
                        scan.ranges = ranges
                        scan_pub.publish(scan)
                    footprint = PolygonStamped()
                    footprint.header.stamp, footprint.header.frame_id = stamp, 'base_link'
                    footprint.polygon.points = [Point32(x=x, y=y, z=0.) for x, y in
                                                [(-.33, -.35), (.33, -.35), (.33, .35), (-.33, .35)]]
                    footprint_pub.publish(footprint)
                rclpy.spin_once(node, timeout_sec=.01)
                if time.monotonic() - start < .7:
                    samples['output'].clear()
                    samples['assisted'].clear()
            return {key: np.asarray(value).reshape(-1, 3) for key, value in samples.items()}

        free = phase(.3, 0.)
        blocked = phase(.3, 0., obstacle=True)
        recovered = phase(.3, 0.)
        turning = phase(0., .5, seconds=3.)
        stopped = phase(0., 0.)
        stale = phase(.3, 0., publish_scan=False)
        report = {
            'free_motion': bool(len(free['output']) and np.max(free['output'][:, 0]) > .2),
            'obstacle_stops_motion': bool(len(blocked['output']) and np.max(np.abs(blocked['output'])) < .001),
            'recovers_after_obstacle_clears': bool(len(recovered['output']) and np.max(recovered['output'][:, 0]) > .2),
            'turn_translation_monitored': bool(len(turning['output']) and np.max(np.abs(turning['assisted'][:, 0])) > .1
                                              and np.max(np.abs(turning['output'][:, 0])) > .1),
            'zero_cancels_turn': bool(len(stopped['output']) and np.max(np.abs(stopped['output'])) < .001),
            'stale_scan_stops_motion': bool(len(stale['output']) and np.max(np.abs(stale['output'])) < .001),
        }
        if args.backend == 'lidar3d':
            parallel = phase(.2, 0., wall=True)
            away = phase(0., 0., wall=True, lateral=-.1)
            toward = phase(0., 0., wall=True, lateral=.1)
            report.update(
                wall_parallel_motion=bool(len(parallel['output']) and np.max(parallel['output'][:, 0]) > .15),
                wall_away_motion=bool(len(away['output']) and np.min(away['output'][:, 2]) < -.08),
                wall_toward_motion_limited=bool(len(toward['output']) and np.max(toward['output'][:, 2]) < .03))
        report['passed'] = all(report.values())
    finally:
        for process in processes:
            process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
    (output / 'validation.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report.get('passed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
