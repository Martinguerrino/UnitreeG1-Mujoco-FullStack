"""Exercise the production LiDAR loop backend with a known loop and odometry drift."""
import argparse
from itertools import pairwise
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import Info, MapData
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

ROOT = Path(__file__).resolve().parents[1]


def scene():
    horizontal = np.arange(-4, 4.01, .12)
    vertical = np.arange(0, 3.01, .12)
    a, b = np.meshgrid(horizontal, horizontal)
    floor = np.column_stack((a.ravel(), b.ravel(), np.zeros(a.size)))
    a, b = np.meshgrid(horizontal, vertical)
    wall_x = np.column_stack((np.full(a.size, -4), a.ravel(), b.ravel()))
    wall_y = np.column_stack((a.ravel(), np.full(a.size, 4), b.ravel()))
    # Non-symmetric geometry avoids matching a repeated corridor segment.
    pillar = np.column_stack((np.full(a.size, 3.2), a.ravel() * .15 - 1, b.ravel()))
    return np.vstack((floor, wall_x, wall_y, pillar)).astype(np.float32)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    logs = []
    processes = []
    rclpy.init()
    node = rclpy.create_node('lidar_loop_validation')
    clouds = node.create_publisher(PointCloud2, '/test/lidar', qos_profile_sensor_data)
    odometry = node.create_publisher(Odometry, '/test/odom', 10)
    broadcaster = TransformBroadcaster(node)
    closures = []
    graph = []
    received_clouds = []
    node.create_subscription(Info, '/slam/info',
                             lambda msg: closures.append((msg.ref_id, msg.proximity_detection_id))
                             if msg.proximity_detection_id > 0 else None, 10)
    node.create_subscription(MapData, '/slam/map_data', lambda msg: graph.append(msg), 10)
    node.create_subscription(PointCloud2, '/slam/map_points', lambda msg: received_clouds.append(msg), 10)
    backend = ROOT / 'artifacts/slam/rootfs/opt/ros/jazzy/lib/rtabmap_slam/rtabmap'
    commands = [
        [str(backend), '--ros-args', '--params-file', str(ROOT / 'configs/navigation/rtabmap_lidar.yaml'),
         '-p', 'use_sim_time:=false', '-p', 'database_path:=' + str(output / 'rtabmap.db'),
         '-r', 'scan_cloud:=/test/lidar', '-r', 'odom:=/test/odom',
         '-r', 'mapData:=/slam/map_data', '-r', 'info:=/slam/info',
         '-r', 'get_map_data:=/slam/get_map_data'],
        [sys.executable, str(ROOT / 'scripts/navigation_lidar_map.py'), '--ros-args',
         '-p', 'use_sim_time:=false', '-p', 'output_dir:=' + str(output)],
    ]
    report = {}
    try:
        for index, command in enumerate(commands):
            log = (output / f'process_{index}.log').open('w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))
        deadline = time.monotonic() + 15
        while clouds.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if clouds.get_subscription_count() == 0:
            raise RuntimeError('LiDAR loop backend did not subscribe')
        world = scene()
        corners = np.array([[0, 0], [2.5, 0], [2.5, 2], [0, 2], [0, 0]])
        path = np.vstack([np.linspace(a, b, 11, endpoint=False) for a, b in pairwise(corners)] + [corners[-1:]])
        for index, position in enumerate(path):
            fraction = index / (len(path) - 1)
            stamp = node.get_clock().now().to_msg()
            estimated = position + [.2 * fraction, -.1 * fraction]
            message = Odometry()
            message.header = Header(stamp=stamp, frame_id='odom')
            message.child_frame_id = 'base_link'
            message.pose.pose.position.x, message.pose.pose.position.y = map(float, estimated)
            message.pose.pose.orientation.w = 1.
            for axis in (0, 7, 14, 21, 28, 35):
                message.pose.covariance[axis] = .01
            odometry.publish(message)
            transform = TransformStamped()
            transform.header = message.header
            transform.child_frame_id = 'base_link'
            transform.transform.translation.x, transform.transform.translation.y = map(float, estimated)
            transform.transform.rotation.w = 1.
            broadcaster.sendTransform(transform)
            local = world - [position[0], position[1], 0]
            clouds.publish(create_cloud_xyz32(Header(stamp=stamp, frame_id='base_link'), local))
            deadline = time.monotonic() + .65
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        links = [link for link in graph[-1].graph.links if link.type in (1, 2)] if graph else []
        corrected_error = None
        if graph:
            last = graph[-1].graph
            latest = last.poses[list(last.poses_id).index(max(last.poses_id))]
            corrected_error = float(np.hypot(latest.position.x, latest.position.y))
        raw_error = float(np.hypot(.2, -.1))
        report = {'accepted_closures': len(set(closures)), 'graph_loop_links': len(links),
                  'uncorrected_final_error': raw_error, 'corrected_final_error': corrected_error,
                  'cloud_points': received_clouds[-1].width if received_clouds else 0,
                  'passed': bool(closures and links and received_clouds and corrected_error is not None
                                 and corrected_error < raw_error * .6)}
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
