"""Publish the full XYZ map rebuilt from RTAB-Map's optimized LiDAR graph."""
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import json
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import Info, MapData
from rtabmap_msgs.srv import GetMap
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_srvs.srv import Trigger
from g1_backend.slam_lidar_graph import ScanDecoder, optimized_cloud, voxel_cloud
from g1_backend.slam_mapping import pose_matrix, write_pcd

ROOT = Path(__file__).resolve().parents[1]


def matrix(value):
    p = value.position if hasattr(value, 'position') else value.translation
    q = value.orientation if hasattr(value, 'orientation') else value.rotation
    return pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


class LidarGraphMap(Node):
    def __init__(self):
        super().__init__('g1_lidar_graph_map')
        self.output = Path(self.declare_parameter('output_dir', 'results/navigation/slam').value)
        self.period = float(self.declare_parameter('update_period', 2.0).value)
        self.max_scans = int(self.declare_parameter('max_scans', 1000).value)
        if self.period <= 0 or self.max_scans < 10:
            raise ValueError('Invalid LiDAR map update period or scan limit')
        self.output.mkdir(parents=True, exist_ok=True)
        self.decoder = ScanDecoder(ROOT / 'artifacts/slam/bin/librtabmap_image.so')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(PointCloud2, '/slam/map_points', qos)
        self.create_subscription(MapData, '/slam/map_data', self.map_data, 10)
        self.create_subscription(Info, '/slam/info', self.info, 10)
        self.client = self.create_client(GetMap, '/slam/get_map_data')
        self.initial_request = None
        self.initial_loaded = False
        self.scans = OrderedDict()
        self.poses = {}
        self.header = None
        self.version = 0
        self.submitted = 0
        self.future = None
        self.next_update = 0.
        self.latest_points = np.empty((0, 3), dtype=np.float32)
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.closures = []
        self.closure_ids = set()
        self.graph_links = 0
        self.correction = np.eye(4)
        self.create_timer(.2, self.poll)
        self.create_service(Trigger, '/slam/save_map', self.save_service)

    def map_data(self, message):
        poses = {key: matrix(pose) for key, pose in zip(message.graph.poses_id, message.graph.poses)
                 if key > 0}
        changed = self.poses.keys() != poses.keys() or any(
            not np.allclose(self.poses[key], pose, atol=1e-5, rtol=0)
            for key, pose in poses.items() if key in self.poses)
        self.poses = poses
        self.header = message.header
        self.correction = matrix(message.graph.map_to_odom)
        self.graph_links = sum(link.type in (1, 2) for link in message.graph.links)
        for key in list(self.scans):
            if key not in poses:
                del self.scans[key]
                changed = True
        for node in message.nodes:
            data = node.data
            if node.id <= 0 or node.id not in poses or node.id in self.scans or not data.laser_scan_compressed:
                continue
            try:
                points = self.decoder.decode(data.laser_scan_compressed, data.laser_scan_format)
                local = matrix(data.laser_scan_local_transform)
                points = points @ local[:3, :3].T + local[:3, 3]
                # Remove points inside the articulated robot's navigation footprint.
                self_hit = ((np.abs(points[:, 0]) < .4) & (np.abs(points[:, 1]) < .4)
                            & (points[:, 2] >= 0) & (points[:, 2] < 1.5))
                self.scans[node.id] = voxel_cloud(points[~self_hit])
                if len(self.scans) > self.max_scans:
                    self.scans.popitem(last=False)
                changed = True
            except ValueError as error:
                self.get_logger().warning(f'Cannot decode LiDAR keyframe {node.id}: {error}')
        if changed:
            self.version += 1

    def info(self, message):
        target = message.proximity_detection_id or message.loop_closure_id
        if target <= 0:
            return
        key = (message.ref_id, target)
        if key in self.closure_ids:
            return
        self.closure_ids.add(key)
        self.closures.append({'from': message.ref_id, 'to': target,
                              'method': 'lidar_icp_proximity',
                              'stamp': message.header.stamp.sec + message.header.stamp.nanosec * 1e-9})
        self.closures = self.closures[-500:]
        self.get_logger().info(f'LiDAR loop closure accepted: {key[0]} -> {key[1]}')
        self.save_report()

    def save_report(self):
        report = {'backend': 'FAST-LIO2 + RTAB-Map LiDAR ICP pose graph',
                  'accepted_closures': len(self.closure_ids), 'closures': self.closures,
                  'graph_loop_links': self.graph_links, 'graph_nodes': len(self.poses),
                  'retained_scans': len(self.scans), 'map_points': len(self.latest_points),
                  'map_to_odom': self.correction.tolist()}
        temporary = self.output / 'loop_closures.tmp.json'
        temporary.write_text(json.dumps(report, indent=2) + '\n')
        temporary.replace(self.output / 'loop_closures.json')

    def reconstruct(self, header, scans, poses):
        points = optimized_cloud(scans, poses)
        write_pcd(self.output / 'map.pcd', points)
        return header, points

    def poll(self):
        if not self.initial_loaded:
            if self.initial_request is None and self.client.service_is_ready():
                self.initial_request = self.client.call_async(
                    GetMap.Request(global_map=True, optimized=True, graph_only=False))
            elif self.initial_request is not None and self.initial_request.done():
                try:
                    response = self.initial_request.result()
                    if response.data.graph.poses_id:
                        self.map_data(response.data)
                    self.initial_loaded = True
                except (RuntimeError, ValueError) as error:
                    self.get_logger().warning(f'Cannot restore LiDAR keyframes: {error}')
                self.initial_request = None
        if self.future is not None:
            if not self.future.done():
                return
            try:
                header, self.latest_points = self.future.result()
                self.publisher.publish(create_cloud_xyz32(header, self.latest_points))
                self.save_report()
            except (RuntimeError, ValueError, OSError) as error:
                self.get_logger().error(f'Cannot rebuild optimized LiDAR map: {error}')
            self.future = None
        if self.scans and self.version != self.submitted and time.monotonic() >= self.next_update:
            self.submitted = self.version
            self.next_update = time.monotonic() + self.period
            self.future = self.pool.submit(self.reconstruct, self.header,
                                           dict(self.scans), dict(self.poses))

    def save_service(self, request, response):
        # Every completed graph snapshot is already saved atomically by the worker.
        response.success = bool(len(self.latest_points)) and (self.output / 'map.pcd').is_file()
        response.message = f'{len(self.latest_points)} optimized points: {self.output / "map.pcd"}'
        self.next_update = 0.
        return response

    def finish(self):
        self.pool.shutdown(wait=True, cancel_futures=False)
        if self.scans:
            _, self.latest_points = self.reconstruct(self.header, dict(self.scans), dict(self.poses))
        self.save_report()


def main():
    rclpy.init()
    node = LidarGraphMap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        try:
            node.finish()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
