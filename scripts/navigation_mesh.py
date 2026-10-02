"""Reconstruct and publish the latest optimized RGB-D map as a colored mesh."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from g1_backend.slam_mapping import write_pcd
from g1_backend.slam_mesh import read_mesh

ROOT = Path(__file__).resolve().parents[1]


class MeshPublisher(Node):
    def __init__(self):
        super().__init__('g1_slam_mesh')
        self.output = Path(self.declare_parameter('output_dir', 'results/navigation/slam').value)
        self.period = float(self.declare_parameter('update_period', 5.0).value)
        if self.period <= 0:
            raise ValueError('update_period must be positive')
        self.output.mkdir(parents=True, exist_ok=True)
        self.binary = ROOT / 'artifacts/slam/bin/g1_cloud_mesh'
        if not self.binary.is_file():
            raise RuntimeError('Run bash scripts/build_navigation_mesh.sh first')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Marker, '/slam/map_mesh', qos)
        self.create_subscription(PointCloud2, '/slam/map_points', self.cloud, qos_profile_sensor_data)
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.latest = None
        self.version = 0
        self.submitted = 0
        self.future = None
        self.next_update = 0.0
        self.create_timer(.2, self.poll)

    def cloud(self, message):
        self.latest = message
        self.version += 1

    def reconstruct(self, message):
        data = read_points(message, skip_nans=False)
        xyz = np.column_stack([data[name].ravel() for name in ('x', 'y', 'z')])
        rgb = data['rgb'].ravel() if 'rgb' in data.dtype.names else np.full(len(xyz), 0xaaaaaa, dtype=np.uint32)
        with tempfile.TemporaryDirectory(prefix='mesh_', dir=self.output) as workspace:
            cloud_path, mesh_path = Path(workspace) / 'cloud.pcd', Path(workspace) / 'mesh.ply'
            write_pcd(cloud_path, xyz, rgb)
            subprocess.run(['nice', '-n', '10', str(self.binary), str(cloud_path), str(mesh_path)],
                           check=True, capture_output=True, timeout=15)
            points, colors, triangles = read_mesh(mesh_path)
            mesh_path.replace(self.output / 'map_mesh.ply')
        return message.header, points, colors, triangles

    def poll(self):
        if self.future is not None:
            if not self.future.done():
                return
            try:
                header, points, colors, triangles = self.future.result()
                marker = Marker()
                marker.header = header
                marker.ns, marker.id = 'slam_mesh', 0
                marker.type = Marker.TRIANGLE_LIST
                marker.action = Marker.ADD if len(triangles) else Marker.DELETE
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = 1.0
                marker.color.a = 1.0
                indices = triangles.ravel()
                marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points[indices]]
                marker.colors = [ColorRGBA(r=float(c[0]) / 255, g=float(c[1]) / 255,
                                           b=float(c[2]) / 255, a=1.0) for c in colors[indices]]
                self.publisher.publish(marker)
                self.get_logger().info(f'Mesh updated: {len(triangles)} triangles')
            except (OSError, ValueError, subprocess.SubprocessError) as error:
                self.get_logger().error(f'Mesh reconstruction failed: {error}')
            self.future = None
        if self.latest is not None and self.version != self.submitted and time.monotonic() >= self.next_update:
            self.submitted = self.version
            self.next_update = time.monotonic() + self.period
            self.future = self.pool.submit(self.reconstruct, self.latest)


def main():
    rclpy.init()
    node = MeshPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.pool.shutdown(wait=True, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
