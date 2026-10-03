"""Fuse RTAB-Map RGB-D keyframes into a colored TSDF mesh for RViz."""
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import json
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import open3d as o3d
import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import MapData
from rtabmap_msgs.srv import GetMap
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker
from g1_backend.slam_tsdf import ImageDecoder, RGBDFrame, TSDFReconstructor, retain_recent_view, transform_matrix

ROOT = Path(__file__).resolve().parents[1]


def pose_matrix(value):
    translation = value.position if hasattr(value, 'position') else value.translation
    rotation = value.orientation if hasattr(value, 'orientation') else value.rotation
    return transform_matrix([translation.x, translation.y, translation.z],
                            [rotation.x, rotation.y, rotation.z, rotation.w])


class MeshPublisher(Node):
    def __init__(self):
        super().__init__('g1_slam_mesh')
        self.output = Path(self.declare_parameter('output_dir', 'results/navigation/slam').value)
        self.period = float(self.declare_parameter('update_period', 5.0).value)
        self.max_frames = int(self.declare_parameter('max_frames', 400).value)
        if self.period <= 0 or not 3 <= self.max_frames <= 2000:
            raise ValueError('Invalid TSDF update period or frame limit')
        self.output.mkdir(parents=True, exist_ok=True)
        self.reconstructor = TSDFReconstructor(
            ImageDecoder(ROOT / 'artifacts/slam/bin/librtabmap_image.so'),
            voxel_size=float(self.declare_parameter('voxel_size', .03).value),
            depth_max=float(self.declare_parameter('depth_max', 6.0).value))
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Marker, '/slam/map_mesh', qos)
        self.create_subscription(MapData, '/slam/map_data', self.map_data, 10)
        self.client = self.create_client(GetMap, '/slam/get_map_data')
        self.initial_request = None
        self.initial_loaded = False
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.frames = OrderedDict()
        self.poses = {}
        self.header = None
        self.version = 0
        self.submitted = 0
        self.future = None
        self.next_update = 0.0
        self.saved_mesh = self.declare_parameter('saved_mesh', '').value
        self.saved_published = False
        self.create_timer(.2, self.poll)
        self.get_logger().info('TSDF mesh: 3 cm default voxels, optimized RTAB-Map RGB-D keyframes')

    def map_data(self, message):
        poses = {key: pose_matrix(pose) for key, pose in zip(message.graph.poses_id, message.graph.poses)
                 if key > 0}
        changed = self.poses.keys() != poses.keys() or any(
            not np.allclose(self.poses[key], pose, atol=1e-5, rtol=0)
            for key, pose in poses.items() if key in self.poses)
        self.poses = poses
        self.header = message.header
        for key in list(self.frames):
            if key not in poses:
                del self.frames[key]
                changed = True
        for node in message.nodes:
            data = node.data
            if (node.id <= 0 or node.id not in poses or node.id in self.frames
                    or not data.left_compressed or not data.right_compressed
                    or len(data.left_camera_info) != 1 or len(data.local_transform) != 1):
                continue
            local = pose_matrix(data.local_transform[0])
            info = data.left_camera_info[0]
            if info.k[0] <= 0 or info.k[4] <= 0:
                continue
            frame = RGBDFrame(
                bytes(data.left_compressed), bytes(data.right_compressed),
                (info.width, info.height, info.k[0], info.k[4], info.k[2], info.k[5]), local)
            retain_recent_view(self.frames, poses, node.id, frame, self.max_frames)
            changed = True
        if changed:
            self.version += 1

    def reconstruct(self, header, frames, poses):
        mesh, integrated, rebuilt = self.reconstructor.update(frames, poses)
        temporary = self.output / 'map_mesh.tmp.ply'
        if not o3d.io.write_triangle_mesh(str(temporary), mesh, write_ascii=True):
            raise OSError('Cannot save TSDF mesh')
        temporary.replace(self.output / 'map_mesh.ply')
        points = np.asarray(mesh.vertices).copy()
        colors = np.clip(np.asarray(mesh.vertex_colors), 0, 1).copy()
        triangles = np.asarray(mesh.triangles).copy()
        metadata = {'algorithm': 'Open3D ScalableTSDFVolume',
                    'voxel_size': self.reconstructor.voxel_size,
                    'depth_max': self.reconstructor.depth_max,
                    'frames': len(self.reconstructor.poses), 'triangles': len(triangles),
                    'rebuilds': self.reconstructor.rebuild_count, 'frame_id': header.frame_id,
                    'view_refresh_policy': 'latest_three_per_view',
                    'retained_frame_ids': list(self.reconstructor.poses)}
        temporary_metadata = self.output / 'map_mesh.tmp.json'
        temporary_metadata.write_text(json.dumps(metadata, indent=2) + '\n')
        temporary_metadata.replace(self.output / 'map_mesh.json')
        return header, points, colors, triangles, integrated, rebuilt

    def publish_result(self, result):
        header, points, colors, triangles, integrated, rebuilt = result
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
        marker.colors = [ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=1.0)
                         for c in colors[indices]]
        self.publisher.publish(marker)
        self.get_logger().info(f'TSDF mesh updated: {len(triangles)} triangles, '
                               f'{integrated} integrated frames, rebuilt={rebuilt}')

    def poll(self):
        if self.saved_mesh:
            if not self.saved_published:
                mesh = o3d.io.read_triangle_mesh(self.saved_mesh)
                if not mesh.has_triangles() or not mesh.has_vertex_colors():
                    raise ValueError('Saved mesh must contain triangles and vertex colors')
                header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
                self.publish_result((header, np.asarray(mesh.vertices),
                                     np.asarray(mesh.vertex_colors), np.asarray(mesh.triangles), 0, False))
                self.saved_published = True
                self.get_logger().info(f'Loaded saved mesh: {self.saved_mesh}')
            return
        # Restore image data if this node starts after RTAB-Map or a database is reused.
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
                    self.get_logger().warning(f'Cannot restore TSDF keyframes: {error}')
                self.initial_request = None
        if self.future is not None:
            if not self.future.done():
                return
            try:
                self.publish_result(self.future.result())
            except (OSError, ValueError, RuntimeError) as error:
                self.get_logger().error(f'TSDF reconstruction failed: {error}')
            self.future = None
        if self.frames and self.version != self.submitted and time.monotonic() >= self.next_update:
            self.submitted = self.version
            self.next_update = time.monotonic() + self.period
            self.future = self.pool.submit(self.reconstruct, self.header,
                                           dict(self.frames), dict(self.poses))

    def finish(self):
        # Finish a current snapshot and export the latest received graph on normal shutdown.
        self.pool.shutdown(wait=True, cancel_futures=False)
        if not self.saved_mesh and self.frames and self.version != self.submitted:
            self.reconstruct(self.header, dict(self.frames), dict(self.poses))


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
        try:
            node.finish()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
