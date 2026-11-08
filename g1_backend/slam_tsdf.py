"""RGB-D TSDF fusion in optimized map coordinates (no ground-truth inputs)."""
from dataclasses import dataclass
import ctypes

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation


def transform_matrix(translation, quaternion):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    result[:3, 3] = translation
    return result


class ImageDecoder:
    """Use RTAB-Map's codec, including its compressed floating-point depth."""
    def __init__(self, library):
        self.library = ctypes.CDLL(str(library))
        self.library.decode_image.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int,
                                             ctypes.c_void_p, ctypes.c_size_t,
                                             ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
        self.library.decode_image.restype = ctypes.c_int

    def decode(self, compressed, depth=False):
        source = np.frombuffer(bytes(compressed), dtype=np.uint8)
        # Bound each decoded image to four million pixels.
        output = np.empty(4_000_000 * (1 if depth else 3), dtype=np.float32 if depth else np.uint8)
        width, height = ctypes.c_int(), ctypes.c_int()
        status = self.library.decode_image(source.ctypes.data, source.size, int(depth),
                                          output.ctypes.data, output.nbytes,
                                          ctypes.byref(width), ctypes.byref(height))
        if status != 0:
            raise ValueError(f'RTAB-Map image decoding failed ({status})')
        shape = (height.value, width.value) if depth else (height.value, width.value, 3)
        return output[:int(np.prod(shape))].reshape(shape).copy()


@dataclass(frozen=True)
class RGBDFrame:
    color: bytes
    depth: bytes
    intrinsics: tuple
    local_transform: np.ndarray


class TSDFReconstructor:
    """Incremental fusion; rebuild when optimized poses or retained nodes change."""
    def __init__(self, decoder, voxel_size=.03, depth_max=6.0, max_triangles=100000):
        if not .005 <= voxel_size <= .2 or not .5 < depth_max <= 20 or max_triangles < 1:
            raise ValueError('Invalid TSDF resolution, range or triangle limit')
        self.decoder = decoder
        self.voxel_size = voxel_size
        self.depth_max = depth_max
        self.max_triangles = max_triangles
        self.poses = {}
        self.volume = None
        self.rebuild_count = 0
        self.gpu_failed = False
        self.backend = 'cpu'

    def update(self, frames, poses):
        from .acceleration import disable, enabled

        if self.backend == 'cuda' and not enabled():
            self.volume, self.poses = None, {}
        try:
            return self._update(frames, poses)
        except Exception as error:
            if self.backend != 'cuda':
                raise
            disable('tsdf', error)
            self.gpu_failed = True
            self.volume, self.poses = None, {}
            return self._update(frames, poses)

    def _update(self, frames, poses):
        camera_poses = {key: poses[key] @ frame.local_transform
                        for key, frame in frames.items() if key in poses}
        # Even small pose corrections must not leave stale surfaces behind.
        rebuild = (self.volume is None or not self.poses.keys() <= camera_poses.keys()
                   or any(not np.allclose(pose, camera_poses[key], atol=1e-5, rtol=0)
                          for key, pose in self.poses.items() if key in camera_poses))
        if rebuild:
            from .acceleration import enabled, disable
            from .cuda.tsdf import CudaTSDFVolume

            self.backend = 'cpu'
            if not self.gpu_failed and enabled() and o3d.core.cuda.is_available():
                try:
                    self.volume = CudaTSDFVolume(o3d, self.voxel_size, self.depth_max)
                    self.backend = 'cuda'
                except Exception as error:  # noqa: BLE001 - retry optional backend on CPU
                    disable('tsdf', error)
                    self.gpu_failed = True
            if self.backend == 'cpu':
                self.volume = o3d.pipelines.integration.ScalableTSDFVolume(
                    voxel_length=self.voxel_size, sdf_trunc=self.voxel_size * 3,
                    color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
                    depth_sampling_stride=2)
            self.poses = {}
            self.rebuild_count += 1
        integrated = 0
        for key, camera_pose in camera_poses.items():
            if key in self.poses:
                continue
            frame = frames[key]
            color = self.decoder.decode(frame.color)
            depth = self.decoder.decode(frame.depth, depth=True)
            if color.shape[:2] != depth.shape:
                raise ValueError('TSDF requires aligned RGB and depth images')
            depth[~np.isfinite(depth) | (depth < .5) | (depth > self.depth_max)] = 0
            width, height, fx, fy, cx, cy = frame.intrinsics
            if depth.shape != (height, width):
                raise ValueError('RGB-D dimensions differ from camera calibration')
            if self.backend == 'cuda':
                self.volume.integrate_arrays(color, depth, frame.intrinsics,
                                             np.linalg.inv(camera_pose))
            else:
                intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
                rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                    o3d.geometry.Image(np.ascontiguousarray(color)),
                    o3d.geometry.Image(np.ascontiguousarray(depth)), depth_scale=1.0,
                    depth_trunc=self.depth_max, convert_rgb_to_intensity=False)
                self.volume.integrate(rgbd, intrinsic, np.linalg.inv(camera_pose))
            self.poses[key] = camera_pose.copy()
            integrated += 1
        mesh = self.volume.extract_triangle_mesh()
        mesh.remove_degenerate_triangles()
        mesh.remove_duplicated_triangles()
        if len(mesh.triangles) > self.max_triangles:
            mesh = mesh.simplify_quadric_decimation(self.max_triangles)
        mesh.remove_unreferenced_vertices()
        return mesh, integrated, rebuild


def same_view(first, second, distance=.02, angle=.02):
    delta = np.linalg.inv(first) @ second
    return (np.linalg.norm(delta[:3, 3]) < distance
            and Rotation.from_matrix(delta[:3, :3]).magnitude() < angle)


def retain_recent_view(frames, poses, key, frame, max_frames, max_repeats=3):
    """Refresh repeated views without accumulating unlimited stationary weight.

    Eviction makes TSDFReconstructor rebuild, removing old surfaces rather than
    merely integrating new observations on top of permanently weighted errors.
    Other viewpoints remain available for the rest of the map.
    """
    camera_pose = poses[key] @ frame.local_transform
    repeats = [old for old, previous in frames.items()
               if old in poses and same_view(camera_pose, poses[old] @ previous.local_transform)]
    for old in repeats[:max(0, len(repeats) - max_repeats + 1)]:
        del frames[old]
    frames[key] = frame
    while len(frames) > max_frames:
        frames.popitem(last=False)
