"""Rebuild bounded LiDAR clouds from optimized scan poses, without stale points."""
import ctypes

import numpy as np


class ScanDecoder:
    def __init__(self, library):
        self.library = ctypes.CDLL(str(library))
        self.library.decode_scan.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int,
                                            ctypes.c_void_p, ctypes.c_size_t,
                                            ctypes.POINTER(ctypes.c_size_t)]
        self.library.decode_scan.restype = ctypes.c_int

    def decode(self, compressed, scan_format):
        source = np.frombuffer(bytes(compressed), dtype=np.uint8)
        output = np.empty((100000, 3), dtype=np.float32)
        count = ctypes.c_size_t()
        status = self.library.decode_scan(source.ctypes.data, source.size, scan_format,
                                         output.ctypes.data, output.size, ctypes.byref(count))
        if status != 0:
            raise ValueError(f'RTAB-Map 3-D scan decoding failed ({status})')
        return output[:count.value].copy()


def voxel_cloud(points, resolution=.08, maximum=400000):
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        return points
    keys = np.floor(points / resolution).astype(np.int64)
    from .cuda.geometry import voxel_indices

    indices = voxel_indices(points, resolution)
    if indices is None:
        _, indices = np.unique(keys, axis=0, return_index=True)
    if len(indices) > maximum:
        indices = indices[np.linspace(0, len(indices) - 1, maximum, dtype=int)]
    return points[indices]


def optimized_cloud(scans, poses, resolution=.08, maximum=400000):
    from .cuda.geometry import optimized_cloud as gpu_cloud

    accelerated = gpu_cloud(scans, poses, resolution, maximum)
    if accelerated is not None:
        return accelerated
    chunks = [points @ poses[key][:3, :3].T + poses[key][:3, 3]
              for key, points in scans.items() if key in poses]
    return voxel_cloud(np.concatenate(chunks) if chunks else np.empty((0, 3)),
                       resolution, maximum)
