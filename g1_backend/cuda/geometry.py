"""Bounded CUDA kernels returning NumPy arrays; None requests the original CPU path."""

import numpy as np
from ..acceleration import _DISABLED, cupy, disable


def attempt(name, size, operation, minimum=32768):
    if size < minimum or name in _DISABLED:
        return None
    cp = cupy()
    if cp is None:
        return None
    try:
        return operation(cp)
    except Exception as error:  # noqa: BLE001 - retry optional backend on CPU
        disable(name, error)
        return None


def transform(points, matrix):
    return attempt(
        'transform',
        len(points),
        lambda cp: cp.asnumpy(
            cp.asarray(points) @ cp.asarray(matrix[:3, :3]).T + cp.asarray(matrix[:3, 3])
        ),
    )


def _voxel_device_indices(cp, points, voxel):
    cells = cp.floor(points / voxel).astype(cp.int64)
    lower, upper = cp.asnumpy(cells.min(axis=0)), cp.asnumpy(cells.max(axis=0))
    widths = [int(b) - int(a) + 1 for a, b in zip(lower, upper)]
    if widths[0] * widths[1] * widths[2] > np.iinfo(np.int64).max:
        raise RuntimeError('Voxel coordinates exceed the int64 key budget')
    # Integer keys preserve NumPy's lexicographic XYZ ordering while avoiding
    # CuPy's expensive multi-column lexsort implementation.
    keys = (
        ((cells[:, 0] - int(lower[0])) * widths[1] + cells[:, 1] - int(lower[1])) * widths[2]
        + cells[:, 2]
        - int(lower[2])
    )
    return cp.unique(keys, return_index=True)[1]


def voxel_indices(points, voxel):
    return attempt(
        'voxel',
        len(points),
        lambda cp: cp.asnumpy(_voxel_device_indices(cp, cp.asarray(points), voxel)),
    )


def plane_inliers(cloud, hypotheses, distance, tilt):
    def compute(cp):
        points = cp.asarray(cloud)
        triangles = points[cp.asarray(hypotheses)]
        normals = cp.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        norms = cp.linalg.norm(normals, axis=1)
        normals /= cp.maximum(norms[:, None], 1e-8)
        eligible = (norms >= 1e-8) & (cp.abs(normals[:, 2]) >= tilt)
        counts = cp.zeros(len(hypotheses), dtype=cp.int64)
        # Bounded scratch space rather than an N-points x N-trials allocation.
        for start in range(0, len(cloud), 8192):
            residual = points[start : start + 8192] @ normals.T
            residual -= cp.sum(triangles[:, 0] * normals, axis=1)[None]
            counts += cp.sum(cp.abs(residual) < distance, axis=0)
        counts = cp.where(eligible, counts, 0)
        index = int(cp.argmax(counts).get())
        if int(counts[index].get()) == 0:
            return np.empty(0, dtype=int)
        return cp.asnumpy(
            cp.flatnonzero(cp.abs((points - triangles[index, 0]) @ normals[index]) < distance)
        )

    return attempt('planes', len(cloud), compute, minimum=8192)


def project_scan(points, position, yaw, config):
    def compute(cp):
        cloud = cp.asarray(points, dtype=cp.float64).reshape(-1, 3)
        delta = cloud[:, :2] - cp.asarray(position)[:2]
        ranges = cp.linalg.norm(delta, axis=1)
        valid = (
            cp.isfinite(cloud).all(axis=1)
            & (cloud[:, 2] >= config.get('scan_min_height', 0.15))
            & (cloud[:, 2] <= config.get('scan_max_height', 1.5))
            & (ranges >= config['min_range'])
            & (ranges <= config['max_range'])
        )
        count = config['horizontal_samples']
        angle = (cp.arctan2(delta[valid, 1], delta[valid, 0]) - yaw + np.pi) % (2 * np.pi)
        bins = cp.floor(angle * count / (2 * np.pi) + 0.5).astype(cp.int64) % count
        nearest = cp.full(count, cp.inf)
        # CuPy minimum.at currently rounds float64 values through float32.
        # Use a double-precision CAS reduction to preserve the sensor contract.
        kernel = cp.RawKernel(
            r"""
        extern "C" __global__ void nearest_range(
            const long long* bins, const double* ranges, double* nearest, int n) {
          int i=blockIdx.x*blockDim.x+threadIdx.x; if(i>=n) return;
          unsigned long long* address=(unsigned long long*)(nearest+bins[i]);
          unsigned long long old=*address, assumed;
          do {
            assumed=old;
            double previous=__longlong_as_double(assumed);
            if(previous<=ranges[i]) break;
            old=atomicCAS(address,assumed,__double_as_longlong(ranges[i]));
          } while(old!=assumed);
        }
        """,
            'nearest_range',
        )
        selected = ranges[valid]
        if len(selected):
            kernel(
                ((len(selected) + 127) // 128,),
                (128,),
                (bins, selected, nearest, np.int32(len(selected))),
            )
        return [float(value) if np.isfinite(value) else None for value in cp.asnumpy(nearest)]

    return attempt('scan_projection', len(points), compute, minimum=4096)


def backproject(depth, rgb, k, settings):
    from .workspace import active

    workspace = active()
    if workspace is not None:
        return attempt(
            'backproject',
            depth.size,
            lambda cp: workspace.backproject(depth, rgb, k, settings),
            minimum=0,
        )

    def compute(cp):
        s = settings
        v, u = cp.mgrid[0 : depth.shape[0] : s.stride, 0 : depth.shape[1] : s.stride]
        z = cp.asarray(depth)[:: s.stride, :: s.stride]
        valid = cp.isfinite(z) & (z >= s.min_depth) & (z <= s.max_depth)
        xyz = cp.stack([(u - k[0, 2]) * z / k[0, 0], (v - k[1, 2]) * z / k[1, 1], z], axis=-1)
        colors = cp.asarray(rgb)[:: s.stride, :: s.stride]
        return cp.asnumpy(xyz[valid]), cp.asnumpy(colors[valid])

    return attempt('backproject', depth.size, compute, minimum=131072)


def optimized_cloud(scans, poses, resolution, maximum):
    def compute(cp):
        chunks = [
            cp.asarray(points, dtype=cp.float32) @ cp.asarray(poses[key][:3, :3]).T
            + cp.asarray(poses[key][:3, 3])
            for key, points in scans.items()
            if key in poses
        ]
        cloud = cp.concatenate(chunks).astype(cp.float32)
        cloud = cloud[cp.isfinite(cloud).all(axis=1)]
        if not len(cloud):
            return np.empty((0, 3), dtype=np.float32)
        indices = _voxel_device_indices(cp, cloud, resolution)
        if len(indices) > maximum:
            indices = indices[cp.asarray(np.linspace(0, len(indices) - 1, maximum, dtype=int))]
        return cp.asnumpy(cloud[indices])

    count = sum(len(points) for key, points in scans.items() if key in poses)
    return attempt('optimized_cloud', count, compute)
