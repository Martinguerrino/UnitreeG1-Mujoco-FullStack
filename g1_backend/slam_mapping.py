"""Sensor-only geometry and bounded mapping helpers shared by 3-D SLAM modes."""
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def pose_matrix(position, quaternion):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    matrix[:3, 3] = position
    return matrix


def navigation_pose(sensor_pose, pelvis_sensor, world_offset):
    """Recover pelvis from estimated sensor pose and articulated local TF only."""
    pelvis = sensor_pose @ np.linalg.inv(pelvis_sensor)
    pelvis[:3, 3] += world_offset
    yaw = Rotation.from_matrix(pelvis[:3, :3]).as_euler('xyz')[2]
    base = np.eye(4)
    base[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
    base[:2, 3] = pelvis[:2, 3]
    return base, np.linalg.inv(base) @ pelvis


class VoxelMap:
    def __init__(self, resolution=0.08, maximum=400000):
        self.resolution = resolution
        self.maximum = maximum
        self.cells = {}

    def add(self, xyz):
        xyz = np.asarray(xyz, dtype=np.float32)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        keys = np.floor(xyz / self.resolution).astype(np.int64)
        from .cuda.geometry import voxel_indices

        indices = voxel_indices(xyz, self.resolution)
        if indices is None:
            _, indices = np.unique(keys, axis=0, return_index=True)
        for index in indices:
            key = tuple(keys[index])
            if key in self.cells or len(self.cells) < self.maximum:
                self.cells[key] = xyz[index]

    def points(self):
        return np.asarray(list(self.cells.values()), dtype=np.float32).reshape(-1, 3)


def write_pcd(path, xyz, rgb=None):
    """Atomic binary PCD, retaining packed RGB bytes exactly when present."""
    xyz = np.asarray(xyz, dtype='<f4').reshape(-1, 3)
    valid = np.isfinite(xyz).all(axis=1)
    xyz = xyz[valid]
    names = ['x', 'y', 'z'] + (['rgb'] if rgb is not None else [])
    records = np.empty(len(xyz), dtype=[(name, '<f4') for name in names])
    for index, name in enumerate(names[:3]):
        records[name] = xyz[:, index]
    if rgb is not None:
        color = np.asarray(rgb)[valid]
        if color.dtype.kind == 'f':
            color = color.astype('<f4').view('<u4')
        records['rgb'] = color.astype('<u4').view('<f4')
    header = (
        '# .PCD v0.7\nVERSION 0.7\nFIELDS ' + ' '.join(names)
        + '\nSIZE ' + ' '.join(['4'] * len(names))
        + '\nTYPE ' + ' '.join(['F'] * len(names))
        + '\nCOUNT ' + ' '.join(['1'] * len(names))
        + f'\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n'
    )
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('wb') as handle:
        handle.write(header.encode('ascii'))
        handle.write(records.tobytes())
    temporary.replace(path)
    return len(xyz)


class OccupancyProjection:
    """Planar obstacle ray tracing from registered measurements in the map frame."""
    def __init__(self, resolution=0.1, extent=50.0):
        self.resolution = resolution
        self.extent = extent
        self.cells = {}

    def add(self, points, origin):
        points = np.asarray(points)
        points = points[np.isfinite(points).all(axis=1)]
        points = points[(points[:, 2] >= 0.15) & (points[:, 2] <= 1.5)]
        points = points[np.linalg.norm(points[:, :2] - origin[:2], axis=1) <= 20]
        # Ray trace the nearest obstacle in each angular sector, rather than
        # thousands of overlapping rays. Keep the odometry callback responsive.
        delta = points[:, :2] - origin[:2]
        sectors = np.floor((np.arctan2(delta[:, 1], delta[:, 0]) + np.pi) * 360 / (2 * np.pi)).astype(int) % 360
        order = np.lexsort((np.linalg.norm(delta, axis=1), sectors))
        _, selected = np.unique(sectors[order], return_index=True)
        points = points[order[selected]]
        endpoints = np.unique(np.floor(points[:, :2] / self.resolution).astype(int), axis=0)
        start = np.floor(origin[:2] / self.resolution).astype(int)
        limit = self.extent / self.resolution
        for end in endpoints:
            if np.any(np.abs(end) >= limit):
                continue
            steps = int(np.max(np.abs(end - start)))
            if steps:
                ray = np.rint(np.linspace(start, end, steps + 1)).astype(int)
                for cell in ray[:-1]:
                    self.cells.setdefault(tuple(cell), 0)
            self.cells[tuple(end)] = 100

    def grid(self):
        if not self.cells:
            return None
        keys = np.asarray(list(self.cells))
        lower, upper = keys.min(axis=0) - 5, keys.max(axis=0) + 5
        width, height = upper - lower + 1
        grid = np.full((height, width), -1, dtype=np.int8)
        for key, value in self.cells.items():
            x, y = np.asarray(key) - lower
            grid[y, x] = value
        return grid, lower * self.resolution
