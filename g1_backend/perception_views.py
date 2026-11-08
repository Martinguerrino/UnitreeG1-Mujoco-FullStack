"""Bounded accumulation of observed surfaces from distinct camera poses."""

from collections import deque
from dataclasses import dataclass

import numpy as np
from scipy.spatial import QhullError
from scipy.spatial.transform import Rotation

from .perception import rectangle


@dataclass
class ViewSettings:
    max_views: int = 6
    max_points: int = 15000
    max_age: float = 4.0
    voxel_size: float = 0.004
    min_translation: float = 0.03
    min_rotation_deg: float = 8.0
    motion_reset: float = 0.08

    def validate(self):
        if self.max_views < 1 or self.max_points < 10:
            raise ValueError('Invalid view/point capacity')
        if (
            min(
                self.max_age,
                self.voxel_size,
                self.min_translation,
                self.min_rotation_deg,
                self.motion_reset,
            )
            <= 0
        ):
            raise ValueError('Multi-view settings must be positive')


class ViewAccumulator:
    def __init__(self, settings=None):
        self.settings = settings or ViewSettings()
        self.settings.validate()
        self.tracks = {}
        self.last_time = None

    def update(self, tables, camera_pose, stamp):
        s = self.settings
        if self.last_time is not None and stamp < self.last_time:
            self.tracks.clear()
        self.last_time = stamp
        self.tracks = {
            key: value
            for key, value in self.tracks.items()
            if stamp - value['last_seen'] <= s.max_age
        }
        for table in tables:
            for item in [table, *table['objects']]:
                points = item.pop('_points', None)
                support = item.pop('_support_center', None)
                normal = item.pop('_support_normal', None)
                if points is None or not len(points):
                    continue
                raw_center = item['raw_pose']['center']
                state = self.tracks.get(item['id'])
                moving = (
                    state is not None
                    and item['kind'] != 'table_surface'
                    and np.linalg.norm(np.array(raw_center) - state['anchor_center'])
                    > s.motion_reset
                )
                if state is None or moving:
                    state = {
                        'views': deque(maxlen=s.max_views),
                        'last_seen': stamp,
                        'raw_center': raw_center,
                        'anchor_center': raw_center,
                    }
                    self.tracks[item['id']] = state
                state['last_seen'], state['raw_center'] = stamp, raw_center
                views = state['views']
                retained = [view for view in views if stamp - view['stamp'] <= s.max_age]
                views.clear()
                views.extend(retained)
                from .cuda.geometry import voxel_indices

                keep = voxel_indices(points, s.voxel_size)
                if keep is None:
                    _, keep = np.unique(
                        np.floor(points / s.voxel_size).astype(np.int64), axis=0, return_index=True
                    )
                points = points[keep]
                if len(points) > s.max_points:
                    points = points[np.linspace(0, len(points) - 1, s.max_points, dtype=int)]
                view = {'pose': camera_pose.copy(), 'points': points, 'stamp': stamp}
                similar = []
                for index, previous in enumerate(views):
                    translation = np.linalg.norm(camera_pose[:3, 3] - previous['pose'][:3, 3])
                    rotation = Rotation.from_matrix(
                        previous['pose'][:3, :3].T @ camera_pose[:3, :3]
                    ).magnitude()
                    if translation < s.min_translation and rotation < np.deg2rad(
                        s.min_rotation_deg
                    ):
                        similar.append(index)
                if similar:
                    views[similar[0]] = view
                else:
                    views.append(view)
                cloud = np.vstack([view['points'] for view in views])
                if len(cloud) > s.max_points:
                    cloud = cloud[np.linspace(0, len(cloud) - 1, s.max_points, dtype=int)]
                axes = Rotation.from_quat(item['orientation_xyzw']).as_matrix()
                origin = np.asarray(item['center'] if support is None else support)
                if normal is not None:
                    z = np.asarray(normal)
                    x = axes[:, 0] - z * (axes[:, 0] @ z)
                    x /= np.linalg.norm(x)
                    axes = np.column_stack([x, np.cross(z, x), z])
                local = (cloud - origin) @ axes
                if len(views) > 1:
                    try:
                        xy, size, yaw = rectangle(local[:, :2])
                        height = (
                            0.0
                            if item['kind'] == 'table_surface'
                            else float(np.quantile(local[:, 2], 0.98))
                        )
                        center = origin + axes @ np.r_[xy, height / 2]
                        rotation = axes @ Rotation.from_euler('z', yaw).as_matrix()
                        item['center'] = center.tolist()
                        item['dimensions'] = [*size.tolist(), height]
                        item['orientation_xyzw'] = (
                            Rotation.from_matrix(rotation).as_quat().tolist()
                        )
                    except QhullError:
                        pass
                item['quality']['view_count'] = len(views)
                item['quality']['accumulated_points'] = len(cloud)
                poses = np.array([view['pose'][:3, 3] for view in views])
                baseline = float(np.max(np.linalg.norm(poses[:, None] - poses[None, :], axis=-1)))
                item['quality']['view_baseline_m'] = baseline
                item['quality']['geometry_complete'] = False
                item['quality']['warnings'].append(
                    'insufficient_view_diversity'
                    if len(views) == 1
                    else 'hidden_faces_still_unknown'
                )
        return tables
