"""Geometric RGB-D perception. No simulator state or object names are inputs."""

from dataclasses import dataclass

import numpy as np
from scipy.spatial import ConvexHull, QhullError, cKDTree
from scipy.spatial.transform import Rotation


@dataclass
class Settings:
    stride: int = 2
    min_depth: float = 0.2
    max_depth: float = 3.5
    voxel_size: float = 0.005
    plane_distance: float = 0.008
    plane_trials: int = 180
    min_table_points: int = 150
    min_table_height: float = 0.45
    max_table_height: float = 1.15
    max_table_tilt_deg: float = 12.0
    min_table_side: float = 0.25
    cluster_radius: float = 0.018
    min_object_points: int = 20
    min_object_height: float = 0.018
    max_object_height: float = 0.4
    max_object_side: float = 0.4
    min_top_fraction: float = 0.15
    min_top_rectangularity: float = 0.85
    max_tables: int = 3

    def validate(self):
        if not np.isfinite(list(vars(self).values())).all():
            raise ValueError('Perception settings must be finite')
        for name in (
            'stride',
            'plane_trials',
            'min_table_points',
            'min_object_points',
            'max_tables',
        ):
            value = getattr(self, name)
            if not isinstance(value, int) or value < 1:
                raise ValueError(f'{name} must be a positive integer')
        for name in (
            'voxel_size',
            'plane_distance',
            'min_table_side',
            'cluster_radius',
            'max_object_side',
            'min_object_height',
        ):
            if getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive')
        if not 0 < self.min_depth < self.max_depth:
            raise ValueError('Depth range must be positive and ordered')
        if not 0 <= self.min_table_height < self.max_table_height:
            raise ValueError('Table height range must be nonnegative and ordered')
        if not self.plane_distance < self.min_object_height < self.max_object_height:
            raise ValueError('Object heights must exceed plane tolerance and be ordered')
        if not 0 < self.max_table_tilt_deg < 45:
            raise ValueError('Table tilt tolerance must be between 0 and 45 degrees')
        if not (0 < self.min_top_fraction <= 1 and 0 < self.min_top_rectangularity <= 1):
            raise ValueError('Top quality thresholds must be in (0, 1]')


def backproject(depth, rgb, k, settings):
    """Return camera-optical XYZ and registered RGB, discarding invalid depths."""
    settings.validate()
    if depth.ndim != 2 or rgb.shape != (*depth.shape, 3):
        raise ValueError('RGB and depth must be registered and have equal resolution')
    k = np.asarray(k, dtype=float).reshape(3, 3)
    if not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError('Invalid camera intrinsics')
    if settings.stride < 1:
        raise ValueError('stride must be positive')
    v, u = np.mgrid[0 : depth.shape[0] : settings.stride, 0 : depth.shape[1] : settings.stride]
    z = depth[:: settings.stride, :: settings.stride]
    valid = np.isfinite(z) & (z >= settings.min_depth) & (z <= settings.max_depth)
    xyz = np.stack([(u - k[0, 2]) * z / k[0, 0], (v - k[1, 2]) * z / k[1, 1], z], axis=-1)
    return xyz[valid], rgb[:: settings.stride, :: settings.stride][valid]


def transform_points(points, matrix):
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


def clusters(points, radius):
    """Connected components of a spatial proximity graph."""
    tree = cKDTree(points)
    visited = np.zeros(len(points), dtype=bool)
    for start in range(len(points)):
        if visited[start]:
            continue
        visited[start] = True
        stack, members = [start], []
        while stack:
            i = stack.pop()
            members.append(i)
            for j in tree.query_ball_point(points[i], radius):
                if not visited[j]:
                    visited[j] = True
                    stack.append(j)
        yield np.asarray(members)


def rectangle(xy):
    """Minimum-area rectangle of observed points; yaw is modulo pi."""
    hull = ConvexHull(xy)
    vertices = xy[hull.vertices]
    edges = np.roll(vertices, -1, axis=0) - vertices
    best = None
    for angle in np.arctan2(edges[:, 1], edges[:, 0]):
        axes = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        local = xy @ axes
        low, high = local.min(axis=0), local.max(axis=0)
        size = high - low
        area = float(np.prod(size))
        if best is None or area < best[0]:
            best = (area, ((low + high) / 2) @ axes.T, size, angle)
    _, center, size, angle = best
    if size[1] > size[0]:
        size = size[::-1]
        angle += np.pi / 2
    return center, size, float((angle + np.pi / 2) % np.pi - np.pi / 2)


def pose(center, axes, output_transform):
    matrix = output_transform[:3, :3] @ axes
    return {
        'center': transform_points(np.asarray(center)[None], output_transform)[0].tolist(),
        'orientation_xyzw': Rotation.from_matrix(matrix).as_quat().tolist(),
    }


def detect(points, colors, settings=None, output_transform=None, *, detect_objects=True):
    """Detect support surfaces and object bounds in a gravity-aligned frame.

    The input frame's Z must point up, with z=0 at floor height. Results may
    be transformed into pelvis/arm frames, including roll/pitch, without
    changing physical dimensions. Surface thickness and hidden faces are unknown.
    """
    s = settings or Settings()
    s.validate()
    output_transform = np.eye(4) if output_transform is None else output_transform
    points, colors = np.asarray(points), np.asarray(colors)
    if points.shape != (len(points), 3) or colors.shape != (len(points), 3):
        raise ValueError('Expected matching Nx3 XYZ and RGB')
    valid = np.isfinite(points).all(axis=1)
    points, colors = points[valid], colors[valid]
    if not len(points):
        return []
    _, keep = np.unique(
        np.floor(points / s.voxel_size).astype(np.int64), axis=0, return_index=True
    )
    points, colors = points[keep], colors[keep]
    remaining = np.flatnonzero(
        (points[:, 2] >= s.min_table_height) & (points[:, 2] <= s.max_table_height)
    )
    rng = np.random.default_rng(0)
    tables = []
    for _ in range(s.max_tables):
        if len(remaining) < s.min_table_points:
            break
        cloud = points[remaining]
        best = np.empty(0, dtype=int)
        for _ in range(s.plane_trials):
            a, b, c = cloud[rng.choice(len(cloud), 3, replace=False)]
            normal = np.cross(b - a, c - a)
            norm = np.linalg.norm(normal)
            if norm < 1e-8:
                continue
            normal /= norm
            if abs(normal[2]) < np.cos(np.deg2rad(s.max_table_tilt_deg)):
                continue
            inliers = np.flatnonzero(np.abs((cloud - a) @ normal) < s.plane_distance)
            if len(inliers) > len(best):
                best = inliers
        if len(best) < s.min_table_points:
            break
        plane_ids = remaining[best]
        remaining = remaining[~np.isin(remaining, plane_ids)]
        # Separate disconnected coplanar tables before fitting their footprints.
        for component in clusters(points[plane_ids], max(0.04, s.cluster_radius * 2)):
            ids = plane_ids[component]
            if len(ids) < s.min_table_points:
                continue
            surface = points[ids]
            origin = surface.mean(axis=0)
            _, _, vh = np.linalg.svd(surface - origin, full_matrices=False)
            normal = vh[-1]
            if normal[2] < 0:
                normal = -normal
            if normal[2] < np.cos(np.deg2rad(s.max_table_tilt_deg)):
                continue
            x = np.array([1.0, 0.0, 0.0])
            x -= normal * (x @ normal)
            x /= np.linalg.norm(x)
            basis = np.column_stack([x, np.cross(normal, x), normal])
            local = (points - origin) @ basis
            try:
                center2, size2, yaw = rectangle(local[ids, :2])
                hull = ConvexHull(local[ids, :2])
            except QhullError:
                continue
            if min(size2) < s.min_table_side:
                continue
            rz = Rotation.from_euler('z', yaw).as_matrix()
            center = origin + basis @ np.r_[center2, 0.0]
            table = dict(
                kind='table_surface',
                **pose(center, basis @ rz, output_transform),
                dimensions=[*size2.tolist(), 0.0],
                point_count=len(ids),
                thickness_known=False,
                objects=[],
            )
            if not detect_objects:
                tables.append(table)
                continue
            inside = np.all(
                local[:, :2] @ hull.equations[:, :2].T + hull.equations[:, 2] <= s.plane_distance,
                axis=1,
            )
            candidates = np.flatnonzero(
                inside
                & (local[:, 2] >= s.min_object_height)
                & (local[:, 2] <= s.max_object_height)
            )
            for component in clusters(points[candidates], s.cluster_radius):
                object_ids = candidates[component]
                if len(object_ids) < s.min_object_points:
                    continue
                observed = local[object_ids]
                try:
                    obj_center2, obj_size2, obj_yaw = rectangle(observed[:, :2])
                except QhullError:
                    continue
                height = float(np.quantile(observed[:, 2], 0.98))
                if min(obj_size2) < s.voxel_size * 2 or max(obj_size2) > s.max_object_side:
                    continue
                # A bounding box alone also fits cylinders and spheres. Require
                # a visible flat, rectangular top before labeling a box candidate.
                top = observed[np.abs(observed[:, 2] - height) <= s.plane_distance]
                if len(top) < 8 or len(top) / len(observed) < s.min_top_fraction:
                    continue
                try:
                    _, top_size, _ = rectangle(top[:, :2])
                    rectangularity = float(ConvexHull(top[:, :2]).volume / np.prod(top_size))
                except QhullError:
                    continue
                if rectangularity < s.min_top_rectangularity:
                    continue
                obj_center = origin + basis @ np.r_[obj_center2, height / 2]
                obj_axes = basis @ Rotation.from_euler('z', obj_yaw).as_matrix()
                table['objects'].append(
                    dict(
                        kind='box_candidate',
                        **pose(obj_center, obj_axes, output_transform),
                        dimensions=[*obj_size2.tolist(), height],
                        point_count=len(object_ids),
                        mean_rgb=colors[object_ids].mean(axis=0).tolist(),
                        top_rectangularity=rectangularity,
                        geometry_complete=False,
                        yaw_ambiguous=bool(obj_size2[0] / obj_size2[1] < 1.15),
                    )
                )
            tables.append(table)
    return tables
