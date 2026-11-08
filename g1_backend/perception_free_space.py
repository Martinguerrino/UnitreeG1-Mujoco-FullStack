"""Conservative observed tabletop patches, never inferred hidden free space."""

from dataclasses import dataclass

import numpy as np
from scipy.ndimage import distance_transform_edt as cpu_distance_transform
from scipy.spatial.transform import Rotation

from .perception import pose


def distance_transform_edt(mask):
    from .cuda.images import distance_transform

    result = distance_transform(mask)
    return cpu_distance_transform(mask) if result is None else result


@dataclass
class FreeSpaceSettings:
    cell_size: float = 0.025
    edge_margin: float = 0.04
    obstacle_margin: float = 0.04
    min_side: float = 0.08
    max_regions: int = 6
    max_cells: int = 20000

    def validate(self):
        limits = [self.cell_size, self.edge_margin, self.obstacle_margin, self.min_side]
        if (
            not np.isfinite(limits).all()
            or min(limits) <= 0
            or self.max_regions < 1
            or self.max_cells < 1
        ):
            raise ValueError('Free-space settings must be positive and finite')


def largest_rectangle(mask):
    """Maximum-area rectangle of true grid cells, using row histograms."""
    heights = np.zeros(mask.shape[1], dtype=int)
    best, bounds = 0, None
    for row, cells in enumerate(mask):
        heights = np.where(cells, heights + 1, 0)
        stack = []
        for col, height in enumerate(np.r_[heights, 0]):
            left = col
            while stack and stack[-1][1] > height:
                start, previous = stack.pop()
                area = previous * (col - start)
                if area > best:
                    best, bounds = area, (row - previous + 1, row + 1, start, col)
                left = start
            if not stack or stack[-1][1] < height:
                stack.append((left, height))
    return bounds


def free_space(table, scene_points, settings=None, plane_distance=0.008, min_height=0.018):
    s = settings or FreeSpaceSettings()
    s.validate()
    dimensions = np.array(table['dimensions'][:2], dtype=float)
    axes = Rotation.from_quat(table['orientation_xyzw']).as_matrix()
    center = np.asarray(table['center'])
    shape = np.ceil(dimensions / s.cell_size).astype(int)
    result = {
        'observed_only': True,
        'cell_size_m': s.cell_size,
        'edge_margin_m': s.edge_margin,
        'obstacle_margin_m': s.obstacle_margin,
        'area_m2': 0.0,
        'status': 'insufficient_observed_surface',
    }
    if min(shape) < 1 or np.prod(shape) > s.max_cells:
        result['status'] = 'surface_grid_limit'
        return result, []
    local = (np.asarray(scene_points).reshape(-1, 3) - center) @ axes
    indices = np.floor((local[:, :2] + dimensions / 2) / s.cell_size).astype(int)
    inside = np.all((indices >= 0) & (indices < shape), axis=1)
    observed = np.zeros(tuple(shape[::-1]), dtype=bool)
    occupied = np.zeros_like(observed)
    for grid, selected in [
        (observed, inside & (abs(local[:, 2]) <= plane_distance)),
        (occupied, inside & (local[:, 2] >= min_height)),
    ]:
        chosen = indices[selected]
        grid[chosen[:, 1], chosen[:, 0]] = True
    # Detection footprints also exclude occluded parts of an observed object.
    yy, xx = np.indices(observed.shape)
    xy = np.stack(
        [
            (xx + 0.5) * s.cell_size - dimensions[0] / 2,
            (yy + 0.5) * s.cell_size - dimensions[1] / 2,
        ],
        axis=-1,
    )
    world = center + xy[..., 0, None] * axes[:, 0] + xy[..., 1, None] * axes[:, 1]
    for obj in table['objects']:
        object_axes = Rotation.from_quat(obj['orientation_xyzw']).as_matrix()
        relative = (world - np.asarray(obj['center'])) @ object_axes
        occupied |= np.all(abs(relative[..., :2]) <= np.array(obj['dimensions'][:2]) / 2, axis=-1)
    # Include outside the observed image/rectangle in edge clearance.
    edge_distance = distance_transform_edt(np.pad(observed, 1))[1:-1, 1:-1] * s.cell_size
    obstacle_distance = (
        distance_transform_edt(~occupied) * s.cell_size
        if occupied.any()
        else np.full_like(xy[:, :, 0], np.inf)
    )
    free = (
        observed
        & (edge_distance > s.edge_margin + s.cell_size / np.sqrt(2))
        & (obstacle_distance > s.obstacle_margin + s.cell_size * np.sqrt(2))
    )
    # Cells that straddle the estimated table boundary cannot be free.
    free &= np.all(abs(xy) + s.cell_size / 2 <= dimensions / 2 - s.edge_margin, axis=-1)
    result['area_m2'] = float(free.sum() * s.cell_size**2)
    result['observed_area_m2'] = float(observed.sum() * s.cell_size**2)
    regions = []
    remaining = free.copy()
    for _ in range(s.max_regions):
        bounds = largest_rectangle(remaining)
        if bounds is None:
            break
        y0, y1, x0, x1 = bounds
        size = np.array([x1 - x0, y1 - y0]) * s.cell_size
        remaining[y0:y1, x0:x1] = False
        if min(size) < s.min_side:
            continue
        local_center = np.array([(x0 + x1) / 2, (y0 + y1) / 2]) * s.cell_size - dimensions / 2
        position = center + axes[:, :2] @ local_center
        regions.append(
            {
                'kind': 'free_surface_candidate',
                **pose(position, axes, np.eye(4)),
                'dimensions': [*size.tolist(), 0.0],
                'area_m2': float(np.prod(size)),
                'observed_only': True,
            }
        )
    if regions:
        result['status'] = 'observed_candidates'
    return result, regions


def attach_free_space(tables, scene_points, settings=None, plane_distance=0.008, min_height=0.018):
    for table in tables:
        table['free_space'], table['free_regions'] = free_space(
            table, scene_points, settings, plane_distance, min_height
        )
    return tables
