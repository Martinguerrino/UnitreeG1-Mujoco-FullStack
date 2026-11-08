"""Refine YOLO instance masks with registered depth and support-plane geometry."""

from dataclasses import dataclass

import cv2
import numpy as np
from scipy.spatial import ConvexHull, QhullError
from scipy.spatial.transform import Rotation

from .perception import Settings, backproject, clusters, detect, pose, rectangle, transform_points
from .perception_tracking import transform_item
from .cuda.neighbors import NeighborIndex
from .perception_yolo import Instance


@dataclass
class RefinementSettings:
    geometric_support: bool = True
    min_mask_pixels: int = 12
    min_points: int = 8
    erosion_pixels: int = 1
    min_valid_fraction: float = 0.35
    neighbor_radius: float = 0.025
    support_margin: float = 0.025
    min_supported_fraction: float = 0.25
    min_component_fraction: float = 0.6
    box_shape_validation: bool = True
    box_max_aspect_ratio: float = 6.0
    duplicate_point_fraction: float = 0.6
    partial_box_confidence: float = 0.55
    partial_box_rectangularity: float = 0.45
    partial_box_min_aspect: float = 1.3

    def validate(self):
        if min(self.min_mask_pixels, self.min_points) < 3 or self.erosion_pixels < 0:
            raise ValueError('Invalid mask support or erosion settings')
        if (
            not 0 < self.min_valid_fraction <= 1
            or min(self.neighbor_radius, self.support_margin) <= 0
        ):
            raise ValueError('Invalid RGB-D refinement thresholds')
        fractions = [
            self.min_supported_fraction,
            self.min_component_fraction,
            self.duplicate_point_fraction,
            self.partial_box_confidence,
            self.partial_box_rectangularity,
        ]
        if (
            not np.isfinite([*fractions, self.box_max_aspect_ratio]).all()
            or any(not 0 < v <= 1 for v in fractions)
            or self.box_max_aspect_ratio < 1
            or not np.isfinite(self.partial_box_min_aspect)
            or self.partial_box_min_aspect < 1
        ):
            raise ValueError('Invalid component, shape or duplicate thresholds')


def mask_points(depth, rgb, k, mask, settings, refinement, transform=None):
    if mask.shape != depth.shape or mask.dtype != bool:
        raise ValueError('Instance mask must match registered depth resolution')
    pixels = int(mask.sum())
    if pixels < refinement.min_mask_pixels:
        return np.empty((0, 3)), np.empty((0, 3)), 0.0
    valid = (
        mask & np.isfinite(depth) & (depth >= settings.min_depth) & (depth <= settings.max_depth)
    )
    fraction = float(valid.sum() / pixels)
    interior = mask
    if refinement.erosion_pixels and pixels > 100:
        width = refinement.erosion_pixels * 2 + 1
        interior = cv2.erode(mask.astype('uint8'), np.ones((width, width), 'uint8')).astype(bool)
    dense = Settings(**{**vars(settings), 'stride': 1})
    from .cuda.workspace import active
    from .cuda.geometry import attempt

    workspace = active()
    accelerated = None
    if workspace is not None:
        accelerated = attempt(
            'mask_backproject',
            depth.size,
            lambda cp: workspace.backproject(
                depth, rgb, np.asarray(k).reshape(3, 3), dense, interior & valid, transform
            ),
            minimum=0,
        )
    if accelerated is None:
        selected = np.where(interior & valid, depth, np.nan)
        points, colors = backproject(selected, rgb, k, dense)
        if transform is not None:
            points = transform_points(points, transform)
    else:
        points, colors = accelerated
    return points, colors, fraction


def quality(instance, points, valid_fraction, inlier_fraction, plane_rmse, image_shape):
    height, width = image_shape
    x0, y0, x1, y1 = instance.box_xyxy
    truncated = x0 <= 1 or y0 <= 1 or x1 >= width - 1 or y1 >= height - 1
    warnings = ['partial_geometry']
    if truncated:
        warnings.append('mask_truncated')
    if valid_fraction < 0.6:
        warnings.append('low_depth_support')
    if len(points) < 30:
        warnings.append('few_3d_points')
    return {
        'segmentation_confidence': instance.confidence,
        'depth_support_scope': 'mask',
        'mask_pixels': int(instance.mask.sum()),
        'valid_depth_fraction': valid_fraction,
        'inlier_fraction': inlier_fraction,
        'plane_rmse_m': float(plane_rmse),
        'status': 'limited' if len(warnings) > 1 else 'observed',
        'warnings': warnings,
    }


def detect_instances(
    depth,
    rgb,
    k,
    instances,
    camera_to_processing,
    processing_to_output=None,
    settings=None,
    refinement=None,
):
    from .cuda.workspace import frame_scope

    with frame_scope(depth, rgb):
        return _detect_instances(
            depth,
            rgb,
            k,
            instances,
            camera_to_processing,
            processing_to_output,
            settings,
            refinement,
        )


def _detect_instances(
    depth,
    rgb,
    k,
    instances,
    camera_to_processing,
    processing_to_output=None,
    settings=None,
    refinement=None,
):
    """Geometry is estimated only from predicted masks; model labels stay explicit."""
    s, r = settings or Settings(), refinement or RefinementSettings()
    s.validate()
    r.validate()
    transform = np.eye(4) if processing_to_output is None else processing_to_output
    tables = []
    for instance_index, instance in enumerate(instances):
        if instance.label != 'table':
            continue
        optical, colors, fraction = mask_points(
            depth, rgb, k, instance.mask, s, r, camera_to_processing
        )
        if fraction < r.min_valid_fraction or len(optical) < s.min_table_points:
            continue
        points = optical
        # Keep the existing constrained plane finder, but prevent unmasked
        # points or heuristic box clusters from becoming semantic detections.
        surfaces = detect(points, colors, s, detect_objects=False)
        for table in surfaces:
            axes = Rotation.from_quat(table['orientation_xyzw']).as_matrix()
            distance = (points - table['center']) @ axes[:, 2]
            inliers = points[np.abs(distance) <= s.plane_distance]
            if len(inliers) < s.min_table_points:
                continue
            rmse = np.sqrt(np.mean(((inliers - table['center']) @ axes[:, 2]) ** 2))
            table['objects'] = []
            table.update(
                instance_index=instance_index,
                semantic_class='table',
                source='yolo_rgbd',
                geometry_complete=False,
                quality=quality(
                    instance, inliers, fraction, len(inliers) / len(points), rmse, depth.shape
                ),
                _points=inliers,
            )
            tables.append(table)
    if r.geometric_support:
        optical, colors = backproject(depth, rgb, k, s)
        points = transform_points(optical, camera_to_processing)
        # A clipped tabletop may not be recognized visually. Keep explicitly
        # geometric support planes so a genuine YOLO object mask can still be
        # placed in 3D. Never turn these planes into a learned table label.
        for table in detect(points, colors, s, detect_objects=False):
            axes = Rotation.from_quat(table['orientation_xyzw']).as_matrix()
            normal = axes[:, 2]
            duplicate = any(
                abs((np.array(table['center']) - other['center']) @ normal) < s.plane_distance * 2
                and normal @ Rotation.from_quat(other['orientation_xyzw']).as_matrix()[:, 2] > 0.95
                and np.linalg.norm(np.array(table['center']) - other['center'])
                < 0.5
                * (
                    np.linalg.norm(table['dimensions'][:2])
                    + np.linalg.norm(other['dimensions'][:2])
                )
                for other in tables
            )
            if duplicate:
                continue
            distance = (points - table['center']) @ normal
            local = (points - table['center']) @ axes
            inlier_mask = (abs(distance) <= s.plane_distance) & np.all(
                abs(local[:, :2]) <= np.array(table['dimensions'][:2]) / 2, axis=1
            )
            inliers = points[inlier_mask]
            if len(inliers) < s.min_table_points:
                continue
            table.update(
                objects=[],
                semantic_class='support_surface',
                source='geometry_support',
                geometry_complete=False,
                _points=inliers,
                quality={
                    'segmentation_confidence': None,
                    'depth_support_scope': 'image',
                    'mask_pixels': 0,
                    'valid_depth_fraction': float(np.isfinite(depth).mean()),
                    'inlier_fraction': float(len(inliers) / max(1, len(points))),
                    'plane_rmse_m': float(np.sqrt(np.mean(distance[inlier_mask] ** 2))),
                    'status': 'limited',
                    'warnings': ['partial_geometry', 'semantic_table_unconfirmed'],
                },
            )
            tables.append(table)
    # A tray lying on a larger tabletop is not another support table. Sort
    # physically plausible surfaces first and reject wholly nested raised ones.
    surfaces = []
    for table in sorted(tables, key=lambda item: np.prod(item['dimensions'][:2]), reverse=True):
        nested = False
        candidate_axes = Rotation.from_quat(table['orientation_xyzw']).as_matrix()
        for parent in surfaces:
            parent_axes = Rotation.from_quat(parent['orientation_xyzw']).as_matrix()
            displacement = np.array(table['center']) - parent['center']
            elevation = displacement @ parent_axes[:, 2]
            if not s.plane_distance * 2 < elevation < s.max_object_height:
                continue
            if np.prod(parent['dimensions'][:2]) < 2 * np.prod(table['dimensions'][:2]):
                continue
            corners = np.array([[a, b, 0.0] for a in (-0.5, 0.5) for b in (-0.5, 0.5)])
            corners *= np.array(table['dimensions'])
            corners = (corners @ candidate_axes.T + displacement) @ parent_axes
            if np.all(
                abs(corners[:, :2]) <= np.array(parent['dimensions'][:2]) / 2 + r.support_margin
            ):
                nested = True
                break
        if not nested:
            surfaces.append(table)
    tables = surfaces
    for instance_index, instance in enumerate(instances):
        if instance.label not in ('box', 'cylinder', 'sphere'):
            continue
        optical, colors, fraction = mask_points(
            depth, rgb, k, instance.mask, s, r, camera_to_processing
        )
        if fraction < r.min_valid_fraction or len(optical) < r.min_points:
            continue
        points = optical
        # Radius support rejects isolated bad depth returns before component selection.
        neighbors = NeighborIndex(points).query_ball_point(
            points, r.neighbor_radius, return_length=True
        )
        points, colors = points[neighbors >= 3], colors[neighbors >= 3]
        if len(points) < r.min_points:
            continue
        choices = []
        for table in tables:
            axes = Rotation.from_quat(table['orientation_xyzw']).as_matrix()
            local = (points - table['center']) @ axes
            inside = np.all(
                np.abs(local[:, :2]) <= np.array(table['dimensions'][:2]) / 2 + r.support_margin,
                axis=1,
            )
            supported = (
                inside
                & (local[:, 2] >= s.min_object_height)
                & (local[:, 2] <= s.max_object_height)
            )
            if supported.sum() >= r.min_points:
                choices.append((int(supported.sum()), table, axes, supported))
        if not choices:
            continue
        _, table, axes, supported = max(choices, key=lambda item: item[0])
        if supported.sum() / len(optical) < r.min_supported_fraction:
            continue
        points, colors = points[supported], colors[supported]
        components = list(clusters(points, r.neighbor_radius))
        component = max(components, key=len)
        observed, colors = points[component], colors[component]
        if len(observed) < r.min_points or len(observed) / len(points) < r.min_component_fraction:
            continue
        local = (observed - table['center']) @ axes
        try:
            xy, size, yaw = rectangle(local[:, :2])
        except QhullError:
            continue
        height = float(np.quantile(local[:, 2], 0.98))
        if min(size) < 0.005 or max(size) > s.max_object_side:
            continue
        rectangularity = None
        partial_shape = False
        if instance.label == 'box' and r.box_shape_validation:
            if len(observed) < s.min_object_points:
                continue
            top = local[abs(local[:, 2] - height) <= s.plane_distance]
            if len(top) >= 8 and len(top) / len(local) >= s.min_top_fraction:
                try:
                    _, top_size, _ = rectangle(top[:, :2])
                    rectangularity = float(ConvexHull(top[:, :2]).volume / np.prod(top_size))
                except QhullError:
                    pass
            full_shape = (
                rectangularity is not None
                and rectangularity >= s.min_top_rectangularity
                and size[0] / size[1] <= r.box_max_aspect_ratio
            )
            if not full_shape:
                x0, y0, x1, y1 = instance.box_xyxy
                truncated = (
                    x0 <= 1 or y0 <= 1 or x1 >= depth.shape[1] - 1 or y1 >= depth.shape[0] - 1
                )
                partial_shape = (
                    instance.confidence >= r.partial_box_confidence
                    and (truncated or size[0] / size[1] >= r.partial_box_min_aspect)
                    and rectangularity is not None
                    and rectangularity >= r.partial_box_rectangularity
                    and supported.sum() / len(optical) >= 0.5
                )
                if not partial_shape:
                    continue
        center = np.asarray(table['center']) + axes @ np.r_[xy, height / 2]
        rotation = axes @ Rotation.from_euler('z', yaw).as_matrix()
        obj = {
            'instance_index': instance_index,
            'kind': 'box_candidate' if instance.label == 'box' else instance.label,
            **pose(center, rotation, np.eye(4)),
            'dimensions': [*size.tolist(), height],
            'semantic_class': instance.label,
            'source': 'yolo_rgbd',
            'mean_rgb': colors.mean(axis=0).tolist(),
            'point_count': len(observed),
            'geometry_complete': False,
            'yaw_ambiguous': bool(instance.label != 'box' or size[0] / size[1] < 1.15),
            'quality': quality(
                instance,
                observed,
                fraction,
                len(observed) / len(optical),
                table['quality']['plane_rmse_m'],
                depth.shape,
            ),
            '_points': observed,
            '_support_center': table['center'],
            '_support_normal': axes[:, 2].tolist(),
        }
        if partial_shape:
            obj['quality']['warnings'].append('box_shape_incomplete')
            obj['quality']['status'] = 'limited'
        obj['quality']['box_geometry'] = (
            'partial'
            if partial_shape
            else ('rectangular_top' if rectangularity is not None else 'not_checked')
        )
        if rectangularity is not None:
            obj['quality']['top_rectangularity'] = rectangularity
        obj['quality']['supported_fraction'] = float(supported.sum() / len(optical))
        obj['quality']['component_fraction'] = float(len(observed) / len(points))
        if obj['yaw_ambiguous']:
            obj['quality']['warnings'].append('orientation_ambiguous')
        table['objects'].append(obj)
    # Depth-space duplicate suppression also handles conflicting visual categories.
    for table in tables:
        accepted = []
        for obj in sorted(
            table['objects'],
            key=lambda item: item['quality']['segmentation_confidence'],
            reverse=True,
        ):
            duplicate = False
            for previous in accepted:
                if np.linalg.norm(np.array(obj['center']) - previous['center']) > 0.04:
                    continue
                if np.linalg.norm(np.array(obj['mean_rgb']) - previous['mean_rgb']) > 80:
                    continue
                small, large = sorted([obj['_points'], previous['_points']], key=len)
                distance, _ = NeighborIndex(large).query(small)
                if np.mean(distance <= s.plane_distance * 2) >= r.duplicate_point_fraction:
                    duplicate = True
                    break
            if not duplicate:
                accepted.append(obj)
        table['objects'] = accepted
    return [transform_item(table, transform) for table in tables]


def validated_instances(depth, rgb, k, instances, tables, camera_to_output, settings=None):
    """Restrict visualization to measured 3D inliers of accepted YOLO instances.

    Run before tracking/multi-view removes the private observation clouds.
    Unconfirmed geometric support surfaces have no YOLO mask to visualize.
    """
    s = settings or Settings()
    dense = Settings(**{**vars(s), 'stride': 1})
    clouds = {}
    for table in tables:
        for item in [table, *table['objects']]:
            if 'instance_index' in item and len(item.get('_points', [])):
                clouds.setdefault(item['instance_index'], []).append(item['_points'])
    accepted = []
    for index, instance in enumerate(instances):
        if index not in clouds:
            continue
        valid = (
            instance.mask & np.isfinite(depth) & (depth >= s.min_depth) & (depth <= s.max_depth)
        )
        pixels = np.flatnonzero(valid)
        optical, _ = backproject(np.where(valid, depth, np.nan), rgb, k, dense)
        points = transform_points(optical, camera_to_output)
        distance, _ = NeighborIndex(np.vstack(clouds[index])).query(points)
        mask = np.zeros(depth.shape, dtype=bool)
        mask.flat[pixels[distance <= s.plane_distance]] = True
        if not mask.any():
            continue
        v, u = np.where(mask)
        bounds = [int(u.min()), int(v.min()), int(u.max() + 1), int(v.max() + 1)]
        accepted.append(Instance(instance.label, instance.confidence, mask, bounds))
    return accepted
