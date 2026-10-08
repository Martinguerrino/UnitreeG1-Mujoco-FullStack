"""Temporal association and filtering in a fixed TF frame, without ground truth."""

from copy import deepcopy
from dataclasses import dataclass

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation, Slerp

from .perception import transform_points


def transform_item(item, transform):
    result = deepcopy(item)
    result['center'] = transform_points([item['center']], transform)[0].tolist()
    rotation = transform[:3, :3] @ Rotation.from_quat(item['orientation_xyzw']).as_matrix()
    result['orientation_xyzw'] = Rotation.from_matrix(rotation).as_quat().tolist()
    if 'raw_pose' in item:
        result['raw_pose'] = transform_item(item['raw_pose'], transform)
    if 'objects' in item:
        result['objects'] = [transform_item(obj, transform) for obj in item['objects']]
    return result


def equivalent_orientation(reference, measured, ambiguous=False):
    """Choose the closest equivalent rectangle orientation before interpolation."""
    previous = Rotation.from_quat(reference)
    current = Rotation.from_quat(measured)
    angles = [0.0, np.pi] if not ambiguous else [0.0, np.pi / 2, np.pi, 3 * np.pi / 2]
    candidates = [current * Rotation.from_euler('z', angle) for angle in angles]
    return min(candidates, key=lambda candidate: (previous.inv() * candidate).magnitude())


@dataclass
class TrackingSettings:
    match_distance: float = 0.18
    dimension_gate: float = 0.08
    timeout: float = 2.0
    alpha: float = 0.35
    confirmation_hits: int = 2

    def validate(self):
        if not np.isfinite(
            [self.match_distance, self.dimension_gate, self.timeout, self.alpha]
        ).all():
            raise ValueError('Tracking settings must be finite')
        if min(self.match_distance, self.dimension_gate, self.timeout) <= 0:
            raise ValueError('Tracking gates and timeout must be positive')
        if not 0 < self.alpha <= 1 or self.confirmation_hits < 1:
            raise ValueError('Tracking alpha must be in (0, 1] and confirmation_hits positive')


class Tracker:
    """One-to-one matching with bounded missed-observation retention.

    Only objects observed in the current capture are returned. Missing tracks
    remain internal for reassociation; they never become invented detections.
    IDs are session-local, monotonic and are not recycled after clock resets.
    """

    def __init__(self, settings=None):
        self.settings = settings or TrackingSettings()
        self.settings.validate()
        self.tracks = {}
        self.next_id = 1
        self.last_time = None

    def expire(self, stamp):
        if self.last_time is not None and stamp < self.last_time:
            self.tracks.clear()
        self.tracks = {
            key: track
            for key, track in self.tracks.items()
            if stamp - track['last_seen'] <= self.settings.timeout
        }
        self.last_time = stamp

    def _group(self, observations, stamp, kind, parent=None):
        candidates = [
            track
            for track in self.tracks.values()
            if track['item']['kind'] == kind and track['parent'] == parent
        ]
        matches = {}
        if candidates and observations:
            costs = np.full((len(candidates), len(observations)), 1e6)
            for i, track in enumerate(candidates):
                previous = track['item']
                for j, measured in enumerate(observations):
                    distance = np.linalg.norm(np.array(previous['center']) - measured['center'])
                    size_delta = np.max(
                        np.abs(np.array(previous['dimensions']) - measured['dimensions'])
                    )
                    # Visible tabletop extents change strongly with viewpoint.
                    gate = self.settings.dimension_gate if kind == 'box_candidate' else np.inf
                    distance_gate = self.settings.match_distance
                    compatible_plane = True
                    if kind == 'table_surface':
                        normal = Rotation.from_quat(previous['orientation_xyzw']).as_matrix()[:, 2]
                        current_normal = Rotation.from_quat(
                            measured['orientation_xyzw']
                        ).as_matrix()[:, 2]
                        displacement = np.array(measured['center']) - previous['center']
                        compatible_plane = (
                            abs(displacement @ normal) < 0.04 and normal @ current_normal > 0.95
                        )
                        distance_gate += 0.25 * (
                            np.linalg.norm(previous['dimensions'][:2])
                            + np.linalg.norm(measured['dimensions'][:2])
                        )
                    if compatible_plane and distance <= distance_gate and size_delta <= gate:
                        color = 0.0
                        if 'mean_rgb' in measured and 'mean_rgb' in previous:
                            color = (
                                np.linalg.norm(
                                    np.array(previous['mean_rgb']) - measured['mean_rgb']
                                )
                                / 255
                            )
                        costs[i, j] = distance + 0.1 * min(size_delta, 1.0) + 0.02 * color
            rows, columns = linear_sum_assignment(costs)
            matches = {
                j: candidates[i] for i, j in zip(rows, columns, strict=True) if costs[i, j] < 1e6
            }
        results = []
        for j, measured in enumerate(observations):
            item = deepcopy(measured)
            item.pop('objects', None)
            raw_pose = {
                key: deepcopy(item[key]) for key in ('center', 'orientation_xyzw', 'dimensions')
            }
            track = matches.get(j)
            if track is None:
                track = {
                    'id': self.next_id,
                    'item': item,
                    'parent': parent,
                    'first_seen': stamp,
                    'last_seen': stamp,
                    'hits': 0,
                }
                self.tracks[self.next_id] = track
                self.next_id += 1
            else:
                alpha = self.settings.alpha
                previous = track['item']
                for key in ('center', 'dimensions'):
                    item[key] = (
                        (1 - alpha) * np.array(previous[key]) + alpha * np.array(item[key])
                    ).tolist()
                ambiguous = item.get('yaw_ambiguous', False)
                if kind == 'table_surface':
                    ambiguous = item['dimensions'][0] / item['dimensions'][1] < 1.15
                closest = equivalent_orientation(
                    previous['orientation_xyzw'], item['orientation_xyzw'], ambiguous
                )
                rotations = Rotation.from_quat([previous['orientation_xyzw'], closest.as_quat()])
                item['orientation_xyzw'] = Slerp([0, 1], rotations)(alpha).as_quat().tolist()
            track['hits'] += 1
            track['last_seen'] = stamp
            track['item'] = deepcopy(item)
            item.update(
                id=f'{"table" if kind == "table_surface" else "box"}_{track["id"]}',
                track_id=track['id'],
                hits=track['hits'],
                confirmed=track['hits'] >= self.settings.confirmation_hits,
                first_seen=track['first_seen'],
                last_seen=stamp,
                age_sec=0.0,
                observed=True,
                raw_pose=raw_pose,
            )
            results.append(item)
        return results

    def update(self, tables, stamp):
        if not np.isfinite(stamp):
            raise ValueError('Capture time must be finite')
        self.expire(stamp)
        tracked = self._group(tables, stamp, 'table_surface')
        for table, measured in zip(tracked, tables, strict=True):
            table['objects'] = self._group(
                measured['objects'], stamp, 'box_candidate', table['id']
            )
        return tracked
