"""Ground truth scoring, kept separate from the perception/tracking runtime."""

from itertools import pairwise

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation


def orientation_error(measured, reference, square=False):
    estimated = Rotation.from_quat(measured)
    truth = Rotation.from_quat(reference)
    angles = [0.0, np.pi] if not square else [0.0, np.pi / 2, np.pi, 3 * np.pi / 2]
    return float(
        min(
            (estimated.inv() * truth * Rotation.from_euler('z', angle)).magnitude()
            for angle in angles
        )
    )


def score(detections, references, max_distance=0.2):
    """One-to-one gated matching. Misses/false positives remain in the report."""
    costs = np.full((len(references), len(detections)), 1e6)
    for i, reference in enumerate(references):
        for j, item in enumerate(detections):
            distance = np.linalg.norm(np.array(reference['center']) - item['center'])
            if distance <= max_distance:
                costs[i, j] = distance
    pairs = []
    if costs.size:
        rows, cols = linear_sum_assignment(costs)
        pairs = [(i, j) for i, j in zip(rows, cols, strict=True) if costs[i, j] < 1e6]
    records = []
    for i, j in pairs:
        truth, item = references[i], detections[j]
        dimensions = np.array(item['dimensions']) - truth['dimensions']
        square = truth['dimensions'][0] / truth['dimensions'][1] < 1.15
        records.append(
            {
                'reference_id': truth['id'],
                'detection_id': item.get('id', f'observation_{j}'),
                'estimated_center': item['center'],
                'center_error_m': float(costs[i, j]),
                'dimension_error_m': dimensions.tolist(),
                'max_dimension_error_m': float(np.abs(dimensions).max()),
                'orientation_error_rad': orientation_error(
                    item['orientation_xyzw'], truth['orientation_xyzw'], square
                ),
                'orientation_symmetry_deg': 90 if square else 180,
            }
        )
    return {
        'reference_count': len(references),
        'detection_count': len(detections),
        'matches': records,
        'missed': [
            item['id'] for i, item in enumerate(references) if i not in {p[0] for p in pairs}
        ],
        'false_positives': [
            item.get('id', f'observation_{j}')
            for j, item in enumerate(detections)
            if j not in {p[1] for p in pairs}
        ],
    }


def aggregate(reports):
    matches = [match for report in reports for match in report['matches']]
    count = len(matches)
    references = sum(report['reference_count'] for report in reports)
    detections = sum(report['detection_count'] for report in reports)
    result = {
        'references': references,
        'detections': detections,
        'matched': count,
        'recall': count / references if references else None,
        'precision': count / detections if detections else None,
        'missed': sum(len(report['missed']) for report in reports),
        'false_positives': sum(len(report['false_positives']) for report in reports),
    }
    for key in ('center_error_m', 'max_dimension_error_m', 'orientation_error_rad'):
        values = np.array([match[key] for match in matches])
        result[key] = (
            {
                'rmse': float(np.sqrt(np.mean(values**2))),
                'median': float(np.median(values)),
                'max': float(values.max()),
            }
            if len(values)
            else None
        )
    return result


def stability(frames, raw_key='raw_boxes', filtered_key='filtered_boxes'):
    """Stationary jitter and identity changes, including missed frames."""
    names = {match['reference_id'] for frame in frames for match in frame[filtered_key]['matches']}
    result = {}
    for name in sorted(names):
        entry = {}
        for label, key in (('raw', raw_key), ('filtered', filtered_key)):
            measurements = [
                match
                for frame in frames
                for match in frame[key]['matches']
                if match['reference_id'] == name
            ]
            points = np.array([match['estimated_center'] for match in measurements])
            entry[label + '_samples'] = len(points)
            entry[label + '_center_std_m'] = (
                float(np.linalg.norm(np.std(points, axis=0))) if len(points) > 1 else None
            )
            if label == 'filtered':
                ids = [match['detection_id'] for match in measurements]
                entry['ids'] = sorted(set(ids))
                entry['id_switches'] = sum(a != b for a, b in pairwise(ids))
                entry['missed_frames'] = len(frames) - len(measurements)
        result[name] = entry
    return result
