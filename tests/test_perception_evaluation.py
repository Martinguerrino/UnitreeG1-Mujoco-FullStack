"""Evaluator counts missed/extra observations and honors orientation symmetry."""

import numpy as np
from scipy.spatial.transform import Rotation

from g1_backend.perception_evaluation import aggregate, score


def item(name, x=0.0, yaw=0.0):
    return {
        'id': name,
        'center': [x, 0.0, 0.8],
        'dimensions': [0.16, 0.08, 0.1],
        'orientation_xyzw': Rotation.from_euler('z', yaw).as_quat().tolist(),
    }


def test_scoring_is_one_to_one_and_keeps_failures():
    report = score([item('a', 0.01), item('extra', 0.04)], [item('truth'), item('missed', 1.0)])
    assert len(report['matches']) == 1
    assert report['matches'][0]['center_error_m'] == 0.01
    assert report['missed'] == ['missed'] and report['false_positives'] == ['extra']
    totals = aggregate([report])
    assert totals['recall'] == 0.5 and totals['precision'] == 0.5
    assert totals['center_error_m']['rmse'] == 0.01


def test_orientation_symmetry_and_empty_results():
    report = score([item('a', yaw=np.pi)], [item('truth')])
    assert report['matches'][0]['orientation_error_rad'] < 1e-10
    square, observed = item('truth'), item('a', yaw=np.pi / 2)
    square['dimensions'] = observed['dimensions'] = [0.1, 0.1, 0.1]
    report = score([observed], [square])
    assert report['matches'][0]['orientation_error_rad'] < 1e-10
    assert report['matches'][0]['orientation_symmetry_deg'] == 90
    totals = aggregate([score([], [item('truth')])])
    assert totals['recall'] == 0 and totals['center_error_m'] is None


def test_stability_records_identity_switches_and_missing_frames():
    from g1_backend.perception_evaluation import stability

    frames = []
    for index in range(3):
        raw = score([item('raw', index * 0.01)], [item('truth')])
        filtered = score([item('box_1' if index < 2 else 'box_2', index * 0.005)], [item('truth')])
        frames.append({'raw_boxes': raw, 'filtered_boxes': filtered})
    frames.append(
        {'raw_boxes': score([], [item('truth')]), 'filtered_boxes': score([], [item('truth')])}
    )
    result = stability(frames)['truth']
    assert result['id_switches'] == 1 and result['missed_frames'] == 1
    assert result['filtered_center_std_m'] < result['raw_center_std_m']
