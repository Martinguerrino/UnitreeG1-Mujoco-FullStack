"""Association, smoothing, symmetry and fixed-frame motion regression checks."""

from copy import deepcopy

import numpy as np
from scipy.spatial.transform import Rotation

from g1_backend.perception_tracking import Tracker, TrackingSettings, transform_item


def box(center=(0.1, 0.05, 0.8), yaw=0.4, color=(255, 0, 0)):
    return {
        'kind': 'box_candidate',
        'center': list(center),
        'dimensions': [0.16, 0.08, 0.1],
        'mean_rgb': list(color),
        'orientation_xyzw': Rotation.from_euler('z', yaw).as_quat().tolist(),
        'yaw_ambiguous': False,
    }


def table(objects):
    return {
        'kind': 'table_surface',
        'center': [0.0, 0.0, 0.75],
        'dimensions': [1.0, 0.7, 0.0],
        'orientation_xyzw': [0.0, 0.0, 0.0, 1.0],
        'objects': objects,
    }


def test_ids_survive_order_changes_and_confirm_only_after_two_hits():
    tracker = Tracker()
    first = tracker.update([table([box(), box((0.4, 0.05, 0.8), color=(0, 255, 0))])], 1)[0]
    assert not first['confirmed'] and all(not obj['confirmed'] for obj in first['objects'])
    ids = [obj['id'] for obj in first['objects']]
    second = tracker.update([table([box((0.4, 0.05, 0.8), color=(0, 255, 0)), box()])], 1.5)[0]
    assert second['id'] == first['id']
    assert [obj['id'] for obj in second['objects']] == ids[::-1]
    assert all(obj['confirmed'] and obj['hits'] == 2 for obj in second['objects'])


def test_filter_reduces_stationary_jitter_and_preserves_raw_observation():
    tracker = Tracker(TrackingSettings(alpha=0.25))
    raw, filtered = [], []
    rng = np.random.default_rng(1)
    for i in range(80):
        position = np.array([0.1, 0.05, 0.8]) + rng.normal(0, 0.012, 3)
        observation = box(position)
        output = tracker.update([table([observation])], i * 0.1)[0]['objects'][0]
        np.testing.assert_allclose(output['raw_pose']['center'], position)
        raw.append(position)
        filtered.append(output['center'])
    assert np.std(filtered[10:], axis=0).mean() < np.std(raw[10:], axis=0).mean() * 0.65


def test_rectangle_symmetry_does_not_spin_a_stationary_box():
    tracker = Tracker()
    a = tracker.update([table([box(yaw=0.4)])], 0)[0]['objects'][0]
    b = tracker.update([table([box(yaw=0.4 + np.pi)])], 0.5)[0]['objects'][0]
    rotation = Rotation.from_quat(a['orientation_xyzw']).inv() * Rotation.from_quat(
        b['orientation_xyzw']
    )
    assert rotation.magnitude() < 1e-10
    square = box(yaw=0.4 + np.pi / 2)
    square['dimensions'] = [0.08, 0.08, 0.1]
    square['yaw_ambiguous'] = True
    c = tracker.update([table([square])], 1)[0]['objects'][0]
    rotation = Rotation.from_quat(a['orientation_xyzw']).inv() * Rotation.from_quat(
        c['orientation_xyzw']
    )
    assert rotation.magnitude() < 1e-10


def test_missing_tracks_are_hidden_reassociate_then_expire_and_clock_reset():
    tracker = Tracker(TrackingSettings(timeout=1.0))
    initial = tracker.update([table([box()])], 10)[0]['objects'][0]
    missing = tracker.update([table([])], 10.5)
    assert missing[0]['objects'] == []
    reappeared = tracker.update([table([box()])], 10.8)[0]['objects'][0]
    assert reappeared['id'] == initial['id']
    tracker.update([], 12)
    expired = tracker.update([table([box()])], 12.1)[0]['objects'][0]
    assert expired['id'] != initial['id']
    reset = tracker.update([table([box()])], 1)[0]['objects'][0]
    assert reset['id'] != expired['id'] and reset['hits'] == 1


def test_association_gate_rejects_far_away_or_wrong_sized_objects():
    tracker = Tracker()
    a = tracker.update([table([box()])], 0)[0]['objects'][0]
    b = tracker.update([table([box((0.6, 0.05, 0.8))])], 0.5)[0]['objects'][0]
    assert a['id'] != b['id']
    observation = box()
    observation['dimensions'] = [0.35, 0.3, 0.1]
    c = tracker.update([table([observation])], 1)[0]['objects'][0]
    assert c['id'] not in (a['id'], b['id'])


def test_robot_motion_is_removed_before_matching_and_output_tracks_current_robot_frame():
    tracker = Tracker()
    world = table([box()])
    before = deepcopy(world)
    a = tracker.update([world], 0)[0]
    world_to_robot = np.eye(4)
    world_to_robot[:3, :3] = Rotation.from_euler('z', -0.7).as_matrix()
    world_to_robot[:3, 3] = [-0.4, -0.1, -0.8]
    observed_in_robot = transform_item(world, world_to_robot)
    recovered_world = transform_item(observed_in_robot, np.linalg.inv(world_to_robot))
    b = tracker.update([recovered_world], 0.5)[0]
    assert b['objects'][0]['id'] == a['objects'][0]['id']
    output = transform_item(b, world_to_robot)
    np.testing.assert_allclose(
        output['objects'][0]['center'], observed_in_robot['objects'][0]['center']
    )
    np.testing.assert_allclose(
        output['objects'][0]['raw_pose']['center'], observed_in_robot['objects'][0]['center']
    )
    assert world == before


def test_table_identity_tolerates_partial_footprint_but_not_another_height():
    tracker = Tracker()
    initial = tracker.update([table([box()])], 0)[0]
    partial = table([box()])
    partial['center'][0] += 0.3
    partial['dimensions'][0] = 0.5
    result = tracker.update([partial], 0.5)[0]
    assert result['id'] == initial['id']
    assert result['objects'][0]['id'] == initial['objects'][0]['id']
    above = table([])
    above['center'][2] += 0.3
    result = tracker.update([above], 1)[0]
    assert result['id'] != initial['id']


def test_stability_requires_history_and_ignores_duplicate_capture():
    tracker = Tracker()
    for stamp in [0, 0, 0.5]:
        obj = tracker.update([table([box()])], stamp)[0]['objects'][0]
        assert not obj['stable']
    obj = tracker.update([table([box(yaw=0.4 + np.pi)])], 1)[0]['objects'][0]
    assert obj['stable'] and obj['stability']['samples'] == 3
    assert obj['stability']['orientation_span_deg'] < 1e-8


def test_raw_motion_is_unstable_even_with_heavy_smoothing_and_recovers():
    tracker = Tracker(TrackingSettings(alpha=0.01))
    for stamp in [0, 0.5, 1]:
        obj = tracker.update([table([box()])], stamp)[0]['objects'][0]
    assert obj['stable']
    moved = box((0.16, 0.05, 0.8))
    obj = tracker.update([table([moved])], 1.5)[0]['objects'][0]
    assert not obj['stable']
    assert 'position_variation' in obj['stability']['reasons']
    assert abs(obj['center'][0] - 0.1) < 0.001
    for stamp in [2, 2.5, 3, 3.5]:
        obj = tracker.update([table([moved])], stamp)[0]['objects'][0]
    assert obj['stable']


def test_stability_resets_after_gap_and_clock_reset():
    tracker = Tracker()
    for stamp in [0, 0.5, 1]:
        obj = tracker.update([table([box()])], stamp)[0]['objects'][0]
    identity = obj['id']
    obj = tracker.update([table([box()])], 2)[0]['objects'][0]
    assert obj['id'] == identity and not obj['stable']
    assert obj['stability']['samples'] == 1
    obj = tracker.update([table([box()])], 0)[0]['objects'][0]
    assert not obj['stable'] and obj['id'] != identity


def test_dimension_and_orientation_variation_reject_stability():
    for change, reason in [
        ('dimensions', 'dimension_variation'),
        ('angle', 'orientation_variation'),
    ]:
        tracker = Tracker()
        for stamp in [0, 0.5]:
            tracker.update([table([box()])], stamp)
        observation = box(yaw=0.8 if change == 'angle' else 0.4)
        if change == 'dimensions':
            observation['dimensions'][0] += 0.04
        obj = tracker.update([table([observation])], 1)[0]['objects'][0]
        assert not obj['stable'] and reason in obj['stability']['reasons']


def test_object_id_survives_support_table_id_change_without_crossing_support_planes():
    tracker = Tracker()
    first = table([box()])
    first['objects'][0].update(_support_center=[0, 0, 0.75], _support_normal=[0, 0, 1])
    initial = tracker.update([first], 0)[0]['objects'][0]
    # Different visible extent creates a new table ID, but object stays in place.
    replacement = deepcopy(first)
    replacement['center'][0] = 2.0
    replacement['dimensions'] = [0.3, 0.3, 0]
    result = tracker.update([replacement], 0.5)[0]['objects'][0]
    assert result['id'] == initial['id']
    another = deepcopy(replacement)
    another['objects'][0]['_support_center'][2] = 1.0
    result = tracker.update([another], 1)[0]['objects'][0]
    assert result['id'] != initial['id']


def test_global_assignment_never_reuses_object_id_in_two_tables():
    tracker = Tracker()
    item = table([box()])
    item['objects'][0].update(_support_center=[0, 0, 0.75], _support_normal=[0, 0, 1])
    tracker.update([item], 0)
    other = deepcopy(item)
    other['center'][0] = 0.4
    tables = tracker.update([item, other], 0.5)
    ids = [obj['id'] for support in tables for obj in support['objects']]
    assert len(ids) == len(set(ids))
