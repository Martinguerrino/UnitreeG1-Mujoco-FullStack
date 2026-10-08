"""View diversity, capacity, expiry and motion prevent stale accumulated geometry."""

from copy import deepcopy

import numpy as np

from g1_backend.perception_tracking import Tracker, transform_item
from g1_backend.perception_views import ViewAccumulator, ViewSettings


def surface(xmin, xmax, offset=0.0):
    x, y = np.meshgrid(np.linspace(xmin, xmax, 25), np.linspace(-0.3, 0.3, 25))
    points = np.column_stack([x.ravel() + offset, y.ravel(), np.full(x.size, 0.75)])
    return {
        'kind': 'table_surface',
        'center': [(xmin + xmax) / 2 + offset, 0.0, 0.75],
        'dimensions': [xmax - xmin, 0.6, 0.0],
        'orientation_xyzw': [0.0, 0.0, 0.0, 1.0],
        'quality': {'warnings': [], 'status': 'observed'},
        '_points': points,
        'objects': [],
    }


def test_views_expand_partial_surface_only_after_camera_moves_and_remain_bounded():
    tracker, views = Tracker(), ViewAccumulator(ViewSettings(max_views=2, max_points=500))
    camera = np.eye(4)
    first = views.update(tracker.update([surface(-0.5, 0.1)], 0), camera, 0)[0]
    assert first['quality']['view_count'] == 1
    repeated = views.update(tracker.update([surface(-0.5, 0.1)], 0.5), camera, 0.5)[0]
    assert repeated['quality']['view_count'] == 1
    camera[0, 3] = 0.1
    second = views.update(tracker.update([surface(-0.1, 0.5)], 1), camera, 1)[0]
    assert second['id'] == first['id']
    assert second['quality']['view_count'] == 2
    assert second['dimensions'][0] > 0.95
    assert second['quality']['accumulated_points'] <= 500
    assert second['quality']['view_baseline_m'] == 0.1
    assert '_points' not in second
    camera[0, 3] = 0.2
    third = views.update(tracker.update([surface(-0.1, 0.5)], 1.5), camera, 1.5)[0]
    assert third['quality']['view_count'] <= 2


def test_points_and_support_plane_transform_with_frame():
    item = surface(-0.1, 0.1)
    item['_support_center'], item['_support_normal'] = [0.0, 0.0, 0.75], [0.0, 0.0, 1.0]
    transform = np.eye(4)
    transform[:3, 3] = [1.0, 2.0, -0.8]
    moved = transform_item(item, transform)
    np.testing.assert_allclose(moved['_points'], item['_points'] + [1, 2, -0.8])
    np.testing.assert_allclose(moved['_support_center'], [1.0, 2.0, -0.05])
    np.testing.assert_allclose(moved['_support_normal'], [0.0, 0.0, 1.0])


def test_old_views_expire_and_clock_reset_clears_memory():
    tracker = Tracker()
    views = ViewAccumulator(ViewSettings(max_age=0.6))
    camera = np.eye(4)
    views.update(tracker.update([surface(-0.5, 0.1)], 2), camera, 2)
    camera[0, 3] = 0.1
    result = views.update(tracker.update([surface(-0.1, 0.5)], 3), camera, 3)[0]
    assert result['quality']['view_count'] == 1
    views.update([], camera, 0)
    assert views.tracks == {}


def test_slow_cumulative_object_motion_resets_geometry():
    views = ViewAccumulator(ViewSettings(motion_reset=0.06))
    camera = np.eye(4)
    item = surface(-0.1, 0.1)
    item.update(
        kind='box_candidate',
        id='box_1',
        raw_pose={'center': [0.0, 0.0, 0.8]},
        _support_center=[0.0, 0.0, 0.7],
        _support_normal=[0.0, 0.0, 1.0],
    )
    wrapper = surface(-0.5, 0.5)
    wrapper.pop('_points')
    wrapper['objects'] = [item]
    views.update([deepcopy(wrapper)], camera, 0)
    for index in range(1, 4):
        moved = deepcopy(wrapper)
        moved['objects'][0]['raw_pose']['center'][0] = index * 0.025
        moved['objects'][0]['_points'][:, 0] += index * 0.025
        camera[0, 3] = index * 0.04
        result = views.update([moved], camera, index * 0.5)
    assert result[0]['objects'][0]['quality']['view_count'] == 1
