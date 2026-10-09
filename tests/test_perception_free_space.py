import numpy as np
from g1_backend.perception_free_space import FreeSpaceSettings, free_space
from g1_backend.perception_tracking import transform_item


def surface():
    x, y = np.meshgrid(np.arange(-0.49, 0.5, 0.01), np.arange(-0.39, 0.4, 0.01))
    points = np.column_stack([x.ravel(), y.ravel(), np.full(x.size, 0.75)])
    table = {
        'center': [0, 0, 0.75],
        'orientation_xyzw': [0, 0, 0, 1],
        'dimensions': [1, 0.8, 0],
        'objects': [],
    }
    return table, points


def test_free_surface_excludes_borders_unknown_and_undetected_obstacles():
    table, points = surface()
    # This obstacle is deliberately not a detected object.
    obstacle = points[(abs(points[:, 0]) < 0.08) & (abs(points[:, 1]) < 0.08)].copy()
    obstacle[:, 2] += 0.1
    points = points[points[:, 0] < 0.2]  # Right side hidden, hence unknown.
    summary, regions = free_space(table, np.vstack([points, obstacle]))
    assert summary['observed_only'] and summary['area_m2'] > 0 and regions
    for region in regions:
        center = np.array(region['center'][:2])
        half = np.array(region['dimensions'][:2]) / 2
        assert np.all(abs(center) + half <= [0.46, 0.36])
        assert center[0] + half[0] < 0.2
        assert np.any(abs(center) - half > 0.1)  # No patch overlaps obstacle.


def test_no_depth_does_not_invent_free_space_and_pose_transforms():
    table, points = surface()
    summary, regions = free_space(table, np.empty((0, 3)))
    assert not regions and summary['area_m2'] == 0
    _, regions = free_space(table, points, FreeSpaceSettings(max_regions=1))
    table['free_regions'] = regions
    transform = np.eye(4)
    transform[:3, 3] = [1, 2, 3]
    moved = transform_item(table, transform)
    np.testing.assert_allclose(
        moved['free_regions'][0]['center'], np.array(regions[0]['center']) + [1, 2, 3]
    )


def test_rotated_detected_object_footprint_excluded_even_when_depth_is_missing():
    table, points = surface()
    table['objects'] = [
        {
            'center': [0, 0, 0.8],
            'orientation_xyzw': [0, 0, 0.382683432, 0.923879533],
            'dimensions': [0.2, 0.1, 0.1],
        }
    ]
    _, regions = free_space(table, points)
    assert regions
    # Every returned rectangular patch is clear of the object center and margin.
    for region in regions:
        half = np.array(region['dimensions'][:2]) / 2
        assert np.any(abs(np.array(region['center'][:2])) - half > 0.04)
