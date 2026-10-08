"""Geometry accuracy and rejection checks independent of ROS and simulator state."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from g1_backend.perception import Settings, backproject, detect, transform_points


def scene(yaw=0.4, noisy=False):
    x, y = np.meshgrid(np.linspace(-0.5, 0.5, 100), np.linspace(-0.35, 0.35, 75))
    table = np.column_stack([x.ravel(), y.ravel(), np.full(x.size, 0.75)])
    x, y = np.meshgrid(np.linspace(-0.08, 0.08, 30), np.linspace(-0.04, 0.04, 20))
    top = np.column_stack([x.ravel(), y.ravel(), np.full(x.size, 0.1)])
    box = top @ Rotation.from_euler('z', yaw).as_matrix().T + [0.1, 0.05, 0.75]
    points = np.vstack([table, box])
    if noisy:
        points += np.random.default_rng(42).normal(0, 0.001, points.shape)
    colors = np.vstack(
        [np.tile([100, 80, 50], (len(table), 1)), np.tile([255, 0, 0], (len(box), 1))]
    )
    return points, colors


@pytest.mark.parametrize('noisy', [False, True])
def test_table_box_pose_dimensions_color_and_orientation(noisy):
    points, colors = scene(noisy=noisy)
    tables = detect(points, colors)
    assert len(tables) == 1
    table = tables[0]
    np.testing.assert_allclose(table['center'], [0, 0, 0.75], atol=0.015)
    np.testing.assert_allclose(table['dimensions'], [1, 0.7, 0], atol=0.015)
    assert table['thickness_known'] is False
    assert len(table['objects']) == 1
    box = table['objects'][0]
    np.testing.assert_allclose(box['center'], [0.1, 0.05, 0.8], atol=0.01)
    np.testing.assert_allclose(box['dimensions'], [0.16, 0.08, 0.1], atol=0.01)
    yaw = Rotation.from_quat(box['orientation_xyzw']).as_euler('xyz')[2]
    assert abs((yaw - 0.4 + np.pi / 2) % np.pi - np.pi / 2) < 0.08
    np.testing.assert_allclose(box['mean_rgb'], [255, 0, 0])
    assert not box['yaw_ambiguous']
    assert not box['geometry_complete']


def test_output_frame_transforms_center_and_full_orientation():
    points, colors = scene()
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('xyz', [0.2, -0.1, 0.7]).as_matrix()
    transform[:3, 3] = [0.3, -0.2, -0.79]
    native = detect(points, colors)[0]['objects'][0]
    output = detect(points, colors, output_transform=transform)[0]['objects'][0]
    np.testing.assert_allclose(
        output['center'], transform_points([native['center']], transform)[0]
    )
    expected = transform[:3, :3] @ Rotation.from_quat(native['orientation_xyzw']).as_matrix()
    np.testing.assert_allclose(
        Rotation.from_quat(output['orientation_xyzw']).as_matrix(), expected
    )
    assert output['dimensions'] == native['dimensions']


def test_floor_vertical_wall_invalid_depth_and_no_objects():
    x, y = np.meshgrid(np.linspace(-1, 1, 50), np.linspace(0, 1, 50))
    floor = np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])
    wall = np.column_stack([x.ravel(), np.zeros(x.size), y.ravel()])
    points = np.vstack([floor, wall, [[np.nan, 0, 0.7]]])
    assert detect(points, np.zeros_like(points)) == []
    points, colors = scene()
    table_only = points[:, 2] < 0.8
    tables = detect(points[table_only], colors[table_only])
    assert len(tables) == 1 and tables[0]['objects'] == []
    assert detect(np.empty((0, 3)), np.empty((0, 3))) == []


def test_projection_uses_metric_depth_optical_axes_and_registered_color():
    depth = np.array([[1.0, np.nan], [0.0, 2.0]])
    rgb = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
    points, colors = backproject(depth, rgb, [2, 0, 0.5, 0, 2, 0.5, 0, 0, 1], Settings(stride=1))
    np.testing.assert_allclose(points, [[-0.25, -0.25, 1], [0.5, 0.5, 2]])
    np.testing.assert_array_equal(colors, [rgb[0, 0], rgb[1, 1]])
    with pytest.raises(ValueError):
        backproject(depth, rgb[:1], np.eye(3), Settings())
    with pytest.raises(ValueError):
        backproject(depth, rgb, np.zeros((3, 3)), Settings())


def test_disconnected_coplanar_tables_and_separate_objects():
    points, colors = scene()
    other = points + [1.5, 0, 0]
    tables = detect(np.vstack([points, other]), np.vstack([colors, colors]))
    assert len(tables) == 2
    assert all(len(t['objects']) == 1 for t in tables)


def test_round_top_is_not_reported_as_a_box():
    points, _colors = scene()
    keep = points[:, 2] < 0.8
    x, y = np.meshgrid(np.linspace(-0.07, 0.07, 40), np.linspace(-0.07, 0.07, 40))
    disk = x * x + y * y < 0.07**2
    cylinder = np.column_stack([x[disk] + 0.1, y[disk] + 0.05, np.full(disk.sum(), 0.85)])
    tables = detect(np.vstack([points[keep], cylinder]), np.zeros((keep.sum() + len(cylinder), 3)))
    assert len(tables) == 1 and tables[0]['objects'] == []
