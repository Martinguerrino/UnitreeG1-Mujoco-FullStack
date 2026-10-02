"""Regression tests for SLAM mount transforms, bounded maps and RGB export."""
import numpy as np
from scipy.spatial.transform import Rotation
from g1_backend.slam_mapping import (
    OccupancyProjection, VoxelMap, navigation_pose, pose_matrix, write_pcd,
)


def test_navigation_pose_compensates_articulated_waist():
    pelvis = pose_matrix([1.2, -0.7, 0.82], Rotation.from_euler('xyz', [.1, -.05, .8]).as_quat())
    mount = pose_matrix([-.04, .002, .17], Rotation.from_euler('xyz', [.03, 0, .3]).as_quat())
    offset = np.array([-.04, 0, .96])
    raw = pelvis @ mount
    raw[:3, 3] -= offset
    base, local_pelvis = navigation_pose(raw, mount, offset)
    np.testing.assert_allclose(base @ local_pelvis, pelvis, atol=1e-12)
    assert base[2, 3] == 0
    np.testing.assert_allclose(base[:2, 3], pelvis[:2, 3])


def test_voxel_map_deduplicates_filters_nan_and_bounds_memory():
    mapping = VoxelMap(resolution=.1, maximum=2)
    mapping.add([[0, 0, 0], [.01, .01, .01], [1, 1, 1], [np.nan, 0, 0]])
    assert len(mapping.points()) == 2
    mapping.add([[2, 2, 2]])
    assert len(mapping.points()) == 2


def test_pcd_keeps_rgb_bits_and_filters_invalid_xyz(tmp_path):
    path = tmp_path / 'map.pcd'
    colors = np.array([0x00123456, 0x00abcdef, 0x00ff0000], dtype='<u4')
    assert write_pcd(path, [[1, 2, 3], [np.nan, 0, 0], [4, 5, 6]], colors.view('<f4')) == 2
    header, data = path.read_bytes().split(b'DATA binary\n')
    assert b'FIELDS x y z rgb' in header
    records = np.frombuffer(data, dtype=[('xyz', '<f4', 3), ('rgb', '<u4')])
    np.testing.assert_array_equal(records['rgb'], colors[[0, 2]])
    assert not path.with_suffix('.pcd.tmp').exists()


def test_occupancy_projection_excludes_floor_and_traces_free_cells():
    mapping = OccupancyProjection(resolution=.1)
    mapping.add([[1, 0, .8], [0, 1, 0], [0, 0, 2]], np.array([0., 0., .9]))
    assert mapping.cells[(10, 0)] == 100
    assert mapping.cells[(5, 0)] == 0
    assert (0, 10) not in mapping.cells
    grid, _origin = mapping.grid()
    assert grid.dtype == np.int8
    assert np.count_nonzero(grid == 100) == 1
