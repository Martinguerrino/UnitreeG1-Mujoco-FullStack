"""Corrected cloud geometry and the native RTAB-Map scan codec."""
from pathlib import Path
import struct
import zlib

import numpy as np
import pytest

from g1_backend.slam_lidar_graph import ScanDecoder, optimized_cloud, voxel_cloud


def test_optimization_moves_previous_scans_without_leaving_duplicate_surfaces():
    points = np.array([[0, 0, 0], [0, 0, 3], [1, 1, 2]], dtype=np.float32)
    first = np.eye(4)
    drifted = first.copy()
    drifted[0, 3] = .3
    before = optimized_cloud({1: points, 2: points}, {1: first, 2: drifted})
    assert len(before) == 6
    after = optimized_cloud({1: points, 2: points}, {1: first, 2: first})
    assert len(after) == 3
    assert after[:, 2].max() == 3  # Full 3-D cloud, beyond the planning height.
    np.testing.assert_array_equal(after[np.lexsort(after.T[::-1])], points[np.lexsort(points.T[::-1])])


def test_cloud_uses_optimized_rotation_and_skips_absent_nodes():
    pose = np.eye(4)
    pose[:2, :2] = [[0, -1], [1, 0]]
    pose[2, 3] = 1
    cloud = optimized_cloud({1: np.array([[1, 0, 2]]), 2: np.array([[100, 100, 100]])}, {1: pose})
    np.testing.assert_allclose(cloud, [[0, 1, 3]])


def test_voxel_filter_bounds_output_and_discards_invalid_points():
    points = np.column_stack([np.arange(100), np.zeros(100), np.zeros(100)])
    points = np.vstack([points, [np.nan, 1, 0], [np.inf, 1, 0]])
    result = voxel_cloud(points, maximum=10)
    assert len(result) == 10 and np.isfinite(result).all()
    assert result[:, 0].min() == 0 and result[:, 0].max() == 99


def test_native_codec_retains_xyz_from_intensity_and_normal_scan_formats():
    library = Path('artifacts/slam/bin/librtabmap_image.so')
    if not library.exists():
        pytest.skip('Build RTAB-Map scan codec first')
    decoder = ScanDecoder(library.resolve())
    for channels, scan_format in ((3, 5), (4, 6), (7, 9)):
        data = np.arange(5 * channels, dtype=np.float32).reshape(5, channels)
        # RTAB-Map compressData: zlib bytes followed by rows, cols, cv::Mat type.
        compressed = zlib.compress(data.tobytes()) + struct.pack('=iii', 1, 5, 5 + ((channels - 1) << 3))
        np.testing.assert_array_equal(decoder.decode(compressed, scan_format), data[:, :3])
    with pytest.raises(ValueError, match='decoding'):
        decoder.decode(compressed, 1)  # Reject 2-D formats instead of misreading intensity as Z.
    with pytest.raises(ValueError, match='decoding'):
        decoder.decode(b'invalid', 5)
    with pytest.raises(ValueError, match='decoding'):
        decoder.decode(zlib.compress(b'') + struct.pack('=iii', 1, 100001, 21), 5)
