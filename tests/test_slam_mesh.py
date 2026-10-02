"""Mesh geometry, colors and bounds; native checks require the PCL helper."""
from pathlib import Path
import subprocess

import numpy as np
import pytest

from g1_backend.slam_mapping import write_pcd
from g1_backend.slam_mesh import read_mesh

ROOT = Path(__file__).resolve().parents[1]


def test_empty_mesh(tmp_path):
    path = tmp_path / 'empty.ply'
    path.write_text('ply\nformat ascii 1.0\nelement vertex 0\nelement face 0\nend_header\n')
    xyz, colors, faces = read_mesh(path)
    assert xyz.shape == colors.shape == faces.shape == (0, 3)


def test_rejects_invalid_face_indices(tmp_path):
    path = tmp_path / 'invalid.ply'
    path.write_text('ply\nformat ascii 1.0\nelement vertex 1\nelement face 1\nend_header\n'
                    '0 0 0 255 0 0\n3 0 1 2\n')
    with pytest.raises(ValueError, match='indices'):
        read_mesh(path)


def test_native_mesh_preserves_color_and_does_not_bridge_surfaces(tmp_path):
    binary = ROOT / 'artifacts/slam/bin/g1_cloud_mesh'
    if not binary.exists():
        pytest.skip('Build the native mesh helper with scripts/build_navigation_mesh.sh')
    x, y = np.meshgrid(np.arange(12) * .08, np.arange(12) * .08)
    floor = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    ceiling = floor + [0, 0, 1]
    xyz = np.vstack((floor, ceiling, [np.nan, 0, 0]))
    source, output = tmp_path / 'cloud.pcd', tmp_path / 'mesh.ply'
    write_pcd(source, xyz, np.full(len(xyz), 0x00ff0000, dtype=np.uint32))
    subprocess.run([str(binary), str(source), str(output)], check=True, capture_output=True)
    vertices, colors, faces = read_mesh(output)
    assert len(faces) > 20
    assert np.isfinite(vertices).all()
    np.testing.assert_array_equal(colors, np.tile([255, 0, 0], (len(colors), 1)))
    triangles = vertices[faces]
    assert np.all(np.ptp(triangles[:, :, 2], axis=1) < 1e-6)
    for edge in ((0, 1), (0, 2), (1, 2)):
        assert np.linalg.norm(triangles[:, edge[0]] - triangles[:, edge[1]], axis=1).max() <= .25001
