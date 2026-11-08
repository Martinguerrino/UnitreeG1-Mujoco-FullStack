"""Metric CUDA fusion and transactional CPU recovery using actual Open3D."""

import numpy as np
import pytest

from g1_backend import acceleration

o3d = pytest.importorskip('open3d')
from g1_backend.slam_tsdf import RGBDFrame, TSDFReconstructor
from g1_backend.cuda import tsdf


class Decoder:
    def decode(self, value, depth=False):
        return value.copy()


def frame():
    return RGBDFrame(
        np.full((48, 64, 3), [255, 0, 0], dtype=np.uint8),
        np.full((48, 64), 2, dtype=np.float32),
        (64, 48, 60.0, 60.0, 31.5, 23.5),
        np.eye(4),
    )


def test_tsdf_gpu_failure_rebuilds_all_frames_on_cpu(monkeypatch):
    monkeypatch.setenv('G1_GPU', 'auto')
    monkeypatch.setattr(o3d.core.cuda, 'is_available', lambda: True)

    class BrokenVolume:
        def __init__(self, *args):
            pass

        def integrate_arrays(self, *args):
            raise RuntimeError('CUDA allocation failed')

    monkeypatch.setattr(tsdf, 'CudaTSDFVolume', BrokenVolume)
    reconstruction = TSDFReconstructor(Decoder())
    rgbd = frame()
    mesh, count, rebuild = reconstruction.update({1: rgbd, 2: rgbd}, {1: np.eye(4), 2: np.eye(4)})
    assert reconstruction.backend == 'cpu' and reconstruction.gpu_failed
    assert count == 2 and rebuild and len(mesh.triangles) > 100
    np.testing.assert_allclose(np.asarray(mesh.vertices)[:, 2], 2, atol=0.01)
    assert set(reconstruction.poses) == {1, 2}
    _, count, rebuild = reconstruction.update({1: rgbd, 2: rgbd}, {1: np.eye(4), 2: np.eye(4)})
    assert count == 0 and not rebuild
    acceleration._DISABLED.discard('tsdf')


def test_real_cuda_tsdf_metric_color_correction_eviction_and_disable(monkeypatch):
    if not o3d.core.cuda.is_available():
        pytest.skip('Open3D CUDA device unavailable')
    monkeypatch.setenv('G1_GPU', 'auto')
    reconstruction = TSDFReconstructor(Decoder())
    rgbd = frame()
    mesh, count, rebuild = reconstruction.update({1: rgbd, 2: rgbd}, {1: np.eye(4), 2: np.eye(4)})
    assert reconstruction.backend == 'cuda'
    assert count == 2 and rebuild and len(mesh.triangles) > 100
    np.testing.assert_allclose(np.asarray(mesh.vertices)[:, 2], 2, atol=0.01)
    np.testing.assert_allclose(
        np.asarray(mesh.vertex_colors), np.tile([1, 0, 0], (len(mesh.vertices), 1)), atol=0.01
    )
    corrected = np.eye(4)
    corrected[2, 3] = 1
    mesh, count, rebuild = reconstruction.update({2: rgbd}, {2: corrected})
    assert reconstruction.backend == 'cuda' and count == 1 and rebuild
    np.testing.assert_allclose(np.asarray(mesh.vertices)[:, 2], 3, atol=0.01)
    monkeypatch.setenv('G1_GPU', 'off')
    mesh, count, rebuild = reconstruction.update({2: rgbd}, {2: corrected})
    assert reconstruction.backend == 'cpu' and count == 1 and rebuild
    np.testing.assert_allclose(np.asarray(mesh.vertices)[:, 2], 3, atol=0.01)
