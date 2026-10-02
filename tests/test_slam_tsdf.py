"""TSDF surface fusion and graph correction, using known RGB-D geometry."""
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip('open3d')
from g1_backend.slam_tsdf import ImageDecoder, RGBDFrame, TSDFReconstructor, same_view


class ArrayDecoder:
    def decode(self, image, depth=False):
        return image.copy()


def plane_frame():
    color = np.full((48, 64, 3), [255, 0, 0], dtype=np.uint8)
    depth = np.full((48, 64), 2, dtype=np.float32)
    return RGBDFrame(color, depth, (64, 48, 60., 60., 31.5, 23.5), np.eye(4))


def test_plane_fusion_is_metric_colored_and_does_not_reintegrate():
    reconstruction = TSDFReconstructor(ArrayDecoder())
    frame = plane_frame()
    mesh, count, rebuilt = reconstruction.update({1: frame, 2: frame}, {1: np.eye(4), 2: np.eye(4)})
    assert count == 2 and rebuilt
    assert len(mesh.triangles) > 100
    vertices = np.asarray(mesh.vertices)
    assert np.max(np.abs(vertices[:, 2] - 2)) < .01
    np.testing.assert_allclose(np.asarray(mesh.vertex_colors), np.tile([1, 0, 0], (len(vertices), 1)))
    _, count, rebuilt = reconstruction.update({1: frame, 2: frame}, {1: np.eye(4), 2: np.eye(4)})
    assert count == 0 and not rebuilt


def test_loop_correction_replaces_old_surface_and_eviction_rebuilds():
    reconstruction = TSDFReconstructor(ArrayDecoder())
    frame = plane_frame()
    reconstruction.update({1: frame, 2: frame}, {1: np.eye(4), 2: np.eye(4)})
    corrected = np.eye(4)
    corrected[2, 3] = 1
    mesh, count, rebuilt = reconstruction.update({1: frame, 2: frame}, {1: corrected, 2: corrected})
    assert count == 2 and rebuilt
    assert np.max(np.abs(np.asarray(mesh.vertices)[:, 2] - 3)) < .01
    _, count, rebuilt = reconstruction.update({2: frame}, {2: corrected})
    assert count == 1 and rebuilt


def test_invalid_depth_is_excluded():
    reconstruction = TSDFReconstructor(ArrayDecoder())
    frame = plane_frame()
    frame.depth[:] = np.nan
    mesh, _, _ = reconstruction.update({1: frame}, {1: np.eye(4)})
    assert len(mesh.triangles) == 0


def test_optical_camera_local_transform_is_composed_with_map_pose():
    reconstruction = TSDFReconstructor(ArrayDecoder())
    frame = plane_frame()
    # Optical forward becomes map +X, with the camera one metre above the origin.
    frame.local_transform[:3, :3] = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
    frame.local_transform[2, 3] = 1
    base_pose = np.eye(4)
    base_pose[0, 3] = .5
    mesh, _, _ = reconstruction.update({1: frame}, {1: base_pose})
    vertices = np.asarray(mesh.vertices)
    assert np.max(np.abs(vertices[:, 0] - 2.5)) < .01
    assert abs(np.mean(vertices[:, 2]) - 1) < .05


def test_stationary_view_gate_checks_rotation_and_translation():
    pose = np.eye(4)
    assert same_view(pose, pose)
    moved = pose.copy()
    moved[0, 3] = .1
    assert not same_view(pose, moved)
    rotated = pose.copy()
    rotated[:2, :2] = [[0, -1], [1, 0]]
    assert not same_view(pose, rotated)


def test_native_codec_decodes_rgb_and_millimeter_depth():
    import cv2
    library = Path('artifacts/slam/bin/librtabmap_image.so')
    if not library.exists():
        pytest.skip('RTAB-Map codec has not been built')
    decoder = ImageDecoder(library.resolve())
    bgr = np.full((10, 12, 3), [10, 20, 30], dtype=np.uint8)
    rgb = decoder.decode(cv2.imencode('.png', bgr)[1].tobytes())
    np.testing.assert_array_equal(rgb, bgr[:, :, ::-1])
    depth = np.full((10, 12), 2345, dtype=np.uint16)
    decoded = decoder.decode(cv2.imencode('.png', depth)[1].tobytes(), depth=True)
    np.testing.assert_allclose(decoded, 2.345)
    with pytest.raises(ValueError, match='decoding'):
        decoder.decode(b'invalid')
