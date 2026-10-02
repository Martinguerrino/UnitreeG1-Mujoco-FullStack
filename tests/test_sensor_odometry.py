import numpy as np
from scipy.spatial.transform import Rotation

from g1_backend.sensor_odometry import (
    CloudReference,
    PoseKalman,
    matrix_pose,
    pose_matrix,
    transform_points,
)


def room_cloud(seed=4):
    rng = np.random.default_rng(seed)
    walls = []
    for axis, value in ((0, -3), (0, 3), (1, -3), (1, 3), (2, -1)):
        points = rng.uniform([-3, -3, -1], [3, 3, 2], (600, 3))
        points[:, axis] = value
        walls.append(points)
    return np.concatenate(walls)


def test_lidar_recovers_rotation_height_and_translation_without_imu():
    previous = room_cloud()
    movement = pose_matrix([0.08, -0.035, 0.025, 0.015, -0.025, 0.06])
    current = transform_points(previous, np.linalg.inv(movement))
    result = CloudReference(previous).register(current)
    assert result is not None
    np.testing.assert_allclose(matrix_pose(result.transform), matrix_pose(movement), atol=0.004)
    assert np.linalg.eigvalsh(result.covariance)[0] > 0


def test_lidar_rejects_missing_returns_and_degenerate_plane():
    previous = room_cloud()
    reference = CloudReference(previous)
    assert reference.register(previous + 30) is None
    assert reference.register(previous[:20]) is None
    assert CloudReference(previous[:600]).register(previous[:600]) is None


def test_kalman_weights_sensor_quality_and_rejects_large_jump():
    filter = PoseKalman()
    filter.predict(1)
    pose = filter.state[:6].copy()
    precise = pose.copy()
    precise[0] = 0.1
    uncertain = pose.copy()
    uncertain[0] = 0.3
    assert filter.update(precise, np.eye(6) * 0.001)
    assert filter.update(uncertain, np.eye(6) * 0.1)
    assert 0.09 < filter.state[0] < 0.12
    assert not filter.update(pose + 100, np.eye(6) * 0.001)
    assert np.linalg.eigvalsh(filter.covariance)[0] > 0


def test_kalman_wraps_yaw_and_does_not_count_prediction_as_sensor_update():
    filter = PoseKalman()
    filter.state[5] = np.pi - 0.01
    filter.predict(1)
    pose = filter.state[:6].copy()
    pose[5] = -np.pi + 0.01
    assert filter.update(pose, np.eye(6) * 0.001)
    assert abs(abs(filter.state[5]) - np.pi) < 0.02
    filter.predict(2)
    assert filter.last_measurement == 1
    assert not filter.predict(1.5)
    assert filter.timestamp == 2


def test_rgbd_depth_produces_metric_points_and_color():
    from g1_backend.rgbd_odometry import depth_cloud

    rgb = np.full((8, 8, 3), 128, dtype=np.uint8)
    depth = np.full((8, 8), 2.0, dtype=np.float32)
    intrinsic = np.array([[4.0, 0, 4], [0, 4, 4], [0, 0, 1]])
    points, colors = depth_cloud(rgb, depth, intrinsic)
    np.testing.assert_allclose(points[0], [-2, -2, 2])
    np.testing.assert_allclose(colors, 128 / 255)


def test_rgbd_pnp_recovers_metric_motion_with_outliers(monkeypatch):
    import cv2
    from g1_backend.rgbd_odometry import RgbdReference

    rng = np.random.default_rng(10)
    intrinsic = np.array([[250.0, 0, 160], [0, 250, 120], [0, 0, 1]])
    objects = rng.uniform([-1, -0.7, 2], [1, 0.7, 4], (100, 3))
    forward = pose_matrix([0.08, -0.03, 0.015, 0.015, -0.02, 0.035])

    def project(points):
        return points[:, :2] / points[:, 2:] * [250, 250] + [160, 120]

    old_pixels = project(objects)
    current_objects = transform_points(objects, forward)
    pixels = project(current_objects)
    pixels[-20:] += [12, -8]  # Deliberate feature mismatches rejected by RANSAC.
    reference = RgbdReference.__new__(RgbdReference)
    reference.intrinsic = intrinsic
    reference.depth = np.full((240, 320), np.nan, dtype=np.float32)
    depth = reference.depth.copy()
    reference.keypoints = [cv2.KeyPoint(float(u), float(v), 3) for u, v in old_pixels]
    for (u, v), z in zip(old_pixels, objects[:, 2]):
        reference.depth[round(v), round(u)] = z
    for (u, v), z in zip(pixels, current_objects[:, 2]):
        depth[round(v), round(u)] = z
    reference.descriptors = np.zeros((100, 32), dtype=np.uint8)
    reference.orb = type(
        'Orb',
        (),
        {
            'detectAndCompute': lambda self, *args: (
                [cv2.KeyPoint(float(u), float(v), 3) for u, v in pixels],
                reference.descriptors,
            )
        },
    )()

    class Matcher:
        def knnMatch(self, *args, **kwargs):
            return [[cv2.DMatch(i, i, 1), cv2.DMatch(i, (i + 1) % 100, 10)] for i in range(100)]

    monkeypatch.setattr(cv2, 'BFMatcher', lambda *args: Matcher())
    result = reference.features(np.zeros((240, 320, 3), np.uint8), depth, intrinsic)
    assert result is not None
    np.testing.assert_allclose(result.transform, np.linalg.inv(forward), atol=0.008)
    assert result.method == 'rgbd_features'


def test_pose_roundtrip():
    pose = [0.1, -0.2, 0.9, 0.05, -0.1, 0.4]
    np.testing.assert_allclose(matrix_pose(pose_matrix(pose)), pose)
    np.testing.assert_allclose(
        Rotation.from_matrix(pose_matrix(pose)[:3, :3]).as_euler('xyz'), pose[3:]
    )


def test_camera_plane_marks_unobservable_motion_with_large_uncertainty():
    plane = room_cloud()[:600]
    result = CloudReference(plane).register(plane, allow_partial=True)
    assert result is not None
    assert np.linalg.eigvalsh(result.covariance)[-1] > 100


def test_kalman_sensor_loss_increases_uncertainty_without_freshening_odometry():
    filter = PoseKalman()
    filter.predict(1)
    assert filter.update(filter.state[:6].copy(), np.eye(6)*0.001)
    before = np.trace(filter.covariance[:6, :6])
    filter.predict(1.5)
    assert np.trace(filter.covariance[:6, :6]) > before
    assert filter.last_measurement == 1
    assert not filter.update(filter.state[:6], -np.eye(6))
