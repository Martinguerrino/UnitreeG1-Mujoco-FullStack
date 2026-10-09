"""Camera mounting rotation and exact reset, independent of robot motion."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from g1_backend.camera_control import camera_pitch, tilted_quaternion


def test_pitch_limits_and_invalid_commands():
    assert camera_pitch(100) == 45
    assert camera_pitch(-100) == -30
    for value in [float('nan'), float('inf'), 'invalid']:
        with pytest.raises(ValueError):
            camera_pitch(value)


def test_pitch_and_reset_preserve_original_mount_and_optical_axes():
    original = np.array([.5, .5, -.5, -.5])
    np.testing.assert_array_equal(tilted_quaternion(original, 0), original)
    mounted = Rotation.from_quat(original[[1, 2, 3, 0]])
    tilted = tilted_quaternion(original, 25)
    actual = Rotation.from_quat(tilted[[1, 2, 3, 0]])
    np.testing.assert_allclose(actual.as_matrix(),
                               (mounted * Rotation.from_euler('x', -25, degrees=True)).as_matrix())
    np.testing.assert_allclose(np.linalg.norm(tilted), 1)
