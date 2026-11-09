import json
import math
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from g1_backend.camera_angle import mount, render_quaternion, optical_quaternion, validate_pitch
from g1_backend.config import load_config
from g1_backend.navigation import NavigationTransport


def matrix(wxyz):
    w, x, y, z = wxyz
    return Rotation.from_quat([x, y, z, w]).as_matrix()


def test_original_urdf_is_exact_default_and_existing_calibration_matches():
    position, rpy = mount()
    view = load_config('configs/experiments/navigation.yaml')['camera']['views'][0]
    np.testing.assert_allclose(position, view['position'])
    np.testing.assert_allclose(render_quaternion(rpy[1], rpy), view['quaternion'])
    assert math.degrees(rpy[1]) == pytest.approx(47.6)


@pytest.mark.parametrize('pitch', [-45., 0., 25., 47.6, 90.])
def test_renderer_and_optical_tf_describe_same_rays_at_all_angles(pitch):
    _, rpy = mount()
    nominal = Rotation.from_euler('xyz', rpy).as_matrix()
    rendered = matrix(render_quaternion(math.radians(pitch), rpy))
    optical = nominal @ matrix(optical_quaternion(math.radians(pitch), rpy))
    np.testing.assert_allclose(optical, rendered @ np.diag([1, -1, -1]), atol=1e-12)


@pytest.mark.parametrize('pitch', [float('nan'), float('inf'), -46., 91.])
def test_invalid_pitch_rejected(pitch):
    with pytest.raises(ValueError):
        validate_pitch(pitch)


def test_camera_command_never_refreshes_or_overwrites_velocity():
    transport = NavigationTransport.__new__(NavigationTransport)
    transport.socket = Mock()
    transport.socket.recvfrom.side_effect = [
        (json.dumps({'camera_pitch_rad': .2}).encode(), None), BlockingIOError()]
    transport.camera_angle_control = SimpleNamespace(value=0.)
    transport.command = np.array([.3, 0, 0])
    transport.received = 0.  # Existing velocity is stale.
    result = transport.velocity()
    assert transport.camera_angle_control.value == .2
    assert transport.received == 0.
    np.testing.assert_array_equal(transport.command, [.3, 0, 0])
    np.testing.assert_array_equal(result, [0, 0, 0])
