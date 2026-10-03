import numpy as np
import pytest

from g1_backend.feet_estimation import FeetEstimator, FeetFusion


@pytest.fixture
def estimator():
    import mujoco
    from pathlib import Path
    import tempfile
    from g1_backend.config import load_config
    from g1_backend.simulator import model_xml

    source, scene = model_xml(load_config('configs/experiments/navigation.yaml'))
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder)
        (path / 'g1_29dof.xml').write_text(source)
        (path / 'scene.xml').write_text(scene)
        model = mujoco.MjModel.from_xml_path(str(path / 'scene.xml'))
    return FeetEstimator(model)


def posture():
    q = np.zeros(29)
    q[[0, 6]] = -.1
    q[[3, 9]] = .3
    q[[4, 10]] = -.2
    return q


def supported_torque(e):
    q = posture()
    e.update(0., q, q, q, [1, 0, 0, 0], [0, 0, 0], [0, 0, 9.81])
    torque = e.data.qfrc_bias[e.vadr].copy()
    for side, body in enumerate(e.bodies):
        point = e.data.xpos[body] + e.data.xmat[body].reshape(3, 3) @ [.025, 0, -.03]
        jac = np.zeros((3, e.model.nv))
        e.mj.mj_jac(e.model, e.data, jac, None, point, body)
        leg = slice(side * 6, side * 6 + 6)
        torque[leg] -= jac[:, e.vadr[leg]].T @ [0, 0, 250]
    return torque


def test_double_support_and_stationary_imu(estimator):
    torque = supported_torque(estimator)
    for i in range(1, 51):
        sample = estimator.update(i * .02, posture(), np.zeros(29), torque,
                                  [1, 0, 0, 0], [0, 0, 0], [0, 0, 9.81])
    assert sample['valid']
    assert all(sample['support'])
    assert min(sample['loads']) > 150
    np.testing.assert_allclose(sample['velocity_body'], 0, atol=1e-6)


def test_unloaded_legs_do_not_claim_support(estimator):
    torque = supported_torque(estimator)
    torque[:] = estimator.data.qfrc_bias[estimator.vadr]
    for i in range(1, 30):
        sample = estimator.update(i * .02, posture(), np.zeros(29), torque,
                                  [1, 0, 0, 0], [0, 0, 0], [0, 0, 9.81])
    assert not sample['valid']
    assert not any(sample['support'])


def test_kinematic_velocity_uses_joint_motion(estimator):
    torque = supported_torque(estimator)
    dq = np.zeros(29)
    dq[[0, 6]] = .1
    for i in range(1, 51):
        sample = estimator.update(i * .02, posture(), dq, torque,
                                  [1, 0, 0, 0], [0, 0, 0], [0, 0, 9.81])
    assert sample['valid']
    assert abs(sample['velocity_body'][0]) > .03


def test_incompatible_support_velocities_reduce_confidence(estimator):
    torque = supported_torque(estimator)
    dq = np.zeros(29)
    dq[0], dq[6] = -.8, .8
    for i in range(1, 51):
        sample = estimator.update(i * .02, posture(), dq, torque,
                                  [1, 0, 0, 0], [0, 0, 0], [0, 0, 9.81])
    assert not sample['valid']


def test_no_support_does_not_accumulate_inertial_drift(estimator):
    q = posture()
    for i in range(100):
        sample = estimator.update(i * .02, q, np.zeros(29), np.zeros(29),
                                  [1, 0, 0, 0], [0, 0, 0], [3, 0, 9.81])
    assert not sample['valid']
    np.testing.assert_array_equal(sample['velocity_body'], [0, 0, 0])


def feed(fusion, valid=True):
    for t in np.arange(0, .201, .02):
        fusion.add({'time': float(t), 'velocity_body': [1., 0, 0],
                    'variance': [.02] * 3, 'valid': valid})


def test_fusion_contributes_motion_and_keeps_native_rotation():
    f = FeetFusion()
    f.update(0, np.eye(4), [.1, .1])
    feed(f)
    native = np.eye(4)
    native[2, 3] = .8
    result = f.update(.2, native, [.1, .1])
    assert .1 < result[0, 3] < .2
    assert f.used == 1
    np.testing.assert_array_equal(result[:3, :3], native[:3, :3])
    assert result[2, 3] == .8


@pytest.mark.parametrize('supported', [True, False])
def test_stale_or_unsupported_falls_back_to_native(supported):
    f = FeetFusion()
    f.update(0, np.eye(4), [.1, .1])
    if not supported:
        feed(f, False)
    native = np.eye(4)
    native[0, 3] = .05
    np.testing.assert_array_equal(f.update(.2, native, [.1, .1]), native)
    assert f.used == 0


def test_sensor_gap_resets_support_and_nonfinite_rejected(estimator):
    torque = supported_torque(estimator)
    for i in range(1, 30):
        estimator.update(i * .02, posture(), np.zeros(29), torque,
                         [1, 0, 0, 0], [0, 0, 0], [0, 0, 9.81])
    sample = estimator.update(2., np.zeros(29), np.zeros(29), np.zeros(29),
                              [1, 0, 0, 0], [0, 0, 0], [0, 0, 9.81])
    assert not sample['valid']
    with pytest.raises(ValueError):
        estimator.update(2.02, np.full(29, np.nan), np.zeros(29), torque,
                         [1, 0, 0, 0], [0, 0, 0], [0, 0, 9.81])
