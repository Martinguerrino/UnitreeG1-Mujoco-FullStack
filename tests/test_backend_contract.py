"""Regression tests for deployment ordering, frames, lifecycle and modularity."""

import hashlib
import importlib.util

import mujoco
import numpy as np
import pytest

from g1_backend.config import load_config, ROOT
from g1_backend.contracts import JointCommand, RobotState
from g1_backend.policies import load_policy, PolicyUnavailable
from g1_backend.policies.locomotion import projected_gravity
from g1_backend.simulator import joint_limits, model_xml
from g1_backend.skills import GoTo, TaskExecutor


def test_maxwell_facility_scene_is_compatible_with_local_robot(tmp_path):
    config = load_config(ROOT / 'configs/experiments/locomotion_maxwell_facility.yaml')
    scene_path = ROOT / config['robot']['scene']
    assert hashlib.sha256(scene_path.read_bytes()).hexdigest() == (
        '8f747d46c5dafb42187dbfcc60f1008c82db8e5ebf4c409ab02a58e21b65a8d3'
    )
    model_source, scene_source = model_xml(config)
    (tmp_path / 'g1_29dof.xml').write_text(model_source)
    (tmp_path / 'scene.xml').write_text(scene_source)
    model = mujoco.MjModel.from_xml_path(str(tmp_path / 'scene.xml'))
    assert 'name="wall_north"' in scene_source
    assert 'name="red_cube"' in scene_source
    assert 'grasp_left_red_cube' not in scene_source
    assert model.nu == 29
    assert model.body('red_cube').id > 0
    assert model.equality('pelvis_fixture').id >= 0


def test_dual_video_cameras_are_attached_to_robot():
    config = load_config(ROOT / 'configs/experiments/locomotion_maxwell_video.yaml')
    model_source, _ = model_xml(config)
    assert 'name="head_camera"' in model_source
    assert 'name="observer_camera"' in model_source


def test_simulation_config_rejects_external_interface():
    config = load_config(ROOT / 'configs/experiments/locomotion.yaml')
    config['robot']['interface'] = 'eth0'
    with pytest.raises(ValueError, match='loopback'):
        load_config(config)


def test_missing_manipulation_is_explicit():
    with pytest.raises(PolicyUnavailable):
        load_policy({'provider': None})


def test_custom_provider_is_independent_of_sdk(tmp_path, monkeypatch):
    (tmp_path / 'test_provider.py').write_text(
        'def create(config):\n    return config["checkpoint"]\n'
    )
    monkeypatch.syspath_prepend(str(tmp_path))
    assert load_policy({'provider': 'test_provider:create', 'checkpoint': 'fixture'}) == 'fixture'


def test_joint_command_rejects_bad_dimensions_and_nan():
    for q in (np.zeros(28), np.full(29, np.nan)):
        with pytest.raises(ValueError):
            JointCommand(q, np.ones(29), np.ones(29)).validate()


def test_gravity_and_goto_use_body_frame():
    np.testing.assert_allclose(projected_gravity([1, 0, 0, 0]), [0, 0, -1])
    np.testing.assert_allclose(
        projected_gravity([np.sqrt(0.5), 0, np.sqrt(0.5), 0]), [1, 0, 0], atol=1e-6
    )
    telemetry = {'position': [0, 0, 0.8], 'quaternion': [np.sqrt(0.5), 0, 0, np.sqrt(0.5)]}
    command = GoTo([1, 0, np.pi / 2]).velocity(telemetry)
    np.testing.assert_allclose(command, [0, -0.3, 0], atol=1e-6)


def test_task_order_and_invalid_skills():
    executor = TaskExecutor([{'skill': 'goto'}, {'skill': 'pick'}, {'skill': 'place'}])
    for name in ('goto', 'pick', 'place'):
        assert executor.current['skill'] == name
        executor.complete_skill()
    assert executor.current is None
    with pytest.raises(ValueError):
        TaskExecutor([{'skill': 'train'}])


def test_model_sensor_layout_and_camera(tmp_path):
    config = load_config(ROOT / 'configs/experiments/locomotion.yaml')
    config['camera']['enabled'] = True
    model_source, scene_source = model_xml(config)
    (tmp_path / 'g1_29dof.xml').write_text(model_source)
    (tmp_path / 'scene.xml').write_text(scene_source)
    model = mujoco.MjModel.from_xml_path(str(tmp_path / 'scene.xml'))
    assert model.nu == 29
    assert model.ncam == 1
    assert model.opt.timestep == 0.002
    assert model.sensor('imu_quat').adr[0] == 87
    assert model.sensor('frame_pos').adr[0] == 97
    assert joint_limits(config).shape == (29, 2)
    # This order is consumed positionally by the official DDS bridge.
    for i in range(29):
        joint = model.actuator_trnid[i, 0]
        assert model.sensor_objid[i] == joint
        assert model.sensor_objid[i + 29] == joint
        assert model.sensor_objid[i + 58] == joint


def test_pinned_policy_history_mapping_and_inference():
    if not (ROOT / 'artifacts/locomotion/policy.onnx').exists():
        pytest.skip('Run scripts/setup_dds.py for optional real checkpoint tests')
    config = load_config(ROOT / 'configs/experiments/locomotion.yaml')
    from g1_backend.config import runtime_path

    runtime_path()
    if importlib.util.find_spec('onnxruntime') is None:
        pytest.skip('ONNX Runtime is optional')
    policy = load_policy(config['locomotion'])
    initial = policy.get_action()
    np.testing.assert_allclose(initial.q[policy.mapping], policy.default)
    state = RobotState(initial.q, np.zeros(29), np.array([1, 0, 0, 0]), np.array([1, 2, 3]), 0)
    obs = policy.get_observation(state, np.zeros(3))
    assert obs.shape == (1, 480)
    np.testing.assert_allclose(obs[0, :15], np.tile([0.2, 0.4, 0.6], 5))
    np.testing.assert_allclose(obs[0, 15:30], np.tile([0, 0, -1], 5))
    moved = RobotState(initial.q + 0.1, np.ones(29), state.quaternion, np.zeros(3), 1)
    obs2 = policy.get_observation(moved, np.zeros(3))
    # Group-major history, oldest first: previous gyro is retained four times.
    np.testing.assert_allclose(obs2[0, :12], np.tile([0.2, 0.4, 0.6], 4))
    np.testing.assert_allclose(obs2[0, 12:15], 0)
    np.testing.assert_allclose(obs2[0, 45 + 4 * 29 : 45 + 5 * 29], 0.1, atol=1e-6)
    policy.infer(obs)
    action = policy.get_action()
    action.validate()
    policy.reset()
    policy.infer(policy.get_observation(state, np.zeros(3)))
    np.testing.assert_allclose(policy.get_action().q, action.q)
