"""Contract tests for the pinned Holosoma FastSAC policy."""

import numpy as np

from g1_backend.config import ROOT
from g1_backend.contracts import RobotState
from g1_backend.policies import load_policy
from g1_backend.policies.holosoma import DEFAULT


def test_holosoma_observation_and_action_contract():
    policy = load_policy({
        'provider': 'holosoma_fastsac',
        'checkpoint': 'artifacts/locomotion_holosoma/fastsac_g1_29dof.onnx',
        'sha256': '8346fd90778439395922a8c7256f24125ae84b8dea949128bac9e23c02bc7717',
    })
    state = RobotState(
        q=DEFAULT.copy(), dq=np.zeros(29), quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
        gyroscope=np.zeros(3), received_at=0.0,
    )
    standing = policy.get_observation(state, np.zeros(3))
    assert standing.shape == (1, 100)
    np.testing.assert_allclose(standing[0, 35:37], [-1.0, -1.0], atol=1e-6)
    turning = policy.get_observation(state, np.array([0.0, 0.0, 0.5]))
    np.testing.assert_allclose(turning[0, 32:35], [0.5, 0.0, 0.0])
    policy.infer(turning)
    command = policy.get_action()
    command.validate()
    np.testing.assert_array_equal(command.q[15:], DEFAULT[15:])
    assert policy.period == 0.005


def test_navigation_returns_to_original_policy_with_turn_assist():
    import yaml

    config = yaml.safe_load((ROOT / 'configs/experiments/navigation.yaml').read_text())
    assert config['locomotion'] == 'configs/locomotion.yaml'
    assert config['navigation']['turn_assist']['enabled'] is True
