"""Lightweight tests independent of the optional torch/LeRobot installation."""

import json
import struct

import pytest

from experiments.okura.checkpoint import STATE_NAMES, tensor_header
from experiments.okura.scene import make_scene, set_initial_arm_pose


def test_joint_layout_is_explicit_and_unique():
    assert len(STATE_NAMES) == len(set(STATE_NAMES)) == 16
    assert STATE_NAMES[0] == 'left_shoulder_pitch_joint'
    assert STATE_NAMES[7] == 'right_shoulder_pitch_joint'
    assert STATE_NAMES[14:] == ('left_gripper', 'right_gripper')


def test_reject_truncated_safetensors(tmp_path):
    header = json.dumps({'x': {'shape': [1], 'dtype': 'F32', 'data_offsets': [0, 4]}}).encode()
    path = tmp_path / 'bad.safetensors'
    path.write_bytes(struct.pack('<Q', len(header)) + header)
    with pytest.raises(ValueError, match='Truncated'):
        tensor_header(path)


def test_box_is_free_and_never_welded_to_hand():
    import mujoco

    model, _data = make_scene()
    assert model.joint('box_free').type[0] == mujoco.mjtJoint.mjJNT_FREE
    assert model.neq == 1
    assert model.eq_obj1id[0] == model.body('pelvis').id
    assert model.eq_obj2id[0] == 0
    assert model.nu == 29  # No Dex1 installed: this is intentionally just an input fixture.


def test_box_falls_under_gravity_without_attachment():
    import mujoco

    model, data = make_scene()
    height_index = model.joint('box_free').qposadr[0] + 2
    data.qpos[height_index] += 0.15  # Initial condition only.
    initial = data.qpos[height_index]
    for _ in range(75):
        mujoco.mj_step(model, data)
    assert data.qpos[height_index] < initial - 0.05


def test_initial_pose_rejects_nan():
    import numpy as np

    model, data = make_scene()
    with pytest.raises(ValueError, match='finite'):
        set_initial_arm_pose(model, data, np.full(16, np.nan))
