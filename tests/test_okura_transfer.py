"""Physical handoff guards and Okura scene contract."""

import numpy as np
import pytest

from g1_backend.skills.transfer import TransferTask
from experiments.okura.scene import ROOT, make_scene
from experiments.okura.transfer import load_transfer_config


def sample(task, now, **overrides):
    values = {'arms': np.arange(14) / 10, 'bilateral': True, 'lift': 0.04, 'hand_contact': True}
    values.update(overrides)
    task.update(now, **values)


def test_contact_without_lift_does_not_start_carry():
    task = TransferTask(pick_timeout=2.0)
    for now in (0.0, 0.5, 1.0, 1.5):
        sample(task, now, lift=0.001)
        assert task.act_enabled and task.arm_hold is None
    sample(task, 2.0, lift=0.001)
    assert task.phase == 'failed' and task.failure == 'pick_timeout'


def test_handoff_requires_sustained_contact_and_latches_pose():
    task = TransferTask()
    sample(task, 0.0)
    sample(task, 0.2, bilateral=False)
    sample(task, 0.3)
    sample(task, 0.5)
    assert task.act_enabled
    sample(task, 0.61)
    assert task.phase == 'hold' and not task.act_enabled
    assert task.gripper_override == 0
    np.testing.assert_allclose(task.arm_hold, np.arange(14) / 10)
    sample(task, 0.62)
    sample(task, 1.63)
    assert task.phase == 'stand'
    sample(task, 3.64)
    assert task.phase == 'carry'
    sample(task, 3.7, bilateral=False)
    sample(task, 3.86, bilateral=False)
    assert task.phase == 'failed' and task.failure == 'object_lost'
    assert task.gripper_override == 0


def test_release_requires_destination_support_open_gripper_and_no_hand_contact():
    task = TransferTask()
    for t in (0.0, 0.31, 0.32, 1.33, 3.34):
        sample(task, t)
    assert task.phase == 'carry'
    sample(task, 3.4, arrived=True)
    sample(task, 4.41, arrived=True)
    assert task.phase == 'lower'
    sample(task, 4.42, release_ready=True, over_table=False)
    assert task.phase == 'lower' and task.gripper_override == 0
    sample(task, 4.43, release_ready=True, over_table=True)
    assert task.phase == 'release' and task.gripper_override == 5.4
    for t in (4.44, 5.1):
        sample(
            task,
            t,
            bilateral=False,
            hand_contact=True,
            table_contact=True,
            over_table=True,
            gripper_open=True,
        )
    assert task.phase == 'release'
    for t in (5.11, 5.7):
        sample(
            task,
            t,
            bilateral=False,
            hand_contact=False,
            table_contact=True,
            over_table=True,
            gripper_open=True,
        )
    assert task.phase == 'done'


def test_fall_aborts_before_any_handoff():
    task = TransferTask()
    sample(task, 0.0, fallen=True)
    assert task.phase == 'failed' and task.failure == 'robot_fell'
    assert task.arm_hold is None


def test_default_transfer_configuration_has_obstacle_clearance_waypoints():
    config = load_transfer_config(ROOT / 'configs/okura_transfer.yaml')
    assert config['object_kind'] == 'okra'
    assert config['carry_waypoints'][0][0] < 0
    assert config['carry_waypoints'][-1] == config['carry_target']


def test_rack_physically_supports_okra_at_pick_height():
    import mujoco

    config = load_transfer_config(ROOT / 'configs/okura_transfer.yaml')
    model, data = make_scene(
        object_kind='okra', transfer_scene=True, box_position=config['object_position']
    )
    for _ in range(1000):
        mujoco.mj_step(model, data)
    position = data.xpos[model.body('box').id]
    assert 0.753 < position[2] < 0.761
    assert np.linalg.norm(position[:2] - config['object_position'][:2]) < 0.01


@pytest.mark.skipif(
    not (ROOT / 'artifacts/dex1/Dex1_base_link.STL').exists(), reason='Dex1 assets required'
)
def test_okra_remains_free_and_second_table_is_physical():
    import mujoco

    model, _ = make_scene(
        dex1=True, box_position=(0.43, -0.15, 0.738), object_kind='okra', transfer_scene=True
    )
    assert model.joint('box_free').type == mujoco.mjtJoint.mjJNT_FREE
    assert model.geom('box_collision').type == mujoco.mjtGeom.mjGEOM_ELLIPSOID
    assert model.body('box').mass[0] == pytest.approx(0.013)
    assert model.geom('table_b').contype[0] != 0
    assert model.neq == 1
    assert model.body(model.eq_obj1id[0]).name == 'pelvis'
