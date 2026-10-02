"""Timestamp compensation selects the prediction for the current simulated time."""

import numpy as np
import pytest

from experiments.okura.timing import ChunkTimeline


def sequence(offset=0):
    return np.repeat((np.arange(100) + offset)[:, None], 16, axis=1).astype(float)


def test_delay_skips_stale_actions_and_expired_chunk_is_not_replayed():
    timeline = ChunkTimeline()
    timeline.add(sequence(), 1.0)
    np.testing.assert_array_equal(timeline.action(1.2), np.full(16, 6))
    assert timeline.action(0.9) is None
    assert timeline.action(1.0 + 100 / 30) is None


def test_zero_is_a_valid_prediction_and_newest_chunk_has_priority():
    timeline = ChunkTimeline()
    timeline.add(np.zeros((100, 16)), 0.0)
    np.testing.assert_array_equal(timeline.action(0.1), np.zeros(16))
    timeline.add(sequence(1000), 0.1)
    np.testing.assert_array_equal(timeline.action(0.2), np.full(16, 1003))


def test_temporal_blend_uses_only_matching_unexpired_predictions():
    timeline = ChunkTimeline(ensemble=True, decay=0)
    timeline.add(sequence(), 0.0)
    timeline.add(sequence(100), 0.1)
    np.testing.assert_allclose(timeline.action(0.2), np.full(16, (6 + 103) / 2))
    # First chunk is expired; only the second remains usable.
    np.testing.assert_allclose(timeline.action(100 / 30), np.full(16, 197))


def test_reject_nonfinite_chunk_and_epoch():
    timeline = ChunkTimeline()
    with pytest.raises(ValueError):
        timeline.add(np.full((100, 16), np.nan), 0.0)
    with pytest.raises(ValueError):
        timeline.add(sequence(), np.nan)


def test_optional_camera_changes_preserve_body_model_and_free_object():
    import mujoco
    from experiments.okura.scene import make_scene

    original, _ = make_scene(dex1=True, object_kind='okra', transfer_scene=True)
    altered, _ = make_scene(dex1=True, object_kind='okra', transfer_scene=True,
                            camera_pitch=60, camera_position=[0.1, 0.04, 0.45],
                            camera_fovy=50, basket_scale=1.4, visual_style='reference')
    assert original.nu == altered.nu == 33
    np.testing.assert_array_equal(original.sensor_adr, altered.sensor_adr)
    assert altered.neq == 1
    assert altered.joint('box_free').type == mujoco.mjtJoint.mjJNT_FREE
    assert original.camera('cam_left_high').fovy[0] == 60
    assert altered.camera('cam_left_high').fovy[0] == 50


def test_position_window_rejects_translation_and_measures_small_solver_jitter():
    from experiments.okura.observer import PositionSpeedWindow

    moving = PositionSpeedWindow(.1)
    resting = PositionSpeedWindow(.1)
    for index in range(101):
        now = index * .002
        speed = moving.measure(now, [now * .1, 0, 0], .1)
        stable = resting.measure(now, [0, 0, .73 + (-1)**index * .00005], .05)
    assert speed > .05  # A moving object cannot satisfy the placement guard.
    assert stable < .005
    warming_up = PositionSpeedWindow(.1)
    assert warming_up.measure(0, [0, 0, 0], .3) == .3
    assert warming_up.measure(.05, [0, 0, 0], .3) == .3


def test_command_pump_keeps_snapshot_and_deliberate_loss_stops_publication():
    from experiments.okura.command_pump import CommandPump
    from g1_backend.contracts import JointCommand

    body, hands = [], []
    pump = CommandPump(lambda command: body.append(command.q.copy()),
                       lambda values: hands.append(values.copy()))
    command = JointCommand(np.ones(29), np.ones(29), np.ones(29))
    grippers = np.array([0., 5.4])
    pump.update(command, grippers)
    command.q[:] = 2
    grippers[:] = 0
    pump.publish_once()
    np.testing.assert_array_equal(body[0], np.ones(29))
    np.testing.assert_array_equal(hands[0], [0, 5.4])
    pump.update(command, grippers, enabled=False)
    pump.publish_once()
    assert len(body) == len(hands) == 1
    pump.close()
