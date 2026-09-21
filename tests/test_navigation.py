"""Navigation sensor geometry and loopback command boundary tests."""

import json
import math
import socket

import mujoco
import numpy as np

from g1_backend.config import ROOT, load_config
from g1_backend.navigation import NavigationTransport, laser_scan
from g1_backend.simulator import model_xml


def test_scan_hits_wall_and_rotates_with_base():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="base"><freejoint/><geom type="sphere" size="0.1"/></body>
      <geom type="box" group="2" pos="3 0 1" size="0.1 5 1"/>
    </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    scan = laser_scan(model, data, count=4)
    assert np.isclose(scan[2], 2.9)
    assert scan[0] is None
    data.qpos[3:7] = [math.sqrt(0.5), 0, 0, math.sqrt(0.5)]
    mujoco.mj_forward(model, data)
    assert np.isclose(laser_scan(model, data, count=4)[1], 2.9)


def test_facility_scan_sees_workbench_top(tmp_path):
    config = load_config(ROOT / 'configs/experiments/navigation.yaml')
    config['robot']['initial_position'] = [4.5, -6, 0.793]
    config['robot']['initial_yaw'] = math.pi / 2
    model_text, scene_text = model_xml(config)
    (tmp_path / 'g1_29dof.xml').write_text(model_text)
    (tmp_path / 'scene.xml').write_text(scene_text)
    model = mujoco.MjModel.from_xml_path(str(tmp_path / 'scene.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert np.isclose(laser_scan(model, data, count=4)[2], 0.95, atol=0.01)


def test_commands_are_bounded_reject_nan_and_expire():
    transport = NavigationTransport({'command_port': 0})
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        address = transport.socket.getsockname()
        sender.sendto(json.dumps({'velocity': [3, -3, 3]}).encode(), address)
        np.testing.assert_allclose(transport.velocity(), [0.55, -0.15, 1.0])
        transport.received -= 1
        sender.sendto(b'{"velocity": [NaN, 0, 0]}', address)
        np.testing.assert_array_equal(transport.velocity(), [0, 0, 0])
    finally:
        sender.close()
        transport.close()


def test_turn_assist_walks_sinusoidally_backwards_first_and_respects_lidar():
    transport = NavigationTransport({'command_port': 0, 'turn_assist': {
        'enabled': True, 'speed': 0.3, 'period': 1.5, 'acceleration': 0.4,
        'min_angular_speed': 0.1, 'max_linear_speed': 0.03,
        'obstacle_clearance': 0.8,
    }})
    # Nav2 commonly adds forward motion to a turn. The assist must still
    # replace it when rotation dominates the command.
    command = np.array([0.2, 0.0, 0.35])
    clear = {'time': 0.0, 'scan': [2.0] * 360}
    try:
        assert transport._assist_turn(command, clear)[0] == 0.0
        clear['time'] = 0.75
        assert np.isclose(transport._assist_turn(command, clear)[0], -0.3)
        clear['time'] = 1.5
        assert np.isclose(transport._assist_turn(command, clear)[0], 0.0)
        clear['time'] = 2.25
        assert np.isclose(transport._assist_turn(command, clear)[0], 0.3)
        blocked = [2.0] * 360
        blocked[180] = 0.5  # forward sector during the positive half-wave
        clear.update(time=2.3, scan=blocked)
        assert np.isclose(transport._assist_turn(command, clear)[0], 0.0)
    finally:
        transport.close()


def test_slow_nav2_turn_keeps_phase_and_stops_on_zero():
    import yaml

    config = yaml.safe_load((ROOT / 'configs/experiments/navigation.yaml').read_text())
    transport = NavigationTransport(dict(config['navigation'], command_port=0))
    try:
        scan = [2.0] * 360
        outputs = []
        for tick in range(201):
            # Small angular commands observed in the user's navigation run,
            # including dips below the entry threshold.
            yaw = -0.005 if tick % 5 == 0 else -0.03
            outputs.append(transport._assist_turn(
                np.array([0.0, 0.0, yaw]), {'time': tick * 0.02, 'scan': scan}
            )[0])
        assert min(outputs) < -0.35
        assert max(outputs) > 0.35
        stopped = transport._assist_turn(np.zeros(3), {'time': 4.02, 'scan': scan})
        np.testing.assert_array_equal(stopped, np.zeros(3))
        assert transport.turn_started is None
    finally:
        transport.close()


def test_turn_boost_ramps_holds_and_reverses_with_nav2():
    import yaml

    config = yaml.safe_load((ROOT / 'configs/experiments/navigation.yaml').read_text())
    transport = NavigationTransport(dict(config['navigation'], command_port=0))
    try:
        def step(t, yaw):
            return transport._assist_turn(
                np.array([0.0, 0.0, yaw]), {'time': t, 'scan': [2.0] * 360}
            )[2]

        assert step(0.0, -0.03) == 0.0
        assert np.isclose(step(0.2, -0.03), -0.8)
        assert np.isclose(step(1.0, -0.03), -1.0)
        assert np.isclose(step(2.0, -0.002), -1.0)
        assert np.isclose(step(3.0, 0.03), 1.0)
        assert step(3.02, 0.0) == 0.0
    finally:
        transport.close()


def test_both_correction_phases_request_equal_turn_rate():
    import yaml

    config = yaml.safe_load((ROOT / 'configs/experiments/navigation.yaml').read_text())
    transport = NavigationTransport(dict(config['navigation'], command_port=0))
    try:
        samples = []
        for tick in range(201):
            t = tick * 0.02
            # Nav2 reduces its angular request during the forward half.
            command = np.array([0.0, 0.0, -0.03 if t < 2 else -0.002])
            out = transport._assist_turn(command, {'time': t, 'scan': [2.0] * 360})
            if tick in (50, 150):
                samples.append(out)
        assert samples[0][0] < 0 < samples[1][0]
        np.testing.assert_allclose(samples[0][2], samples[1][2])
        assert samples[0][2] == -1.0
    finally:
        transport.close()


def test_turn_assist_releases_when_nav2_starts_forward_path_following():
    import yaml

    config = yaml.safe_load((ROOT / 'configs/experiments/navigation.yaml').read_text())
    transport = NavigationTransport(dict(config['navigation'], command_port=0))
    try:
        telemetry = {'time': 0.0, 'scan': [2.0] * 360}
        transport._assist_turn(np.array([0.0, 0.0, 0.2]), telemetry)
        telemetry['time'] = 0.5
        command = np.array([0.1, 0.0, 0.2])
        np.testing.assert_allclose(
            transport._assist_turn(command, telemetry), [0.1, 0.0, 0.6]
        )
        assert transport.turn_started is None
        telemetry['time'] = 1.0
        np.testing.assert_allclose(
            transport._assist_turn(np.array([0.2, 0.0, 0.01]), telemetry),
            [0.2, 0.0, 0.03],
        )
        telemetry['time'] = 1.02
        np.testing.assert_array_equal(
            transport._assist_turn(np.zeros(3), telemetry), np.zeros(3)
        )
    finally:
        transport.close()
