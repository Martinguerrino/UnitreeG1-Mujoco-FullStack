"""Exercise real spawned sensors, latest-only delivery and capture timestamps."""

import ctypes.util
import multiprocessing as mp
import time

import numpy as np
import pytest

from g1_backend.sensor_worker import LatestMailbox, SensorWorker


def test_mailbox_replaces_pending_and_never_waits():
    mailbox = LatestMailbox(mp.get_context('spawn'), 4096)
    assert mailbox.put({'time': 1})
    assert mailbox.put({'time': 2})
    assert mailbox.get() == {'time': 2}
    assert mailbox.get() is None
    mailbox.lock.acquire()
    try:
        assert not mailbox.put({'time': 3})
        assert mailbox.get() is None
    finally:
        mailbox.lock.release()
    assert mailbox.put({'time': 4})
    assert mailbox.get() == {'time': 4}


@pytest.mark.parametrize('backend', ['egl', 'osmesa', 'cpu', 'auto'])
def test_spawned_rgb_depth_renderer(backend, monkeypatch):
    if backend == 'osmesa' and not ctypes.util.find_library('OSMesa'):
        pytest.skip('OSMesa is not installed; CPU coverage uses software EGL')
    mujoco = pytest.importorskip('mujoco')
    # The worker must override the parent's viewer backend before importing GL.
    monkeypatch.setenv('MUJOCO_GL', 'glfw')
    if backend == 'auto':
        monkeypatch.setenv('MUJOCO_EGL_DEVICE_ID', '99999')
    source = '''<mujoco><visual><global offwidth="32" offheight="24"/></visual>
      <worldbody><camera name="policy_camera" pos="0 0 3"/>
      <body pos="0 0 0.5"><freejoint/><geom type="sphere" size="0.3"/></body>
      <geom type="plane" size="3 3 .1"/></worldbody></mujoco>'''
    config = {'camera': {'enabled': True, 'width': 32, 'height': 24,
                         'rgb': True, 'depth': True, 'backend': backend}}
    model = mujoco.MjModel.from_xml_string(source)
    data = mujoco.MjData(model)
    spec = mujoco.mjtState.mjSTATE_INTEGRATION
    state = np.empty(mujoco.mj_stateSize(model, spec))
    data.time = 1.25
    mujoco.mj_getState(model, data, state, spec)
    worker = SensorWorker('camera', config, (source, source), len(state))
    try:
        if backend in ('cpu', 'auto'):
            assert worker.info['backend'] in ('osmesa', 'egl-cpu')
        else:
            assert worker.info['backend'] == backend
        if backend in ('osmesa', 'cpu', 'auto'):
            assert worker.info['acceleration'] == 'cpu'
        assert worker.submit(1.25, state)
        deadline = time.monotonic() + 10
        frame = None
        while frame is None and time.monotonic() < deadline:
            frame = worker.receive()
            time.sleep(.01)
        assert frame is not None
        assert frame['time'] == 1.25
        assert frame['sensors']['rgb'].shape == (24, 32, 3)
        assert frame['sensors']['depth'].shape == (24, 32)
        assert np.isfinite(frame['sensors']['depth']).all()
        assert worker.process.pid != __import__('os').getpid()
    finally:
        worker.close()
    assert worker.process is None


def test_spawned_lidar_uses_snapshot_and_capture_time():
    mujoco = pytest.importorskip('mujoco')
    source = '''<mujoco><worldbody><body name="torso_link" pos="0 0 .78">
      <freejoint/><geom type="sphere" size=".1"/></body>
      <geom type="box" pos="2 0 .78" size=".1 2 2" group="2"/>
      </worldbody></mujoco>'''
    config = {'camera': {}, 'navigation': {'lidar': {
        'enabled': True, 'body': 'torso_link', 'position': [.2, 0, 0],
        'rpy': [0, 0, 0], 'horizontal_samples': 16,
        'vertical_angles_deg': [0], 'min_range': .05, 'max_range': 20}}}
    model = mujoco.MjModel.from_xml_string(source)
    data = mujoco.MjData(model)
    data.time = 3.5
    spec = mujoco.mjtState.mjSTATE_INTEGRATION
    state = np.empty(mujoco.mj_stateSize(model, spec))
    mujoco.mj_getState(model, data, state, spec)
    worker = SensorWorker('lidar', config, (source, source), len(state))
    try:
        worker.submit(3.5, state)
        deadline = time.monotonic() + 10
        frame = None
        while frame is None and time.monotonic() < deadline:
            frame = worker.receive()
            time.sleep(.01)
        assert frame is not None
        assert frame['time'] == 3.5
        assert len(frame['scan']) == 16
        assert frame['sensors']['mid360_points'].shape[1] == 3
        assert len(frame['sensors']['mid360_points']) > 0
        assert frame['scan'][8] == pytest.approx(1.9, abs=1e-5)
    finally:
        worker.close()


def test_camera_buttons_change_render_and_reset(monkeypatch):
    import json
    import socket

    mujoco = pytest.importorskip('mujoco')
    monkeypatch.setenv('MUJOCO_GL', 'glfw')
    source = '''<mujoco><visual><global offwidth="32" offheight="24"/></visual>
      <worldbody><camera name="d435i" pos="0 0 3"/>
      <body pos="0 0 0.5"><freejoint/><geom type="sphere" size="0.3" rgba="1 0 0 1"/></body>
      <geom type="plane" size="3 3 .1"/></worldbody></mujoco>'''
    config = {'navigation': {'enabled': True}, 'camera': {
        'enabled': True, 'width': 32, 'height': 24, 'rgb': True, 'depth': True,
        'backend': 'cpu', 'views': [{'name': 'd435i'}]}}
    model = mujoco.MjModel.from_xml_string(source)
    data = mujoco.MjData(model)
    spec = mujoco.mjtState.mjSTATE_INTEGRATION
    state = np.empty(mujoco.mj_stateSize(model, spec))
    mujoco.mj_getState(model, data, state, spec)
    worker = SensorWorker('camera', config, (source, source), len(state))
    command = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        frames = []
        for index, angle in enumerate([0, 25, 0]):
            command.sendto(json.dumps({'pitch_deg': angle}).encode(), ('127.0.0.1', 19423))
            worker.submit(float(index), state)
            deadline = time.monotonic() + 10
            frame = None
            while frame is None and time.monotonic() < deadline:
                frame = worker.receive()
                time.sleep(.01)
            assert frame is not None and frame['time'] == index
            assert frame['sensors']['camera_pitch_deg'] == angle
            frames.append(frame['sensors'])
        assert not np.array_equal(frames[0]['d435i_rgb'], frames[1]['d435i_rgb'])
        np.testing.assert_array_equal(frames[0]['d435i_rgb'], frames[2]['d435i_rgb'])
        np.testing.assert_array_equal(frames[0]['d435i_depth'], frames[2]['d435i_depth'])
    finally:
        command.close()
        worker.close()
