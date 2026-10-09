"""Independent sensor processes with bounded, latest-only shared mailboxes."""

import atexit
import multiprocessing as mp
import os
import pickle
import signal
import socket
import json
import tempfile
import traceback
from pathlib import Path

import numpy as np
from .sensor_effects import SensorEffects, DelayedFrames


class LatestMailbox:
    """One shared slot; readers and writers never wait for the other process."""

    def __init__(self, context, capacity):
        self.buffer = context.RawArray('B', capacity)
        self.length = context.RawValue('Q', 0)
        self.sequence = context.RawValue('Q', 0)
        self.lock = context.Lock()
        self.seen = 0

    def put(self, value):
        payload = pickle.dumps(value, protocol=pickle.HIGHEST_PROTOCOL)
        if len(payload) > len(self.buffer):
            raise ValueError('Sensor mailbox capacity exceeded')
        if not self.lock.acquire(False):
            return False
        try:
            np.frombuffer(self.buffer, dtype=np.uint8)[:len(payload)] = np.frombuffer(
                payload, dtype=np.uint8
            )
            self.length.value = len(payload)
            self.sequence.value += 1
        finally:
            self.lock.release()
        return True

    def get(self):
        if not self.lock.acquire(False):
            return None
        try:
            if self.sequence.value == self.seen:
                return None
            payload = bytes(np.frombuffer(self.buffer, dtype=np.uint8)[:self.length.value])
            self.seen = self.sequence.value
        finally:
            self.lock.release()
        return pickle.loads(payload)


def _run_sensor(kind, config, sources, backend, states, frames, stop, status):
    # The parent owns shutdown through the stop event.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    renderer = None
    control_socket = None
    try:
        # Set before importing MuJoCo: GL backend selection is process-local.
        if backend:
            os.environ['MUJOCO_GL'] = 'egl' if backend == 'egl-cpu' else backend
            os.environ['PYOPENGL_PLATFORM'] = os.environ['MUJOCO_GL']
            if backend == 'egl-cpu':
                os.environ['LIBGL_ALWAYS_SOFTWARE'] = 'true'
                os.environ['MESA_LOADER_DRIVER_OVERRIDE'] = 'llvmpipe'
                vendor = Path('/usr/share/glvnd/egl_vendor.d/50_mesa.json')
                if vendor.exists():
                    os.environ['__EGL_VENDOR_LIBRARY_FILENAMES'] = str(vendor)
        import mujoco
        if backend == 'egl-cpu':
            # MuJoCo's default EGL path selects a hardware device. Mesa's
            # surfaceless display supports llvmpipe on machines without one.
            import mujoco.egl
            from mujoco.egl import egl_ext as EGL

            display = EGL.eglGetPlatformDisplayEXT(0x31DD, EGL.EGL_DEFAULT_DISPLAY, None)
            if not EGL.eglInitialize(display, None, None):
                raise RuntimeError('Unable to initialize software Mesa/EGL')
            mujoco.egl.EGL_DISPLAY = display
            atexit.register(EGL.eglTerminate, display)

        with tempfile.TemporaryDirectory(prefix='g1_sensor_') as directory:
            path = Path(directory)
            (path / 'g1_29dof.xml').write_text(sources[0])
            (path / 'scene.xml').write_text(sources[1])
            model = mujoco.MjModel.from_xml_path(str(path / 'scene.xml'))
        data = mujoco.MjData(model)
        effects = (SensorEffects(config['sensor_effects'], config.get('seed', 0) + (101 if kind == 'camera' else 202))
                   if config.get('sensor_effects') else None)
        spec = mujoco.mjtState.mjSTATE_INTEGRATION
        if kind == 'camera':
            camera = config['camera']
            from .camera_control import camera_pitch, tilted_quaternion
            pitch = 0.0
            camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'd435i')
            if config.get('navigation') and camera_id >= 0:
                control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                control_socket.bind(('127.0.0.1', 19423))
                control_socket.setblocking(False)
                original_quaternion = model.cam_quat[camera_id].copy()
            views = camera.get('views') or [{'name': 'policy_camera'}]
            renderer = mujoco.Renderer(model, camera['height'], camera['width'])
            mujoco.mj_forward(model, data)
            for view in views:
                renderer.update_scene(data, camera=view['name'])
                renderer.render()
            if camera['depth']:
                renderer.enable_depth_rendering()
                renderer.render()
                renderer.disable_depth_rendering()
            from OpenGL import GL
            device = GL.glGetString(GL.GL_RENDERER)
            device = device.decode(errors='replace') if device else 'unknown'
            acceleration = 'cpu' if backend in ('osmesa', 'egl-cpu') or any(
                name in device.lower() for name in ('llvmpipe', 'softpipe', 'swrast', 'software')
            ) else 'gpu'
            if backend == 'egl-cpu' and not any(
                name in device.lower() for name in ('llvmpipe', 'softpipe', 'swrast', 'software')
            ):
                raise RuntimeError(f'CPU rendering requested but got {device}')
            status.send({'ready': True, 'backend': backend, 'device': device,
                         'acceleration': acceleration})
        else:
            status.send({'ready': True})
        while not stop.is_set():
            snapshot = states.get()
            if snapshot is None:
                stop.wait(0.002)
                continue
            captured, state = snapshot
            if control_socket is not None:
                for _ in range(100):
                    try:
                        payload, _ = control_socket.recvfrom(1024)
                    except BlockingIOError:
                        break
                    try:
                        pitch = camera_pitch(json.loads(payload)['pitch_deg'])
                    except (ValueError, TypeError, KeyError):
                        continue
                model.cam_quat[camera_id] = tilted_quaternion(original_quaternion, pitch)
            mujoco.mj_setState(model, data, state, spec)
            mujoco.mj_forward(model, data)
            result = {'time': captured, 'sensors': {}}
            if kind == 'camera':
                for view in views:
                    renderer.update_scene(data, camera=view['name'])
                    prefix = f"{view['name']}_" if 'views' in camera else ''
                    if camera['rgb']:
                        result['sensors'][f'{prefix}rgb'] = renderer.render().copy()
                    if camera['depth']:
                        renderer.enable_depth_rendering()
                        try:
                            result['sensors'][f'{prefix}depth'] = renderer.render().copy()
                        finally:
                            renderer.disable_depth_rendering()
            else:
                from .navigation import laser_scan, lidar_scan
                lidar = config['navigation'].get('lidar', {})
                if lidar.get('enabled'):
                    points, result['scan'] = lidar_scan(model, data, lidar, effects)
                    result['sensors']['mid360_points'] = points
                else:
                    result['scan'] = laser_scan(model, data)
            if control_socket is not None:
                result['sensors']['camera_pitch_deg'] = np.array(pitch)
            if kind == 'camera' and effects is not None:
                result['sensors'] = effects.camera(result['sensors'])
            frames.put(result)
    except BaseException:  # noqa: BLE001 - report child failures to the parent
        status.send({'error': traceback.format_exc()})
    finally:
        if control_socket is not None:
            control_socket.close()
        if renderer is not None:
            renderer.close()
        status.close()


class SensorWorker:
    def __init__(self, kind, config, sources, state_size):
        timing = config.get('sensor_effects', {}).get(kind, {})
        self.delivery = DelayedFrames(timing.get('latency_s', 0), timing.get('jitter_s', 0),
                                      config.get('seed', 0) + (303 if kind == 'camera' else 404))
        self.context = mp.get_context('spawn')
        self.states = LatestMailbox(self.context, state_size * 8 + 4096)
        camera = config['camera']
        if kind == 'camera':
            pixels = camera['width'] * camera['height']
            views = len(camera.get('views') or [camera])
            capacity = pixels * views * (3 * bool(camera['rgb']) + 4 * bool(camera['depth']))
        else:
            lidar = config['navigation'].get('lidar', {})
            samples = lidar.get('horizontal_samples', 360)
            capacity = samples * (12 * len(lidar.get('vertical_angles_deg', [])) + 32)
        self.frames = LatestMailbox(self.context, capacity + 65536)
        self.process = None
        self.status = None
        self.stop = None
        self.info = {}
        preference = camera.get('backend', 'auto')
        backends = {'auto': ['egl', 'osmesa', 'egl-cpu'],
                    'cpu': ['osmesa', 'egl-cpu']}.get(preference, [preference])
        errors = []
        for backend in backends if kind == 'camera' else [None]:
            self.stop = self.context.Event()
            self.status, child = self.context.Pipe(duplex=False)
            self.process = self.context.Process(
                target=_run_sensor,
                args=(kind, config, sources, backend, self.states, self.frames,
                      self.stop, child),
                name=f'g1-{kind}-{backend or "rays"}',
            )
            self.process.start()
            child.close()
            try:
                if not self.status.poll(60):
                    raise RuntimeError(f'{kind} initialization timed out')
                self.info = self.status.recv()
                if 'error' in self.info:
                    raise RuntimeError(self.info['error'])
                return
            except (RuntimeError, EOFError) as error:
                errors.append(f'{backend}: {error}')
                self.close()
        raise RuntimeError(f'Unable to initialize {kind}: ' + '\n'.join(errors))

    def submit(self, captured, state):
        return self.states.put((captured, state))

    def receive(self, sim_time=None):
        if self.status.poll():
            try:
                message = self.status.recv()
            except EOFError:
                message = {'error': 'Sensor process closed unexpectedly'}
            if 'error' in message:
                raise RuntimeError(message['error'])
        if not self.process.is_alive():
            raise RuntimeError(f'{self.process.name} exited ({self.process.exitcode})')
        frame = self.frames.get()
        if sim_time is None or (self.delivery.latency == 0 and self.delivery.jitter == 0):
            return frame
        return self.delivery.receive(frame, sim_time)

    def close(self):
        if self.stop is not None:
            self.stop.set()
        if self.process is not None:
            self.process.join(timeout=2)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=2)
            if self.process.is_alive():
                self.process.kill()
                self.process.join()
            self.process.close()
            self.process = None
        if self.status is not None:
            self.status.close()
            self.status = None
