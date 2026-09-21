"""MuJoCo process wrapping the pinned official unitree_mujoco Python bridge.

The pipe only carries lifecycle and sensor telemetry. Motor targets use DDS.
"""

import hashlib
import importlib.util
import json
import os
import resource
from pathlib import Path
import sys
import tempfile
import threading
import time
import traceback
import types
import xml.etree.ElementTree as ET

import numpy as np

from .config import ROOT, runtime_path


def model_xml(config):
    robot = config['robot']
    model = ET.fromstring((ROOT / robot['model']).read_text())
    model.find('compiler').set('meshdir', str(ROOT / robot['meshes']))
    scene = ET.fromstring((ROOT / robot['scene']).read_text())
    option = scene.find('option')
    if option is None:
        option = ET.SubElement(scene, 'option')
    option.set('timestep', str(robot['timestep']))
    # Externally sourced worlds may include optional constraints for robot
    # variants (for example Dex3 palms) absent from our 29-DoF model.
    body_names = {body.get('name') for body in model.findall('.//body')}
    equality = scene.find('equality')
    if equality is None:
        equality = ET.SubElement(scene, 'equality')
    else:
        for constraint in list(equality):
            references = [constraint.get(key) for key in ('body1', 'body2')]
            if any(reference and reference not in body_names for reference in references):
                equality.remove(constraint)
    if equality.find("weld[@name='pelvis_fixture']") is None:
        ET.SubElement(equality, 'weld', name='pelvis_fixture', body1='pelvis')
    # Retain the existing fixture through warmup; release after DDS controller is ready.
    pelvis = model.find(".//body[@name='pelvis']")
    pelvis.set('pos', ' '.join(map(str, robot['initial_position'])))
    yaw = robot['initial_yaw']
    pelvis.set('quat', f'{np.cos(yaw / 2)} 0 0 {np.sin(yaw / 2)}')
    camera = config['camera']
    if camera['enabled']:
        for view in camera.get('views') or [camera]:
            parent = model.find(f".//body[@name='{view['body']}']")
            if parent is None:
                raise ValueError('Camera parent body does not exist')
            ET.SubElement(
                parent,
                'camera',
                name=view.get('name', 'policy_camera'),
                pos=' '.join(map(str, view['position'])),
                quat=' '.join(map(str, view['quaternion'])),
                fovy=str(view['fovy']),
            )
        visual = scene.find('visual')
        global_settings = visual.find('global')
        global_settings.set('offwidth', str(camera['width']))
        global_settings.set('offheight', str(camera['height']))
    return ET.tostring(model, encoding='unicode'), ET.tostring(scene, encoding='unicode')


def joint_limits(config):
    model = ET.fromstring((ROOT / config['robot']['model']).read_text())
    names = [act.attrib['joint'] for act in model.find('actuator')]
    return np.array(
        [
            list(map(float, model.find(f".//joint[@name='{n}']").attrib['range'].split()))
            for n in names
        ]
    )


def run_simulator(config, initial_q, pipe, stop, release):
    renderer = None
    viewer = None
    try:
        runtime_path()
        live_view = config.get('viewer', {}).get('enabled', False)
        if live_view:
            os.environ['MUJOCO_GL'] = 'glfw'
        else:
            os.environ.setdefault('MUJOCO_GL', 'egl')
        import mujoco
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.utils.crc import CRC

        robot = config['robot']
        bridge_path = ROOT / robot['bridge']
        lockfile = json.loads((ROOT / 'configs/sources.lock.json').read_text())
        if (
            hashlib.sha256(bridge_path.read_bytes()).hexdigest()
            != lockfile['files'][robot['bridge']]['sha256']
        ):
            raise ValueError('Official bridge checksum mismatch')
        upstream_config = types.ModuleType('config')
        upstream_config.ROBOT = 'g1'
        sys.modules['config'] = upstream_config
        spec = importlib.util.spec_from_file_location('official_unitree_bridge', bridge_path)
        upstream = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(upstream)
        model_source, scene_source = model_xml(config)
        with tempfile.TemporaryDirectory(prefix='g1_dds_') as directory:
            path = Path(directory)
            (path / 'g1_29dof.xml').write_text(model_source)
            (path / 'scene.xml').write_text(scene_source)
            model = mujoco.MjModel.from_xml_path(str(path / 'scene.xml'))
        data = mujoco.MjData(model)
        # Episode initialization only. Runtime control is exclusively LowCmd -> bridge -> ctrl.
        for i in range(model.nu):
            data.qpos[model.jnt_qposadr[model.actuator_trnid[i, 0]]] = initial_q[i]
        mujoco.mj_forward(model, data)
        if live_view:
            import mujoco.viewer

            viewer = mujoco.viewer.launch_passive(model, data)
        ChannelFactoryInitialize(robot['domain_id'], robot['interface'])
        mutex = threading.RLock()

        class Bridge(upstream.UnitreeSdk2Bridge):
            def __init__(self):
                self.latest = None
                self.last_command = time.monotonic()
                self.invalid_commands = 0
                self.watchdog_events = 0
                self.timed_out = False
                self.crc_check = CRC()
                # Upstream initializes names without underscores but publishes using underscores.
                self.have_imu_ = self.have_frame_sensor_ = False
                super().__init__(model, data)

            def LowCmdHandler(self, message):
                with mutex:
                    values = [[m.q, m.dq, m.tau, m.kp, m.kd] for m in message.motor_cmd[:29]]
                    if (
                        message.mode_pr != 0
                        or not np.isfinite(values).all()
                        or message.crc != self.crc_check.Crc(message)
                    ):
                        self.invalid_commands += 1
                        return
                    self.latest = message
                    self.last_command = time.monotonic()

            def PublishLowState(self):
                with mutex:
                    super().PublishLowState()

            def PublishHighState(self):
                with mutex:
                    super().PublishHighState()

            def advance(self):
                stale = (
                    self.latest is not None
                    and time.monotonic() - self.last_command > robot['command_timeout']
                )
                if stale:
                    if not self.timed_out:
                        self.watchdog_events += 1
                    data.ctrl[:] = -2 * data.sensordata[29:58]
                elif self.latest is not None:
                    super().LowCmdHandler(self.latest)
                self.timed_out = stale

        camera = config['camera']
        camera_views = camera.get('views') or [{'name': 'policy_camera'}]
        if camera['enabled']:
            renderer = mujoco.Renderer(model, camera['height'], camera['width'])
            # EGL shader compilation can take longer than the state watchdog.
            # Warm the renderer before announcing readiness or starting DDS threads.
            for view in camera_views:
                renderer.update_scene(data, camera=view['name'])
                renderer.render()
            if camera['depth']:
                renderer.enable_depth_rendering()
                renderer.render()
                renderer.disable_depth_rendering()
        bridge = Bridge()
        pipe.send({'ready': True})
        started = time.monotonic()
        cpu_started = time.process_time()
        rendered_frames = 0
        steps = 0
        collisions = 0
        previous_contacts = set()
        next_telemetry = 0.0
        next_camera = 0.0
        next_scan = 0.0
        scan = None
        scan_time = None
        next_viewer = 0.0
        sensor_frame = None
        sensor_timestamp = None
        fixture = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, 'pelvis_fixture')
        if fixture < 0:
            raise ValueError('Warmup requires the existing pelvis_fixture')
        while not stop.is_set():
            if viewer is not None and not viewer.is_running():
                stop.set()
                break
            before = time.monotonic()
            with mutex:
                if release.is_set():
                    data.eq_active[fixture] = 0
                bridge.advance()
                mujoco.mj_step(model, data)
                steps += 1
                contacts = set()
                for contact in data.contact:
                    bodies = [
                        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[g])
                        for g in (contact.geom1, contact.geom2)
                    ]
                    # Foot-ground support is expected, all other contact pairs are counted.
                    if 'world' in bodies and any('ankle_roll' in b for b in bodies):
                        continue
                    contacts.add(tuple(sorted((int(contact.geom1), int(contact.geom2)))))
                collisions += len(contacts - previous_contacts)
                previous_contacts = contacts
                if config.get('navigation') and data.time >= next_scan:
                    from .navigation import laser_scan

                    scan = laser_scan(model, data)
                    scan_time = float(data.time)
                    next_scan = data.time + 0.1
                if renderer and data.time >= next_camera:
                    # Physics is paused in this process while rendering. Let the
                    # DDS publisher read the stable MjData instead of starving
                    # LowState for the duration of two EGL renders.
                    mutex.release()
                    try:
                        rendered_frames += 1
                        sensor_frame = {}
                        multiple_views = 'views' in camera
                        for view in camera_views:
                            renderer.update_scene(data, camera=view['name'])
                            prefix = f"{view['name']}_" if multiple_views else ''
                            if camera['rgb']:
                                sensor_frame[f'{prefix}rgb'] = renderer.render().copy()
                            if camera['depth']:
                                renderer.enable_depth_rendering()
                                sensor_frame[f'{prefix}depth'] = renderer.render().copy()
                                renderer.disable_depth_rendering()
                    finally:
                        mutex.acquire()
                    sensor_timestamp = float(data.time)
                    next_camera = data.time + 1 / camera['fps']
                if data.time >= next_telemetry:
                    pipe.send(
                        {
                            'time': float(data.time),
                            'scan': scan,
                            'scan_time': scan_time,
                            'position': data.qpos[:3].tolist(),
                            'quaternion': data.qpos[3:7].tolist(),
                            'linear_velocity': data.qvel[:3].tolist(),
                            'steps': steps,
                            'steps_per_second': steps / (time.monotonic() - started),
                            'simulator_cpu_percent': 100
                            * (time.process_time() - cpu_started)
                            / (time.monotonic() - started),
                            'simulator_peak_ram_bytes': resource.getrusage(
                                resource.RUSAGE_SELF
                            ).ru_maxrss
                            * 1024,
                            'render_fps': rendered_frames / (time.monotonic() - started),
                            'collisions': collisions,
                            'watchdog_events': bridge.watchdog_events,
                            'watchdog_active': bridge.timed_out,
                            'damping_active': bridge.latest is not None
                            and all(m.kp == 0 and m.kd == 2 for m in bridge.latest.motor_cmd[:29]),
                            'invalid_commands': bridge.invalid_commands,
                            'sensors': sensor_frame,
                            'sensor_time': sensor_timestamp,
                            'camera_frame_id': rendered_frames,
                        }
                    )
                    next_telemetry = data.time + 0.02
            if viewer is not None and time.monotonic() >= next_viewer:
                viewer.sync()
                next_viewer = time.monotonic() + 1 / 30
            time.sleep(max(0, robot['timestep'] - (time.monotonic() - before)))
    except BaseException:  # noqa: BLE001 - forward child failures to parent
        pipe.send({'error': traceback.format_exc()})
    finally:
        if viewer:
            viewer.close()
        if renderer:
            renderer.close()
        pipe.close()
        # Official bridge recurrent threads are daemon threads; process owns their lifecycle.
