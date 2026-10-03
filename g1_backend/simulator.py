"""MuJoCo process wrapping the pinned official unitree_mujoco Python bridge.

The pipe only carries lifecycle and sensor telemetry. Motor targets use DDS.
"""

import hashlib
import importlib.util
import json
import os
import resource
import signal
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
    dexterity = robot.get('dexterity', 'hand')
    if dexterity not in ('hand', 'claw'):
        raise ValueError('dexterity must be hand or claw')
    if dexterity == 'claw':
        from .dex1 import add_dex1

        add_dex1(model)
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
            # Some imported scenes contain a fixed camera placeholder. The
            # robot-mounted sensor is authoritative and camera names are global
            # in MuJoCo, so remove a same-named top-level placeholder first.
            worldbody = scene.find('worldbody')
            for placeholder in list(worldbody.findall('camera')):
                if placeholder.get('name') == view.get('name', 'policy_camera'):
                    worldbody.remove(placeholder)
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
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    workers = {}
    viewer = None
    grippers = None
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
        body_model = model
        if robot.get('dexterity', 'hand') == 'claw':
            body_config = {**config, 'robot': {**robot, 'dexterity': 'hand'}}
            body_source, body_scene = model_xml(body_config)
            with tempfile.TemporaryDirectory(prefix='g1_body_') as directory:
                path = Path(directory)
                (path / 'g1_29dof.xml').write_text(body_source)
                (path / 'scene.xml').write_text(body_scene)
                body_model = mujoco.MjModel.from_xml_path(str(path / 'scene.xml'))
        data = mujoco.MjData(model)
        # Episode initialization only. Runtime control is exclusively LowCmd -> bridge -> ctrl.
        for i in range(29):
            data.qpos[model.jnt_qposadr[model.actuator_trnid[i, 0]]] = initial_q[i]
        mujoco.mj_forward(model, data)
        if live_view:
            import mujoco.viewer

            viewer = mujoco.viewer.launch_passive(model, data)
        ChannelFactoryInitialize(robot['domain_id'], robot['interface'])
        plant = None
        if robot.get('dexterity', 'hand') == 'claw':
            from .dex1 import GripperDds, GripperPlant, motor_to_slide

            plant = GripperPlant(model)
            data.qpos[plant.qadr] = motor_to_slide(5.4)
            grippers = GripperDds(server=True)
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
                super().__init__(body_model, data)

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
                    data.ctrl[:29] = -2 * data.sensordata[29:58]
                elif self.latest is not None:
                    super().LowCmdHandler(self.latest)
                self.timed_out = stale

        camera = config['camera']
        from .sensor_worker import SensorWorker

        state_spec = mujoco.mjtState.mjSTATE_INTEGRATION
        state_size = mujoco.mj_stateSize(model, state_spec)
        sensor_state = np.empty(state_size)
        sources = (model_source, scene_source)
        if camera['enabled']:
            workers['camera'] = SensorWorker('camera', config, sources, state_size)
        if config.get('navigation'):
            workers['lidar'] = SensorWorker('lidar', config, sources, state_size)
        bridge = Bridge()
        pipe.send({'ready': True})
        started = time.monotonic()
        cpu_started = time.process_time()
        rendered_frames = 0
        steps = 0
        collisions = 0
        previous_contacts = set()
        next_telemetry = 0.0
        navigation_imu = []
        feet_samples = []
        feet_estimator = None
        next_feet = 0.0
        if config.get('navigation', {}).get('feet_estimation'):
            from .feet_estimation import FeetEstimator
            feet_estimator = FeetEstimator(body_model)
        from .sensor_effects import SensorEffects
        effects = (SensorEffects(config['sensor_effects'], config['seed'] + 505)
                   if config.get('sensor_effects') else None)
        feet_effects = (SensorEffects(config['sensor_effects'], config['seed'] + 506)
                        if effects is not None and feet_estimator is not None else None)
        next_camera = 0.0
        next_scan = 0.0
        scan = None
        scan_time = None
        next_viewer = 0.0
        sensor_frame = None
        sensor_times = {}
        sensor_timestamp = None
        camera_timestamp = None
        fixture = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, 'pelvis_fixture')
        torso_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'torso_link')
        joint_ids = model.actuator_trnid[:, 0]
        joint_names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(joint_id))
            for joint_id in joint_ids
        ]
        joint_qpos_addresses = model.jnt_qposadr[joint_ids]
        if fixture < 0:
            raise ValueError('Warmup requires the existing pelvis_fixture')
        while not stop.is_set():
            if viewer is not None and not viewer.is_running():
                stop.set()
                break
            before = time.monotonic()
            received_frames = {kind: worker.receive(sim_time=float(data.time)) for kind, worker in workers.items()}
            with mutex:
                if release.is_set():
                    data.eq_active[fixture] = 0
                bridge.advance()
                if plant is not None:
                    try:
                        gripper_target = grippers.read()
                    except TimeoutError:
                        gripper_target = None
                    plant.advance(data, gripper_target)
                    grippers.send(plant.encoder_state(data))
                if feet_estimator is not None and data.time >= next_feet:
                    inertial = {'time': max(0., float(data.time - robot['timestep'])),
                                'gyro': data.sensor('imu_gyro').data.tolist(),
                                'acc': data.sensor('imu_acc').data.tolist()}
                    if feet_effects is not None:
                        inertial = feet_effects.imu(inertial)
                    sample = feet_estimator.update(
                        inertial['time'], data.sensordata[:29], data.sensordata[29:58],
                        data.sensordata[58:87], data.sensor('imu_quat').data,
                        inertial['gyro'], inertial['acc'])
                    if sample is not None:
                        feet_samples.append(sample)
                    next_feet = float(data.time) + .02
                mujoco.mj_step(model, data)
                if config.get('navigation'):
                    sample = {
                        # mj_step's sensors describe the beginning of this step.
                        # Preserve every physics sample (500 Hz): downsampling
                        # foot-impact accelerations aliases them into false drift.
                        'time': float(data.time - robot['timestep']),
                        **{name: data.sensor('secondary_imu_' + name).data.tolist()
                           for name in ('gyro', 'acc')},
                    }
                    navigation_imu.append(effects.imu(sample) if effects is not None else sample)
                # mj_step leaves Cartesian poses at the previous integration step.
                if (data.time >= next_telemetry
                        or (config.get('navigation') and data.time >= next_scan)
                        or ('camera' in workers and data.time >= next_camera)):
                    mujoco.mj_forward(model, data)
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
                camera_due = 'camera' in workers and data.time >= next_camera
                lidar_due = 'lidar' in workers and data.time >= next_scan
                if camera_due or lidar_due:
                    mujoco.mj_getState(model, data, sensor_state, state_spec)
                    if camera_due and workers['camera'].submit(float(data.time), sensor_state):
                        next_camera = data.time + 1 / camera['fps']
                    if lidar_due and workers['lidar'].submit(float(data.time), sensor_state):
                        lidar = config['navigation'].get('lidar', {})
                        next_scan = data.time + (1 / lidar['fps'] if lidar.get('enabled') else 0.1)
                for kind, frame in received_frames.items():
                    if frame is None:
                        continue
                    if kind == 'camera':
                        rendered_frames += 1
                        camera_timestamp = frame['time']
                    else:
                        scan = frame['scan']
                        scan_time = frame['time']
                    if frame['sensors']:
                        sensor_frame = sensor_frame or {}
                        sensor_frame.update(frame['sensors'])
                        sensor_times.update({name: frame['time'] for name in frame['sensors']})
                        sensor_timestamp = frame['time']
                if data.time >= next_telemetry:
                    qw, qx, qy, qz = data.qpos[3:7]
                    base_yaw = np.arctan2(
                        2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)
                    )
                    cy, sy = np.cos(base_yaw), np.sin(base_yaw)
                    base_rotation = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
                    torso_rotation = base_rotation.T @ data.xmat[torso_id].reshape(3, 3)
                    torso_quaternion = np.empty(4)
                    mujoco.mju_mat2Quat(torso_quaternion, torso_rotation.ravel())
                    torso_position = base_rotation.T @ (
                        data.xpos[torso_id] - data.qpos[:3]
                    )
                    pipe.send(
                        {
                            'time': float(data.time),
                            'imu': {name: data.sensor('imu_' + name).data.tolist()
                                    for name in ('quat', 'gyro', 'acc')},
                            'torso_imu': navigation_imu.copy(),
                            'feet_estimation': feet_samples.copy(),
                            'scan': scan,
                            'scan_time': scan_time,
                            'position': data.qpos[:3].tolist(),
                            'quaternion': data.qpos[3:7].tolist(),
                            'linear_velocity': data.qvel[:3].tolist(),
                            'joint_names': joint_names,
                            'joint_positions': data.qpos[joint_qpos_addresses].tolist(),
                            'torso_position_base': torso_position.tolist(),
                            'torso_quaternion_base': torso_quaternion.tolist(),
                            'steps': steps,
                            'steps_per_second': steps / (time.monotonic() - started),
                            'simulator_cpu_percent': 100
                            * (time.process_time() - cpu_started)
                            / (time.monotonic() - started),
                            'simulator_peak_ram_bytes': resource.getrusage(
                                resource.RUSAGE_SELF
                            ).ru_maxrss
                            * 1024,
                            'camera_renderer': workers['camera'].info if 'camera' in workers else None,
                            'render_fps': rendered_frames / (time.monotonic() - started),
                            'collisions': collisions,
                            'watchdog_events': bridge.watchdog_events,
                            'watchdog_active': bridge.timed_out,
                            'damping_active': bridge.latest is not None
                            and all(m.kp == 0 and m.kd == 2 for m in bridge.latest.motor_cmd[:29]),
                            'invalid_commands': bridge.invalid_commands,
                            'sensors': sensor_frame,
                            'sensor_time': sensor_timestamp,
                            'sensor_times': sensor_times.copy(),
                            'camera_frame_id': rendered_frames,
                            'camera_time': camera_timestamp,
                        }
                    )
                    # Navigation forwards a sensor frame immediately when it
                    # arrives. Do not pickle and copy the same RGB-D arrays in
                    # every 50 Hz telemetry message until the next 10 Hz frame.
                    if config.get('navigation'):
                        sensor_frame = None
                        sensor_times = {}
                    next_telemetry = data.time + 0.02
                    navigation_imu.clear()
                    feet_samples.clear()
            if viewer is not None and time.monotonic() >= next_viewer:
                viewer.sync()
                next_viewer = time.monotonic() + 1 / 30
            time.sleep(max(0, robot['timestep'] - (time.monotonic() - before)))
    except BaseException:  # noqa: BLE001 - forward child failures to parent
        pipe.send({'error': traceback.format_exc()})
    finally:
        if grippers is not None:
            grippers.close()
        if viewer:
            viewer.close()
        for worker in workers.values():
            worker.close()
        pipe.close()
        # Official bridge recurrent threads are daemon threads; process owns their lifecycle.
