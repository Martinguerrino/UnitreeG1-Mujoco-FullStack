"""Physical downstream diagnostics seeded with a prepared grasp, never ACT success.

The object is positioned once at reset, closed on with gravity disabled, then
held only by contacts under normal gravity. Each phase uses an independent fixture.
"""

import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from safetensors.numpy import load_file

from g1_backend.config import ROOT
from g1_backend.contracts import JointCommand, RobotState
from g1_backend.dex1 import GripperPlant, motor_to_slide
from .checkpoint import DEFAULT_CHECKPOINT, STATE_NAMES
from .scene import make_scene, set_initial_arm_pose
from .transfer import TransferController, load_transfer_config


def scene_at_base(position):
    model, _ = make_scene(dex1=True, object_kind='okra', transfer_scene=True)
    with tempfile.TemporaryDirectory(prefix='okura_phase_') as directory:
        path = Path(directory) / 'scene.xml'
        mujoco.mj_saveLastXML(str(path), model)
        root = ET.parse(path).getroot()
        root.find(".//body[@name='pelvis']").set('pos', ' '.join(map(str, position)))
        equality = root.find('equality')
        equality.clear()
        ET.SubElement(equality, 'weld', name='pelvis_fixture', body1='pelvis')
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    return model, mujoco.MjData(model)


def run(phase, output, duration=45, placement_height=1.0, timestep=.002, speed_window=0.):
    output.mkdir(parents=True, exist_ok=False)
    is_placement = phase in ('lower', 'release')
    model, data = scene_at_base([0, 1.2, .793] if is_placement else [-.3, 0, .793])
    model.opt.timestep = timestep
    mujoco.mj_forward(model, data)
    config = load_transfer_config(ROOT / 'configs/okura_transfer.yaml')
    if speed_window:
        from .observer import WindowedTransferController

        controller = WindowedTransferController(model, data, config, window=speed_window)
    else:
        controller = TransferController(model, data, config)
    controller.initialize_body()
    initial = load_file(str(DEFAULT_CHECKPOINT / 'policy_preprocessor_step_3_normalizer_processor.safetensors'))['observation.state.mean'].copy()
    set_initial_arm_pose(model, data, initial)
    qadr = model.jnt_qposadr[model.actuator_trnid[:29, 0]]
    vadr = model.jnt_dofadr[model.actuator_trnid[:29, 0]]
    plant = GripperPlant(model)
    data.qpos[plant.qadr] = motor_to_slide(5.4)
    wrist = model.body('right_wrist_yaw_link').id
    if is_placement:
        from scipy.optimize import least_squares

        arm_qadr = np.array([model.joint(name).qposadr[0] for name in STATE_NAMES[7:14]])
        start = data.qpos[arm_qadr].copy()
        mujoco.mj_forward(model, data)
        orientation = data.xmat[wrist].reshape(3, 3)[:, 0].copy()
        desired = np.array([.55, 1.2, placement_height])

        def residual(q):
            data.qpos[arm_qadr] = q
            mujoco.mj_forward(model, data)
            rotation = data.xmat[wrist].reshape(3, 3)
            center = data.xpos[wrist] + rotation @ np.array([.16, 0, 0])
            return np.r_[center - desired, .005 * (q - start), .02 * (rotation[:, 0] - orientation)]

        fit = least_squares(residual, start,
                            bounds=(controller.arm_limits[:, 0], controller.arm_limits[:, 1]))
        data.qpos[arm_qadr] = fit.x
    mujoco.mj_forward(model, data)
    rotation = data.xmat[wrist].reshape(3, 3).copy()
    object_position = data.xpos[wrist] + rotation @ np.array([.16, 0, 0])
    obj_adr = model.joint('box_free').qposadr[0]
    data.qpos[obj_adr:obj_adr + 3] = object_position
    mujoco.mju_mat2Quat(data.qpos[obj_adr + 3:obj_adr + 7], rotation.ravel())
    mujoco.mj_forward(model, data)
    controller.initial_z = float(data.xpos[controller.body, 2])
    target = data.qpos[qadr].copy()
    kp, kd = np.full(29, 80.), np.full(29, 4.)
    kp[19:22] = kp[26:29] = 20
    kd[19:22] = kd[26:29] = 2
    grip = np.zeros(2)
    renderer = mujoco.Renderer(model, 480, 640)
    frames, rows = [], []
    next_frame = 0.
    expected = {'hold': 'stand', 'stand': 'carry', 'carry': 'lower',
                'lower': 'release', 'release': 'done'}[phase]
    report = {'phase_tested': phase, 'expected_next_phase': expected, 'phase_passed': False,
              'act_executed': False, 'prepared_grasp_fixture': True, 'object_attached': False,
              'pelvis_fixed_during_test': phase in ('hold', 'lower', 'release'),
              'preparation_gravity_disabled_seconds': 1.0, 'completed': False,
              'error': None, 'physical_preparation_valid': False,
              'initial_object_position': object_position.tolist(),
              'requested_placement_fixture_height': placement_height, 'timestep': timestep,
              'placement_speed_window': speed_window}
    (output / 'provenance.json').write_text(json.dumps({
        'source_sha256': {str(path): hashlib.sha256((ROOT / path).read_bytes()).hexdigest()
                          for path in ('experiments/okura/phase_probe.py',
                                       'experiments/okura/transfer.py',
                                       'experiments/okura/scene.py',
                                       'g1_backend/skills/transfer.py', 'g1_backend/dex1.py',
                                       'experiments/okura/observer.py')},
        'configuration': config, 'diagnostic_phase': phase,
    }, indent=2) + '\n')

    def state():
        imu = model.sensor('imu_quat').adr[0]
        gyro = model.sensor('imu_gyro').adr[0]
        return RobotState(data.qpos[qadr].copy(), data.qvel[vadr].copy(),
                          data.sensordata[imu:imu + 4].copy(),
                          data.sensordata[gyro:gyro + 3].copy(), time.monotonic())

    def apply(command, grippers):
        force = command.kp * (command.q - data.qpos[qadr]) - command.kd * data.qvel[vadr]
        data.ctrl[:29] = np.clip(force, model.actuator_ctrlrange[:29, 0],
                                model.actuator_ctrlrange[:29, 1])
        plant.advance(data, grippers)
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)

    def record():
        nonlocal next_frame
        if data.time >= next_frame:
            renderer.update_scene(data, camera='cam_left_high')
            frames.append(renderer.render().copy())
            observed = controller.observe(state(), plant.encoder_state(data))
            rows.append({'time': float(data.time), 'q': data.qpos[qadr].copy(),
                         'physics_qpos': data.qpos.copy(),
                         'object_position': data.xpos[controller.body].copy(),
                         'gripper': plant.encoder_state(data),
                         'phase': controller.task.phase,
                         'bilateral': observed['bilateral'],
                         'table_contact': observed['table_contact'],
                         'hand_contact': observed['hand_contact'],
                         'base_position': data.qpos[:3].copy(),
                         'object_speed': observed['object_speed'],
                         'instantaneous_object_speed': float(np.linalg.norm(
                             data.qvel[controller.free_dof:controller.free_dof + 3])),
                         'over_table': observed['over_table'],
                         'gripper_open': observed['gripper_open']})
            next_frame += .1

    try:
        model.opt.gravity[:] = 0
        while data.time < 1.0:
            apply(JointCommand(target, kp, kd), grip)
            record()
        model.opt.gravity[:] = [0, 0, -9.81]
        stable = 0.
        preparation_deadline = float(data.time) + 2.
        while data.time < preparation_deadline and stable < .5:
            apply(JointCommand(target, kp, kd), grip)
            observed = controller.observe(state(), plant.encoder_state(data))
            stable = stable + model.opt.timestep if observed['bilateral'] else 0.
            record()
        report['physical_preparation_valid'] = stable >= .5
        if not report['physical_preparation_valid']:
            report['failure'] = 'preparation_did_not_retain_bilateral_grasp_under_gravity'
        else:
            controller.task.arm_hold = data.qpos[qadr][controller.arms].copy()
            controller.arm_target = controller.task.arm_hold.copy()
            controller.task._transition(phase, float(data.time), 'diagnostic_prepared_grasp')
            while data.time < duration and not controller.task.finished:
                s = state()
                controller.update(s, plant.encoder_state(data))
                if controller.task.phase == expected:
                    report['phase_passed'] = True
                    break
                if controller.task.finished:
                    break
                if phase in ('lower', 'release'):
                    # Fixed-pelvis isolation of the same placement controller and guards.
                    if controller.task.phase == 'lower':
                        controller._lower_arm()
                    command = JointCommand(target.copy(), kp, kd)
                    command.q[controller.arms] = controller.arm_target
                    grip[1] = controller.task.gripper_override
                else:
                    command, grip = controller.compose(s, JointCommand(target.copy(), kp, kd), grip)
                apply(command, grip)
                if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                    raise ValueError('Nonfinite physics')
                record()
            if not report['phase_passed']:
                report['failure'] = controller.task.failure or 'diagnostic_duration_timeout'
        report['completed'] = True
    except Exception as error:  # noqa: BLE001 - preserve failed diagnostic evidence
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        renderer.close()
        report['simulation_seconds'] = float(data.time)
        report['transfer'] = controller.report()
        if controller.last_observation is not None:
            report['last_observation'] = {k: v.tolist() if isinstance(v, np.ndarray) else v
                                          for k, v in controller.last_observation.items()}
        report['pelvis_fixture_active_at_end'] = bool(data.eq_active[controller.fixture])
        report['final_object_position'] = data.xpos[controller.body].tolist()
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        if rows:
            np.savez_compressed(output / 'trajectory.npz',
                                **{k: np.array([r[k] for r in rows]) for k in rows[0]})
        if frames:
            from PIL import Image
            import av

            Image.fromarray(frames[0]).save(output / 'initial_camera.png')
            Image.fromarray(frames[-1]).save(output / 'final_camera.png')
            with av.open(str(output / 'camera.mp4'), 'w') as video:
                stream = video.add_stream('libx264', rate=10)
                stream.width, stream.height, stream.pix_fmt = 640, 480, 'yuv420p'
                for frame in frames:
                    for packet in stream.encode(av.VideoFrame.from_ndarray(frame, format='rgb24')):
                        video.mux(packet)
                for packet in stream.encode():
                    video.mux(packet)
        print(json.dumps(report, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase', choices=['hold', 'stand', 'carry', 'lower', 'release'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--placement-height', type=float, default=1.0)
    parser.add_argument('--timestep', type=float, default=.002)
    parser.add_argument('--speed-window', type=float, default=0.0)
    args = parser.parse_args()
    if not np.isfinite(args.placement_height) or not .8 <= args.placement_height <= 1.1:
        parser.error('placement-height must be in [0.8, 1.1] meters')
    if not np.isfinite(args.timestep) or not .0005 <= args.timestep <= .002:
        parser.error('timestep must be in [0.0005, 0.002] seconds')
    if not np.isfinite(args.speed_window) or not 0 <= args.speed_window <= .5:
        parser.error('speed-window must be in [0, 0.5] seconds')
    run(args.phase, args.output, placement_height=args.placement_height,
        timestep=args.timestep, speed_window=args.speed_window)


if __name__ == '__main__':
    main()
