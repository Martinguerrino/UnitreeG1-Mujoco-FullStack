"""Execute original Okura ACT in a fixed-base Dex1 scene over local SDK2/DDS.

This is an experimental grasp evaluation, not a calibrated reproduction of Okura.
Run in a fresh process: upstream DDS and recurrent threads are process-global.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import importlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import time
import types

import numpy as np

from g1_backend.config import ROOT, runtime_path
from g1_backend.contracts import JointCommand
from g1_backend.dex1 import GripperDds, GripperPlant, motor_to_slide
from g1_backend.transport import DdsRobot
from .checkpoint import DEFAULT_CHECKPOINT, STATE_NAMES
from .scene import make_scene, set_initial_arm_pose
from .timing import ChunkTimeline


def official_bridge_class():
    path = ROOT / 'artifacts/unitree_mujoco/unitree_sdk2py_bridge.py'
    entry = json.loads((ROOT / 'configs/sources.lock.json').read_text())['files'][
        str(path.relative_to(ROOT))
    ]
    if hashlib.sha256(path.read_bytes()).hexdigest() != entry['sha256']:
        raise ValueError('Official bridge checksum mismatch')
    config = types.ModuleType('config')
    config.ROBOT = 'g1'
    sys.modules['config'] = config
    spec = importlib.util.spec_from_file_location('okura_official_bridge', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.UnitreeSdk2Bridge


def run(args):
    os.environ.setdefault('MUJOCO_GL', 'egl')
    runtime_path()
    import mujoco
    from PIL import Image
    from safetensors.numpy import load_file
    from unitree_sdk2py.utils.crc import CRC

    output = args.output
    output.mkdir(parents=True, exist_ok=False)
    source_files = [
        'experiments/okura/control.py',
        'experiments/okura/infer.py',
        'experiments/okura/scene.py',
        'experiments/okura/timing.py',
        'experiments/okura/command_pump.py',
        'experiments/okura/observer.py',
        'g1_backend/dex1.py',
        'experiments/okura/transfer.py',
        'g1_backend/skills/transfer.py',
        'configs/dex1.sources.json',
        'configs/sources.lock.json',
        'src/g1_mujoco/mjcf/g1_29dof.xml',
    ]
    (output / 'provenance.json').write_text(
        json.dumps(
            {
                'arguments': {
                    k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()
                },
                'source_sha256': {
                    p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in source_files
                },
                'dex1_assets': json.loads((ROOT / 'configs/dex1.sources.json').read_text()),
            },
            indent=2,
        )
        + '\n'
    )
    report = {
        'completed': False,
        'physical_grasp_tested': False,
        'grasp_success': False,
        'mode': args.mode,
        'camera_calibrated': False,
        'base_fixed': True,
        'object_attached': False,
        'error': None,
    }
    (output / 'report.json').write_text(json.dumps(report, indent=2))
    stats = load_file(
        str(args.checkpoint / 'policy_preprocessor_step_3_normalizer_processor.safetensors')
    )
    initial = stats['observation.state.mean'].copy()
    transfer = None
    config = None
    if args.mode in ('transfer', 'pick'):
        from .transfer import TransferController, load_transfer_config

        config = load_transfer_config(args.transfer_config)
        (output / 'transfer_config.json').write_text(json.dumps(config, indent=2) + '\n')
        initial[7:14] = config['initial_right_state'][:7]
        initial[15] = config['initial_right_state'][7]
        object_position = config['object_position']
        model, data = make_scene(
            dex1=True, box_position=object_position, object_kind='okra',
            transfer_scene=args.mode == 'transfer' or config.get('demonstration_fixture', False),
            camera_pitch=args.camera_pitch, camera_position=args.camera_position,
            camera_fovy=args.camera_fovy, basket_scale=args.basket_scale,
            basket_position=config.get("basket_position"), visual_style=args.visual_style
        )
        if args.mode == 'transfer':
            if args.placement_speed_window:
                from .observer import WindowedTransferController

                transfer = WindowedTransferController(
                    model, data, config, window=args.placement_speed_window
                )
            else:
                transfer = TransferController(model, data, config)
            transfer.initialize_body()
        else:
            from g1_backend.policies.locomotion import UnitreeVelocityPolicy
            import yaml

            body_q = UnitreeVelocityPolicy(
                yaml.safe_load((ROOT / 'configs/locomotion.yaml').read_text())
            ).get_action().q
            for i in range(15):
                joint = model.actuator_trnid[i, 0]
                data.qpos[model.jnt_qposadr[joint]] = body_q[i]
    else:
        object_position = [args.box_x, args.box_y, 0.755]
        model, data = make_scene(dex1=True, box_position=object_position, camera_pitch=args.camera_pitch)
    # The unchanged official bridge describes only body motors. Its sensor indices
    # match the first 29 actuators / original sensors in the extended physical model.
    body_model, _ = make_scene()
    assert body_model.nu == 29 and model.nu == 33
    assert np.array_equal(body_model.sensor_adr, model.sensor_adr)
    assert [body_model.actuator(i).name for i in range(29)] == [
        model.actuator(i).name for i in range(29)
    ]
    plant = GripperPlant(model)
    if args.reset_pose == 'mean':
        initial = stats['observation.state.mean'].copy()
    elif args.reset_pose == 'park-left':
        from g1_backend.policies.locomotion import UnitreeVelocityPolicy
        import yaml

        initial[:7] = UnitreeVelocityPolicy(
            yaml.safe_load((ROOT / 'configs/locomotion.yaml').read_text())
        ).get_action().q[15:22]
    set_initial_arm_pose(model, data, initial)
    data.qpos[plant.qadr] = np.repeat(motor_to_slide(initial[-2:]), 2)
    mujoco.mj_forward(model, data)
    qadr = model.jnt_qposadr[model.actuator_trnid[:29, 0]]
    limits = model.jnt_range[model.actuator_trnid[:29, 0]]
    arm_indices = np.array(
        [
            next(i for i in range(29) if model.joint(model.actuator_trnid[i, 0]).name == name)
            for name in STATE_NAMES[:14]
        ]
    )
    target = data.qpos[qadr].copy()
    gripper_target = initial[-2:].copy()
    kp, kd = np.full(29, 80.0), np.full(29, 4.0)
    for i in arm_indices:
        if 'wrist' in model.joint(model.actuator_trnid[i, 0]).name:
            kp[i], kd[i] = 20.0, 2.0
    kp[arm_indices] *= args.arm_gain_scale
    kd[arm_indices] *= np.sqrt(args.arm_gain_scale)
    kp[12:15] *= args.waist_gain_scale
    kd[12:15] *= np.sqrt(args.waist_gain_scale)
    renderer = mujoco.Renderer(model, 480, 640)
    renderer.update_scene(data, camera='cam_left_high')
    image = renderer.render().copy()
    image_time = float(data.time)
    Image.fromarray(image).save(output / 'initial_camera.png')
    policy = None
    if args.mode in ('act', 'transfer', 'pick'):
        module, factory = args.policy_factory.split(':', 1)
        policy = getattr(importlib.import_module(module), factory)(args.checkpoint, threads=2)
        report['policy_factory'] = args.policy_factory
        report['checkpoint'] = {
            k: v
            for k, v in getattr(policy, 'contract', {}).items()
            if k != 'physical_grasp_tested'
        }
    robot = DdsRobot({'domain_id': args.domain, 'interface': 'lo', 'state_timeout': 0.5}, limits)
    mutex = threading.RLock()
    crc = CRC()
    upstream = official_bridge_class()

    class Bridge(upstream):
        def __init__(self):
            self.latest, self.received = None, 0.0
            self.invalid, self.watchdog = 0, 0
            self.stale = False
            self.have_imu_ = self.have_frame_sensor_ = False
            super().__init__(body_model, data)

        def LowCmdHandler(self, msg):
            with mutex:
                values = [[m.q, m.dq, m.tau, m.kp, m.kd] for m in msg.motor_cmd[:29]]
                if msg.mode_pr != 0 or not np.isfinite(values).all() or msg.crc != crc.Crc(msg):
                    self.invalid += 1
                    return
                self.latest, self.received = msg, time.monotonic()

        def PublishLowState(self):
            with mutex:
                super().PublishLowState()

        def PublishHighState(self):
            with mutex:
                super().PublishHighState()

        def advance(self):
            stale = time.monotonic() - self.received > 0.25
            if stale:
                if not self.stale and self.latest is not None:
                    self.watchdog += 1
                data.ctrl[:29] = -2 * data.sensordata[29:58]
            elif self.latest is not None:
                super().LowCmdHandler(self.latest)
            self.stale = stale

    bridge = Bridge()
    server = GripperDds(server=True)
    hands = GripperDds()
    state_stop = threading.Event()

    def publish_gripper_state():
        # Publish the measured state independently of rendering and CPU inference.
        while not state_stop.is_set():
            with mutex:
                server.send(plant.encoder_state(data))
            state_stop.wait(0.002)

    state_thread = threading.Thread(target=publish_gripper_state, daemon=True)
    state_thread.start()
    pump = None
    if args.async_commands:
        from .command_pump import CommandPump

        pump = CommandPump(robot.send, hands.send)
    pool = ThreadPoolExecutor(max_workers=1)
    future, chunk, chunk_index = None, None, 0
    timeline = ChunkTimeline(ensemble=args.action_timing == 'ensemble')
    timing_rows = []
    speed_clips = 0
    chunk_observed_at = None
    observation_time = None
    observation_image_time = None
    frames, rows, chunks, observations, inference_times = [], [], [], [], []
    next_frame, next_action, next_inference = 0.0, 0.0, 1.0
    started = time.monotonic()
    command_count, clipped, finger_stale = 0, 0, 0
    lift_hold, max_lift_hold = 0.0, 0.0
    max_contacts, max_lift = 0, 0.0
    box_id = model.body('box').id
    box_geom = model.geom('box_collision').id
    initial_box_z = float(data.xpos[box_id, 2])
    initial_q = data.qpos[qadr].copy()
    mechanical = {}
    error = None
    try:
        # Discover both state and command topics before starting the physical episode.
        deadline = time.monotonic() + 5
        while True:
            server.send(plant.encoder_state(data))
            hands.send(gripper_target)
            try:
                robot.read()
                hands.read()
                server.read()
                break
            except TimeoutError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(0.01)
        if pump is not None:
            pump.update(JointCommand(target.copy(), kp.copy(), kd.copy()), gripper_target)
            pump.start()
        while data.time < args.duration:
            if pump is not None:
                pump.check()
            tick = time.monotonic()
            state = robot.read()
            grip_state = hands.read()
            if args.mode == 'mechanical':
                gripper_target = np.full(
                    2, 5.4 if data.time < 2 else 0.0 if data.time < 4 else 5.4
                )
                for label, t in [('open', 1.9), ('closed', 3.9), ('reopened', 5.9)]:
                    if data.time >= t and label not in mechanical:
                        mechanical[label] = grip_state.tolist()
            act_enabled = transfer is None or transfer.task.act_enabled
            if policy and act_enabled and future is None and data.time >= next_inference:
                observation = np.concatenate((state.q[arm_indices], grip_state)).astype(np.float32)
                observations.append(observation)
                np.savez_compressed(
                    output / f'input_{len(observations):03d}.npz',
                    state=observation,
                    image=image,
                    time=float(data.time),
                    image_time=image_time,
                )
                observation_time = float(data.time)
                observation_image_time = image_time
                inference_start = time.monotonic()
                future = pool.submit(policy.chunk, observation, image.copy())
                next_inference = float('inf')
            if future is not None and future.done():
                chunk = future.result()
                if chunk.shape != (100, 16) or not np.isfinite(chunk).all():
                    raise ValueError('Policy must return a finite 100x16 action chunk')
                inference_times.append(time.monotonic() - inference_start)
                chunks.append(chunk)
                chunk_observed_at = observation_time
                timeline.add(chunk, observation_time)
                timing_rows.append({
                    'observed_sim_time': observation_time,
                    'image_sim_time': observation_image_time,
                    'applied_sim_time': float(data.time),
                    'host_inference_seconds': inference_times[-1],
                    'stale_action_steps': int((data.time - observation_time) * 30),
                })
                future, chunk_index = None, 0
                next_inference = data.time + args.execute_steps / 30.0
            if data.time >= next_action:
                action = None
                if act_enabled and chunk is not None:
                    if args.action_timing == 'legacy' and chunk_index < len(chunk):
                        action = chunk[chunk_index]
                    elif args.action_timing != 'legacy':
                        action = timeline.action(float(data.time))
                if action is not None:
                    raw = action[:14]
                    bounded = np.clip(raw, limits[arm_indices, 0], limits[arm_indices, 1])
                    clipped += int(np.count_nonzero(raw != bounded))
                    # Bounded transition from the reset pose; original ACT output is logged.
                    delta = bounded - target[arm_indices]
                    step_limit = args.arm_speed / 30.0
                    speed_clips += int(np.count_nonzero(np.abs(delta) > step_limit))
                    target[arm_indices] += np.clip(delta, -step_limit, step_limit)
                    gripper_target = np.clip(action[-2:], 0, 5.4)
                    chunk_index += 1
                    command_count += 1
                next_action += 1 / 30.0
            outage = args.test_watchdog and 6.2 <= data.time < 6.7
            command = JointCommand(target.copy(), kp.copy(), kd.copy())
            if transfer:
                transfer.update(state, grip_state)
                if transfer.task.finished:
                    break
                command, gripper_target = transfer.compose(state, command, gripper_target)
            if pump is not None:
                pump.update(command, gripper_target, enabled=not outage)
            elif not outage:
                robot.send(command)
                hands.send(gripper_target)
            with mutex:
                bridge.advance()
                try:
                    plant.advance(data, server.read())
                except TimeoutError:
                    finger_stale += 1
                    plant.advance(data, None)
                mujoco.mj_step(model, data)
                server.send(plant.encoder_state(data))
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                raise ValueError('Nonfinite physics state')
            contacts = set()
            for contact in data.contact:
                if box_geom in (contact.geom1, contact.geom2):
                    other = contact.geom2 if contact.geom1 == box_geom else contact.geom1
                    name = model.body(model.geom_bodyid[other]).name
                    if 'dex1_finger' in name:
                        contacts.add(name)
            bilateral = any(
                all(f'{s}_dex1_finger_link_{i}' in contacts for i in (1, 2))
                for s in ('left', 'right')
            )
            lifted = data.xpos[box_id, 2] > initial_box_z + 0.05
            max_contacts = max(max_contacts, len(contacts))
            max_lift = max(max_lift, float(data.xpos[box_id, 2] - initial_box_z))
            lift_hold = lift_hold + 0.002 if bilateral and lifted else 0.0
            max_lift_hold = max(max_lift_hold, lift_hold)
            if data.time >= next_frame:
                # Rendering doesn't mutate physics. DDS publisher can continue during rasterization.
                renderer.update_scene(data, camera='cam_left_high')
                image = renderer.render().copy()
                image_time = float(data.time)
                frames.append(image)
                rows.append(
                    {
                        'time': float(data.time),
                        'q': state.q.copy(),
                        'target': command.q.copy(),
                        'gripper': grip_state.copy(),
                        'gripper_target': gripper_target.copy(),
                        'box': data.xpos[box_id].copy(),
                        'finger_contacts': len(contacts),
                        'bilateral': bilateral,
                        'phase': transfer.task.phase if transfer else args.mode,
                        'base_position': data.qpos[:3].copy(),
                        'finger_slide': data.qpos[plant.qadr].copy(),
                        'physics_qpos': data.qpos.copy(),
                        'actuator_force': data.actuator_force.copy(),
                        'image_age_sim_s': float(data.time - image_time),
                        'chunk_age_sim_s': float(data.time - chunk_observed_at)
                        if chunk_observed_at is not None else -1.0,
                        'right_finger_midpoint': np.mean([
                            data.xpos[model.body(f'right_dex1_finger_link_{i}').id]
                            for i in (1, 2)
                        ], axis=0),
                    }
                )
                next_frame += 0.1
            time.sleep(max(0.0, 0.002 - (time.monotonic() - tick)))
        if transfer and not transfer.task.finished:
            transfer.task.fail(float(data.time), 'episode_timeout')
        report['completed'] = True
    except Exception as exc:  # noqa: BLE001 - save evidence, then re-raise
        error = exc
        report['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        if transfer:
            if error and not transfer.task.finished:
                transfer.task.fail(float(data.time), str(error))
            report['transfer'] = transfer.report()
            report['base_fixed'] = not transfer.released_fixture
            (output / 'task_events.json').write_text(
                json.dumps(transfer.task.events, indent=2) + '\n'
            )
        if pump is not None:
            pump.close()
        robot.stop()
        pool.shutdown(wait=True, cancel_futures=True)
        renderer.close()
        # Stop upstream publishers before closing transport; avoid background use of model/data.
        for thread in (
            bridge.lowStateThread,
            bridge.HighStateThread,
            bridge.WirelessControllerThread,
        ):
            thread.Wait()
        bridge.low_cmd_suber.Close()
        for pub in (
            bridge.low_state_puber,
            bridge.high_state_puber,
            bridge.wireless_controller_puber,
        ):
            pub.Close()
        state_stop.set()
        state_thread.join()
        hands.close()
        server.close()
        robot.close()
        if rows:
            np.savez_compressed(
                output / 'trajectory.npz', **{k: np.array([r[k] for r in rows]) for k in rows[0]}
            )
        if chunks:
            np.savez_compressed(
                output / 'policy.npz',
                chunks=np.array(chunks),
                observations=np.array(observations[: len(chunks)]),
            )
        if frames:
            Image.fromarray(frames[-1]).save(output / 'final_camera.png')
            import av

            with av.open(str(output / 'camera.mp4'), 'w') as container:
                stream = container.add_stream('libx264', rate=10)
                stream.width, stream.height, stream.pix_fmt = 640, 480, 'yuv420p'
                for frame in frames:
                    for packet in stream.encode(av.VideoFrame.from_ndarray(frame, format='rgb24')):
                        container.mux(packet)
                for packet in stream.encode():
                    container.mux(packet)
        (output / 'inference_timing.json').write_text(json.dumps(timing_rows, indent=2) + '\n')
        report['watchdog_test_requested'] = args.test_watchdog
        report['watchdog_test_passed'] = (
            args.test_watchdog and bridge.watchdog > 0 and finger_stale > 0
        )
        report.update(
            {
                'physical_grasp_tested': args.mode in ('act', 'transfer', 'pick') and command_count > 0,
                'grasp_success': max_lift_hold >= 0.5,
                'max_lift_with_two_finger_contact_seconds': max_lift_hold,
                'max_object_lift_m': max_lift,
                'max_finger_contacts': max_contacts,
                'act_actions_executed': command_count,
                'chunks': len(chunks),
                'inference_seconds': inference_times,
                'arm_limit_clips': clipped,
                'body_watchdog_events': bridge.watchdog,
                'finger_stale_steps': finger_stale,
                'invalid_body_commands': bridge.invalid,
                'invalid_hand_commands': server.invalid,
                'mechanical_positions_motor_rad': mechanical,
                'mechanical_passed': args.mode == 'mechanical'
                and all(
                    np.max(np.abs(np.array(mechanical.get(k, [999, 999])) - v)) < 0.15
                    for k, v in [('open', 5.4), ('closed', 0), ('reopened', 5.4)]
                ),
                'max_arm_motion_rad': max(
                    (
                        float(np.max(np.abs(r['q'][arm_indices] - initial_q[arm_indices])))
                        for r in rows
                    ),
                    default=0,
                ),
                'sim_seconds': float(data.time),
                'wall_seconds': time.monotonic() - started,
                'box_initial_position': object_position,
                'object_kind': 'okra' if config else 'box',
                'action_rate_hz': 30,
                'execute_steps_per_chunk': args.execute_steps,
                'arm_target_speed_limit_rad_s': args.arm_speed,
                'arm_gain_scale': args.arm_gain_scale,
                'waist_gain_scale': args.waist_gain_scale,
                'action_timing': args.action_timing,
                'speed_limit_clips': speed_clips,
                'reset_pose': args.reset_pose,
                'camera_pitch': args.camera_pitch,
                'camera_position': args.camera_position,
                'camera_fovy': args.camera_fovy,
                'basket_scale': args.basket_scale,
                'async_commands': args.async_commands,
                'visual_style': args.visual_style,
                'placement_speed_window': args.placement_speed_window,
                'encoder_slide_resolution_m': 0.001,
                'note': 'Local Dex1 MJCF from official URDF. Camera/table/object are uncalibrated; no training or object attachment.',
            }
        )
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
    if error:
        raise error
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['mechanical', 'act', 'transfer', 'pick'], default='act')
    parser.add_argument('--duration', type=float)
    parser.add_argument(
        '--transfer-config', type=Path, default=ROOT / 'configs/okura_transfer.yaml'
    )
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=43)
    parser.add_argument('--camera-pitch', type=float, help='Downward camera tilt in degrees')
    parser.add_argument('--camera-position', nargs=3, type=float)
    parser.add_argument('--camera-fovy', type=float)
    parser.add_argument('--basket-scale', type=float, default=1.0)
    parser.add_argument('--reset-pose', choices=['default', 'mean', 'park-left'], default='default')
    parser.add_argument('--arm-gain-scale', type=float, default=1.0)
    parser.add_argument('--waist-gain-scale', type=float, default=1.0)
    parser.add_argument('--arm-speed', type=float, default=1.5)
    parser.add_argument('--action-timing', choices=['legacy', 'compensate', 'ensemble'], default='legacy')
    parser.add_argument('--visual-style', choices=['legacy', 'reference'], default='legacy')
    parser.add_argument('--async-commands', action='store_true')
    parser.add_argument('--placement-speed-window', type=float, default=0.0)
    parser.add_argument('--box-x', type=float, default=0.4)
    parser.add_argument('--box-y', type=float, default=-0.15)
    parser.add_argument('--execute-steps', type=int, default=100)
    parser.add_argument('--checkpoint', type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument('--policy-factory', default='experiments.okura.infer:OkuraPolicy')
    parser.add_argument(
        '--test-watchdog', action='store_true', help='Inject command loss at 6.2..6.7 seconds'
    )
    args = parser.parse_args()
    if args.duration is None:
        args.duration = 90.0 if args.mode == 'transfer' else 12.0
    if (
        not 1 <= args.domain <= 232
        or not 1 <= args.execute_steps <= 100
        or (args.camera_pitch is not None and (not np.isfinite(args.camera_pitch)
            or not 0 < args.camera_pitch < 90))
        or not np.isfinite(args.duration)
        or args.duration <= 0
    ):
        parser.error('Invalid domain, duration or execute-steps')
    for name in ('arm_gain_scale', 'waist_gain_scale', 'arm_speed', 'basket_scale'):
        value = getattr(args, name)
        if not np.isfinite(value) or value <= 0:
            parser.error(f'{name} must be positive and finite')
    if args.camera_position is not None and not np.isfinite(args.camera_position).all():
        parser.error('Camera position must be finite')
    if args.camera_fovy is not None and not 0 < args.camera_fovy < 180:
        parser.error('Camera fovy must be in (0, 180)')
    if not np.isfinite(args.placement_speed_window) or not 0 <= args.placement_speed_window <= .5:
        parser.error('placement-speed-window must be in [0, 0.5] seconds')
    run(args)


if __name__ == '__main__':
    main()
