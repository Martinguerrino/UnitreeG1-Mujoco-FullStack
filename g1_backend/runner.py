"""Reproducible experiments with separate simulator and DDS controller."""

from datetime import datetime, timezone
import fcntl
import hashlib
import importlib.metadata
import json
import multiprocessing as mp
import platform
import resource
import subprocess
import time
import tempfile
import uuid

import numpy as np
import yaml

from .config import ROOT, load_config
from .policies import load_policy, PolicyUnavailable
from .policies.locomotion import projected_gravity
from .simulator import joint_limits, run_simulator
from .skills import GoTo, Manipulate, TaskExecutor
from .transport import DdsRobot
from .recording import SensorRecorder


def provenance(config):
    def git(*args):
        return subprocess.run(
            ['git', '-C', str(ROOT), *args], capture_output=True, text=True, check=False
        ).stdout.strip()

    versions = {}
    for package in ('mujoco', 'numpy', 'onnxruntime', 'cyclonedds', 'unitree_sdk2py'):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = None
    checkpoints = {}
    for name in ('locomotion', 'manipulation'):
        checkpoint = config[name].get('checkpoint')
        if checkpoint and (ROOT / checkpoint).is_file():
            checkpoints[name] = {
                'path': checkpoint,
                'sha256': hashlib.sha256((ROOT / checkpoint).read_bytes()).hexdigest(),
            }
    return {
        'schema_version': 1,
        'git_commit': git('rev-parse', 'HEAD'),
        'git_dirty': bool(git('status', '--porcelain')),
        'timestamp': datetime.now(timezone.utc).isoformat(),
        'seed': config['seed'],
        'versions': versions,
        'python': platform.python_version(),
        'checkpoints': checkpoints,
        'sources': json.loads((ROOT / 'configs/sources.lock.json').read_text()),
        'pose_source': 'simulator ground truth; navigation is not hardware localization',
        'source_sha256': {
            str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (ROOT / 'g1_backend').rglob('*.py')
        },
        'asset_sha256': {
            str(path.relative_to(ROOT) if path.is_relative_to(ROOT) else path):
                hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [
                ROOT / config['robot']['model'],
                ROOT / config['robot']['scene'],
                *(ROOT / config['robot']['meshes']).glob('*'),
            ]
            if path.is_file()
        },
    }


def run_experiment(source):
    config = load_config(source)
    run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S') + '_' + uuid.uuid4().hex[:8]
    directory = ROOT / config.get('results_dir', 'results') / run_id
    directory.mkdir(parents=True)
    (directory / 'config.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    metrics = {
        'schema_version': 1,
        'status': 'running',
        'success': False,
        'falls': 0,
        'collisions': None,
        'failed_skill': None,
        'retries': 0,
        'grasp_success': None,
        'lift_success': None,
        'place_success': None,
        'object_drops': None,
        'placement_error': None,
        'gpu_utilization': None,
        'vram_bytes': None,
        'simulation_fps': None,
        'render_fps': None,
    }
    process = robot = stream = None
    domain_lock = None
    stop = release = pipe = None
    start = time.monotonic()
    run_started = start
    control_started = False
    cpu_started = time.process_time()
    telemetry = None
    executor = None
    last_target = None
    target = None
    phase_durations = {}
    previous_phase = None
    phase_tick = None
    recorder = None
    navigation = None
    try:
        kind = config['kind']
        if kind not in ('locomotion', 'communication', 'task', 'manipulation', 'navigation'):
            raise ValueError(f'Unknown experiment kind: {kind}')
        if kind == 'navigation':
            from .navigation import NavigationTransport

            navigation = NavigationTransport(config['navigation'])
        task_steps = config['task']['steps'] if kind == 'task' else []
        needs_manipulation = kind == 'manipulation' or any(
            x['skill'] in ('pick', 'place') for x in task_steps
        )
        manipulation = load_policy(config['manipulation']) if needs_manipulation else None
        locomotion = load_policy(config['locomotion'])
        np.random.seed(config['seed'])
        limits = joint_limits(config)
        domain_lock = open(  # noqa: SIM115 - held through process cleanup in finally
            f'{tempfile.gettempdir()}/g1_backend_domain_{config["robot"]["domain_id"]}.lock', 'a'
        )
        try:
            fcntl.flock(domain_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError('Another local experiment already owns this DDS domain') from error
        robot = DdsRobot(config['robot'], limits)
        ctx = mp.get_context('spawn')
        pipe, child = ctx.Pipe(duplex=False)
        stop, release = ctx.Event(), ctx.Event()
        process = ctx.Process(
            target=run_simulator,
            args=(config, locomotion.get_action().q, child, stop, release),
            daemon=False,
        )
        process.start()
        child.close()
        if not pipe.poll(210):
            raise TimeoutError('Simulator did not start within 210 seconds')
        ready = pipe.recv()
        if 'error' in ready:
            raise RuntimeError(ready['error'])
        start = time.monotonic()
        control_started = True
        use_simulation_clock = config.get('viewer', {}).get('enabled', False) or navigation is not None
        next_infer = 0.0 if use_simulation_clock else start
        initial_state = None
        command = locomotion.get_action()
        warmup = config.get('warmup', 2.0)
        duration = config['duration']
        stand = config.get('stand_duration', 2.0)
        settling = config.get('stop_duration', 2.0)
        executor = (
            TaskExecutor(task_steps, config['task'].get('defaults')) if kind == 'task' else None
        )
        active = None
        target = GoTo(
            config.get('target', [0.5, 0, 0]),
            config.get('max_speed', 0.3),
            config.get('position_tolerance', 0.12),
            config.get('orientation_tolerance', 0.15),
        )
        stop_since = None
        stream = (directory / 'trajectory.jsonl').open('w')
        recorder = SensorRecorder(directory, config['camera'])
        policy_tick_count = 0
        policy_tick_seconds = 0.0
        physics_at_release = None
        watchdog_observed = damping_observed = False
        maximum_displacement = 0.0
        telemetry_received_at = time.monotonic()
        previous_phase = 'warmup'
        phase_tick = 0.0 if use_simulation_clock else start
        while True:
            tick = time.monotonic()
            while pipe.poll():
                incoming = pipe.recv()
                if 'error' in incoming:
                    raise RuntimeError(incoming['error'])
                if 'position' in incoming:
                    if navigation:
                        navigation.publish(incoming)
                    telemetry = incoming
                    telemetry_received_at = time.monotonic()
            elapsed = (
                telemetry['time'] if use_simulation_clock and telemetry else tick - start
            )
            if elapsed >= warmup + duration:
                break
            if time.monotonic() - telemetry_received_at > 2:
                raise TimeoutError('Simulator telemetry is stale')
            if not process.is_alive():
                raise RuntimeError(f'Simulator exited: {process.exitcode}')
            try:
                state = robot.read()
            except TimeoutError:
                if initial_state is None and elapsed < 5:
                    time.sleep(0.002)
                    continue
                raise
            if initial_state is None:
                initial_state = state
            if telemetry is None:
                time.sleep(0.002)
                continue
            if kind == 'communication':
                maximum_displacement = max(
                    maximum_displacement, float(state.q[19] - initial_state.q[19])
                )
                watchdog_observed |= telemetry['watchdog_active']
                damping_observed |= telemetry['damping_active']
            if elapsed >= warmup and kind != 'communication' and kind != 'manipulation':
                release.set()
                if physics_at_release is None:
                    physics_at_release = telemetry['time']
            if elapsed >= warmup and telemetry['position'][2] < 0.45:
                metrics['falls'] = 1
                raise RuntimeError('Fall detected: base height below 0.45 m')
            if projected_gravity(state.quaternion)[2] > -0.54:
                metrics['falls'] = 1
                raise RuntimeError('Fall detected: base tilt exceeds about 57 degrees')
            policy_clock = elapsed if use_simulation_clock else tick
            if policy_clock >= next_infer:
                before = time.monotonic()
                velocity = np.zeros(3)
                phase = 'warmup'
                if kind == 'communication':
                    # Small fixed-base wrist displacement proves command/measurement round trip.
                    command = locomotion.get_action()
                    command.q[19] += 0.08
                    phase = 'communication'
                elif elapsed < warmup:
                    command = locomotion.get_action()
                else:
                    phase = 'stand'
                    if kind == 'navigation':
                        requested = navigation.velocity(telemetry)
                        if elapsed >= warmup + stand:
                            velocity = requested
                            phase = 'navigation'
                    elif kind == 'task' and elapsed >= warmup + stand:
                        step = executor.current
                        if step is None:
                            metrics['policy_completed'] = True
                            metrics['success'] = None if needs_manipulation else True
                            break
                        if executor.check_deadline(elapsed):
                            active = None
                        if active is None:
                            active = (
                                GoTo(
                                    step['target'],
                                    step.get('max_speed', config.get('max_speed', 0.3)),
                                    step.get(
                                        'position_tolerance',
                                        config.get('position_tolerance', 0.12),
                                    ),
                                    step.get(
                                        'orientation_tolerance',
                                        config.get('orientation_tolerance', 0.15),
                                    ),
                                )
                                if step['skill'] == 'goto'
                                else Manipulate(manipulation, step)
                            )
                        phase = step['skill']
                        if isinstance(active, GoTo):
                            last_target = active
                            velocity = active.velocity(telemetry)
                            settled = active.is_done(telemetry) and np.linalg.norm(
                                telemetry['linear_velocity'][:2]
                            ) <= step.get('stop_speed_tolerance', 0.1)
                            if executor.ready_to_complete(settled, elapsed):
                                position_error, orientation_error = active.errors(telemetry)
                                executor.complete_skill(
                                    elapsed,
                                    position_error=position_error,
                                    orientation_error=orientation_error,
                                )
                                active = None
                        else:
                            command = active.tick(state, telemetry['sensors'])
                            if executor.ready_to_complete(manipulation.is_done(), elapsed):
                                executor.complete_skill(elapsed, physical_success=None)
                                active = None
                    elif kind == 'manipulation':
                        if active is None:
                            active = Manipulate(manipulation, {'skill': 'pick'})
                        command = active.tick(state, telemetry['sensors'])
                        phase = 'pick'
                        if manipulation.is_done():
                            metrics['policy_completed'] = True
                            metrics['success'] = None  # no object evaluator/scene configured
                            break
                    elif kind == 'locomotion' and elapsed >= warmup + stand:
                        if target.is_done(telemetry):
                            stop_since = stop_since or policy_clock
                            phase = 'stop'
                            if policy_clock - stop_since >= settling:
                                metrics['success'] = True
                                break
                        else:
                            stop_since = None
                            velocity = target.velocity(telemetry)
                            phase = 'walk'
                    if phase not in ('pick', 'place'):
                        locomotion.infer(locomotion.get_observation(state, velocity))
                        command = locomotion.get_action()
                policy_tick_count += 1
                phase_durations[previous_phase] = (
                    phase_durations.get(previous_phase, 0.0) + policy_clock - phase_tick
                )
                previous_phase, phase_tick = phase, policy_clock
                policy_tick_seconds += time.monotonic() - before
                next_infer = policy_clock + (
                    manipulation.period if phase in ('pick', 'place') else locomotion.period
                )
                row = {k: v for k, v in telemetry.items() if k != 'sensors'}
                row.update(
                    {
                        'wall_elapsed': elapsed,
                        'phase': phase,
                        'q': state.q.tolist(),
                        'dq': state.dq.tolist(),
                        'imu_quaternion': state.quaternion.tolist(),
                        'imu_gyroscope': state.gyroscope.tolist(),
                        'lowstate_age': time.monotonic() - state.received_at,
                        'target_q': command.q.tolist(),
                        'velocity_command': velocity.tolist(),
                    }
                )
                stream.write(json.dumps(row) + '\n')
                recorder.capture_telemetry(telemetry)
            if kind == 'communication' and warmup <= elapsed < warmup + 0.5:
                pass  # Deliberate publisher outage to exercise simulator watchdog.
            elif kind == 'communication' and warmup + 0.5 <= elapsed < warmup + 1.0:
                robot.stop()
            else:
                robot.send(command)
            time.sleep(max(0, config['robot']['timestep'] - (time.monotonic() - tick)))
        metrics.update(
            {
                'status': 'completed' if metrics['success'] else 'failed',
                'lowstate_samples': robot.count,
                'clipped_joint_targets': robot.clipped,
                'mean_policy_tick_seconds': policy_tick_seconds / max(policy_tick_count, 1),
                'control_elapsed': time.monotonic() - start,
                'phase_durations': phase_durations,
                'walking_time': phase_durations.get('walk', 0.0)
                + phase_durations.get('goto', 0.0),
                'manipulation_time': phase_durations.get('pick', 0.0)
                + phase_durations.get('place', 0.0),
            }
        )
        if kind == 'communication':
            difference = maximum_displacement
            metrics.update(
                {
                    'wrist_displacement': difference,
                    'success': 0.02 < difference < 0.15 and watchdog_observed and damping_observed,
                    'watchdog_observed': watchdog_observed,
                    'damping_observed': damping_observed,
                    'lowstate_hz': robot.count / (time.monotonic() - start),
                }
            )
            metrics['status'] = 'completed' if metrics['success'] else 'failed'
        if navigation:
            # Goal success is reported by Nav2 actions, not this velocity bridge.
            metrics.update(status='completed', success=None, reason='Navigation session duration elapsed')
        if executor and executor.current:
            metrics['failed_skill'] = executor.current['skill']
        if telemetry and (kind == 'locomotion' or last_target is not None):
            position_error, orientation_error = (last_target or target).errors(telemetry)
            metrics.update(
                {'final_position_error': position_error, 'orientation_error': orientation_error}
            )
        if not metrics['success'] and 'reason' not in metrics:
            metrics['reason'] = 'Episode ended without measured success'
    except PolicyUnavailable as error:
        metrics.update(status='blocked', reason=str(error), failed_skill='manipulation')
    except (Exception, KeyboardInterrupt) as error:  # noqa: BLE001 - persist failed experiment
        metrics.update(status='failed', reason=str(error) or 'Interrupted')
    finally:
        final_target = last_target or (target if config['kind'] == 'locomotion' else None)
        if telemetry is not None and final_target is not None:
            position_error, orientation_error = final_target.errors(telemetry)
            metrics.update(
                final_position_error=position_error, orientation_error=orientation_error
            )
        if control_started:
            metrics['control_elapsed'] = time.monotonic() - start
        if robot:
            metrics.update(lowstate_samples=robot.count, clipped_joint_targets=robot.clipped)
        if phase_tick is not None:
            final_clock = (
                telemetry['time']
                if (config.get('viewer', {}).get('enabled', False) or navigation) and telemetry
                else time.monotonic()
            )
            phase_durations[previous_phase] = (
                phase_durations.get(previous_phase, 0.0) + final_clock - phase_tick
            )
            metrics['phase_durations'] = phase_durations
            metrics['walking_time'] = phase_durations.get('walk', 0.0) + phase_durations.get(
                'goto', 0.0
            )
            metrics['manipulation_time'] = phase_durations.get('pick', 0.0) + phase_durations.get(
                'place', 0.0
            )
        if executor is not None:
            if executor.current:
                executor.abort(time.monotonic() - start, metrics.get('reason', 'Episode ended'))
            metrics.update(
                completed_skills=executor.index,
                retries=executor.retries,
                failed_skill=executor.current['skill'] if executor.current else None,
                failed_skill_index=executor.index if executor.current else None,
            )
            (directory / 'skills.json').write_text(json.dumps(executor.events, indent=2) + '\n')
        if stop:
            stop.set()
        if navigation:
            navigation.close()
        if robot:
            try:
                robot.close()
            except Exception as error:  # noqa: BLE001 - cleanup must preserve the report
                metrics['cleanup_error'] = str(error)
        if process:
            # Drain telemetry so the child cannot remain blocked writing to its pipe.
            deadline = time.monotonic() + 3
            while process.is_alive() and time.monotonic() < deadline:
                if pipe.poll(0.05):
                    try:
                        pipe.recv()
                    except EOFError:
                        break
            process.join(timeout=6)
            if process.is_alive():
                process.terminate()
                process.join(timeout=2)
        if pipe:
            pipe.close()
        if domain_lock:
            domain_lock.close()
        if stream:
            stream.close()
        if recorder:
            metrics.update(recorder.close())
        if telemetry:
            metrics.update(
                {
                    k: telemetry[k]
                    for k in (
                        'collisions',
                        'steps_per_second',
                        'watchdog_events',
                        'invalid_commands',
                        'simulator_cpu_percent',
                        'simulator_peak_ram_bytes',
                        'render_fps',
                        'camera_renderer',
                    )
                }
            )
            metrics['simulation_time'] = telemetry['time']
        metrics['total_completion_time'] = time.monotonic() - run_started
        metrics['controller_cpu_percent'] = (
            100 * (time.process_time() - cpu_started) / max(metrics['total_completion_time'], 1e-6)
        )
        metrics['controller_peak_ram_bytes'] = (
            resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
        )
        (directory / 'metadata.json').write_text(json.dumps(provenance(config), indent=2) + '\n')
        (directory / 'metrics.json').write_text(
            json.dumps(metrics, indent=2, allow_nan=False) + '\n'
        )
    return {'results_dir': str(directory), **metrics}
