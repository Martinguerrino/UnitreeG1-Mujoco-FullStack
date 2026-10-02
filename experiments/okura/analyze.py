"""Summarize recorded Okura episodes without altering policies or physics."""

import argparse
import json
from pathlib import Path

import numpy as np
from safetensors.numpy import load_file

from .checkpoint import DEFAULT_CHECKPOINT, STATE_NAMES


def analyze(directory, stats):
    report = json.loads((directory / 'report.json').read_text())
    result = {
        'directory': str(directory),
        'valid_policy_episode': bool(report.get('completed') and report.get('physical_grasp_tested')
                                     and not report.get('error')),
        **{key: report.get(key) for key in (
            'sim_seconds', 'grasp_success', 'max_finger_contacts', 'max_object_lift_m',
            'max_lift_with_two_finger_contact_seconds', 'act_actions_executed',
            'body_watchdog_events', 'finger_stale_steps', 'arm_limit_clips',
            'invalid_body_commands', 'invalid_hand_commands', 'mechanical_passed',
            'watchdog_test_passed', 'error',
        )},
    }
    result['clean_command_stream'] = (report.get('body_watchdog_events', 0) == 0
                                      and report.get('finger_stale_steps', 0) == 0)
    result['clean_policy_episode'] = (result['valid_policy_episode']
                                      and result['clean_command_stream'])
    result['experimental_parameters'] = {key: report.get(key) for key in (
        'arm_gain_scale', 'waist_gain_scale', 'arm_target_speed_limit_rad_s', 'reset_pose', 'action_timing',
        'camera_pitch', 'camera_position', 'camera_fovy', 'basket_scale',
        'async_commands', 'placement_speed_window', 'speed_limit_clips',
    )}
    timing_file = directory / 'inference_timing.json'
    if timing_file.exists():
        events = json.loads(timing_file.read_text())
        if events:
            result['observation_to_chunk_median_sim_seconds'] = float(np.median([
                event['applied_sim_time'] - event['observed_sim_time'] for event in events
            ]))
            result['image_age_at_observation_median_sim_seconds'] = float(np.median([
                event['observed_sim_time'] - event['image_sim_time'] for event in events
            ]))
            result['stale_action_steps_median'] = float(np.median([
                event['stale_action_steps'] for event in events
            ]))
    observations = [np.load(path)['state'] for path in sorted(directory.glob('input_*.npz'))]
    if observations:
        states = np.stack(observations)
        normalized = (states - stats['observation.state.mean']) / (
            stats['observation.state.std'] + 1e-8
        )
        result['state_normalized_max_abs'] = dict(zip(
            STATE_NAMES, np.max(np.abs(normalized), axis=0).tolist(), strict=True
        ))
        result['state_outside_training_minmax_fraction'] = float(np.mean(
            (states < stats['observation.state.min']) | (states > stats['observation.state.max'])
        ))
    times = report.get('inference_seconds', [])
    if times:
        result['inference_median_host_seconds'] = float(np.median(times))
    trajectory = directory / 'trajectory.npz'
    if trajectory.exists():
        with np.load(trajectory) as data:
            error = np.abs(data['q'][:, 15:29] - data['target'][:, 15:29])
            result['arm_tracking_error_p95_rad'] = float(np.percentile(error, 95))
            result['arm_tracking_error_max_rad'] = float(error.max())
            error = np.abs(data['gripper'][:, 1] - data['gripper_target'][:, 1])
            result['right_gripper_tracking_error_p95_motor_rad'] = float(np.percentile(error, 95))
            result['right_gripper_measured_range_motor_rad'] = [
                float(data['gripper'][:, 1].min()), float(data['gripper'][:, 1].max())
            ]
            result['object_displacement_xyz_m'] = (
                data['box'][-1] - data['box'][0]
            ).tolist()
            if 'right_finger_midpoint' in data:
                distance = np.linalg.norm(data['right_finger_midpoint'] - data['box'], axis=1)
                result['min_finger_body_midpoint_to_object_center_m'] = float(distance.min())
                result['distance_note'] = 'Body-origin midpoint, not collision-surface clearance.'
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plot', type=Path)
    args = parser.parse_args()
    stats = load_file(str(DEFAULT_CHECKPOINT / 'policy_preprocessor_step_3_normalizer_processor.safetensors'))
    summaries = [analyze(directory, stats) for directory in args.directories]
    args.output.write_text(json.dumps(summaries, indent=2) + '\n')
    if args.plot:
        from .plot import plot

        plot(args.directories, args.plot)
    print(json.dumps([{k: row.get(k) for k in (
        'directory', 'valid_policy_episode', 'grasp_success', 'max_finger_contacts',
        'max_object_lift_m', 'arm_tracking_error_p95_rad',
        'right_gripper_tracking_error_p95_motor_rad', 'mechanical_passed', 'error',
    )} for row in summaries], indent=2))


if __name__ == '__main__':
    main()
