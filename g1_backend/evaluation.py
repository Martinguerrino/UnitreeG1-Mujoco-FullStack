"""Benchmark orchestration outside the SDK process, preserving every trial."""

from collections import Counter
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import statistics
import signal
import subprocess
import sys
import uuid

import yaml

from .config import ROOT, load_config


METRICS = (
    'final_position_error',
    'orientation_error',
    'walking_time',
    'control_elapsed',
    'total_completion_time',
    'steps_per_second',
    'collisions',
    'falls',
)


def summarize(trials):
    """Include all attempts in success rate; never replace missing measurements by zero."""
    if not trials:
        raise ValueError('Cannot summarize an empty benchmark')
    status = Counter(trial['status'] for trial in trials)
    result = {
        'attempts': len(trials),
        'statuses': dict(status),
        'successes': sum(trial.get('success') is True for trial in trials),
    }
    result['success_rate'] = result['successes'] / len(trials)
    result['metrics'] = {}
    for metric in METRICS:
        values = [
            trial[metric]
            for trial in trials
            if isinstance(trial.get(metric), (float, int))
            and not isinstance(trial[metric], bool)
            and math.isfinite(trial[metric])
        ]
        result['metrics'][metric] = {
            'measured': len(values),
            'missing': len(trials) - len(values),
            'mean': statistics.mean(values) if values else None,
            'median': statistics.median(values) if values else None,
            'min': min(values) if values else None,
            'max': max(values) if values else None,
        }
    return result


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.partial')
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def run_process(arguments, log_path, timeout):
    """Own a process group so deadline/interrupt cleanup also stops its simulator."""
    with (
        log_path.open('w') as log,
        subprocess.Popen(
            arguments,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        ) as process,
    ):
        try:
            return process.wait(timeout=timeout)
        except BaseException:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            raise


def run_benchmark(source):
    """Each trial runs in a fresh interpreter, resetting DDS singletons and simulator."""
    import copy

    specification = (
        copy.deepcopy(source)
        if isinstance(source, dict)
        else yaml.safe_load(Path(source).read_text())
    )
    repetitions = specification.get('repetitions', 1)
    if type(repetitions) is not int or repetitions < 1:
        raise ValueError('repetitions must be a positive integer')
    cases = specification['cases']
    if not cases or len({case['name'] for case in cases}) != len(cases):
        raise ValueError('Benchmark cases must be nonempty with unique names')
    base = load_config(ROOT / specification['experiment'])
    configs = []
    for case in cases:
        for repeat in range(repetitions):
            config = load_config(base)
            config['robot']['initial_position'] = case['initial_position']
            config['robot']['initial_yaw'] = case['initial_yaw']
            config['target'] = case['target']
            config['max_speed'] = case['max_speed']
            config['seed'] = specification.get('seed', base['seed']) + len(configs)
            configs.append((case['name'], repeat, load_config(config)))
    run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S') + '_' + uuid.uuid4().hex[:8]
    directory = ROOT / specification.get('results_dir', 'results/benchmarks') / run_id
    directory.mkdir(parents=True)
    (directory / 'benchmark.yaml').write_text(yaml.safe_dump(specification, sort_keys=False))
    summary = {'results_dir': str(directory), 'trials': []}
    for index, (name, repeat, config) in enumerate(configs):
        trial_dir = directory / f'trial_{index:03d}'
        trial_dir.mkdir()
        config['results_dir'] = str(trial_dir / 'episodes')
        config_path = trial_dir / 'config.yaml'
        config_path.write_text(yaml.safe_dump(config, sort_keys=False))
        report_path = trial_dir / 'report.json'
        timeout = config['duration'] + config.get('warmup', 2) + 40
        try:
            returncode = run_process(
                [
                    sys.executable,
                    str(ROOT / 'scripts/run_experiment.py'),
                    '--config',
                    str(config_path),
                    '--report',
                    str(report_path),
                ],
                log_path=trial_dir / 'process.log',
                timeout=timeout,
            )
            if report_path.exists():
                report = json.loads(report_path.read_text())
                if returncode != 0 and report.get('success') is True:
                    report.update(
                        status='failed', success=False, reason='Process exit disagrees with report'
                    )
            else:
                report = {
                    'status': 'failed',
                    'success': False,
                    'reason': f'Process exited with code {returncode} without a report',
                }
            report['returncode'] = returncode
        except subprocess.TimeoutExpired:
            report = {'status': 'failed', 'success': False, 'reason': 'Experiment process timeout'}
        report.update(
            case=name,
            repetition=repeat,
            seed=config['seed'],
            process_log=str(trial_dir / 'process.log'),
        )
        write_json(report_path, report)
        summary['trials'].append(report)
        summary['aggregate'] = summarize(summary['trials'])
        summary['by_case'] = {
            case['name']: summarize([t for t in summary['trials'] if t['case'] == case['name']])
            for case in cases
            if any(t['case'] == case['name'] for t in summary['trials'])
        }
        write_json(directory / 'summary.json', summary)
        print(
            f'Trial {index + 1}/{len(configs)} ({name}, repeat {repeat}): {report["status"]}',
            flush=True,
        )
    return summary
