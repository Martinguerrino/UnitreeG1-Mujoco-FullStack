from pathlib import Path
import math
import os
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]


def runtime_path():
    prefix = ROOT / 'artifacts/cyclonedds'
    if prefix.exists():
        os.environ['CYCLONEDDS_HOME'] = str(prefix)
    path = ROOT / 'artifacts/dds_runtime'
    if path.is_dir() and str(path) not in sys.path:
        sys.path.insert(0, str(path))


def finite(value, name, minimum=None):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    if minimum is not None and value <= minimum:
        raise ValueError(f'{name} must be greater than {minimum}')


def vector(value, name, size):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f'{name} must have {size} components')
    for component in value:
        finite(component, name)


def load_config(source):
    if isinstance(source, dict):
        import copy

        config = copy.deepcopy(source)
    else:
        config = yaml.safe_load(Path(source).read_text())
    # Component references are always relative to the repository, not the cwd.
    for name in ('robot', 'locomotion', 'manipulation', 'camera', 'task'):
        if isinstance(config.get(name), str):
            config[name] = yaml.safe_load((ROOT / config[name]).read_text())
    robot = config['robot']
    if (
        robot['interface'] != 'lo'
        or type(robot['domain_id']) is not int
        or not 1 <= robot['domain_id'] <= 232
    ):
        raise ValueError('This simulation launcher requires loopback and DDS domain 1..232')
    for key in ('timestep', 'state_timeout', 'command_timeout'):
        finite(robot[key], key, 0)
    vector(robot['initial_position'], 'initial_position', 3)
    finite(robot['initial_yaw'], 'initial_yaw')
    finite(config['duration'], 'duration', 0)
    for key in ('warmup', 'stand_duration', 'stop_duration'):
        if key in config:
            finite(config[key], key)
            if config[key] < 0:
                raise ValueError(f'{key} must not be negative')
    for key in ('max_speed', 'position_tolerance', 'orientation_tolerance'):
        if key in config:
            finite(config[key], key, 0)
    if 'target' in config:
        vector(config['target'], 'target', 3)
    if type(config['seed']) is not int or not 0 <= config['seed'] < 2**32:
        raise ValueError('seed must be an integer in [0, 2**32)')
    cam = config['camera']
    if cam.get('record') and not cam.get('enabled'):
        raise ValueError('Camera recording requires an enabled camera')
    if cam.get('enabled'):
        for key in ('width', 'height', 'max_recorded_frames'):
            value = cam.get(key, 10000)
            if type(value) is not int or value <= 0:
                raise ValueError(f'camera {key} must be a positive integer')
        finite(cam['fps'], 'camera fps', 0)
        if not (cam['rgb'] or cam['depth']):
            raise ValueError('Camera requires FOV <180 and at least one modality')
        views = cam.get('views') or [cam]
        names = set()
        for view in views:
            name = view.get('name', 'policy_camera')
            if not isinstance(name, str) or not name or name in names:
                raise ValueError('Camera view names must be unique non-empty strings')
            names.add(name)
            finite(view['fovy'], f'camera {name} fovy', 0)
            if view['fovy'] >= 180:
                raise ValueError('Camera requires FOV <180 and at least one modality')
            vector(view['position'], f'camera {name} position', 3)
            vector(view['quaternion'], f'camera {name} quaternion', 4)
            if sum(component**2 for component in view['quaternion']) < 1e-12:
                raise ValueError('Camera quaternion must be nonzero')
    if config['kind'] == 'task':
        from .skills import GoTo, TaskExecutor

        executor = TaskExecutor(config['task']['steps'], config['task'].get('defaults'))
        for step in executor.steps:
            if step['skill'] == 'goto':
                GoTo(step['target'], step.get('max_speed', config.get('max_speed', 0.3)))
                for key in ('position_tolerance', 'orientation_tolerance', 'stop_speed_tolerance'):
                    if key in step:
                        finite(step[key], key, 0)
    return config
