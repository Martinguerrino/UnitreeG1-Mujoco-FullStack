"""Adapter for the pinned G1 29-DoF RL Lab velocity/v0 export."""

from collections import deque
import hashlib
import json

import numpy as np
import yaml

from ..config import ROOT, runtime_path
from ..contracts import JointCommand


def projected_gravity(quaternion):
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.isfinite(q).all() or norm < 1e-6:
        raise ValueError('Invalid IMU quaternion')
    w, x, y, z = q / norm
    return np.array([2 * (w * y - x * z), -2 * (y * z + w * x), 2 * (x * x + y * y) - 1])


class UnitreeVelocityPolicy:
    def __init__(self, config):
        runtime_path()
        import onnxruntime as ort

        checkpoint = ROOT / config['checkpoint']
        params = ROOT / config['deployment']
        lock = json.loads((ROOT / 'configs/sources.lock.json').read_text())
        for path in (checkpoint, params):
            expected = lock['files'][str(path.relative_to(ROOT))]['sha256']
            if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                raise ValueError(f'Artifact checksum mismatch: {path}')
        self.cfg = yaml.safe_load(params.read_text())
        self.command_limits = dict(self.cfg['commands']['base_velocity']['ranges'])
        self.command_limits.update(config.get('command_ranges', {}))
        self.mapping = np.array(self.cfg['joint_ids_map'])
        if sorted(self.mapping) != list(range(29)):
            raise ValueError('Invalid SDK joint mapping')
        self.period = self.cfg['step_dt']
        self.default = np.asarray(self.cfg['default_joint_pos'], dtype=np.float32)
        self.kp = np.asarray(self.cfg['stiffness'], dtype=np.float32)  # SDK order upstream
        self.kd = np.asarray(self.cfg['damping'], dtype=np.float32)
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        from ..acceleration import OnnxSession

        self.session = OnnxSession(ort, str(checkpoint), options, calibrate=True)
        if self.session.get_inputs()[0].shape != [1, 480]:
            raise ValueError('Expected the pinned 480-observation export')
        if self.session.backend == 'cuda':
            # Warm and compare providers before entering the control loop.
            self.session.run(None, {'obs': np.zeros((1, 480), dtype=np.float32)})
        self.reset()

    def reset(self):
        self.history = {}
        self.action = np.zeros(29, dtype=np.float32)

    def get_observation(self, robot_state, velocity):
        limits = self.command_limits
        velocity = np.asarray(velocity)
        if velocity.shape != (3,) or not np.isfinite(velocity).all():
            raise ValueError('Velocity command must be finite vx, vy, yaw rate')
        command = np.array(
            [
                np.clip(v, *limits[k])
                for v, k in zip(velocity, ('lin_vel_x', 'lin_vel_y', 'ang_vel_z'), strict=True)
            ]
        )
        values = {
            'base_ang_vel': robot_state.gyroscope,
            'projected_gravity': projected_gravity(robot_state.quaternion),
            'velocity_commands': command,
            'joint_pos_rel': robot_state.q[self.mapping] - self.default,
            'joint_vel_rel': robot_state.dq[self.mapping],
            'last_action': self.action,
        }
        groups = []
        for name, term in self.cfg['observations'].items():
            value = np.asarray(values[name], dtype=np.float32) * term['scale']
            if not np.isfinite(value).all():
                raise ValueError(f'Nonfinite observation: {name}')
            if name not in self.history:
                self.history[name] = deque(
                    [value.copy() for _ in range(term['history_length'])],
                    maxlen=term['history_length'],
                )
            else:
                self.history[name].append(value.copy())
            groups.append(np.concatenate(self.history[name]))
        return np.concatenate(groups).astype(np.float32)[None, :]

    def infer(self, observation):
        self.action = self.session.run(None, {'obs': observation})[0][0]
        if self.action.shape != (29,) or not np.isfinite(self.action).all():
            raise ValueError('Invalid policy output')

    def get_action(self):
        action_cfg = self.cfg['actions']['JointPositionAction']
        target = np.empty(29, dtype=np.float32)
        target[self.mapping] = self.action * action_cfg['scale'] + action_cfg['offset']
        return JointCommand(target, self.kp.copy(), self.kd.copy())
