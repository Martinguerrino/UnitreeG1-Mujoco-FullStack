"""Adapter for the public Holosoma FastSAC G1 29-DoF locomotion policy."""

import hashlib
import json

import numpy as np

from ..config import ROOT, runtime_path
from ..contracts import JointCommand
from .locomotion import projected_gravity


DEFAULT = np.zeros(29, dtype=np.float32)
DEFAULT[[0, 6]] = -0.312
DEFAULT[[3, 9]] = 0.669
DEFAULT[[4, 10]] = -0.363
DEFAULT[[15, 22]] = 0.2
DEFAULT[16] = 0.2
DEFAULT[23] = -0.2
DEFAULT[[18, 25]] = 0.6


class HolosomaFastSACPolicy:
    """Exact 100-D observation contract used by the public FastSAC export."""

    period = 0.005  # LeRobot reference controller runs this policy at 200 Hz.
    action_scale = 0.25
    gait_period = 0.5

    def __init__(self, config):
        runtime_path()
        import onnxruntime as ort

        checkpoint = ROOT / config['checkpoint']
        digest = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
        if digest != config['sha256']:
            raise ValueError(f'Holosoma checkpoint checksum mismatch: {digest}')
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(
            str(checkpoint), options, providers=['CPUExecutionProvider']
        )
        if self.session.get_inputs()[0].shape != [1, 100]:
            raise ValueError('Expected Holosoma 100-observation export')
        if self.session.get_outputs()[0].shape != [1, 29]:
            raise ValueError('Expected Holosoma 29-action export')
        metadata = self.session.get_modelmeta().custom_metadata_map
        self.kp = np.asarray(json.loads(metadata['kp']), dtype=np.float32)
        self.kd = np.asarray(json.loads(metadata['kd']), dtype=np.float32)
        self.reset()

    def reset(self):
        self.action = np.zeros(29, dtype=np.float32)
        self.phase = np.array([0.0, np.pi], dtype=np.float32)
        self.standing = True

    def get_observation(self, robot_state, velocity):
        command = np.clip(np.asarray(velocity, dtype=np.float32), -1.0, 1.0)
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError('Velocity command must be finite vx, vy, yaw rate')
        if np.linalg.norm(command[:2]) < 0.01 and abs(command[2]) < 0.01:
            self.phase[:] = np.pi
            self.standing = True
        elif self.standing:
            self.phase[:] = [0.0, np.pi]
            self.standing = False
        else:
            increment = 2 * np.pi * self.period / self.gait_period
            self.phase = (self.phase + increment + np.pi) % (2 * np.pi) - np.pi

        # The reference controller hides arms from locomotion so teleoperation
        # can own them independently.  We likewise expose fixed arm state and
        # only apply policy outputs to legs and waist (SDK joints 0..14).
        q = np.asarray(robot_state.q, dtype=np.float32).copy()
        dq = np.asarray(robot_state.dq, dtype=np.float32).copy()
        q[15:] = DEFAULT[15:]
        dq[15:] = 0.0
        observation = np.concatenate([
            self.action,
            np.asarray(robot_state.gyroscope, dtype=np.float32) * 0.25,
            command[2:3],
            command[:2],
            np.cos(self.phase),
            q - DEFAULT,
            dq * 0.05,
            projected_gravity(robot_state.quaternion).astype(np.float32),
            np.sin(self.phase),
        ]).astype(np.float32)[None, :]
        if observation.shape != (1, 100) or not np.isfinite(observation).all():
            raise ValueError('Invalid Holosoma observation')
        return observation

    def infer(self, observation):
        name = self.session.get_inputs()[0].name
        action = self.session.run(None, {name: observation})[0][0]
        if action.shape != (29,) or not np.isfinite(action).all():
            raise ValueError('Invalid Holosoma policy output')
        self.action = np.clip(action, -100.0, 100.0).astype(np.float32)

    def get_action(self):
        target = DEFAULT.copy()
        target[:15] += self.action[:15] * self.action_scale
        return JointCommand(target, self.kp.copy(), self.kd.copy())
