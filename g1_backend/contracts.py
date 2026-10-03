"""Policy contracts contain no simulator or SDK types."""

from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class RobotState:
    q: np.ndarray  # 29 positions, SDK motor order, radians
    dq: np.ndarray
    quaternion: np.ndarray  # wxyz, base orientation in world
    gyroscope: np.ndarray  # base frame, rad/s
    received_at: float  # monotonic host time
    mode_machine: int = 0
    accelerometer: np.ndarray | None = None
    tau_est: np.ndarray | None = None


@dataclass(frozen=True)
class JointCommand:
    q: np.ndarray  # absolute radians, SDK motor order
    kp: np.ndarray
    kd: np.ndarray

    def validate(self):
        for value in (self.q, self.kp, self.kd):
            if np.shape(value) != (29,) or not np.isfinite(value).all():
                raise ValueError('Commands require 29 finite values in SDK motor order')
        if np.any(self.kp < 0) or np.any(self.kd < 0):
            raise ValueError('Negative PD gains')


class LocomotionPolicy(Protocol):
    period: float

    def reset(self): ...
    def get_observation(self, robot_state: RobotState, velocity: np.ndarray) -> np.ndarray: ...
    def infer(self, observation: np.ndarray): ...
    def get_action(self) -> JointCommand: ...


class ManipulationPolicy(Protocol):
    """Future provider owns preprocessing and checkpoint-specific temporal state.

    observe accepts a sensor dict with RGB/depth arrays and RobotState.
    get_action must return a complete body command after resolving balance
    ownership; gripper transport is a separate provider-specific dependency.
    is_done reports policy completion, NOT measured grasp/place success.
    """

    period: float

    def reset(self): ...
    def observe(self, sensors: dict, robot_state: RobotState): ...
    def infer(self, observation): ...
    def get_action(self) -> JointCommand: ...
    def is_done(self) -> bool: ...
