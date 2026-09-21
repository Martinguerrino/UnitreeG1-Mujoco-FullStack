"""Official SDK2 messages; policies never receive SDK objects."""

import threading
import time

import numpy as np

from .config import runtime_path
from .contracts import RobotState


class DdsRobot:
    _initialized_domain = None

    def __init__(self, config, limits):
        if self._initialized_domain not in (None, config['domain_id']):
            raise ValueError('SDK2 domain is process-global; use a new process to change domains')
        runtime_path()
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.utils.crc import CRC

        self.timeout = config['state_timeout']
        self.limits = limits
        self.state = None
        self.count = 0
        self.clipped = 0
        self.lock = threading.Lock()
        self.crc = CRC()
        self.command = unitree_hg_msg_dds__LowCmd_()
        ChannelFactoryInitialize(config['domain_id'], config['interface'])
        DdsRobot._initialized_domain = config['domain_id']
        self.publisher = ChannelPublisher('rt/lowcmd', LowCmd_)
        self.publisher.Init()
        self.subscriber = ChannelSubscriber('rt/lowstate', LowState_)
        self.subscriber.Init(self._receive, 10)

    def _receive(self, message):
        motors = message.motor_state[:29]
        state = RobotState(
            np.array([m.q for m in motors]),
            np.array([m.dq for m in motors]),
            np.array(message.imu_state.quaternion),
            np.array(message.imu_state.gyroscope),
            time.monotonic(),
            message.mode_machine,
        )
        with self.lock:
            self.state = state
            self.count += 1

    def read(self):
        with self.lock:
            state = self.state
        if state is None or time.monotonic() - state.received_at > self.timeout:
            raise TimeoutError('LowState missing or stale')
        if not all(
            np.isfinite(x).all() for x in (state.q, state.dq, state.quaternion, state.gyroscope)
        ):
            raise ValueError('Nonfinite LowState')
        return state

    def send(self, command):
        command.validate()
        state = self.read()
        q = np.clip(command.q, self.limits[:, 0], self.limits[:, 1])
        self.clipped += int(np.count_nonzero(q != command.q))
        self.command.mode_pr = 0
        self.command.mode_machine = state.mode_machine
        for i in range(29):
            motor = self.command.motor_cmd[i]
            motor.mode = 1
            motor.q = float(q[i])
            motor.dq = 0.0
            motor.tau = 0.0
            motor.kp = float(command.kp[i])
            motor.kd = float(command.kd[i])
        self.command.crc = self.crc.Crc(self.command)
        self.publisher.Write(self.command)

    def stop(self):
        # Damping is a simulation stop, not a hardware-certified emergency stop.
        for motor in self.command.motor_cmd:
            motor.mode = 1
            motor.q = motor.dq = motor.tau = motor.kp = 0.0
            motor.kd = 2.0
        self.command.crc = self.crc.Crc(self.command)
        self.publisher.Write(self.command)

    def close(self):
        self.stop()
        self.subscriber.Close()
        self.publisher.Close()
