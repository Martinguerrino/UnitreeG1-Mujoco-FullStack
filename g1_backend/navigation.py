"""Loopback transport and ideal, level 2-D LiDAR for the simulation-only ROS bridge."""

import json
import math
import socket
import time

import numpy as np

from .skills import yaw_from_quaternion


def laser_scan(model, data, count=360, maximum=20.0):
    import mujoco

    yaw = yaw_from_quaternion(data.qpos[3:7])
    origin = np.array([*data.qpos[:2], 0.78])
    # A virtual stabilized horizontal scanner. Group 2 contains the imported
    # world's geometry, excluding robot visual/collision meshes (groups 0/1).
    groups = np.array([0, 0, 1, 0, 0, 0], dtype=np.uint8)
    ranges = []
    hit = np.zeros(1, dtype=np.int32)
    for angle in np.linspace(-math.pi, math.pi, count, endpoint=False):
        direction = np.array([math.cos(yaw + angle), math.sin(yaw + angle), 0.0])
        distance = mujoco.mj_ray(model, data, origin, direction, groups, 1, -1, hit)
        ranges.append(float(distance) if 0.05 <= distance <= maximum else None)
    return ranges


class NavigationTransport:
    def __init__(self, config):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', config.get('command_port', 19421)))
        self.socket.setblocking(False)
        self.destination = ('127.0.0.1', config.get('telemetry_port', 19420))
        self.command = np.zeros(3)
        self.received = -math.inf
        self.last_time = None
        self.turn_assist = config.get('turn_assist', {})
        self.turn_started = None
        self.assist_linear = 0.0
        self.assist_angular = 0.0
        self.assist_time = None

    @staticmethod
    def _sector_clear(scan, forward, clearance):
        if not scan:
            return False
        count = len(scan)
        center = count // 2 if forward else 0
        half_width = max(1, round(count * 20 / 360))
        indices = [(center + offset) % count for offset in range(-half_width, half_width + 1)]
        distances = [scan[index] for index in indices if scan[index] is not None]
        return bool(distances) and min(distances) >= clearance

    def _assist_turn(self, command, telemetry):
        cfg = self.turn_assist
        now = telemetry['time']
        dt = 0.0 if self.assist_time is None else max(0.0, now - self.assist_time)
        self.assist_time = now
        turning = (
            cfg.get('enabled', False)
            and abs(command[1]) <= cfg.get('max_linear_speed', 0.03)
            and abs(command[2]) >= (
                cfg.get('min_angular_speed', 0.01)
                if self.turn_started is None else cfg.get('release_angular_speed', 0.001)
            )
            and abs(command[0]) <= cfg.get('max_forward_command', 0.5)
            and abs(command[2]) >= (
                cfg.get('angular_linear_ratio', 0.7) * abs(command[0])
            )
        )
        target = 0.0
        if turning:
            if self.turn_started is None:
                self.turn_started = now
            half_period = max(float(cfg.get('period', 1.5)), 1e-3)
            elapsed = now - self.turn_started
            # Equal raised-cosine pulses: backwards first, then forwards.
            # Each half-wave starts and ends with zero slope, so both phases
            # accelerate identically without a sharp direction change.
            half_wave = int(elapsed / half_period)
            phase = (elapsed / half_period) - half_wave
            direction = -1.0 if half_wave % 2 == 0 else 1.0
            magnitude = 0.5 * (1.0 - math.cos(2.0 * math.pi * phase))
            target = direction * cfg.get('speed', 0.3) * magnitude
            if not self._sector_clear(
                telemetry.get('scan'), target > 0, cfg.get('obstacle_clearance', 0.8)
            ):
                target = 0.0
                self.assist_linear = 0.0
        else:
            self.turn_started = None
        change = cfg.get('acceleration', 0.4) * dt
        self.assist_linear += np.clip(target - self.assist_linear, -change, change)
        assisted = command.copy()
        if turning:
            assisted[0] = self.assist_linear
            angular_magnitude = max(
                abs(command[2]) * cfg.get('angular_gain', 1.0),
                cfg.get('minimum_assist_turn_rate', 0.0),
            )
            angular_target = math.copysign(
                min(angular_magnitude, cfg.get('max_angular_speed', 1.0)), command[2]
            )
        else:
            self.assist_linear = 0.0
            angular_target = np.clip(
                command[2] * cfg.get('tracking_angular_gain', 1.0),
                -cfg.get('max_angular_speed', 1.0),
                cfg.get('max_angular_speed', 1.0),
            )

        if cfg.get('enabled', False) and command[2] != 0.0:
            # Preserve the angular ramp across the oscillation/path-following
            # handoff. Translation always remains Nav2's command when tracking.
            angular_change = cfg.get('angular_acceleration', 4.0) * dt
            self.assist_angular += np.clip(
                angular_target - self.assist_angular, -angular_change, angular_change
            )
            assisted[2] = self.assist_angular
        else:
            self.assist_angular = 0.0
        return assisted

    def velocity(self, telemetry=None):
        while True:
            try:
                payload, _ = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            try:
                value = np.asarray(json.loads(payload)['velocity'], dtype=float)
                if value.shape != (3,) or not np.isfinite(value).all():
                    continue
                self.command = np.clip(value, [-0.5, -0.15, -1.0], [0.55, 0.15, 1.0])
                self.received = time.monotonic()
            except (ValueError, KeyError, TypeError):
                continue
        command = self.command.copy() if time.monotonic() - self.received < 0.5 else np.zeros(3)
        return self._assist_turn(command, telemetry) if telemetry is not None else command

    def publish(self, telemetry):
        if telemetry['time'] == self.last_time:
            return
        self.last_time = telemetry['time']
        keys = ('time', 'position', 'quaternion', 'linear_velocity', 'scan', 'scan_time')
        self.socket.sendto(
            json.dumps({key: telemetry[key] for key in keys}).encode(), self.destination
        )

    def close(self):
        self.socket.close()
