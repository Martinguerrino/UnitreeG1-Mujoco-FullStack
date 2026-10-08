"""Stateful humanoid turn assistance, evaluated before ROS collision monitoring."""
import math

import numpy as np


def merge_parameters(base, overrides):
    for key, value in overrides.items():
        if isinstance(value, dict):
            merge_parameters(base.setdefault(key, {}), value)
        else:
            base[key] = value


class TurnAssist:
    def __init__(self, config):
        self.turn_assist = config
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
            ) or not self._swept_footprint_clear(telemetry.get('scan'), target > 0):
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

    def _swept_footprint_clear(self, scan, forward):
        """Check diagonal obstacles along the full width of a translation pulse."""
        cfg = self.turn_assist
        if 'robot_half_width' not in cfg:
            return True
        if not scan:
            return False
        margin = cfg.get('turn_obstacle_margin', 0.1)
        width = cfg['robot_half_width'] + margin
        length = cfg['robot_half_length'] + margin
        reach = max(length, cfg.get('obstacle_clearance', 0.5))
        lower, upper = (-length, reach) if forward else (-reach, length)
        for index, distance in enumerate(scan):
            if distance is None or not math.isfinite(distance):
                continue
            angle = -math.pi + 2 * math.pi * index / len(scan)
            x, y = distance * math.cos(angle), distance * math.sin(angle)
            if lower <= x <= upper and abs(y) <= width:
                return False
        return True
