"""Loopback transport and ideal, level 2-D LiDAR for the simulation-only ROS bridge."""

import json
import io
import math
import socket
import struct
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


def lidar_scan(model, data, config):
    """Return a MID-360-like XYZ cloud and its horizontal Nav2 projection."""
    import mujoco

    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, config['body'])
    if body_id < 0:
        raise ValueError(f"LiDAR parent body does not exist: {config['body']}")
    roll, pitch, yaw = config['rpy']
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
        math.cos(yaw), math.sin(yaw),
    )
    mount_rotation = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    body_rotation = data.xmat[body_id].reshape(3, 3)
    rotation = body_rotation @ mount_rotation
    origin = data.xpos[body_id] + body_rotation @ np.asarray(config['position'])
    groups = np.array([0, 0, 1, 0, 0, 0], dtype=np.uint8)
    hit = np.zeros(1, dtype=np.int32)
    angles = np.linspace(-math.pi, math.pi, config['horizontal_samples'], endpoint=False)
    elevations = np.radians(config['vertical_angles_deg'])
    points = []
    horizontal = [None] * len(angles)
    horizontal_row = int(np.argmin(np.abs(elevations)))
    for row, elevation in enumerate(elevations):
        ce, se = math.cos(elevation), math.sin(elevation)
        for column, azimuth in enumerate(angles):
            local_direction = np.array([ce * math.cos(azimuth), ce * math.sin(azimuth), se])
            distance = mujoco.mj_ray(
                model, data, origin, rotation @ local_direction, groups, 1, -1, hit
            )
            if config['min_range'] <= distance <= config['max_range']:
                points.append((local_direction * distance).tolist())
                if row == horizontal_row:
                    horizontal[column] = float(distance)
    return np.asarray(points, dtype=np.float32), horizontal


class NavigationTransport:
    def __init__(self, config):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 * 1024 * 1024)
        self.socket.bind(('127.0.0.1', config.get('command_port', 19421)))
        self.socket.setblocking(False)
        self.destination = ('127.0.0.1', config.get('telemetry_port', 19420))
        self.sensor_destination = ('127.0.0.1', config.get('sensor_port', 19422))
        self.command = np.zeros(3)
        self.received = -math.inf
        self.last_time = None
        self.last_sensor_frame = None
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
        keys = (
            'time', 'position', 'quaternion', 'linear_velocity', 'scan', 'scan_time',
            'torso_position_base', 'torso_quaternion_base',
        )
        self.socket.sendto(
            json.dumps({key: telemetry[key] for key in keys}).encode(), self.destination
        )
        frame_id = telemetry.get('camera_frame_id')
        sensors = telemetry.get('sensors')
        if sensors and frame_id != self.last_sensor_frame:
            self.last_sensor_frame = frame_id
            # Keep each modality in an independent compressed message. A raw
            # RGB-D pair is larger than common UDP receive buffers; independent
            # sequences prevent one dropped image from discarding LiDAR too.
            for modality, (name, value) in enumerate(sensors.items()):
                buffer = io.BytesIO()
                np.savez_compressed(
                    buffer, time=telemetry['sensor_time'], **{name: value}
                )
                payload = buffer.getvalue()
                chunk_size = 60000
                count = math.ceil(len(payload) / chunk_size)
                sequence = frame_id * 16 + modality
                for index in range(count):
                    header = struct.pack('!IHH', sequence, index, count)
                    self.socket.sendto(
                        header + payload[index * chunk_size:(index + 1) * chunk_size],
                        self.sensor_destination,
                    )

    def close(self):
        self.socket.close()
