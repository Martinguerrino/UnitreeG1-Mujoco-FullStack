"""Loopback transport and mounted 3-D / ideal planar navigation sensors."""

import json
import io
import math
import socket
import struct
import time

import numpy as np

from .skills import yaw_from_quaternion
from .navigation_control import TurnAssist


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


def lidar_scan(model, data, config, effects=None):
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
    # Include robot collision geometry as an occluder, then discard self hits.
    # Visual duplicates (group 1) are excluded; world geometry is group 2.
    groups = np.array([1, 0, 1, 0, 0, 0], dtype=np.uint8)
    angles = np.linspace(-math.pi, math.pi, config['horizontal_samples'], endpoint=False)
    elevations = np.radians(config['vertical_angles_deg'])
    local_directions = np.asarray([
        [math.cos(elevation) * math.cos(azimuth),
         math.cos(elevation) * math.sin(azimuth), math.sin(elevation)]
        for elevation in elevations for azimuth in angles
    ], dtype=np.float64)
    world_directions = np.ascontiguousarray(local_directions @ rotation.T)
    distances = np.empty(len(world_directions), dtype=np.float64)
    geom_ids = np.empty(len(world_directions), dtype=np.int32)
    mujoco.mj_multiRay(
        model, data, origin, world_directions.ravel(), groups, 1, -1, geom_ids,
        distances, len(world_directions), config['max_range'],
    )
    valid = (distances >= config['min_range']) & (distances <= config['max_range'])
    root_id = int(model.body_rootid[body_id])
    self_hit = np.zeros(len(distances), dtype=bool)
    hits = geom_ids >= 0
    self_hit[hits] = model.body_rootid[model.geom_bodyid[geom_ids[hits]]] == root_id
    valid &= ~self_hit
    points = (local_directions[valid] * distances[valid, None]).astype(np.float32)
    if effects is not None:
        points = effects.lidar(points, config['max_range'])
    world_points = origin + points.astype(np.float64) @ rotation.T
    return points, project_scan(world_points, data.qpos[:3],
                                yaw_from_quaternion(data.qpos[3:7]), config)


def project_scan(world_points, base_position, yaw, config):
    """Bin actual 3-D returns in a level, yaw-only frame at base XY, z=0.78.

    Ranges are planar distances, not slant ranges. All elevations contribute;
    ground/overhead points are rejected by world height. No synthetic rays.
    """
    count = config['horizontal_samples']
    points = np.asarray(world_points, dtype=float).reshape(-1, 3)
    delta = points[:, :2] - np.asarray(base_position)[:2]
    ranges = np.linalg.norm(delta, axis=1)
    valid = (np.isfinite(points).all(axis=1)
             & (points[:, 2] >= config.get('scan_min_height', 0.15))
             & (points[:, 2] <= config.get('scan_max_height', 1.5))
             & (ranges >= config['min_range']) & (ranges <= config['max_range']))
    angle = (np.arctan2(delta[valid, 1], delta[valid, 0]) - yaw + math.pi) % (2 * math.pi)
    bins = np.floor(angle * count / (2 * math.pi) + 0.5).astype(int) % count
    nearest = np.full(count, np.inf)
    np.minimum.at(nearest, bins, ranges[valid])
    return [float(value) if np.isfinite(value) else None for value in nearest]



class NavigationTransport(TurnAssist):
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
        self.sensor_sequence = 0
        super().__init__(config.get('turn_assist', {}))

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
            'torso_position_base', 'torso_quaternion_base', 'joint_names',
            'joint_positions',
        )
        self.socket.sendto(
            json.dumps({**{key: telemetry[key] for key in keys},
                        'torso_imu': telemetry.get('torso_imu', []),
                        'feet_estimation': telemetry.get('feet_estimation', []),
                        **({'imu': telemetry['imu']} if 'imu' in telemetry else {})}).encode(), self.destination
        )
        sensors = telemetry.get('sensors')
        if sensors:
            # Keep each modality in an independent compressed message. A raw
            # RGB-D pair is larger than common UDP receive buffers; independent
            # sequences prevent one dropped image from discarding LiDAR too.
            for name, value in sensors.items():
                buffer = io.BytesIO()
                np.savez_compressed(
                    buffer, time=telemetry.get('sensor_times', {}).get(name, telemetry['sensor_time']),
                    **{name: value}
                )
                payload = buffer.getvalue()
                chunk_size = 60000
                count = math.ceil(len(payload) / chunk_size)
                self.sensor_sequence += 1
                sequence = self.sensor_sequence
                for index in range(count):
                    header = struct.pack('!IHH', sequence, index, count)
                    self.socket.sendto(
                        header + payload[index * chunk_size:(index + 1) * chunk_size],
                        self.sensor_destination,
                    )

    def close(self):
        self.socket.close()
