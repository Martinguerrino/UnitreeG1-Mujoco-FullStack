"""Seeded, approximate sensor imperfections; never alter simulator truth."""
from collections import deque
import numpy as np


class SensorEffects:
    def __init__(self, profile, seed=0):
        self.profile = profile
        self.rng = np.random.default_rng(seed)
        imu = profile.get('imu', {})
        self.bias = {name: self.rng.normal(0, imu.get(key, 0), 3)
                     for name, key in [('gyro', 'gyro_bias_sigma_rad_s'),
                                       ('acc', 'acc_bias_sigma_m_s2')]}

    def lidar(self, points, maximum):
        cfg = self.profile['lidar']
        points = np.asarray(points, dtype=np.float32)
        ranges = np.linalg.norm(points, axis=1)
        valid = ranges >= cfg['min_range_m']
        valid &= self.rng.random(len(points)) >= cfg['dropout_probability']
        measured = ranges + self.rng.normal(0, cfg['range_sigma_m'] + cfg['range_sigma_per_m'] * ranges)
        valid &= (measured >= cfg['min_range_m']) & (measured <= maximum)
        return (points[valid] * (measured[valid] / ranges[valid])[:, None]).astype(np.float32)

    def camera(self, sensors):
        from .cuda.sensors import camera

        accelerated = camera(self, sensors)
        if accelerated is not None:
            return accelerated
        cfg = self.profile['camera']
        output = dict(sensors)
        for name, image in sensors.items():
            if name.endswith('rgb'):
                gain = 1 + self.rng.normal(0, cfg['exposure_sigma'])
                noisy = image.astype(np.float32) * gain + self.rng.normal(0, cfg['rgb_sigma'], image.shape)
                output[name] = np.clip(np.rint(noisy), 0, 255).astype(np.uint8)
            elif name.endswith('depth'):
                depth = np.asarray(image, dtype=np.float32)
                valid = np.isfinite(depth) & (depth >= cfg['depth_min_m']) & (depth <= cfg['depth_max_m'])
                safe = np.where(valid, depth, 0)
                edge = np.zeros(depth.shape, dtype=bool)
                jump = np.abs(np.diff(safe, axis=1)) > cfg['depth_edge_m']
                edge[:, 1:] |= jump
                edge[:, :-1] |= jump
                jump = np.abs(np.diff(safe, axis=0)) > cfg['depth_edge_m']
                edge[1:] |= jump
                edge[:-1] |= jump
                sigma = cfg['depth_sigma_m'] + cfg['depth_sigma_per_m2'] * safe**2
                measured = safe + self.rng.normal(0, sigma)
                measured = np.rint(measured / cfg['depth_quantization_m']) * cfg['depth_quantization_m']
                valid &= self.rng.random(depth.shape) >= cfg['depth_dropout_probability']
                valid &= ~(edge & (self.rng.random(depth.shape) < cfg['edge_dropout_probability']))
                valid &= (measured >= cfg['depth_min_m']) & (measured <= cfg['depth_max_m'])
                output[name] = np.where(valid, measured, 0).astype(np.float32)
        return output

    def imu(self, sample):
        cfg = self.profile['imu']
        return {**sample, **{name: (np.asarray(sample[name]) + self.bias[name]
                                  + self.rng.normal(0, cfg[key], 3)).tolist()
                            for name, key in [('gyro', 'gyro_sigma_rad_s'), ('acc', 'acc_sigma_m_s2')]}}


class DelayedFrames:
    """Bounded delivery queue, preserving original capture timestamps."""
    def __init__(self, latency=0, jitter=0, seed=0):
        self.latency, self.jitter = latency, jitter
        self.rng = np.random.default_rng(seed)
        self.pending = deque(maxlen=8)

    def receive(self, frame, now):
        if frame is not None:
            delay = max(0., self.latency + self.rng.uniform(-self.jitter, self.jitter))
            self.pending.append((frame['time'] + delay, frame))
        latest = None
        while self.pending and self.pending[0][0] <= now:
            latest = self.pending.popleft()[1]
        return latest
