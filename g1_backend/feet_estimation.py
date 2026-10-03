"""Sensor-only leg/IMU velocity and conservative planar odometry fusion.

Contact forces are inferred with a quasi-static joint torque model, not read
from simulation contacts. They are confidence cues, not force measurements.
"""
from collections import deque

import numpy as np


class FeetEstimator:
    def __init__(self, model):
        import mujoco

        self.mj = mujoco
        self.model = model
        # Private kinematic state: never copy simulator poses, velocities or contacts.
        self.data = mujoco.MjData(model)
        self.joints = model.actuator_trnid[:29, 0]
        self.qadr = model.jnt_qposadr[self.joints]
        self.vadr = model.jnt_dofadr[self.joints]
        self.bodies = [model.body(side + '_ankle_roll_link').id for side in ('left', 'right')]
        self.velocity = np.zeros(3)
        self.variance = np.full(3, .25)
        self.bias = np.zeros(3)
        self.confidence = np.zeros(2)
        self.support = np.zeros(2, dtype=bool)
        self.timestamp = None
        self.last_support = None

    def update(self, timestamp, q, dq, torque, quaternion, gyro, acceleration):
        arrays = [np.asarray(x, dtype=float) for x in (q, dq, torque, quaternion, gyro, acceleration)]
        if [x.shape for x in arrays] != [(29,), (29,), (29,), (4,), (3,), (3,)]:
            raise ValueError('Invalid feet estimator sensor dimensions')
        if not all(np.isfinite(x).all() for x in arrays) or not np.isfinite(timestamp):
            raise ValueError('Nonfinite feet estimator input')
        q, dq, torque, quaternion, gyro, acceleration = arrays
        norm = np.linalg.norm(quaternion)
        if norm < .5:
            raise ValueError('Invalid IMU quaternion')
        dt = .02 if self.timestamp is None else timestamp - self.timestamp
        if dt <= 0:
            return None
        if dt > .1:
            self.velocity[:] = 0
            self.variance[:] = .25
            self.confidence[:] = 0
            self.support[:] = False
            self.last_support = None
        dt = min(dt, .1)
        self.timestamp = timestamp
        rotation = np.empty(9)
        self.mj.mju_quat2Mat(rotation, quaternion / norm)
        rotation = rotation.reshape(3, 3)
        d, m = self.data, self.model
        d.qpos[:3] = [0, 0, 3]  # Keep all geometry clear of the floor.
        d.qpos[3:7] = quaternion / norm
        d.qpos[self.qadr] = q
        d.qvel[:] = 0
        self.mj.mj_kinematics(m, d)
        self.mj.mj_comPos(m, d)
        self.mj.mj_comVel(m, d)
        self.mj.mj_rne(m, d, 0, d.qfrc_bias)
        positions, velocities, loads, scores = [], [], [], []
        for side, body in enumerate(self.bodies):
            point = d.xpos[body] + d.xmat[body].reshape(3, 3) @ np.array([.025, 0, -.03])
            jac = np.zeros((3, m.nv))
            self.mj.mj_jac(m, d, jac, None, point, body)
            leg = slice(side * 6, side * 6 + 6)
            columns = self.vadr[leg]
            # Gravity minus actuator effort equals approximate external joint load.
            j = jac[:, columns].T
            residual = d.qfrc_bias[columns] - torque[leg]
            force = np.linalg.solve(j.T @ j + np.eye(3) * .0001, j.T @ residual)
            fit = np.linalg.norm(j @ force - residual)
            load = max(0., float(force[2]))
            position = rotation.T @ (point - d.qpos[:3])
            foot_velocity = rotation.T @ (jac[:, self.vadr] @ dq)
            body_velocity = -foot_velocity - np.cross(gyro, position)
            positions.append(position)
            velocities.append(rotation @ body_velocity)
            loads.append(load)
            scores.append(np.clip((load - 35.) / 140., 0., 1.) * np.exp(-fit / 12.))
        positions, velocities = np.asarray(positions), np.asarray(velocities)
        heights = (positions @ rotation.T)[:, 2]
        scores = np.asarray(scores) * np.exp(-np.maximum(heights - min(heights) - .025, 0) / .04)
        inertial = rotation @ (acceleration - self.bias) + np.array([0, 0, -9.81])
        # Foot impacts can produce very large acceleration impulses. They must
        # not become a prolonged velocity prediction without trusted support.
        if np.linalg.norm(inertial) < 60:
            self.velocity += inertial * dt
        self.variance += .8 * dt
        # Two claimed supports must agree on body velocity. Neither is privileged.
        disagreement = np.linalg.norm(velocities[0] - velocities[1])
        if min(scores) > .3 and disagreement > .25:
            scores *= np.exp(-(disagreement - .25) / .2)
        self.confidence += (1 - np.exp(-dt / .08)) * (scores - self.confidence)
        self.support = np.where(self.support, self.confidence > .25, self.confidence > .55)
        weights = self.confidence * self.support
        valid = bool(weights.sum() > .25)
        measurement_variance = .5
        if valid:
            self.last_support = timestamp
            measured = np.average(velocities, axis=0, weights=weights)
            spread = np.average(np.sum((velocities - measured)**2, axis=1), weights=weights)
            measurement_variance = .015 + .08 * (1 - min(1., weights.sum())) + spread
            gain = self.variance / (self.variance + measurement_variance)
            self.velocity += gain * (measured - self.velocity)
            self.variance *= 1 - gain
            if np.linalg.norm(measured) < .03 and np.linalg.norm(gyro) < .05:
                expected = rotation.T @ np.array([0, 0, 9.81])
                self.bias += min(dt / 10., .01) * (acceleration - expected - self.bias)
        elif self.last_support is None or timestamp - self.last_support > .15:
            self.velocity[:] = 0
            self.variance[:] = .5
        if np.linalg.norm(self.velocity) > 2.:
            valid = False
            self.velocity[:] = 0
            self.variance[:] = .5
        return {'time': float(timestamp), 'velocity_body': (rotation.T @ self.velocity).tolist(),
                'variance': np.maximum(self.variance, measurement_variance).tolist(),
                'confidence': self.confidence.tolist(), 'support': self.support.tolist(),
                'loads': loads, 'feet_position_body': positions.tolist(), 'valid': valid}


class FeetFusion:
    """Fuse leg velocity increments with native SLAM XY; keep native rotation/Z.

    Does not fuse IMU orientation twice or treat leg-integrated position as an
    independent absolute pose. Requires fresh supported samples across interval.
    """
    def __init__(self):
        self.samples = deque(maxlen=200)
        self.position = None
        self.timestamp = None
        self.variance = np.full(2, .02)
        self.used = 0
        self.fallbacks = 0

    def add(self, sample):
        velocity = np.asarray(sample['velocity_body'])
        variance = np.asarray(sample['variance'])
        if (velocity.shape != (3,) or variance.shape != (3,) or
                not np.isfinite(np.r_[velocity, variance, sample['time']]).all() or
                np.any(variance <= 0)):
            return
        if not self.samples or sample['time'] > self.samples[-1]['time']:
            self.samples.append(sample)

    def update(self, timestamp, base, native_variance):
        result = base.copy()
        if self.timestamp is not None and timestamp <= self.timestamp:
            return result
        available = [s for s in self.samples if s['time'] <= timestamp]
        valid = self.timestamp is not None and 0 < timestamp - self.timestamp <= .3
        delta, uncertainty = np.zeros(2), np.zeros(2)
        cursor = self.timestamp
        previous = None
        if valid:
            for sample in available:
                if sample['time'] <= cursor:
                    previous = sample
                    continue
                if previous is None or not previous['valid'] or sample['time'] - cursor > .06:
                    valid = False
                    break
                dt = sample['time'] - cursor
                delta += (base[:3, :3] @ np.asarray(previous['velocity_body']))[:2] * dt
                uncertainty += np.max(previous['variance']) * dt * (timestamp - self.timestamp)
                cursor, previous = sample['time'], sample
            if previous is None or not previous['valid'] or timestamp - previous['time'] > .06:
                valid = False
            elif valid:
                dt = timestamp - cursor
                delta += (base[:3, :3] @ np.asarray(previous['velocity_body']))[:2] * dt
                uncertainty += np.max(previous['variance']) * dt * (timestamp - self.timestamp)
        if valid:
            predicted = self.position + delta
            prior = self.variance + uncertainty + .002 * (timestamp - self.timestamp)
            noise = np.clip(np.asarray(native_variance), .0025, .25)
            innovation = base[:2, 3] - predicted
            if np.sum(innovation**2 / (prior + noise)) < 16:
                gain = prior / (prior + noise)
                self.position = predicted + gain * innovation
                self.variance = np.maximum((1 - gain) * prior, .0025)
                result[:2, 3] = self.position
                self.used += 1
            else:
                valid = False
        if not valid:
            self.position = base[:2, 3].copy()
            self.variance[:] = .02
            self.fallbacks += 1
        self.timestamp = timestamp
        return result
