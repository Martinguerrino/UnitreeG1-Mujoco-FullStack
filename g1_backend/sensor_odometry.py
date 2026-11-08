"""Sensor-only SE(3) registration and covariance-weighted pose/velocity filtering."""

from dataclasses import dataclass

import numpy as np
from .cuda.neighbors import NeighborIndex
from scipy.spatial.transform import Rotation


def wrap(angles):
    return (np.asarray(angles) + np.pi) % (2 * np.pi) - np.pi


def pose_matrix(pose):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('xyz', pose[3:]).as_matrix()
    transform[:3, 3] = pose[:3]
    return transform


def matrix_pose(transform):
    return np.r_[transform[:3, 3], Rotation.from_matrix(transform[:3, :3]).as_euler('xyz')]


def transform_points(points, transform):
    return points @ transform[:3, :3].T + transform[:3, 3]


def reduce_cloud(points, colors=None, voxel=0.08, limit=2500):
    valid = np.isfinite(points).all(axis=1)
    points = np.asarray(points)[valid]
    colors = None if colors is None else np.asarray(colors)[valid]
    _, indices = np.unique(np.floor(points / voxel).astype(np.int64), axis=0, return_index=True)
    if len(indices) > limit:
        indices = indices[np.linspace(0, len(indices) - 1, limit).astype(int)]
    return points[indices], None if colors is None else colors[indices]


@dataclass
class Registration:
    transform: np.ndarray  # reference <- current
    covariance: np.ndarray  # translation XYZ, small rotation XYZ
    rmse: float
    overlap: float
    method: str = 'point_to_plane'


class CloudReference:
    def __init__(self, points, colors=None):
        self.points, self.colors = reduce_cloud(points, colors)
        self.tree = NeighborIndex(self.points)
        if len(self.points) < 60:
            self.normals = np.zeros_like(self.points)
            self.planar = np.zeros(len(self.points), dtype=bool)
            return
        distances, indices = self.tree.query(self.points, k=12)
        neighbors = self.points[indices]
        centered = neighbors - neighbors.mean(axis=1, keepdims=True)
        eigenvalues, eigenvectors = np.linalg.eigh(np.einsum('nki,nkj->nij', centered, centered))
        self.normals = eigenvectors[:, :, 0]
        self.planar = (
            (eigenvalues[:, 0] < 0.12 * np.maximum(eigenvalues[:, 1], 1e-10))
            & (eigenvalues[:, 1] > 0.0001)
            & (distances[:, -1] < 2.0)
        )

    def register(self, points, initial=None, colors=None, allow_partial=False):
        """Robust point-to-plane ICP; geometry, rather than IMU, supplies all rotations."""
        points, colors = reduce_cloud(points, colors)
        if min(len(points), len(self.points)) < 60 or self.planar.sum() < 60:
            return None
        transform = np.eye(4) if initial is None else np.array(initial, copy=True)
        for _ in range(25):
            source = transform_points(points, transform)
            distances, indices = self.tree.query(source)
            valid = (distances < 0.5) & self.planar[indices]
            if colors is not None and self.colors is not None:
                valid &= np.linalg.norm(colors - self.colors[indices], axis=1) < 0.45
            if valid.sum() < 60 or valid.mean() < 0.15:
                return None
            p, target, normal = (
                source[valid],
                self.points[indices[valid]],
                self.normals[indices[valid]],
            )
            residual = np.sum(normal * (target - p), axis=1)
            weights = np.minimum(1.0, 0.04 / np.maximum(np.abs(residual), 1e-8))
            jacobian = np.c_[normal, np.cross(p, normal)]
            hessian = jacobian.T @ (weights[:, None] * jacobian)
            eigenvalues = np.linalg.eigvalsh(hessian)
            # A single plane/corridor must not claim a well-constrained six-axis pose.
            if not allow_partial and eigenvalues[0] < 0.0001 * eigenvalues[-1]:
                return None
            increment = np.linalg.pinv(hessian, rcond=0.0001) @ (jacobian.T @ (weights * residual))
            if np.linalg.norm(increment[:3]) > 0.4 or np.linalg.norm(increment[3:]) > 0.25:
                return None
            step = np.eye(4)
            step[:3, :3] = Rotation.from_rotvec(increment[3:]).as_matrix()
            step[:3, 3] = increment[:3]
            transform = step @ transform
            if np.linalg.norm(increment[:3]) < 0.0002 and np.linalg.norm(increment[3:]) < 0.0001:
                break
        source = transform_points(points, transform)
        distances, indices = self.tree.query(source)
        valid = (distances < 0.25) & self.planar[indices]
        if colors is not None and self.colors is not None:
            valid &= np.linalg.norm(colors - self.colors[indices], axis=1) < 0.45
        if valid.sum() < 60 or valid.mean() < 0.2:
            return None
        p, normal = source[valid], self.normals[indices[valid]]
        residual = np.sum(normal * (self.points[indices[valid]] - p), axis=1)
        rmse = float(np.sqrt(np.mean(residual**2)))
        if rmse > 0.07:
            return None
        jacobian = np.c_[normal, np.cross(p, normal)]
        hessian = jacobian.T @ jacobian
        eigenvalues, eigenvectors = np.linalg.eigh(hessian)
        observable = eigenvalues > eigenvalues[-1] * 0.0001
        if not allow_partial and not observable.all():
            return None
        # Neighbor returns are correlated: inflate the least-squares covariance
        # and retain conservative floors rather than dividing confidence by N.
        variances = np.full(6, 1000.0)
        variances[observable] = max(rmse**2, 0.0001) * 30 / eigenvalues[observable]
        covariance = (eigenvectors * variances) @ eigenvectors.T
        covariance += np.diag([0.0004] * 3 + [0.0001] * 3)
        return Registration(transform, covariance, rmse, float(valid.mean()))


class PoseKalman:
    """12-state constant-velocity filter: XYZ, Euler XYZ, and their rates.

    Euler innovations wrap at +/-pi. Intended for upright navigation, not
    motion near the Euler pitch singularity. No accelerometer/control input.
    """

    def __init__(self):
        self.state = np.zeros(12)
        self.state[2] = 0.793  # Initial standing-height convention only.
        self.covariance = np.diag([0.1] * 3 + [0.03] * 3 + [0.2] * 6)
        self.timestamp = None
        self.last_measurement = None

    def predict(self, timestamp):
        if self.timestamp is None:
            self.timestamp = timestamp
            return True
        dt = timestamp - self.timestamp
        if dt < -1e-6:
            return False
        transition = np.eye(12)
        transition[:6, 6:] = np.eye(6) * dt
        acceleration_noise = np.diag([0.8] * 3 + [1.2] * 3)
        process = np.block(
            [
                [acceleration_noise * dt**3 / 3, acceleration_noise * dt**2 / 2],
                [acceleration_noise * dt**2 / 2, acceleration_noise * dt],
            ]
        )
        self.state = transition @ self.state
        self.state[3:6] = wrap(self.state[3:6])
        self.covariance = transition @ self.covariance @ transition.T + process
        self.timestamp = timestamp
        return True

    def update(self, pose, covariance):
        pose, covariance = np.asarray(pose), np.asarray(covariance)
        if (
            pose.shape != (6,)
            or covariance.shape != (6, 6)
            or not np.isfinite(pose).all()
            or not np.isfinite(covariance).all()
            or not np.allclose(covariance, covariance.T)
            or np.linalg.eigvalsh(covariance)[0] <= 0
        ):
            return False
        innovation = pose - self.state[:6]
        innovation[3:] = wrap(innovation[3:])
        residual_covariance = self.covariance[:6, :6] + covariance
        if innovation @ np.linalg.solve(residual_covariance, innovation) > 22.46:
            return False
        gain = np.linalg.solve(residual_covariance, self.covariance[:6, :]).T
        self.state += gain @ innovation
        self.state[3:6] = wrap(self.state[3:6])
        residual = np.eye(12)
        residual[:, :6] -= gain
        self.covariance = residual @ self.covariance @ residual.T + gain @ covariance @ gain.T
        self.covariance = (self.covariance + self.covariance.T) / 2
        self.last_measurement = self.timestamp
        return True
