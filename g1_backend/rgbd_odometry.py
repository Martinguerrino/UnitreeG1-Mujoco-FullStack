"""RGB feature/depth odometry with a colored depth registration fallback."""

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from .sensor_odometry import CloudReference, Registration, transform_points


def depth_cloud(rgb, depth, intrinsic, stride=4):
    v, u = np.mgrid[0 : depth.shape[0] : stride, 0 : depth.shape[1] : stride]
    z = depth[v, u]
    valid = np.isfinite(z) & (z > 0.15) & (z < 12)
    u, v, z = u[valid], v[valid], z[valid]
    xyz = np.c_[
        (u - intrinsic[0, 2]) * z / intrinsic[0, 0], (v - intrinsic[1, 2]) * z / intrinsic[1, 1], z
    ]
    return xyz, rgb[v, u].astype(float) / 255


class RgbdReference:
    def __init__(self, rgb, depth, intrinsic):
        self.intrinsic = np.asarray(intrinsic)
        self.depth = depth
        from .cuda.images import Orb

        self.orb = Orb(cv2, nfeatures=1600, fastThreshold=8)
        self.keypoints, self.descriptors = self.orb.detectAndCompute(rgb, None)
        points, colors = depth_cloud(rgb, depth, intrinsic)
        self.geometry = CloudReference(points, colors)

    def register(self, rgb, depth, intrinsic, initial=None):
        """Return reference camera <- current camera, in meters from depth."""
        feature_result = self.features(rgb, depth, intrinsic)
        if feature_result is not None:
            return feature_result
        points, colors = depth_cloud(rgb, depth, intrinsic)
        result = self.geometry.register(points, initial, colors, allow_partial=True)
        if result is not None:
            result.method = 'rgbd_colored_depth'
            result.covariance *= 2  # A narrow camera view is less informative than LiDAR.
        return result

    def features(self, rgb, depth, intrinsic):
        keypoints, descriptors = self.orb.detectAndCompute(rgb, None)
        if self.descriptors is None or descriptors is None or len(descriptors) < 15:
            return None
        matches = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(self.descriptors, descriptors, k=2)
        matches = [
            pair[0]
            for pair in matches
            if len(pair) == 2 and pair[0].distance < 0.7 * pair[1].distance
        ]
        if len(matches) < 15:
            return None
        objects, pixels, current_depth = [], [], []
        for match in matches:
            u, v = self.keypoints[match.queryIdx].pt
            cu, cv = keypoints[match.trainIdx].pt
            z = self.depth[round(v), round(u)]
            cz = depth[round(cv), round(cu)]
            if not (np.isfinite(z) and np.isfinite(cz) and 0.15 < z < 12 and 0.15 < cz < 12):
                continue
            objects.append(
                [
                    (u - self.intrinsic[0, 2]) * z / self.intrinsic[0, 0],
                    (v - self.intrinsic[1, 2]) * z / self.intrinsic[1, 1],
                    z,
                ]
            )
            pixels.append([cu, cv])
            current_depth.append(cz)
        if len(objects) < 15:
            return None
        objects, pixels = np.asarray(objects, float), np.asarray(pixels, float)
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            objects,
            pixels,
            np.asarray(intrinsic),
            None,
            iterationsCount=100,
            reprojectionError=2.0,
            confidence=0.995,
            flags=cv2.SOLVEPNP_EPNP,
        )
        if not ok or inliers is None or len(inliers) < 15 or len(inliers) / len(objects) < 0.4:
            return None
        indices = inliers.ravel()
        # Refine only the RANSAC inliers and verify against current metric depth.
        rvec, tvec = cv2.solvePnPRefineLM(
            objects[indices], pixels[indices], intrinsic, None, rvec, tvec
        )
        forward = np.eye(4)  # current camera <- reference camera
        forward[:3, :3] = Rotation.from_rotvec(rvec.ravel()).as_matrix()
        forward[:3, 3] = tvec.ravel()
        predicted = transform_points(objects[indices], forward)
        depth_error = np.abs(predicted[:, 2] - np.asarray(current_depth)[indices])
        if np.median(depth_error) > 0.12:
            return None
        projected, _ = cv2.projectPoints(objects[indices], rvec, tvec, intrinsic, None)
        rmse = float(np.sqrt(np.mean(np.sum((projected[:, 0] - pixels[indices]) ** 2, axis=1))))
        if rmse > 2.0:
            return None
        # Use actual feature distribution to reject concentration in one image patch.
        if np.linalg.eigvalsh(np.cov(pixels[indices].T))[0] < 100:
            return None
        noise = max(0.02, float(np.median(depth_error)), 0.01 * rmse)
        covariance = np.diag([noise**2] * 3 + [max(0.01, rmse / 100) ** 2] * 3)
        return Registration(
            np.linalg.inv(forward), covariance, rmse, len(indices) / len(objects), 'rgbd_features'
        )
