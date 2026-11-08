"""Resident nearest-neighbor search with bounded scratch and CPU speed gating."""

import time
import numpy as np
from scipy.spatial import cKDTree
from ..acceleration import cupy, enabled, disable


_CHOICES = {}


class NeighborIndex:
    def __init__(self, points):
        self.cpu = cKDTree(points)
        self.points = np.asarray(points)
        self.device_points = None
        self.selected = None

    def query(self, points, k=1):
        # Dense brute-force CUDA is quadratic. Large refinement/overlay clouds
        # must stay on the indexed CPU search, not calibrate billions of pairs.
        size = len(np.asarray(points)) if np.asarray(points).ndim == 2 else 0
        key = (len(self.points).bit_length(), size.bit_length(), k)
        if len(self.points) > 4096 or size > 4096 or _CHOICES.get(key) is False:
            return self.cpu.query(points, k=k)
        cp = cupy() if enabled() and len(self.points) >= 1024 else None
        if cp is None or self.selected is False:
            return self.cpu.query(points, k=k)
        try:
            points = np.asarray(points)
            if points.ndim != 2 or len(points) < 512 or k > len(self.points):
                return self.cpu.query(points, k=k)
            if self.device_points is None:
                self.device_points = cp.asarray(self.points, dtype=cp.float64)
            result = self._query(cp, points, k)
            if self.selected is None:
                start = time.perf_counter()
                expected = self.cpu.query(points, k=k)
                cpu_time = time.perf_counter() - start
                start = time.perf_counter()
                result = self._query(cp, points, k)
                gpu_time = time.perf_counter() - start
                self.selected = bool(
                    gpu_time < cpu_time
                    and np.allclose(result[0], expected[0])
                    and np.array_equal(result[1], expected[1])
                )
                _CHOICES[key] = self.selected
                if not self.selected:
                    self.device_points = None
                    return expected
            return result
        except Exception as error:  # noqa: BLE001 - retry optional backend on CPU
            self.selected = False
            self.device_points = None
            disable('neighbors', error)
            return self.cpu.query(points, k=k)

    def _query(self, cp, points, k):
        distances, indices = [], []
        reference = self.device_points
        block = max(1, min(256, 4_000_000 // len(reference)))
        for start in range(0, len(points), block):
            query = cp.asarray(points[start : start + block], dtype=cp.float64)
            # Direct differences preserve accuracy for points far from the origin.
            squared = cp.zeros((len(query), len(reference)), dtype=cp.float64)
            for axis in range(3):
                squared += (query[:, None, axis] - reference[None, :, axis]) ** 2
            selected = cp.argpartition(squared, k - 1, axis=1)[:, :k]
            values = cp.take_along_axis(squared, selected, axis=1)
            order = cp.argsort(values, axis=1)
            selected = cp.take_along_axis(selected, order, axis=1)
            values = cp.sqrt(cp.take_along_axis(values, order, axis=1))
            distances.append(cp.asnumpy(values[:, 0] if k == 1 else values))
            indices.append(cp.asnumpy(selected[:, 0] if k == 1 else selected))
        return np.concatenate(distances), np.concatenate(indices)

    def query_ball_point(self, points, radius, **kwargs):
        from .spatial import radius_counts

        if kwargs == {'return_length': True} and np.array_equal(points, self.points):
            result = radius_counts(self.points, radius)
            if result is not None:
                return result
        return self.cpu.query_ball_point(points, radius, **kwargs)
