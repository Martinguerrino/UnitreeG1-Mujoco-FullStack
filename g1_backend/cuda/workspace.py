"""Per-frame device workspace reused across RGB-D masks and support geometry."""

from contextlib import contextmanager
from contextvars import ContextVar

_ACTIVE = ContextVar('g1_cuda_frame', default=None)


class FrameWorkspace:
    def __init__(self, cp, depth, rgb):
        self.cp = cp
        self.host_depth, self.host_rgb = depth, rgb
        self.depth, self.rgb = cp.asarray(depth), cp.asarray(rgb)
        self.grids = {}

    def backproject(self, depth, rgb, k, settings, mask=None, transform=None):
        import numpy as np

        settings.validate()
        if depth.ndim != 2 or rgb.shape != (*depth.shape, 3):
            raise ValueError('RGB and depth must be registered and have equal resolution')
        k = np.asarray(k, dtype=float).reshape(3, 3)
        if not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
            raise ValueError('Invalid camera intrinsics')
        cp, s = self.cp, settings
        shape = (depth.shape, s.stride)
        if shape not in self.grids:
            self.grids[shape] = cp.mgrid[
                0 : depth.shape[0] : s.stride, 0 : depth.shape[1] : s.stride
            ]
        v, u = self.grids[shape]
        values = self.depth if depth is self.host_depth else cp.asarray(depth)
        colors = self.rgb if rgb is self.host_rgb else cp.asarray(rgb)
        z = values[:: s.stride, :: s.stride]
        valid = cp.isfinite(z) & (z >= s.min_depth) & (z <= s.max_depth)
        if mask is not None:
            valid &= cp.asarray(mask)[:: s.stride, :: s.stride]
        points = cp.stack([(u - k[0, 2]) * z / k[0, 0], (v - k[1, 2]) * z / k[1, 1], z], axis=-1)
        points = points[valid]
        if transform is not None:
            points = points @ cp.asarray(transform[:3, :3]).T + cp.asarray(transform[:3, 3])
        return cp.asnumpy(points), cp.asnumpy(colors[:: s.stride, :: s.stride][valid])


def active():
    return _ACTIVE.get()


@contextmanager
def frame_scope(depth, rgb):
    from ..acceleration import cupy, disable, enabled, _DISABLED

    workspace = None
    cp = cupy() if enabled() and 'workspace' not in _DISABLED and depth.size >= 131072 else None
    if cp is not None:
        try:
            workspace = FrameWorkspace(cp, depth, rgb)
        except Exception as error:  # noqa: BLE001 - optional allocation must not break CPU
            disable('workspace', error)
    token = _ACTIVE.set(workspace)
    try:
        yield workspace
    finally:
        _ACTIVE.reset(token)
