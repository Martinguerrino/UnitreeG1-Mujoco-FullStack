"""Image sensor arithmetic on CUDA; preserve NumPy's seeded random samples."""

import copy
import numpy as np
from ..acceleration import cupy, disable, enabled


def camera(owner, sensors):
    if not enabled() or sum(value.size for value in sensors.values()) < 500000:
        return None
    cp = cupy()
    if cp is None or getattr(owner, '_cuda_failed', False):
        return None
    saved_state = copy.deepcopy(owner.rng.bit_generator.state)
    try:
        cfg = owner.profile['camera']
        output = dict(sensors)
        for name, image in sensors.items():
            if name.endswith('rgb'):
                gain = 1 + owner.rng.normal(0, cfg['exposure_sigma'])
                noise = cp.asarray(owner.rng.normal(0, cfg['rgb_sigma'], image.shape))
                noisy = cp.asarray(image, dtype=cp.float32) * gain + noise
                output[name] = cp.asnumpy(cp.clip(cp.rint(noisy), 0, 255).astype(cp.uint8))
            elif name.endswith('depth'):
                depth = np.asarray(image, dtype=np.float32)
                valid = (
                    np.isfinite(depth)
                    & (depth >= cfg['depth_min_m'])
                    & (depth <= cfg['depth_max_m'])
                )
                safe = np.where(valid, depth, 0)
                # NumPy generates identical noise and advances the original RNG once.
                sigma = cfg['depth_sigma_m'] + cfg['depth_sigma_per_m2'] * safe**2
                noise = cp.asarray(owner.rng.normal(0, sigma))
                dropped = cp.asarray(
                    owner.rng.random(depth.shape) < cfg['depth_dropout_probability']
                )
                edge_drop = cp.asarray(
                    owner.rng.random(depth.shape) < cfg['edge_dropout_probability']
                )
                values = cp.asarray(safe)
                edge = cp.zeros(depth.shape, dtype=bool)
                jump = cp.abs(cp.diff(values, axis=1)) > cfg['depth_edge_m']
                edge[:, 1:] |= jump
                edge[:, :-1] |= jump
                jump = cp.abs(cp.diff(values, axis=0)) > cfg['depth_edge_m']
                edge[1:] |= jump
                edge[:-1] |= jump
                measured = values + noise
                measured = (
                    cp.rint(measured / cfg['depth_quantization_m']) * cfg['depth_quantization_m']
                )
                valid = cp.asarray(valid) & ~dropped & ~(edge & edge_drop)
                valid &= (measured >= cfg['depth_min_m']) & (measured <= cfg['depth_max_m'])
                output[name] = cp.asnumpy(cp.where(valid, measured, 0).astype(cp.float32))
        return output
    except Exception as error:  # noqa: BLE001 - optional CUDA must retain CPU operation
        owner.rng.bit_generator.state = saved_state
        owner._cuda_failed = True
        disable('sensor_effects', error)
        return None
