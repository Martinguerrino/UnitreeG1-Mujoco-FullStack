"""Measure warmed CPU/CUDA boundaries including host/device transfers."""

import argparse
import json
import os
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_backend.cuda import geometry
from g1_backend.cuda.spatial import radius_counts
from scipy.spatial import cKDTree


def measure(callback, repeats):
    start = time.perf_counter()
    result = callback()
    warmup = time.perf_counter() - start
    if result is None:
        return {'available': False, 'warmup_seconds': warmup}, None
    samples = []
    for _ in range(repeats):
        start = time.perf_counter()
        result = callback()
        samples.append((time.perf_counter() - start) * 1000)
    return {
        'available': True,
        'warmup_seconds': warmup,
        'median_ms': float(np.median(samples)),
        'p95_ms': float(np.percentile(samples, 95)),
    }, result


def run(repeats):
    rng = np.random.default_rng(42)
    cloud = rng.normal(size=(40000, 3))
    matrix = np.eye(4)
    matrix[:3, 3] = [1, 2, 3]
    neighbors = rng.uniform(-1, 1, (10000, 3))
    tree = cKDTree(neighbors)
    jobs = {
        'transform_40000': (
            lambda: cloud @ matrix[:3, :3].T + matrix[:3, 3],
            lambda: geometry.transform(cloud, matrix),
        ),
        'voxel_40000': (
            lambda: np.unique(np.floor(cloud / 0.08).astype(np.int64), axis=0, return_index=True)[
                1
            ],
            lambda: geometry.voxel_indices(cloud, 0.08),
        ),
        'radius_counts_10000': (
            lambda: tree.query_ball_point(neighbors, 0.08, return_length=True),
            lambda: radius_counts(neighbors, 0.08),
        ),
    }
    original = os.environ.get('G1_GPU')
    report = {}
    try:
        for name, (cpu, gpu) in jobs.items():
            os.environ['G1_GPU'] = 'off'
            cpu_report, expected = measure(cpu, repeats)
            os.environ['G1_GPU'] = 'auto'
            gpu_report, actual = measure(gpu, repeats)
            result = {'cpu': cpu_report, 'cuda': gpu_report}
            if actual is not None:
                result['equivalent'] = bool(np.allclose(actual, expected, rtol=1e-7, atol=1e-10))
                result['speedup'] = cpu_report['median_ms'] / gpu_report['median_ms']
            report[name] = result
    finally:
        if original is None:
            os.environ.pop('G1_GPU', None)
        else:
            os.environ['G1_GPU'] = original
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repeats', type=int, default=7)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not 1 <= args.repeats <= 100:
        parser.error('--repeats must be between 1 and 100')
    report = run(args.repeats)
    text = json.dumps(report, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + '\n')
    print(text)
