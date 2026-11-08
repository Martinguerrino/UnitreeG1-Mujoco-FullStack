"""Optional acceleration with a process-wide CPU override and observable fallbacks."""

import importlib
import logging
import os
import ctypes
from pathlib import Path
import sys
import time

import numpy as np

_LOG = logging.getLogger(__name__)
_DISABLED = set()
_LIBRARIES = []
_PRELOADED = False


def preload_libraries():
    """Discover optional pip CUDA libraries without changing system installations."""
    global _PRELOADED
    if _PRELOADED:
        return
    _PRELOADED = True
    roots = [Path(entry) / 'nvidia' for entry in sys.path if entry]
    for library in (
        'libcudart.so.12',
        'libnvrtc.so.12',
        'libcublasLt.so.12',
        'libcublas.so.12',
        'libcudnn.so.9',
    ):
        for root in roots:
            matches = list(root.glob('*/lib/' + library)) if root.is_dir() else []
            if matches:
                try:
                    _LIBRARIES.append(ctypes.CDLL(str(matches[0]), mode=ctypes.RTLD_GLOBAL))
                    break
                except OSError:
                    pass


def enabled():
    return os.environ.get('G1_GPU', 'auto').lower() not in ('0', 'false', 'off', 'cpu')


def disable(component, error):
    if component not in _DISABLED:
        _LOG.warning('GPU %s unavailable; using CPU: %s', component, error)
    _DISABLED.add(component)


def cupy():
    if not enabled() or 'cupy' in _DISABLED:
        return None
    try:
        preload_libraries()
        cp = importlib.import_module('cupy')
        if cp.cuda.runtime.getDeviceCount() < 1:
            return None
        return cp
    except ModuleNotFoundError:
        _DISABLED.add('cupy')
        return None
    except Exception as error:  # noqa: BLE001 - retry optional backend on CPU
        disable('cupy', error)
        return None


def torch_device():
    if not enabled() or 'torch' in _DISABLED:
        return 'cpu'
    try:
        torch = importlib.import_module('torch')
        if torch.cuda.is_available():
            # Availability alone does not establish that this wheel supports the GPU.
            torch.empty(1, device='cuda').add_(1)
            torch.cuda.synchronize()
            return 'cuda'
    except Exception as error:  # noqa: BLE001 - retry optional backend on CPU
        disable('torch', error)
    return 'cpu'


class OnnxSession:
    """CPU-compatible session; retry on CPU if CUDA initialization or execution fails."""

    def __init__(self, ort, path, options, gpu=True, calibrate=False):
        self.ort, self.path, self.options = ort, path, options
        self.backend = 'cpu'
        self.calibrate = calibrate
        self.calibrated = False
        if gpu and enabled() and 'onnx' not in _DISABLED:
            try:
                if 'CUDAExecutionProvider' in ort.get_available_providers():
                    preload_libraries()
                    if hasattr(ort, 'preload_dlls'):
                        ort.preload_dlls(directory='')
                    session = ort.InferenceSession(
                        path,
                        sess_options=options,
                        providers=['CUDAExecutionProvider', 'CPUExecutionProvider'],
                    )
                    if 'CUDAExecutionProvider' in session.get_providers():
                        self.session, self.backend = session, 'cuda'
                        return
            except Exception as error:  # noqa: BLE001 - retry optional backend on CPU
                disable('onnx', error)
        self._cpu()

    def _cpu(self):
        self.session = self.ort.InferenceSession(
            self.path, sess_options=self.options, providers=['CPUExecutionProvider']
        )
        self.backend = 'cpu'

    def run(self, outputs, inputs):
        if self.backend == 'cuda' and not enabled():
            self._cpu()
        try:
            if self.backend == 'cuda' and self.calibrate and not self.calibrated:
                return self._compare_latency(outputs, inputs)
            return self.session.run(outputs, inputs)
        except Exception as error:
            if self.backend != 'cuda':
                raise
            disable('onnx', error)
            self._cpu()
            return self.session.run(outputs, inputs)

    def _compare_latency(self, outputs, inputs):
        """Small control networks should use CUDA only when latency improves."""
        cpu = self.ort.InferenceSession(
            self.path, sess_options=self.options, providers=['CPUExecutionProvider']
        )
        expected = cpu.run(outputs, inputs)
        self.session.run(outputs, inputs)  # warm CUDA before timing
        cpu_samples, gpu_samples = [], []
        for _ in range(3):
            start = time.perf_counter()
            expected = cpu.run(outputs, inputs)
            cpu_samples.append(time.perf_counter() - start)
            start = time.perf_counter()
            actual = self.session.run(outputs, inputs)
            gpu_samples.append(time.perf_counter() - start)
        self.calibrated = True
        equivalent = len(actual) == len(expected) and all(
            np.allclose(a, b, rtol=1e-5, atol=1e-5) for a, b in zip(actual, expected)
        )
        if not equivalent or np.median(gpu_samples) >= np.median(cpu_samples):
            self.session, self.backend = cpu, 'cpu'
            return expected
        return actual

    def __getattr__(self, name):
        return getattr(self.session, name)
