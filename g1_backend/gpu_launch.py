"""Propagate project GPU packages to Python workers without changing the ROS CLI."""

import json
import os
from pathlib import Path
import subprocess
import sys
import logging

_PROBE = """import importlib.metadata as m, json, sys, sysconfig
names = ('onnxruntime-gpu', 'cupy-cuda12x', 'cupy-cuda11x')
found = False
for name in names:
    try:
        m.distribution(name)
        found = True
    except m.PackageNotFoundError:
        pass
print(json.dumps({'version': list(sys.version_info[:2]),
                  'site': sysconfig.get_path('purelib'), 'gpu': found}))
"""


def gpu_process_environment(python, environ=None):
    """Return worker overrides only when packages match the launcher's Python ABI."""
    env = os.environ if environ is None else environ
    if env.get('G1_GPU', 'auto').lower() in ('off', 'false', '0', 'cpu'):
        return {}
    try:
        result = json.loads(
            subprocess.check_output(
                [str(python), '-c', _PROBE], text=True, timeout=10, stderr=subprocess.PIPE
            )
        )
        if result['version'] != list(sys.version_info[:2]):
            return {}
        site = env.get('G1_GPU_PYTHON_PATH') or result['site']
        if not (result['gpu'] or env.get('G1_GPU_PYTHON_PATH')) or not Path(site).is_dir():
            return {}
        paths = [site, *filter(None, env.get('PYTHONPATH', '').split(os.pathsep))]
        return {'G1_GPU_PYTHON_PATH': site, 'PYTHONPATH': os.pathsep.join(dict.fromkeys(paths))}
    except (OSError, subprocess.SubprocessError, ValueError, KeyError) as error:
        logging.getLogger(__name__).warning('GPU worker environment unavailable: %s', error)
        return {}
