"""Automatic worker environment without modifying the parent ROS environment."""

import json
import subprocess
import sys

from g1_backend.gpu_launch import gpu_process_environment


def test_cpu_mode_does_not_probe_or_change_environment(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('CPU mode must not probe GPU packages')

    monkeypatch.setattr(subprocess, 'check_output', forbidden)
    assert gpu_process_environment('/python', {'G1_GPU': 'off'}) == {}


def test_gpu_worker_keeps_ros_paths_and_deduplicates_site(monkeypatch, tmp_path):
    monkeypatch.setattr(
        subprocess,
        'check_output',
        lambda *a, **k: json.dumps(
            {'version': list(sys.version_info[:2]), 'site': str(tmp_path), 'gpu': True}
        ),
    )
    env = {'PYTHONPATH': '/ros:' + str(tmp_path)}
    overrides = gpu_process_environment('/python', env)
    assert overrides == {
        'G1_GPU_PYTHON_PATH': str(tmp_path),
        'PYTHONPATH': str(tmp_path) + ':/ros',
    }
    assert env == {'PYTHONPATH': '/ros:' + str(tmp_path)}


def test_no_packages_or_python_abi_mismatch_keep_original_worker(monkeypatch, tmp_path):
    for version, gpu in ((list(sys.version_info[:2]), False), ([0, 0], True)):
        monkeypatch.setattr(
            subprocess,
            'check_output',
            lambda *a, version=version, gpu=gpu, **k: json.dumps(
                {'version': version, 'site': str(tmp_path), 'gpu': gpu}
            ),
        )
        assert gpu_process_environment('/python', {}) == {}


def test_failed_probe_keeps_cpu_launcher(monkeypatch):
    def timeout(*args, **kwargs):
        raise subprocess.TimeoutExpired('/python', 10)

    monkeypatch.setattr(subprocess, 'check_output', timeout)
    assert gpu_process_environment('/python', {}) == {}
