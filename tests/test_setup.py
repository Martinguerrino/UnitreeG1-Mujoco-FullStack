"""Portable interpreter selection and the installer's orchestration contract."""

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

from g1_backend.python_runtime import resolve_python

ROOT = Path(__file__).resolve().parents[1]


def executable(path, text='#!/bin/sh\nexit 0\n'):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    path.chmod(0o755)
    return str(path)


def test_explicit_python_overrides_saved_and_active_environments(tmp_path):
    saved = executable(tmp_path / 'saved/bin/python')
    chosen = executable(tmp_path / 'custom env/bin/python')
    (tmp_path / '.g1-runtime.json').write_text(json.dumps({'python': saved}))
    assert resolve_python(tmp_path, {'G1_PYTHON': chosen, 'CONDA_PREFIX': '/unused'}) == chosen
    assert resolve_python(tmp_path, {'CONDA_PREFIX': '/unused'}) == saved


@pytest.mark.parametrize('variable', ['CONDA_PREFIX', 'VIRTUAL_ENV'])
def test_uses_any_active_environment_name(tmp_path, variable):
    prefix = tmp_path / 'my-robot-environment'
    python = executable(prefix / 'bin/python')
    assert resolve_python(tmp_path, {variable: str(prefix)}) == python


def test_venv_symlink_is_preserved_and_missing_configuration_is_reported(tmp_path):
    target = executable(tmp_path / 'system-python')
    python = tmp_path / '.venv/bin/python'
    python.parent.mkdir(parents=True)
    python.symlink_to(target)
    assert resolve_python(tmp_path, {}) == str(python)
    with pytest.raises(RuntimeError, match='setup.sh'):
        resolve_python(tmp_path, {'G1_PYTHON': str(tmp_path / 'deleted/bin/python')})


def test_bundled_model_checksum_is_checked_before_install(tmp_path):
    source = tmp_path / 'assets/perception'
    source.mkdir(parents=True)
    model = source / 'g1_objects_yolo11n_seg.onnx'
    model.write_bytes(b'test-model')
    (source / 'model_manifest.json').write_text(
        json.dumps({'onnx_sha256': hashlib.sha256(model.read_bytes()).hexdigest()})
    )
    spec = importlib.util.spec_from_file_location(
        'install_model', ROOT / 'scripts/install_perception_model.py'
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.install_model(tmp_path)
    destination = tmp_path / 'artifacts/perception' / model.name
    assert destination.read_bytes() == b'test-model'
    model.write_bytes(b'corrupted-model')
    with pytest.raises(ValueError, match='SHA-256'):
        module.install_model(tmp_path)
    assert destination.read_bytes() == b'test-model'


def setup_fixture(tmp_path):
    shutil.copy2(ROOT / 'setup.sh', tmp_path / 'setup.sh')
    (tmp_path / 'g1_backend').mkdir()
    shutil.copy2(ROOT / 'g1_backend/python_runtime.py', tmp_path / 'g1_backend/python_runtime.py')
    for name in [
        'install_host_dependencies',
        'install_ros_dependencies',
        'bootstrap',
        'setup_perception_yolo',
        'setup_okura',
        'install_navigation_slam',
        'build',
        'check_installation',
    ]:
        executable(
            tmp_path / 'scripts' / f'{name}.sh',
            f'#!/bin/sh\nprintf "{name}\\n" >> "$SETUP_TEST_LOG"\n',
        )
    python = executable(
        tmp_path / 'my environment/bin/python',
        '#!/bin/sh\nprintf "python %s\\n" "$*" >> "$SETUP_TEST_LOG"\n',
    )
    env = {
        key: value
        for key, value in os.environ.items()
        if key not in ('G1_PYTHON', 'CONDA_PREFIX', 'VIRTUAL_ENV')
    }
    env['SETUP_TEST_LOG'] = str(tmp_path / 'calls.log')
    return python, env


def test_full_setup_calls_every_component_and_saves_custom_python(tmp_path):
    if 'VERSION_ID="24.04"' not in Path('/etc/os-release').read_text():
        pytest.skip('Full setup targets Ubuntu 24.04')
    python, env = setup_fixture(tmp_path)
    result = subprocess.run(
        ['bash', str(tmp_path / 'setup.sh'), '--python', python],
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert json.loads((tmp_path / '.g1-runtime.json').read_text())['python'] == python
    calls = (tmp_path / 'calls.log').read_text()
    for component in [
        'install_host_dependencies',
        'install_ros_dependencies',
        'bootstrap',
        'setup_dds.py',
        'setup_dex1.py',
        'setup_perception_yolo',
        'setup_okura',
        'install_navigation_slam',
        'build',
        'check_installation',
    ]:
        assert component in calls


def test_dry_run_on_clean_clone_creates_no_environment_or_configuration(tmp_path):
    _, env = setup_fixture(tmp_path)
    result = subprocess.run(
        ['bash', str(tmp_path / 'setup.sh'), '--dry-run'],
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert '-m venv' in result.stdout
    assert not (tmp_path / '.venv').exists()
    assert not (tmp_path / '.g1-runtime.json').exists()
    assert not (tmp_path / 'calls.log').exists()


def test_skip_options_omit_installation_of_unselected_components(tmp_path):
    python, env = setup_fixture(tmp_path)
    result = subprocess.run(
        [
            'bash',
            str(tmp_path / 'setup.sh'),
            '--python',
            python,
            '--skip-system',
            '--skip-slam3d',
            '--skip-experiments',
            '--skip-build',
        ],
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    calls = (tmp_path / 'calls.log').read_text()
    for component in [
        'install_host_dependencies',
        'install_ros_dependencies',
        'setup_okura',
        'install_navigation_slam',
        'build',
    ]:
        assert component not in calls
    assert 'setup_perception_yolo' in calls


def test_failed_component_stops_setup_before_reporting_success(tmp_path):
    python, env = setup_fixture(tmp_path)
    executable(tmp_path / 'scripts/bootstrap.sh', '#!/bin/sh\nexit 17\n')
    result = subprocess.run(
        ['bash', str(tmp_path / 'setup.sh'), '--python', python],
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 17
    assert 'setup_perception_yolo' not in (tmp_path / 'calls.log').read_text()
    assert 'Instalación finalizada.' not in result.stdout
