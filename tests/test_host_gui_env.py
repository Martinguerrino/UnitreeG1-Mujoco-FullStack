"""Native GTK must not load VS Code Snap schemas; GPU selection stays intact."""
import json
from pathlib import Path
import subprocess


HELPER = Path(__file__).parents[1] / 'scripts/host_gui_env.sh'
KEYS = ('GSETTINGS_SCHEMA_DIR', 'GIO_MODULE_DIR', 'GIO_EXTRA_MODULES',
        'XDG_DATA_DIRS', 'GTK_IM_MODULE_FILE', 'MUJOCO_GL',
        '__GLX_VENDOR_LIBRARY_NAME', '__EGL_VENDOR_LIBRARY_FILENAMES',
        'CUDA_VISIBLE_DEVICES')


def cleaned(extra):
    code = 'import json,os; print(json.dumps({k:os.environ.get(k) for k in ' + repr(KEYS) + '}))'
    command = 'source "$1"; source "$1"; /usr/bin/python3 -c "$2"'
    result = subprocess.run(['bash', '-eu', '-c', command, 'gui-test', str(HELPER), code],
                            env={'PATH': '/usr/bin:/bin', **extra},
                            capture_output=True, text=True, check=True)
    return json.loads(result.stdout)


def test_snap_schema_and_modules_are_removed_and_original_paths_restored():
    result = cleaned({
        'GSETTINGS_SCHEMA_DIR': '/home/user/snap/code/276/.local/share/glib-2.0/schemas',
        'GIO_MODULE_DIR': '/home/user/snap/code/common/.cache/gio-modules',
        'GIO_EXTRA_MODULES': '/snap/code/276/usr/lib/gio/modules',
        'GTK_IM_MODULE_FILE': '/home/user/snap/code/common/immodules.cache',
        'XDG_DATA_DIRS': '/snap/code/276/usr/share:/usr/share',
        'XDG_DATA_DIRS_VSCODE_SNAP_ORIG': '/usr/local/share:/usr/share',
    })
    for key in ('GSETTINGS_SCHEMA_DIR', 'GIO_MODULE_DIR', 'GIO_EXTRA_MODULES', 'GTK_IM_MODULE_FILE'):
        assert result[key] is None
    assert result['XDG_DATA_DIRS'] == '/usr/local/share:/usr/share'


def test_without_original_paths_filters_snap_and_retains_host_desktop_paths():
    result = cleaned({'XDG_DATA_DIRS': '/home/user/snap/code/276:/snap/code/276/usr/share:/usr/share:/var/lib/snapd/desktop'})
    assert result['XDG_DATA_DIRS'] == '/usr/share:/var/lib/snapd/desktop'
    assert cleaned({'XDG_DATA_DIRS': '/snap/code/276/usr/share'})['XDG_DATA_DIRS'] == '/usr/local/share:/usr/share'


def test_native_configuration_and_nvidia_selection_are_preserved():
    supplied = {'GSETTINGS_SCHEMA_DIR': '/opt/custom/schemas',
                'GIO_MODULE_DIR': '/opt/custom/gio', 'XDG_DATA_DIRS': '/usr/share',
                'MUJOCO_GL': 'egl', '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
                '__EGL_VENDOR_LIBRARY_FILENAMES': '/usr/share/glvnd/egl_vendor.d/10_nvidia.json',
                'CUDA_VISIBLE_DEVICES': '0'}
    result = cleaned(supplied)
    for key, value in supplied.items():
        assert result[key] == value
