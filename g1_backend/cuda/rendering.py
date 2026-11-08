"""EGL selection for offline rendering; physics is unchanged."""

import atexit
import os
from ..acceleration import enabled

# These must be set before importing MuJoCo/OpenGL in offline entry points.
os.environ.setdefault('MUJOCO_GL', 'egl')
os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
if not enabled():
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = 'true'
    os.environ['__EGL_VENDOR_LIBRARY_FILENAMES'] = '/usr/share/glvnd/egl_vendor.d/50_mesa.json'


def initialize_renderer():
    import mujoco.egl
    from mujoco.egl import egl_ext as EGL

    display = EGL.EGL_NO_DISPLAY
    if enabled():
        try:
            display = mujoco.egl.create_initialized_egl_device_display()
        except RuntimeError:
            pass
    if display == EGL.EGL_NO_DISPLAY:
        display = EGL.eglGetPlatformDisplayEXT(0x31DD, EGL.EGL_DEFAULT_DISPLAY, None)
        if not EGL.eglInitialize(display, None, None):
            raise RuntimeError('Unable to initialize EGL GPU or software rendering')
    mujoco.egl.EGL_DISPLAY = display
    atexit.register(EGL.eglTerminate, display)
