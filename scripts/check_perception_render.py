"""Validate perception on actual MuJoCo-rendered RGB-D of a table and rotated box."""

import atexit
import json
import os
from pathlib import Path
import sys

os.environ['MUJOCO_GL'] = 'egl'
os.environ['PYOPENGL_PLATFORM'] = 'egl'
os.environ['LIBGL_ALWAYS_SOFTWARE'] = 'true'
os.environ['__EGL_VENDOR_LIBRARY_FILENAMES'] = '/usr/share/glvnd/egl_vendor.d/50_mesa.json'

import mujoco
import mujoco.egl
from mujoco.egl import egl_ext as EGL
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_backend.perception import Settings, backproject, detect, transform_points


def initialize_renderer():
    display = EGL.eglGetPlatformDisplayEXT(0x31DD, EGL.EGL_DEFAULT_DISPLAY, None)
    if not EGL.eglInitialize(display, None, None):
        raise RuntimeError('Unable to initialize Mesa/EGL')
    mujoco.egl.EGL_DISPLAY = display
    atexit.register(EGL.eglTerminate, display)


def main():
    initialize_renderer()
    model = mujoco.MjModel.from_xml_string("""
    <mujoco>
      <visual><global offwidth="640" offheight="480"/></visual>
      <worldbody>
        <light pos="0 -1 3"/>
        <camera name="rgbd" pos="0 -1.1 1.4" xyaxes="1 0 0 0 .5087293 .8609265" fovy="58"/>
        <geom type="plane" size="2 2 .1" rgba=".3 .3 .3 1"/>
        <geom type="box" pos="0 0 .72" size=".5 .35 .03" rgba=".5 .35 .2 1"/>
        <geom type="box" pos=".1 .05 .8" size=".08 .04 .05" euler="0 0 22.918312" rgba="1 0 0 1"/>
      </worldbody>
    </mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with mujoco.Renderer(model, height=480, width=640) as renderer:
        renderer.update_scene(data, camera='rgbd')
        rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        depth = renderer.render().copy()
    focal = 480 / (2 * np.tan(np.deg2rad(58) / 2))
    k = [focal, 0, 319.5, 0, focal, 239.5, 0, 0, 1]
    # This is the camera's known extrinsic (the equivalent of TF). Object
    # ground truth below is used only to evaluate results, never by detect().
    extrinsic = np.eye(4)
    extrinsic[:3, :3] = data.cam_xmat[0].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
    extrinsic[:3, 3] = data.cam_xpos[0]
    points, colors = backproject(depth, rgb, k, Settings())
    tables = detect(transform_points(points, extrinsic), colors)
    boxes = [box for table in tables for box in table['objects']]
    assert len(tables) == 1 and len(boxes) == 1, tables
    box = boxes[0]
    center_error = float(np.linalg.norm(np.array(box['center']) - [0.1, 0.05, 0.8]))
    size_error = float(np.max(np.abs(np.array(box['dimensions']) - [0.16, 0.08, 0.1])))
    yaw = Rotation.from_quat(box['orientation_xyzw']).as_euler('xyz')[2]
    yaw_error = float(abs((yaw - 0.4 + np.pi / 2) % np.pi - np.pi / 2))
    report = {
        'tables': tables,
        'center_error_m': center_error,
        'max_dimension_error_m': size_error,
        'yaw_error_rad': yaw_error,
        'passed': center_error < 0.025 and size_error < 0.025 and yaw_error < 0.12,
    }
    output = Path('results/perception')
    output.mkdir(parents=True, exist_ok=True)
    (output / 'render_validation.json').write_text(json.dumps(report, indent=2))
    np.savez_compressed(
        output / 'render_rgbd.npz', rgb=rgb, depth=depth, k=k, camera_to_processing=extrinsic
    )
    print(json.dumps({key: value for key, value in report.items() if key != 'tables'}, indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
