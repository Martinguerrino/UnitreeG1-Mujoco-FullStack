"""Isolated visual fixture for ACT input tests, not a Dex1 grasp environment."""

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .checkpoint import STATE_NAMES


ROOT = Path(__file__).resolve().parents[2]


def make_scene() -> tuple[mujoco.MjModel, mujoco.MjData]:
    root = ET.parse(ROOT / 'src/g1_mujoco/mjcf/g1_29dof.xml').getroot()
    root.find('compiler').set('meshdir', str(ROOT / 'src/g1_description/meshes'))
    ET.SubElement(root, 'option', timestep='0.002', integrator='implicitfast')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='640', offheight='480')
    world = root.find('worldbody')
    ET.SubElement(world, 'light', pos='0 0 3', dir='0 0 -1', diffuse='0.8 0.8 0.8')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='3 3 .1', rgba='.25 .3 .35 1')
    ET.SubElement(
        world,
        'geom',
        name='table',
        type='box',
        pos='.55 0 .70',
        size='.30 .5 .025',
        rgba='.65 .5 .35 1',
    )
    box = ET.SubElement(world, 'body', name='box', pos='.4 -.15 .755')
    ET.SubElement(box, 'freejoint', name='box_free')
    ET.SubElement(
        box,
        'geom',
        name='box_collision',
        type='box',
        size='.03 .03 .03',
        mass='.1',
        rgba='.85 .2 .1 1',
        friction='.8 .01 .001',
    )
    ET.SubElement(
        world,
        'site',
        name='destination',
        pos='.4 .15 .726',
        type='box',
        size='.055 .055 .001',
        rgba='.1 .7 .2 .5',
    )
    # Optical placement is an uncalibrated head-height hypothesis, not the dataset camera.
    torso = root.find(".//body[@name='torso_link']")
    ET.SubElement(
        torso,
        'camera',
        name='cam_left_high',
        pos='.08 .025 .4',
        xyaxes='0 -1 0 .7071 0 .7071',
        fovy='60',
    )
    equality = ET.SubElement(root, 'equality')
    ET.SubElement(equality, 'weld', name='pelvis_fixture', body1='pelvis')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def set_initial_arm_pose(model, data, state: np.ndarray) -> None:
    """Reset-only pose assignment, never an execution or grasp mechanism."""
    if state.shape != (16,) or not np.isfinite(state).all():
        raise ValueError('Expected a finite 16-element state')
    for name, value in zip(STATE_NAMES[:14], state[:14], strict=True):
        joint = model.joint(name)
        if not joint.range[0] <= value <= joint.range[1]:
            raise ValueError(f'Initial joint outside model limits: {name}')
        data.qpos[joint.qposadr[0]] = value
    mujoco.mj_forward(model, data)


def render_head(model, data) -> np.ndarray:
    with mujoco.Renderer(model, height=480, width=640) as renderer:
        renderer.update_scene(data, camera='cam_left_high')
        return renderer.render().copy()
