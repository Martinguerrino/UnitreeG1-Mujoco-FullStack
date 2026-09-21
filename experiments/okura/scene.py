"""Okura fixtures: original box diagnostic and optional Dex1/okra transfer scene."""

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .checkpoint import STATE_NAMES


ROOT = Path(__file__).resolve().parents[2]


def make_scene(
    *, dex1=False, box_position=(0.4, -0.15, 0.755), object_kind='box', transfer_scene=False
) -> tuple[mujoco.MjModel, mujoco.MjData]:
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
    box = ET.SubElement(world, 'body', name='box', pos=' '.join(map(str, box_position)))
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
    if object_kind == 'okra':
        geom = box.find('geom')
        geom.set('type', 'ellipsoid')
        geom.set('size', '.045 .01 .009')
        geom.set('mass', '.012')
        geom.set('rgba', '.28 .38 .06 1')
        ET.SubElement(
            box,
            'geom',
            name='okra_stem',
            type='capsule',
            fromto='-.045 0 0 -.055 0 0',
            size='.003',
            mass='.001',
            rgba='.42 .45 .12 1',
        )
    elif object_kind != 'box':
        raise ValueError('Unknown object kind')
    if transfer_scene:
        world.find("geom[@name='table']").set('rgba', '.12 .12 .12 1')
        ET.SubElement(
            world,
            'geom',
            name='table_b',
            type='box',
            pos='.55 1.2 .70',
            size='.3 .3 .025',
            rgba='.3 .32 .34 1',
        )
        # Approximation of the demonstration's dark tray and white open basket.
        ET.SubElement(
            world,
            'geom',
            name='tray',
            type='box',
            pos='.5 0 .727',
            size='.23 .27 .002',
            rgba='.025 .025 .025 1',
        )
        # The source video shows raised pegs: support the okra above the tray
        # while leaving space for the fingers. Heights are an explicit estimate.
        for x in np.arange(0.29, 0.72, 0.03):
            for y in np.arange(-0.24, 0.25, 0.015):
                ET.SubElement(
                    world,
                    'geom',
                    type='capsule',
                    fromto=f'{x} {y} .729 {x} {y} .749',
                    size='.0015',
                    rgba='.035 .035 .035 1',
                    friction='.8 .01 .001',
                )
        basket = ET.SubElement(world, 'body', name='basket', pos='.53 .16 .731')
        ET.SubElement(basket, 'geom', type='box', size='.09 .075 .003', rgba='.9 .94 .9 1')
        for z in (0.025, 0.075, 0.125, 0.16):
            for y in (-0.075, 0.075):
                ET.SubElement(
                    basket,
                    'geom',
                    type='box',
                    pos=f'0 {y} {z}',
                    size='.09 .004 .004',
                    rgba='.9 .94 .9 1',
                )
            for x in (-0.09, 0.09):
                ET.SubElement(
                    basket,
                    'geom',
                    type='box',
                    pos=f'{x} 0 {z}',
                    size='.004 .075 .004',
                    rgba='.9 .94 .9 1',
                )
        for x in np.linspace(-0.09, 0.09, 7):
            for y in (-0.075, 0.075):
                ET.SubElement(
                    basket,
                    'geom',
                    type='box',
                    pos=f'{x} {y} .08',
                    size='.003 .003 .08',
                    rgba='.9 .94 .9 1',
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
    if transfer_scene:
        torso.find("camera[@name='cam_left_high']").set('xyaxes', '0 -1 0 .819152 0 .573576')
        world.find("site[@name='destination']").set('rgba', '0 0 0 0')
    equality = ET.SubElement(root, 'equality')
    ET.SubElement(equality, 'weld', name='pelvis_fixture', body1='pelvis')
    if dex1:
        from g1_backend.dex1 import add_dex1

        add_dex1(root)
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
