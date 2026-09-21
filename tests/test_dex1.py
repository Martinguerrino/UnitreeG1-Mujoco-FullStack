"""Dex1 conversion, dynamics and body-bridge layout regressions."""

import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from g1_backend.dex1 import ASSETS, GripperPlant, motor_to_slide, slide_to_motor
from experiments.okura.scene import make_scene


def test_motor_units_and_roundtrip():
    np.testing.assert_allclose(motor_to_slide([0, 5.4]), [-0.02, 0.024])
    samples = np.linspace(0, 5.4, 31)
    np.testing.assert_allclose(slide_to_motor(motor_to_slide(samples)), samples, atol=1e-14)
    np.testing.assert_allclose(motor_to_slide([-10, 100]), [-0.02, 0.024])
    with pytest.raises(ValueError):
        motor_to_slide(np.nan)


def test_force_servo_moves_both_fingers_and_damps_on_loss():
    root = ET.Element('mujoco')
    ET.SubElement(root, 'option', timestep='.002', gravity='0 0 0')
    world, actuators = ET.SubElement(root, 'worldbody'), ET.SubElement(root, 'actuator')
    for side in ('left', 'right'):
        for finger in (1, 2):
            name = f'{side}_dex1_finger_joint_{finger}'
            body = ET.SubElement(world, 'body')
            ET.SubElement(
                body, 'inertial', pos='0 0 0', mass='.086783', diaginertia='.001 .001 .001'
            )
            ET.SubElement(body, 'joint', name=name, type='slide', range='-.02 .0245')
            ET.SubElement(actuators, 'motor', name=name, joint=name, ctrlrange='-20 20')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    data, plant = mujoco.MjData(model), GripperPlant(model)
    for target in ([5.4, 0.0], [0.0, 5.4]):
        for _ in range(1000):
            plant.advance(data, target)
            assert np.max(np.abs(data.ctrl)) <= 20
            mujoco.mj_step(model, data)
        np.testing.assert_allclose(plant.state(data), target, atol=0.01)
    data.qvel[:] = 0.1
    plant.advance(data, None)
    assert np.all(data.ctrl < 0)
    data.qpos[plant.qadr] = -0.02 + 0.00001
    np.testing.assert_array_equal(plant.encoder_state(data), [0, 0])
    assert np.all(plant.state(data) > 0)  # Quantized DDS state, not a qpos mutation.


@pytest.mark.skipif(
    not (ASSETS / 'Dex1_base_link.STL').exists(), reason='Run scripts/setup_dex1.py'
)
def test_extended_model_preserves_sdk_sensor_layout_and_free_object():
    base, _ = make_scene()
    model, _ = make_scene(dex1=True)
    assert model.nu == 33
    np.testing.assert_array_equal(model.sensor_adr, base.sensor_adr)
    for i in range(29):
        assert model.actuator(i).name == base.actuator(i).name
    assert model.joint('box_free').type == mujoco.mjtJoint.mjJNT_FREE
    assert model.neq == 1
    assert model.body(model.eq_obj1id[0]).name == 'pelvis'
    for side in ('left', 'right'):
        for finger in (1, 2):
            joint = model.joint(f'{side}_dex1_finger_joint_{finger}')
            assert joint.type == mujoco.mjtJoint.mjJNT_SLIDE
            np.testing.assert_allclose(joint.range, [-0.02, 0.0245])


@pytest.mark.skipif(
    not (ASSETS / 'Dex1_base_link.STL').exists(), reason='Run scripts/setup_dex1.py'
)
@pytest.mark.parametrize(
    'shape,size,mass', [('box', '.03 .03 .03', '.1'), ('ellipsoid', '.045 .01 .009', '.013')]
)
def test_official_collision_mesh_grips_then_releases_free_object(shape, size, mass):
    from g1_backend.dex1 import add_dex1

    root = ET.Element('mujoco')
    ET.SubElement(root, 'compiler', angle='radian')
    ET.SubElement(root, 'option', timestep='.002', integrator='implicitfast', gravity='0 0 0')
    ET.SubElement(root, 'asset')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(root, 'actuator')
    for side, y in [('left', 0), ('right', 1)]:
        ET.SubElement(world, 'body', name=f'{side}_wrist_yaw_link', pos=f'0 {y} .3')
    box = ET.SubElement(world, 'body', name='test_box', pos='.16 0 .3')
    ET.SubElement(box, 'freejoint', name='box_free')
    ET.SubElement(
        box,
        'geom',
        name='box',
        type=shape,
        size=size,
        mass=mass,
        friction='1 .01 .001',
        condim='4',
    )
    add_dex1(root)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    data, plant = mujoco.MjData(model), GripperPlant(model)
    data.qpos[plant.qadr] = motor_to_slide(5.4)
    mujoco.mj_forward(model, data)

    # Physical opening direction is verified against mesh vertices, not motor labels.
    def inner_gap():
        bounds = []
        for finger in (1, 2):
            geom = model.geom(f'left_dex1_finger_link_{finger}_collision')
            mesh = geom.dataid[0]
            start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            verts = (
                model.mesh_vert[start : start + count] @ data.geom_xmat[geom.id].reshape(3, 3).T
                + data.geom_xpos[geom.id]
            )
            bounds.append((verts[:, 1].min(), verts[:, 1].max()))
        return bounds[1][0] - bounds[0][1]

    assert inner_gap() > 0.085
    for _ in range(500):
        plant.advance(data, [0, 0])
        mujoco.mj_step(model, data)
    touched = set()
    for contact in data.contact:
        if model.geom('box').id in (contact.geom1, contact.geom2):
            other = contact.geom2 if contact.geom1 == model.geom('box').id else contact.geom1
            touched.add(model.body(model.geom_bodyid[other]).name)
    assert {'left_dex1_finger_link_1', 'left_dex1_finger_link_2'} <= touched
    assert plant.state(data)[0] > 0.1  # Contact blocks the zero-motor closure target.
    # Gravity is enabled after closure in this isolated contact fixture.
    # There are no equality constraints and the object is never attached.
    assert model.neq == 0
    model.opt.gravity[:] = [0, 0, -9.81]
    for _ in range(500):
        plant.advance(data, [0, 0])
        mujoco.mj_step(model, data)
    assert abs(data.xpos[model.body('test_box').id, 2] - 0.3) < 0.005
    for _ in range(250):
        plant.advance(data, [5.4, 5.4])
        mujoco.mj_step(model, data)
    assert data.xpos[model.body('test_box').id, 2] < 0.15
