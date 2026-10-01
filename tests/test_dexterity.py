"""End-effector variants preserve the body controller's model contract."""

import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from g1_backend.config import ROOT, load_config
from g1_backend.dexterity import description_with_claws
from g1_backend.simulator import model_xml


def test_claws_preserve_body_sensors_and_actuators(tmp_path):
    config = load_config(ROOT / 'configs/experiments/navigation.yaml')
    models = []
    for variant in ('hand', 'claw'):
        config['robot']['dexterity'] = variant
        model, scene = model_xml(config)
        (tmp_path / 'g1_29dof.xml').write_text(model)
        (tmp_path / 'scene.xml').write_text(scene)
        models.append(mujoco.MjModel.from_xml_path(str(tmp_path / 'scene.xml')))
    hand, claw = models
    assert hand.nu == 29 and claw.nu == 33
    assert [hand.actuator(i).name for i in range(29)] == [
        claw.actuator(i).name for i in range(29)
    ]
    np.testing.assert_array_equal(hand.sensor_adr, claw.sensor_adr)
    assert claw.sensor('imu_quat').adr[0] == 87
    assert claw.body('left_dex1_base_link').id > 0
    with pytest.raises(KeyError):
        claw.geom('left_rubber_hand')


def test_description_replaces_hands_and_resolves_meshes():
    xml = '<robot name="test"><link name="left_rubber_hand"/><joint name="palm" type="fixed"><parent link="left_wrist_yaw_link"/><child link="left_rubber_hand"/></joint></robot>'
    root = ET.fromstring(description_with_claws(xml))
    assert root.find("link[@name='left_rubber_hand']") is None
    assert root.find("joint[@name='palm']") is None
    assert len(root.findall('link')) == 6
    assert len(root.findall('joint')) == 6
    assert all(mesh.get('filename').startswith('file:///') for mesh in root.findall('.//mesh'))


def test_invalid_dexterity_is_rejected():
    config = load_config(ROOT / 'configs/experiments/navigation.yaml')
    config['robot']['dexterity'] = 'invalid'
    with pytest.raises(ValueError, match='dexterity'):
        load_config(config)
