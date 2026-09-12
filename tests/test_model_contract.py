"""Static contracts shared by URDF, MJCF and ros2_control configuration."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).parents[1]
URDF = ROOT / 'src/g1_description/urdf/g1_29dof.urdf.xacro'
MJCF = ROOT / 'src/g1_mujoco/mjcf/g1_29dof.xml'
CONTROLLERS = ROOT / 'src/g1_control/config/controllers.yaml'


def _controlled_joints():
    root = ET.parse(URDF).getroot()
    control = root.find('ros2_control')
    assert control is not None
    return {element.attrib['name'] for element in control.findall('joint')}


def test_exactly_29_controlled_joints():
    assert len(_controlled_joints()) == 29


def test_urdf_mjcf_controller_joint_names_match():
    controlled = _controlled_joints()
    mjcf_root = ET.parse(MJCF).getroot()
    actuated = {element.attrib['joint'] for element in mjcf_root.findall('./actuator/*')}
    config = yaml.safe_load(CONTROLLERS.read_text())
    configured = set(config['whole_body_controller']['ros__parameters']['joints'])
    assert controlled == actuated == configured


def test_every_referenced_mesh_exists():
    root = ET.parse(URDF).getroot()
    prefix = 'package://g1_description/meshes/'
    refs = [mesh.attrib['filename'] for mesh in root.findall('.//mesh')]
    assert refs
    assert all(ref.startswith(prefix) for ref in refs)
    missing = [
        ref
        for ref in refs
        if not (ROOT / 'src/g1_description/meshes' / ref.removeprefix(prefix)).is_file()
    ]
    assert missing == []


def test_effort_limits_are_present():
    root = ET.parse(MJCF).getroot()
    joints = {
        joint.attrib['name']: joint for joint in root.findall('.//joint') if 'name' in joint.attrib
    }
    for name in _controlled_joints():
        assert 'actuatorfrcrange' in joints[name].attrib
