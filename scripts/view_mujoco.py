#!/usr/bin/env python
"""Load the repository G1 scene in the official MuJoCo viewer."""

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import tempfile

import mujoco
import mujoco.viewer


ROOT = Path(__file__).parents[1]
MODEL = ROOT / 'src/g1_mujoco/mjcf/g1_29dof.xml'
SCENE = ROOT / 'src/g1_mujoco/scenes/scene_29dof.xml'
MESHES = ROOT / 'src/g1_description/meshes'


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dexterity', choices=['claw', 'hand'], default='hand')
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='g1_viewer_') as directory:
        generated = Path(directory)
        xml = MODEL.read_text().replace('meshdir="meshes"', f'meshdir="{MESHES}"', 1)
        if args.dexterity == 'claw':
            sys.path.insert(0, str(ROOT))
            from g1_backend.dex1 import add_dex1

            root = ET.fromstring(xml)
            add_dex1(root)
            xml = ET.tostring(root, encoding='unicode')
        (generated / MODEL.name).write_text(xml)
        (generated / SCENE.name).write_text(SCENE.read_text())
        model = mujoco.MjModel.from_xml_path(str(generated / SCENE.name))
        data = mujoco.MjData(model)
        mujoco.viewer.launch(model, data)


if __name__ == '__main__':
    main()
