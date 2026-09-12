"""Load the real G1 scene through MuJoCo's parser."""

from pathlib import Path
import tempfile

import mujoco


ROOT = Path(__file__).parents[1]


def test_scene_loads_and_has_expected_dimensions():
    model_source = ROOT / 'src/g1_mujoco/mjcf/g1_29dof.xml'
    scene_source = ROOT / 'src/g1_mujoco/scenes/scene_29dof.xml'
    mesh_dir = ROOT / 'src/g1_description/meshes'
    with tempfile.TemporaryDirectory(prefix='g1_test_') as directory:
        temp = Path(directory)
        model_xml = model_source.read_text().replace(
            'meshdir="meshes"', f'meshdir="{mesh_dir}"', 1
        )
        (temp / 'g1_29dof.xml').write_text(model_xml)
        (temp / 'scene_29dof.xml').write_text(scene_source.read_text())
        model = mujoco.MjModel.from_xml_path(str(temp / 'scene_29dof.xml'))
    assert model.nu == 29
    assert model.nq == 36  # floating base (7) + 29 hinges
    assert model.nv == 35  # floating base (6) + 29 hinges
