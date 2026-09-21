# maxwellrobotics/g1-ros2 scenario

The MuJoCo facility scenario used by this project is copied verbatim from:

- repository: `https://github.com/maxwellrobotics/g1-ros2`
- revision: `ace298393ec6cadc1f4a66e70a3311e1d2c4d7ff`
- source: `workspace/src/g1_bringup/mjcf/g1_navigation_scene.xml`
- destination: `src/g1_mujoco/scenes/maxwellrobotics/g1_navigation_scene.xml`
- SHA-256: `8f747d46c5dafb42187dbfcc60f1008c82db8e5ebf4c409ab02a58e21b65a8d3`

The upstream BSD 3-Clause license is preserved in `LICENSE`.

The source scene declares inactive grasp welds for Dex3 palm bodies. The local
29-DoF model does not contain those hands, so `g1_backend.simulator.model_xml`
removes constraints that reference unavailable bodies while constructing the
runtime XML. Geometry, materials, lighting and object poses remain unchanged.
