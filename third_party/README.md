# Componentes de terceros

Los archivos `src/g1_description/meshes/*.STL`, el URDF base y el MJCF base se
derivan de repositorios oficiales de Unitree Robotics. Se redistribuyen bajo la
licencia BSD de tres cláusulas incluida en `UNITREE_LICENSE`.

- `unitree_ros`, commit `7d6075f7f58588b189b940130e3edab3c839b2df`
- `unitree_mujoco`, commit `4134cb5dc7ff1ba7f484deda48b5274b58694519`

Las modificaciones propias añaden las interfaces de `ros2_control`, el escenario
de laboratorio, launch files, controladores y pruebas.
