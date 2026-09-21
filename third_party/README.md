# Componentes de terceros

Los archivos `src/g1_description/meshes/*.STL`, el URDF base y el MJCF base se
derivan de repositorios oficiales de Unitree Robotics. Se redistribuyen bajo la
licencia BSD de tres cláusulas incluida en `UNITREE_LICENSE`.

- `unitree_ros`, commit `7d6075f7f58588b189b940130e3edab3c839b2df`
- `unitree_mujoco`, commit `4134cb5dc7ff1ba7f484deda48b5274b58694519`

Las modificaciones propias añaden las interfaces de `ros2_control`, el escenario
de laboratorio, launch files, controladores y pruebas.

Las mallas y el URDF Dex1 opcionales se descargan de `unitree_ros` a
`artifacts/dex1`, bajo la misma licencia `UNITREE_LICENSE`. Las revisiones y
hashes de esos assets y de los controladores consultados se fijan en
`configs/dex1.sources.json`; la conversión MJCF y el servo son adaptaciones locales.
