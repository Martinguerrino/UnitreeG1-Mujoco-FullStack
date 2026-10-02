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

## Dependencias de SLAM descargadas

`bash scripts/install_navigation_slam.sh` descarga dentro de `artifacts/slam`
(excluido de Git):

- FAST-LIO2, port ROS2: https://github.com/Ericsii/FAST_LIO_ROS2,
  commit `2fffc570a25d0df172720bac034fbdb6a13d2162`, con ikd-Tree fijado por
  su submódulo. Consultar sus archivos de licencia (GPL-2.0 y avisos por archivo).
  `scripts/prepare_fast_lio.py` aplica ajustes de compilación e instantáneas
  MuJoCo; no reemplaza el algoritmo original.
- Interfaces Livox: https://github.com/Livox-SDK/livox_ros_driver2,
  commit `21445540f0d100dc86a7e6df312dd70bbdb4afdf`, licencia MIT conservada en
  `LICENSE.txt`. Solo se compilan `CustomMsg` y `CustomPoint` oficiales.
- RTAB-Map ROS2: https://github.com/introlab/rtabmap_ros/tree/ros2,
  paquetes binarios ROS Jazzy del repositorio APT configurado; sus versiones y
  licencias se conservan en los `.deb` y archivos extraídos. La versión instalada
  durante la integración fue 0.23.7.
