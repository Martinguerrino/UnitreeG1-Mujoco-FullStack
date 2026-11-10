# Parámetros de `navigation.sh`

Referencia de todas las opciones de [navigation.sh](navigation.sh) y
[navigation.launch.py](navigation.launch.py). Ejecutar desde la raíz del proyecto.
Para arquitectura, instalación y detalles de SLAM, consultar la
[guía de navegación](../docs/navigation.md).

## Uso rápido

```bash
# Configuración por defecto: MuJoCo + sensores + SLAM Toolbox + Nav2 + RViz
bash scripts/navigation.sh

# Agregar detección de objetos y superficies mediante RGB-D
bash scripts/navigation.sh perception:=true

# Simulación sin ventanas, con duración de 60 segundos simulados
bash scripts/navigation.sh viewer:=false rviz:=false duration:=60

# SLAM 3D usando LiDAR e IMU
bash scripts/navigation.sh --slam lidar3d

# SLAM RGB-D y reconstrucción de malla
bash scripts/navigation.sh --slam rgb-d

# Pinzas Dex1 y percepción
bash scripts/navigation.sh --dexterity claw perception:=true
```

Cada arranque recupera el dominio DDS 44 y los puertos UDP `19420–19423`
ocupados por procesos anteriores de esta simulación. Intenta SIGTERM y luego
SIGKILL si no responden. No cierra procesos ajenos al proyecto.

## Cómo escribir las opciones

Los parámetros del lanzador usan **`nombre:=valor`**. Los booleanos se escriben
como `true` o `false`, en minúsculas. Se pueden combinar varios parámetros:

```bash
bash scripts/navigation.sh perception:=true camera_fps:=15 duration:=120
```

Estos atajos del script equivalen a parámetros del lanzador:

| Atajo | Equivalente | Valores admitidos |
| --- | --- | --- |
| `--slam lidar3d` | `slam:=lidar3d` | `lidar3d`, `rgb-d` |
| `--dexterity claw` | `dexterity:=claw` | `claw`, `hand` |
| `--feet-estimation` | `feet_estimation:=true` | Activa estimación con pies e IMU |
| `--real-odom` | `real_odom:=true` | Activa odometría estimada |
| `--real` | `real_sensors:=true` | Activa efectos de sensores simulados |
| `--ideal` | `ideal:=true` | Activa el modo ligero con láser planar ideal |

También se aceptan `--slam=rgb-d` y `--dexterity=claw`.
Para SLAM Toolbox usar `slam:=slam_toolbox` o no indicar `slam`;
el atajo `--slam` solo admite los dos modos 3D.
Los demás parámetros, como `perception`, se pasan con `:=`.

## Ventanas, ejecución y posición inicial

| Parámetro | Por defecto | Qué hace / valores |
| --- | --- | --- |
| `viewer` | `true` | Abre el visor interactivo de MuJoCo. `false` ejecuta sin esa ventana. |
| `rviz` | `true` | Abre RViz a los 20 segundos de pared. `false` lo desactiva. |
| `rviz_software_rendering` | `false` | `true` usa Mesa por CPU con dos hilos únicamente en RViz; útil ante errores gráficos de RViz. |
| `duration` | `3600.0` | Duración en segundos de simulación, además del calentamiento de 2 s. El tiempo de pared depende del rendimiento del equipo. |
| `initial_x` | `0.0` | Posición inicial X del robot en metros dentro del mundo MuJoCo. |
| `initial_y` | `0.0` | Posición inicial Y del robot en metros. |
| `initial_yaw` | `0.0` | Orientación inicial alrededor de Z, en radianes. `1.5708` equivale aproximadamente a 90°. |
| `scene` | Vacío | Ruta a un XML de escena MuJoCo existente. Vacío usa la escena de `configs/robot_navigation.yaml`. |
| `sim_python` | Se resuelve automáticamente | Intérprete Python para el simulador y SDK2. ROS sigue usando Python del sistema. |

```bash
# Iniciar en otra pose
bash scripts/navigation.sh initial_x:=1.0 initial_y:=-0.5 initial_yaw:=1.5708

# Cambiar la escena; el XML debe ser compatible con el modelo G1 del proyecto
bash scripts/navigation.sh scene:="$PWD/src/g1_mujoco/scenes/maxwellrobotics/g1_navigation_scene.xml"

# Cambiar solo el intérprete del simulador
bash scripts/navigation.sh sim_python:=/ruta/al/entorno/bin/python

# RViz con renderizado por CPU
bash scripts/navigation.sh rviz_software_rendering:=true
```

La selección automática de `sim_python` busca, en orden: `G1_PYTHON`,
`.g1-runtime.json`, el entorno Conda/virtual activo, `.venv/bin/python` y
`python3` disponible. El intérprete debe tener las dependencias del simulador.
Cerrar MuJoCo o presionar Ctrl+C detiene el conjunto.

## Navegación, SLAM y mapas

| Parámetro | Por defecto | Qué hace / valores |
| --- | --- | --- |
| `nav2` | `true` | Inicia planificación, seguimiento y filtro de comandos de Nav2. `false` mantiene la simulación, el puente y el backend de SLAM/localización, sin Nav2. |
| `mode` | `mapping` | `mapping` construye un mapa; `localization` reutiliza uno guardado. |
| `slam` | `slam_toolbox` | `slam_toolbox`: mapa 2D con `/scan`; `lidar3d`: FAST-LIO + RTAB-Map con LiDAR/IMU; `rgb-d`: odometría RGB-D + RTAB-Map y malla TSDF. |
| `map` | Vacío | Archivo de mapa requerido en `localization`: YAML de ocupación con SLAM Toolbox/AMCL, o `rtabmap.db` con los modos 3D. |
| `slam_output` | Vacío | Carpeta de sesión para `lidar3d` o `rgb-d`. Vacío crea `results/navigation/<backend>_<fecha UTC>`. No se usa con SLAM Toolbox. |

Con `nav2:=false` no habrá ejecución de metas Nav2; el backend puede recibir
comandos externos por el puente ROS. El script sigue comprobando la instalación
de `nav2_bringup` aunque se desactive Nav2.

```bash
# Solo simulación y SLAM, sin navegación automática de Nav2
bash scripts/navigation.sh nav2:=false

# Localización 2D con mapa guardado
bash scripts/navigation.sh mode:=localization map:="$PWD/maps/facility.yaml"

# Guardar una sesión LiDAR en una carpeta elegida
bash scripts/navigation.sh --slam lidar3d slam_output:="$PWD/results/slam_lidar_sesion"

# Localización RGB-D desde una base RTAB-Map guardada
bash scripts/navigation.sh --slam rgb-d mode:=localization \
  map:="$PWD/maps/sesion_rgbd/rtabmap.db" \
  slam_output:="$PWD/results/localizacion_rgbd"
```

En localización 2D, usar **2D Pose Estimate** en RViz antes de enviar una meta.
La base de datos 3D debe corresponder al backend elegido. Si `slam_output`
contiene una sesión de otro backend, el lanzador la rechaza.
Para guardar mapas 2D, ver [Guardar un mapa y reutilizarlo](../docs/navigation.md#guardar-un-mapa-y-reutilizarlo).

## Sensores, odometría, percepción y manos

| Parámetro | Por defecto | Qué hace / valores |
| --- | --- | --- |
| `camera_fps` | `0` | Frecuencia de captura RGB-D en Hz. `0` selecciona automáticamente 20 Hz para `rgb-d` y 10 Hz para los otros backends. Un valor explícito debe ser mayor que cero. El rendimiento real puede ser menor. |
| `perception` | `false` | Activa detección RGB-D de superficies y candidatos a objetos, con el backend de `configs/perception.yaml`. Con YOLO requiere el modelo configurado. |
| `dexterity` | `hand` | `hand` usa las manos originales; `claw` instala las dos pinzas Dex1 simuladas. |
| `real_odom` | `false` | Usa odometría estimada en lugar de la pose ideal del simulador. Se activa también automáticamente con `real_sensors:=true` o un backend SLAM 3D. |
| `feet_estimation` | `false` | Habilita estimación con pies e IMU para SLAM 3D. Solo se acepta con `lidar3d` o `rgb-d`. |
| `real_sensors` | `false` | Aplica ruido, sesgos, latencia y otros efectos definidos en `configs/navigation/sensors_real.yaml`. Activa odometría estimada. Sigue siendo una simulación. |
| `ideal` | `false` | Desactiva RGB-D y LiDAR 3D, conserva el escaneo planar ideal y usa una configuración ligera. |

```bash
# Odometría estimada con el backend 2D
bash scripts/navigation.sh --real-odom

# Sensores simulados con efectos realistas
bash scripts/navigation.sh --real perception:=true

# SLAM LiDAR con estimación de pies
bash scripts/navigation.sh --slam lidar3d --feet-estimation

# Modo ligero
bash scripts/navigation.sh --ideal
```

## Reconstrucción de malla RGB-D

Estas opciones se usan únicamente con `slam:=rgb-d`.

| Parámetro | Por defecto | Qué hace / valores |
| --- | --- | --- |
| `mesh_voxel_size` | `0.03` | Tamaño del vóxel TSDF en metros. Menor tamaño aumenta el detalle y el consumo de recursos. Usar un valor positivo. |
| `mesh_update_period` | `5.0` | Período de actualización de la malla en segundos de pared. Debe ser mayor que cero. |
| `mesh_max_frames` | `400` | Máximo de cuadros RGB-D retenidos para TSDF. Entero entre `3` y `2000`. |

```bash
bash scripts/navigation.sh --slam rgb-d \
  camera_fps:=15 mesh_voxel_size:=0.05 \
  mesh_update_period:=10 mesh_max_frames:=200
```

## Combinaciones y dependencias

| Caso | Requisito / restricción |
| --- | --- |
| `perception:=true` | Requiere RGB-D; no se combina con `--ideal`. Si falta el modelo YOLO: `bash scripts/setup_perception_yolo.sh`. |
| `--ideal` | No se combina con `--real-odom`, `--real` ni SLAM 3D. |
| `--feet-estimation` | Requiere `--slam lidar3d` o `--slam rgb-d`. |
| `real_odom:=true`, `--real` o SLAM 3D | Requieren OpenCV y SciPy en Python del sistema: `sudo apt install python3-opencv python3-scipy`. |
| SLAM 3D | Instalar dependencias con `bash scripts/install_navigation_slam.sh`. El arranque ejecuta también `build_navigation_mesh.sh`. |
| SLAM RGB-D | Requiere además Open3D en el entorno TSDF: `bash scripts/install_navigation_tsdf.sh`. |
| `mode:=localization` | Requiere `map:=...` apuntando a un archivo existente. |

## Configuración que se edita en archivos

Los siguientes ajustes no son argumentos de `navigation.sh`:

| Archivo | Ajustes |
| --- | --- |
| [configs/experiments/navigation.yaml](../configs/experiments/navigation.yaml) | Resolución y montaje de cámara, geometría/frecuencia/alcance del LiDAR, calentamiento y configuración base de navegación. |
| [configs/robot_navigation.yaml](../configs/robot_navigation.yaml) | Escena por defecto, modelo y configuración del robot/DDS. |
| [configs/navigation/nav2_tuning.yaml](../configs/navigation/nav2_tuning.yaml) | Ajustes compartidos de control Nav2 y asistencia de giro. |
| [configs/navigation/sensors_real.yaml](../configs/navigation/sensors_real.yaml) | Efectos activados por `--real`. |
| [configs/perception.yaml](../configs/perception.yaml) | Modelo, umbrales y procesamiento de percepción. |
| [configs/navigation_rviz_layout.yaml](../configs/navigation_rviz_layout.yaml) | Distribución de paneles de RViz. |

Los resultados de cada simulación se guardan en
`results/navigation/<fecha UTC>_<id>/`, con `config.yaml`, `metrics.json` y,
si comenzó el control, `trajectory.jsonl`.
