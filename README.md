# Unitree G1 — MuJoCo + ROS 2

Integración inicial de [Nav2 + SLAM Toolbox](docs/navigation.md):
`bash scripts/navigation.sh` abre MuJoCo y RViz para seleccionar destinos.
La navegación publica además un MID-360 3D simulado y una D435i RGB-D en tópicos
ROS estándar, conservando `/scan` como proyección 2D para Nav2.
También permite elegir FAST-LIO2 o RTAB-Map para generar mapas 3D:

```bash
bash scripts/install_navigation_slam.sh
bash scripts/navigation.sh --slam lidar3d
bash scripts/navigation.sh --slam rgb-d
```

Los mapas se guardan en `results/navigation/`. Dependencias, sensores y
limitaciones se detallan en la [guía de navegación](docs/navigation.md).

Percepción con YOLO11n-seg y RGB-D de mesas, cajas, cilindros y esferas:
`bash scripts/navigation.sh perception:=true`. Publica centro, dimensiones y
orientación en `pelvis`, con visualización en RViz. Uso, salidas y límites en
[percepción](docs/perception.md).
Incluye máscaras, seguimiento en `odom`, acumulación de vistas, IDs y etiquetas/ejes en RViz.
Preparación del modelo y uso: [segmentación YOLO](docs/perception_yolo.md).
`scripts/evaluate_perception.py` compara estimaciones con la referencia de
MuJoCo en Maxwell, variando distancia, ángulo y ruido.

Stack educativo y modular de locomanipulación para el Unitree G1 de 29 DOF. La
versión actual llega hasta planificación de brazos con MoveIt 2:

```text
pose objetivo → MoveIt 2 (IK + colisiones + OMPL)
              → JointTrajectoryController → ros2_control
              → MuJoCo/G1 → /joint_states y TF
```

## Backend de políticas preentrenadas por SDK2/DDS

Se agregó una ruta independiente de ROS para ejecutar la política oficial G1
29 DoF de Unitree RL Lab mediante el bridge oficial `unitree_mujoco`.
Incluye configuraciones, test de comunicación, GoTo, benchmark y API externa.
ACT con Dex1 se prueba en un ejecutable aislado; su incorporación al Task Executor
sigue pendiente de un agarre validado.

```bash
conda activate mujoco-unitree
python scripts/setup_dds.py
python scripts/run_experiment.py --config configs/experiments/locomotion.yaml
```

Para ejecutar la misma política en el escenario reproducible de instalaciones
de `maxwellrobotics/g1-ros2` (revisión y SHA-256 fijados localmente):

```bash
python scripts/run_experiment.py \
  --config configs/experiments/locomotion_maxwell_facility.yaml
```

Para guardar dos videos sincronizados —vista desde la cabeza y vista externa—
en una subcarpeta identificada por prueba:

```bash
python scripts/run_video_test.py \
  --name locomotion_maxwell \
  --config configs/experiments/locomotion_maxwell_video.yaml
```

Agregar `--viewer` abre el visor interactivo de MuJoCo mientras se ejecuta y
graba exactamente el mismo experimento:

```bash
python scripts/run_video_test.py \
  --name table_to_table_live \
  --config configs/experiments/table_to_table_stability_video.yaml \
  --viewer
```

Para observar el mismo test sin grabar ni renderizar cámaras adicionales, usar
`--viewer --no-record`.

Cada corrida crea `videos/locomotion_maxwell/<fecha_id>/head_camera.mp4` y
`observer_camera.mp4`. Se puede cambiar `--name` para separar otros tests.

[Uso y API del backend](docs/backend.md) · [Contratos de políticas](docs/policy_interfaces.md)
· [Resultados de validación](docs/validation.md)
· [Avance del plan y tareas con varios destinos](docs/plan_progress.md).

La ruta ROS/MoveIt descrita abajo conserva sus comandos originales.

## Estado

Experimento opcional: [Dex1 + ACT por DDS](docs/dex1_act.md) incorpora pinzas
con geometría oficial y ejecuta el checkpoint Okura sobre el robot simulado.
**Apertura/cierre validados; los ensayos ACT todavía no consiguen agarrar la caja.**
No cambia el lanzamiento ROS existente.

El nuevo modo [traslado con objeto Okura](docs/okra_transfer.md) incorpora
agarre ACT, retención de pinza, recorrido a una segunda mesa y liberación con
verificación física. **El ensayo actual logra contacto bilateral, pero no eleva
el objeto; el traslado completo todavía no está validado.**

- Fase 0: workspace modular, scripts, dependencias fijadas y tests.
- Fase 1: G1 oficial de 29 DOF cargable en MuJoCo.
- Fase 2: interfaces bidireccionales de esfuerzo mediante `ros2_control`.
- Fase 3: URDF, `robot_state_publisher`, `world → pelvis` y árbol TF.
- Fase 4: trayectorias nombradas `home` y `reach_forward` para ambos brazos.
- Fase 5: MoveIt 2 para brazo izquierdo, derecho o ambos; planificación por
  pose, ejecución y mesa como objeto de colisión.
- Fase 6: acción Pick implementada; backend de agarre y validación física pendientes.
  Consulta [exp/09_pick.md](exp/09_pick.md).
- Fases 7–16: pendientes.

## Inicio rápido

Requisitos del host: Ubuntu 24.04, ROS 2 Jazzy y Miniforge en
`/home/martin/miniforge3`. La instalación razonada está en
[`exp/01_instalacion.md`](exp/01_instalacion.md).

```bash
./scripts/install_ros_dependencies.sh  # una vez; sudo pedirá tu contraseña
source /home/martin/miniforge3/etc/profile.d/conda.sh
conda activate mujoco-unitree
./scripts/bootstrap.sh
./scripts/build.sh
source install/setup.bash
ros2 launch g1_bringup simulation.launch.py
```

Para instalar, compilar, probar y ejecutar el smoke test de MoveIt de una sola
vez también se puede usar:

```bash
./scripts/setup_phase5.sh
```

Modo sin ventana (útil para CI):

```bash
ros2 launch g1_bringup simulation.launch.py headless:=true
```

RViz se activa de forma explícita con `rviz:=true`. Para comprobar solo el
modelo físico desde Conda, sin ROS: `python scripts/view_mujoco.py`.

Para abrir la simulación con MoveIt y RViz ya configurado:

```bash
./scripts/sim.sh moveit:=true rviz:=true
```

En otra terminal se puede planificar primero sin ejecutar y luego mover el
brazo. Las posiciones están en metros y los ángulos RPY en radianes, respecto a
`world`:

```bash
source scripts/activate.sh
ros2 run g1_manipulation move_to_pose --group left_arm --named-home --plan-only
ros2 run g1_manipulation move_to_pose --group left_arm \
  --pose 0.35 0.25 1.05 0.0 1.5708 0.0
```

El ejemplo agrega una mesa a la escena por defecto; `--no-table` permite
comparar el resultado sin ese obstáculo. La explicación completa está en
[`exp/08_moveit_planificacion.md`](exp/08_moveit_planificacion.md).

La simulación empieza con `fixed_base:=true`: la pelvis queda fijada mientras se
desarrollan los brazos. `fixed_base:=false` libera la base, pero el robot caerá
hasta que una policy de locomoción o un controlador de balance cierre ese lazo.

En otra terminal:

```bash
source scripts/activate.sh
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 run g1_control arm_pose.py home
ros2 run g1_control arm_pose.py reach_forward --duration 4.0
```

## Validación

```bash
source scripts/activate.sh
./scripts/check.sh
```

Los tests verifican que los mismos 29 joints existan en URDF, MJCF y YAML, que
ningún mesh falte y que MuJoCo pueda compilar el escenario real.

## Mapa del repositorio

```text
src/g1_description  geometría, cinemática, TF e interfaces ros2_control
src/g1_mujoco       física MJCF y escena
src/g1_control      controladores y comandos seguros de ejemplo
src/g1_bringup      launch central de toda la aplicación
src/g1_moveit_config SRDF, IK, OMPL, límites y conexión a controladores
src/g1_manipulation cliente de planificación por pose y escena de colisiones
tests               contratos entre paquetes y carga física
exp                 explicaciones progresivas en español
scripts             tareas repetibles de desarrollo
```

El modelo y los meshes de Unitree conservan su licencia y procedencia en
[`third_party/README.md`](third_party/README.md). El código propio usa Apache-2.0.
# UnitreeG1-Mujoco-FullStack

Para elegir el extremo de los brazos al iniciar la simulación:

```bash
bash scripts/navigation.sh --dexterity claw   # Dos pinzas Dex1
bash scripts/navigation.sh --dexterity hand   # Manos originales (predeterminado)
bash scripts/sim.sh --dexterity claw
```

También disponible en `scripts/view_mujoco.py`, `scripts/navigation_sim.py` y
`scripts/run_experiment.py`. En ROS launch se usa `dexterity:=claw`; en las
configuraciones del backend, `robot.dexterity: claw`. Las pinzas requieren los
assets preparados con `python scripts/setup_dex1.py`. La selección genera una
copia temporal del modelo y conserva los archivos originales. En navegación,
las pinzas usan sus tópicos DDS independientes `rt/dex1/{left,right}/{cmd,state}`;
sin comandos aplican amortiguación. El stack ROS de `sim.sh` muestra y simula
las pinzas, pero sus controladores corporales existentes no comandan los dedos.
La selección no activa por sí sola la política ACT ni valida un agarre.
