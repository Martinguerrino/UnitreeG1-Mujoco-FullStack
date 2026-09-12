# Unitree G1 — MuJoCo + ROS 2

Stack educativo y modular de locomanipulación para el Unitree G1 de 29 DOF. La
versión actual llega hasta planificación de brazos con MoveIt 2:

```text
pose objetivo → MoveIt 2 (IK + colisiones + OMPL)
              → JointTrajectoryController → ros2_control
              → MuJoCo/G1 → /joint_states y TF
```

## Estado

Experimento opcional: [ACT Okura](exp/10_okura_act.md) ya carga pesos y produce
acciones con imágenes de MuJoCo. **Dex1, agarre y traslado aún no validados**;
no cambia el lanzamiento ROS existente.

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
