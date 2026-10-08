# Auditoría del estado inicial

Fecha: 2026-09-12. Antes de modificar código para `exp/plan.md`.

Este documento conserva la auditoría inicial. El estado actualizado está en
[avance del plan](plan_progress.md) y [pruebas Dex1 + ACT](dex1_act.md).

## Componentes existentes y reutilizables

- Siete paquetes ROS 2 Jazzy bajo `src/g1_*`: descripción, MuJoCo, control,
  bringup, interfaces, configuración MoveIt y manipulación.
- G1 oficial: 29 motores, base flotante (nq=36, nv=35), manos rígidas.
  Procedencia y revisiones en `third_party/README.md`. No es Menagerie.
- MuJoCo 3.3.6 en `environment.yml`; escena existente con suelo, sin necesidad
  de construir un escenario nuevo. Paso de física 0.002 s (500 Hz nominales).
- `mujoco_ros2_control` fijado en `dependencies.repos`; control por esfuerzo,
  JointTrajectoryController a 500 Hz y estados a 100 Hz configurados.
- Inicio mediante `scripts/sim.sh` o launch `g1_bringup simulation.launch.py`.
  Base soldada por defecto; MoveIt opcional. El lanzamiento libre carece de
  balance activo. `use_sim_time=false` por problema documentado del reloj ROS.
- MJCF con sensores de posición, velocidad, esfuerzo, IMU y pose de base.
- Tests de contratos URDF/MJCF/YAML, carga MuJoCo, configuración MoveIt y ACT.
- Experimento opcional Okura: checkpoint fijado, verificación SHA-256,
  procesadores LeRobot e inferencia RGB. No envía acciones a motores.

## Componentes incompletos y problemas

- No existe transporte Unitree SDK2, LowCmd/LowState ni configuración de domain
  para el G1. El DDS de ROS 2 no constituye por sí solo esa interfaz.
- No hay locomoción preentrenada integrada ni control de balance validado.
- Pick de MoveIt tiene contrato de acción; agarre físico no validado.
- Cámara del experimento Okura aproximada, sin calibración con entrenamiento.
- Manos rígidas incompatibles con pinzas Dex1 del checkpoint ACT. Manipulación
  aplazada por instrucción del usuario; debe poder incorporarse como proveedor.
- No hay benchmark integrado, ejecutor de tareas ni API MuJoCo independiente.

## Dependencias externas

Actuales: ROS 2 Jazzy, MoveIt, plugin mujoco_ros2_control, Conda/Python 3.12,
MuJoCo 3.3.6, NumPy, YAML; opcionales Torch 2.7.1 y LeRobot 0.4.1.
SDK2 Python, CycloneDDS, pygame y ONNX Runtime no estaban instalados en el
intérprete auditado. No se necesita instalar Isaac Lab ni entrenar políticas.

## Evidencia inicial

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 "$G1_PYTHON" -m pytest -q tests`:
**18 passed in 17.56s**. Esto valida contratos, carga e inferencia opcional;
no equivale a un nuevo ensayo ROS/MoveIt ni a locomoción o agarre físico.
El único archivo no versionado al comenzar era `exp/plan.md`.
