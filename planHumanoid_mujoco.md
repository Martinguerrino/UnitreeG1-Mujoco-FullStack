# Plan de Proyecto — Stack Modular de Locomanipulación para Unitree G1

## 1. Objetivo general

Desarrollar progresivamente un stack modular de locomanipulación para el **Unitree G1** utilizando **MuJoCo como simulador** y **ROS 2 Jazzy como middleware**, capaz de ejecutar una tarea completa de:

> **Recibir una tarea → navegar/aproximarse → alcanzar un objeto → agarrarlo → transportarlo → colocarlo en un destino.**

El proyecto no se plantea como un único algoritmo de RL ni como una política end-to-end. El objetivo principal es construir una arquitectura cercana a un stack profesional de robótica humanoide, donde cada módulo tenga una responsabilidad clara y pueda reemplazarse o mejorarse posteriormente.

La arquitectura final buscada es:

```text
┌─────────────────────────────────────────────┐
│              HIGH-LEVEL TASK                │
│                                             │
│     Task command / Planner / Behavior Tree  │
└──────────────────────┬──────────────────────┘
                       │
          ┌────────────┴────────────┐
          ▼                         ▼
   LOCOMOTION                    MANIPULATION
   GoTo / Approach               Pick / Place
          │                         │
          └────────────┬────────────┘
                       ▼
                  CONTROL LAYER
             ros2_control / WBC
                       │
                       ▼
                   MuJoCo
                       │
                       ▼
                  Unitree G1
```

---

# 2. Principio de desarrollo

El proyecto debe desarrollarse de forma incremental.

Cada fase debe cumplir tres condiciones:

1. **Agregar una capacidad necesaria para la demo final.**
2. **Funcionar y poder demostrarse de forma independiente.**
3. **Reutilizarse en las fases posteriores.**

No se debe comenzar implementando visión, VLM, WBC o RL.

Primero se construye una cadena funcional:

```text
Simulación
   ↓
ROS 2
   ↓
Control
   ↓
Manipulación
   ↓
Locomoción
   ↓
Orquestación
```

Luego se agregan capacidades avanzadas.

---

# 3. Stack tecnológico

## Base

- Ubuntu 24.04
- ROS 2 Jazzy
- Python
- C++
- Git
- Docker cuando sea necesario

## Simulación

- MuJoCo

Repositorio:

https://github.com/google-deepmind/mujoco

## Integración con ROS 2

- ros2_control
- mujoco_ros2_control

Repositorios:

https://github.com/ros-controls/ros2_control

https://github.com/ros-controls/mujoco_ros2_control

## Manipulación

- MoveIt 2

Repositorio:

https://github.com/moveit/moveit2

Tutoriales:

https://github.com/moveit/moveit2_tutorials

## Orquestación de tareas

- BehaviorTree.CPP

Repositorio:

https://github.com/BehaviorTree/BehaviorTree.CPP

## Robot learning

- Unitree RL Mjlab
- MuJoCo Playground

Repositorios:

https://github.com/unitreerobotics/unitree_rl_mjlab

https://github.com/google-deepmind/mujoco_playground

## Referencia arquitectónica

Repositorio cercano al objetivo final:

https://github.com/maxwellrobotics/g1-ros2

Este repositorio debe utilizarse como referencia para estudiar decisiones de arquitectura, integración y organización, no como sustituto del desarrollo propio.

---

# 4. Arquitectura del repositorio

Propuesta inicial:

```text
g1_locomanipulation/
│
├── src/
│   │
│   ├── g1_description/
│   │   ├── urdf/
│   │   ├── meshes/
│   │   └── config/
│   │
│   ├── g1_mujoco/
│   │   ├── mjcf/
│   │   ├── scenes/
│   │   └── launch/
│   │
│   ├── g1_bringup/
│   │   ├── launch/
│   │   └── config/
│   │
│   ├── g1_control/
│   │   ├── controllers/
│   │   └── config/
│   │
│   ├── g1_locomotion/
│   │
│   ├── g1_manipulation/
│   │
│   ├── g1_navigation/
│   │
│   ├── g1_interfaces/
│   │   ├── action/
│   │   ├── msg/
│   │   └── srv/
│   │
│   └── g1_orchestration/
│       ├── behavior_trees/
│       └── nodes/
│
├── scripts/
├── docs/
├── tests/
├── README.md
└── docker/
```

La regla es evitar un único paquete gigante. Cada módulo debe representar una capacidad del robot.

---

# 5. Fase 0 — Preparación de la arquitectura

## Objetivo

Crear el repositorio, workspace de ROS 2 y estructura de paquetes.

## Implementar

- Workspace ROS 2.
- Paquetes vacíos principales.
- Convenciones de nombres.
- README inicial.
- Scripts de build y ejecución.
- Configuración de dependencias.

## Criterio de finalización

Debe existir un comando central equivalente a:

```bash
ros2 launch g1_bringup simulation.launch.py
```

aunque inicialmente solo arranque una simulación básica.

---

# 6. Fase 1 — Unitree G1 funcionando en MuJoCo

## Objetivo

Ejecutar correctamente el modelo del G1 dentro de MuJoCo.

## Verificar

- Gravedad.
- Articulaciones.
- Actuadores.
- Límites articulares.
- Contactos.
- Estabilidad.
- Sensores básicos si están disponibles.

## Demo

```text
G1
 ↓
MuJoCo Physics
 ↓
Joint states
```

## Criterio de finalización

El G1 debe cargar correctamente y responder a comandos básicos dentro de MuJoCo.

No avanzar todavía hacia RL ni ROS complejo.

---

# 7. Fase 2 — Integración MuJoCo + ROS 2 + ros2_control

## Objetivo

Conectar MuJoCo al grafo de ROS 2 utilizando `mujoco_ros2_control`.

Arquitectura:

```text
MuJoCo
   ↕
mujoco_ros2_control
   ↕
ros2_control
   ↕
ROS 2
```

## Resultado esperado

Publicación de estados:

```bash
ros2 topic echo /joint_states
```

Visualización de controladores:

```bash
ros2 control list_controllers
```

## Primera demo

```text
ROS command
      ↓
Controller
      ↓
ros2_control
      ↓
MuJoCo
      ↓
G1 moves
      ↓
Joint state returns
```

## Criterio de finalización

Debe existir control bidireccional entre ROS 2 y la simulación.

---

# 8. Fase 3 — Descripción del robot y TF

## Objetivo

Construir una representación consistente del G1 para el resto del stack.

```text
                 G1 Description
                       │
           ┌───────────┼───────────┐
           ▼           ▼           ▼
          TF         MoveIt      Control
```

## Implementar

- URDF/Xacro.
- `robot_state_publisher`.
- Árbol TF.
- Frames de base.
- Frames de torso.
- Frames de manos.
- Frames de pies.
- Frame de IMU.

## Herramientas

- RViz.
- tf2.
- URDF/Xacro.

## Criterio de finalización

El árbol TF debe ser coherente y el robot debe visualizarse correctamente en RViz.

---

# 9. Fase 4 — Control básico de brazos

## Objetivo

Controlar los brazos del G1 mediante ROS 2.

Arquitectura:

```text
ROS Action / Command
        ↓
JointTrajectoryController
        ↓
ros2_control
        ↓
MuJoCo
        ↓
G1 Arm
```

## Implementar

Dos movimientos básicos:

```text
home
```

y:

```text
reach_forward
```

## Criterio de finalización

El robot debe poder ejecutar trayectorias de brazos desde ROS 2.

---

# 10. Fase 5 — Integración con MoveIt 2

## Objetivo

Reemplazar trayectorias hardcodeadas por planificación de movimiento.

Arquitectura:

```text
Target Pose
     ↓
MoveIt 2
     ↓
IK
     ↓
Collision Checking
     ↓
Trajectory Planning
     ↓
ros2_control
     ↓
G1
```

## Primera demo

Enviar una pose objetivo para la mano.

MoveIt debe:

1. Resolver cinemática.
2. Generar una trayectoria.
3. Ejecutarla.

## Segunda demo

Agregar una mesa como obstáculo.

La trayectoria debe evitar colisiones.

## Criterio de finalización

El brazo debe alcanzar poses arbitrarias dentro de su workspace mediante planificación.

---

# 11. Fase 6 — Skill de Pick

## Objetivo

Crear una habilidad reutilizable:

```text
Pick(object_pose)
```

## Flujo

```text
Object Pose
     ↓
Pre-Grasp Pose
     ↓
MoveIt Plan
     ↓
Execute
     ↓
Close Hand
     ↓
Attach Object
     ↓
Lift
```

## Interfaz

Crear una acción ROS propia, por ejemplo:

```text
Pick.action
```

Conceptualmente:

```text
Goal:
    object_pose

Result:
    success
```

## Criterio de finalización

Una capa superior debe poder pedir simplemente:

```text
Pick(object_pose)
```

sin conocer detalles internos de MoveIt o del gripper.

---

# 12. Fase 7 — Skill de Place

## Objetivo

Crear:

```text
Place(target_pose)
```

## Flujo

```text
Move to target
      ↓
Open hand
      ↓
Detach object
      ↓
Return to safe pose
```

## Criterio de finalización

El sistema debe poder ejecutar:

```text
Pick()
 ↓
Place()
```

de forma autónoma.

---

# 13. Fase 8 — Interfaz de locomoción

## Objetivo

Integrar una solución de locomoción existente antes de entrenar una propia.

Referencia:

https://github.com/unitreerobotics/unitree_rl_mjlab

Arquitectura buscada:

```text
Goal / Velocity Command
          ↓
Locomotion Interface
          ↓
Locomotion Controller / Policy
          ↓
Joint Commands
          ↓
G1
```

## Primera meta

Control mediante:

```text
vx
vy
yaw_rate
```

Por ejemplo:

```text
vx > 0
↓
G1 walks forward
```

## Criterio de finalización

La locomoción debe exponerse como una interfaz clara dentro de ROS 2.

---

# 14. Fase 9 — GoTo y aproximación

## Objetivo

Crear una skill:

```text
GoToPose(x, y, theta)
```

Inicialmente no es necesario utilizar Nav2.

El objetivo es convertir un target relativo o absoluto en comandos de locomoción.

Arquitectura:

```text
Current Robot Pose
        +
Goal Pose
        ↓
GoTo Controller
        ↓
vx / vy / yaw
        ↓
Locomotion Policy
        ↓
G1
```

## Criterio de finalización

El G1 debe acercarse y orientarse frente a un objeto.

---

# 15. Fase 10 — Primera locomanipulación

## Objetivo

Conectar locomoción y manipulación.

Escenario:

```text
        Object
           ●


           |
           |
          G1
```

Flujo:

```text
1. Detect / receive object pose
2. Walk toward object
3. Stop at manipulation distance
4. Reach
5. Pick
```

## Primera lógica

Inicialmente:

```text
if distance > threshold:
    locomotion
else:
    manipulation
```

No utilizar RL todavía.

## Criterio de finalización

El robot debe realizar:

```text
Approach → Reach → Pick
```

---

# 16. Fase 11 — Transporte y Place

## Objetivo

Completar la tarea:

```text
START
 ↓
Approach Object
 ↓
Pick
 ↓
Transport Object
 ↓
Approach Destination
 ↓
Place
 ↓
SUCCESS
```

En esta fase ya existe una primera demo end-to-end funcional.

---

# 17. Fase 12 — BehaviorTree.CPP

## Objetivo

Reemplazar la máquina de estados simple por una arquitectura de ejecución de tareas más robusta.

Repositorio:

https://github.com/BehaviorTree/BehaviorTree.CPP

Árbol inicial:

```text
Sequence
│
├── NavigateToObject
│
├── PickObject
│
├── NavigateToDestination
│
└── PlaceObject
```

Las skills deben exponerse como nodos del árbol.

Ejemplo:

```text
NavigateToObject
        ↓
Pick
        ↓
NavigateToDestination
        ↓
Place
```

## Criterio de finalización

La misión completa debe poder modificarse cambiando el árbol y no reescribiendo toda la lógica del sistema.

---

# 18. Fase 13 — Whole-Body Control

Esta fase debe comenzar únicamente cuando la arquitectura básica funcione.

## Problema

Un brazo aislado tiene un workspace limitado.

Para alcanzar ciertos objetivos:

```text
Hand Target
     ↓
Arm alone is insufficient
     ↓
Torso motion required
     ↓
Balance must be maintained
```

## Arquitectura futura

```text
Hand Target
      +
Torso Target
      +
Foot Contacts
      ↓
Whole-Body Controller
      ↓
┌────────┬────────┬────────┐
│ Arms   │ Torso  │ Legs   │
└────────┴────────┴────────┘
```

## Objetivo

Estudiar e integrar un controlador de cuerpo completo existente antes de intentar desarrollar uno propio.

Esta fase representa una extensión natural de MoveIt y la manipulación hacia un humanoide real.

---

# 19. Fase 14 — Percepción

## Objetivo

Eliminar gradualmente las poses hardcodeadas.

Arquitectura:

```text
Camera
   ↓
Object Detection
   ↓
Object Pose Estimation
   ↓
/object_pose
   ↓
Pick Action
```

La arquitectura de skills no debe cambiar.

Antes:

```text
Hardcoded Pose
      ↓
Pick()
```

Después:

```text
Camera
  ↓
Perception
  ↓
Object Pose
  ↓
Pick()
```

Esto permite agregar percepción sin modificar locomoción ni manipulación.

---

# 20. Fase 15 — Lenguaje y VLM

## Objetivo

Agregar instrucciones de alto nivel.

Ejemplo:

> "Llevá el vaso a la otra mesa."

Arquitectura:

```text
Language Command
       +
Visual Context
       ↓
VLM
       ↓
Structured Task
       ↓
Behavior Tree
```

Salida conceptual:

```json
{
    "object": "glass",
    "destination": "table_b"
}
```

El VLM no controla articulaciones.

Su responsabilidad es transformar lenguaje y contexto visual en información estructurada para el sistema robótico.

Esto permite usar modelos pequeños o cuantizados, algo importante considerando la RTX 2060.

---

# 21. Fase 16 — Investigación con RL

RL se incorpora solamente cuando ya existe una plataforma funcional.

## Opción A — Política de locomoción

Utilizar como base:

https://github.com/unitreerobotics/unitree_rl_mjlab

Posibles modificaciones:

- Robustez.
- Domain randomization.
- Nuevas observaciones.
- Nuevas rewards.
- Recuperación ante perturbaciones.

---

## Opción B — Manipulación aprendida

Entrenar una policy para una habilidad específica:

```text
Robot State
+
Object Pose
+
Target
      ↓
RL / Imitation Learning
      ↓
Manipulation Commands
```

---

## Opción C — Coordinación locomoción-manipulación

Esta es la opción más interesante para investigar después de construir el stack.

Arquitectura:

```text
                 STATE
                   │
                   ▼
             RL Coordinator
              /          \
             ▼            ▼
      Locomotion       Manipulation
```

La política no controla directamente todos los torques del robot.

Puede producir:

```text
vx
vy
yaw_rate

+

manipulation activation
hand target
```

Esto reduce la dimensionalidad de la acción y permite estudiar la coordinación entre skills especializadas.

---

# 22. Orden obligatorio de implementación

```text
PHASE 0
Repository architecture
        ↓
PHASE 1
G1 in MuJoCo
        ↓
PHASE 2
MuJoCo ↔ ROS 2 ↔ ros2_control
        ↓
PHASE 3
URDF + TF + RViz
        ↓
PHASE 4
Basic arm control
        ↓
PHASE 5
MoveIt 2
        ↓
PHASE 6
Pick
        ↓
PHASE 7
Place
        ↓
PHASE 8
Locomotion interface
        ↓
PHASE 9
GoTo / Approach
        ↓
PHASE 10
Approach + Pick
        ↓
PHASE 11
Transport + Place
        ↓
PHASE 12
BehaviorTree.CPP
        ↓
PHASE 13
Whole-Body Control
        ↓
PHASE 14
Perception
        ↓
PHASE 15
Language / VLM
        ↓
PHASE 16
RL research extension
```

---

# 23. Demo mínima funcional

La primera demo realmente importante del proyecto será:

```text
┌───────────────────────────────────┐
│             SCENE                 │
│                                   │
│   [Object]              [Target]  │
│      ●                     □       │
│                                   │
│                 G1                │
└───────────────────────────────────┘
```

El robot debe:

```text
START
  ↓
Receive object pose
  ↓
GoTo object
  ↓
Approach
  ↓
Pick
  ↓
GoTo target
  ↓
Place
  ↓
SUCCESS
```

---

# 24. Criterio de éxito del proyecto

El resultado final no se evalúa solamente por una política entrenada.

El objetivo es disponer de una plataforma modular donde sea posible:

```text
                    Task
                     │
                     ▼
               Behavior Tree
                     │
       ┌─────────────┼─────────────┐
       ▼             ▼             ▼
   Locomotion      Manipulation    Perception
       │             │             │
       └─────────────┼─────────────┘
                     ▼
              Control Layer
                     │
                     ▼
                    G1
```

Cada módulo debe poder evolucionar independientemente.

Por ejemplo:

```text
Classic Perception
        ↓
        VLM
```

o:

```text
MoveIt manipulation
        ↓
Learned manipulation
```

o:

```text
Existing locomotion policy
        ↓
Custom RL policy
```

sin tener que reconstruir el proyecto completo.

---

# 25. Consideraciones de hardware

El desarrollo debe tener en cuenta una RTX 2060.

Por lo tanto:

- No depender inicialmente de entrenar modelos gigantes.
- No entrenar una política end-to-end de cuerpo completo como requisito para tener una demo funcional.
- Priorizar controladores existentes y skills modulares.
- Usar RL para componentes concretos donde aporte una ventaja clara.
- Mantener percepción y VLM desacoplados del loop de control.
- Considerar modelos pequeños o cuantizados para lenguaje/visión.
- Priorizar MuJoCo para experimentación de control y políticas pequeñas.

La arquitectura modular permite avanzar incluso con hardware limitado.

---

# 26. Referencias principales

## Simulación

- MuJoCo: https://github.com/google-deepmind/mujoco
- MuJoCo Playground: https://github.com/google-deepmind/mujoco_playground

## ROS 2 y control

- ros2_control: https://github.com/ros-controls/ros2_control
- mujoco_ros2_control: https://github.com/ros-controls/mujoco_ros2_control

## Manipulación

- MoveIt 2: https://github.com/moveit/moveit2
- MoveIt Tutorials: https://github.com/moveit/moveit2_tutorials

## Unitree G1 y robot learning

- Unitree RL Mjlab: https://github.com/unitreerobotics/unitree_rl_mjlab

## Orquestación

- BehaviorTree.CPP: https://github.com/BehaviorTree/BehaviorTree.CPP

## Referencia arquitectónica

- g1-ros2: https://github.com/maxwellrobotics/g1-ros2

---

# 27. Decisión actual

El punto de partida inmediato es:

```text
Unitree G1
    +
MuJoCo
    +
ROS 2 Jazzy
    +
ros2_control
```

La primera meta concreta es conseguir:

```text
ROS 2 command
      ↓
ros2_control
      ↓
MuJoCo
      ↓
G1 moves
      ↓
Joint states return to ROS 2
```

No avanzar a MoveIt, locomoción, WBC, percepción, VLM o RL hasta que esta integración básica funcione correctamente.

Una vez conseguida esta base, cada fase agrega una capacidad directamente necesaria para la demo final de locomanipulación.
