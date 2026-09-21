Quiero que trabajes sobre MI REPOSITORIO DE MUJOCO:



Quiero convertirlo en un entorno de investigación/integración para Unitree G1 en MuJoCo, utilizando políticas preentrenadas de locomoción y manipulación.

IMPORTANTE:

NO quiero entrenar las políticas en este proyecto.

La locomoción será una política preentrenada.

La manipulación será una política preentrenada.

El objetivo es integrar ambas políticas con Unitree G1 + MuJoCo + Unitree SDK2/DDS y conseguir un workflow reproducible que posteriormente pueda utilizarse como backend MuJoCo de un repositorio principal de comparación MuJoCo vs Isaac Sim.

==================================================

1. REFERENCIAS OFICIALES DE UNITREE
   ==================================================

Utilizar como referencias principales:

### Unitree MuJoCo

https://github.com/unitreerobotics/unitree_mujoco

Es el simulador/interfaz principal que quiero utilizar.

Estudiar especialmente:

* G1;
* comunicación DDS;
* LowCmd;
* LowState;
* IMU;
* configuración del robot;
* configuración de simulation;
* domain ID;
* interfaces utilizadas por SDK2.

El objetivo es mantener la ruta:

Policy / Controller
↓
Unitree SDK2 / DDS
↓
unitree_mujoco
↓
MuJoCo
↓
G1

NO quiero que las políticas controlen directamente qpos/qvel del modelo MuJoCo como mecanismo principal de ejecución.

---

### Unitree SDK2

https://github.com/unitreerobotics/unitree_sdk2

Utilizar este repositorio como referencia para la interfaz oficial de comunicación/control del robot.

Estudiar especialmente los ejemplos de G1 y:

* LowCmd;
* LowState;
* IMU;
* arm control;
* joint control;
* DDS;
* control loop.

---

### Unitree SDK2 Python

https://github.com/unitreerobotics/unitree_sdk2_python

Utilizarlo cuando resulte conveniente para las partes Python del proyecto.

No duplicar implementaciones del SDK dentro de mi repositorio.

---

### Unitree RL Lab

https://github.com/unitreerobotics/unitree_rl_lab

ESTA REFERENCIA ES MUY IMPORTANTE.

Analizar especialmente:

* G1 29 DoF;
* despliegue de políticas;
* `deploy/robots/g1_29dof`;
* uso de `unitree_sdk2`;
* workflow sim2sim;
* comunicación con `unitree_mujoco`;
* transición hacia hardware real.

El repositorio documenta un flujo:

Isaac Lab
↓
trained policy
↓
MuJoCo sim2sim
↓
Unitree SDK2
↓
real robot

Quiero aprovechar la arquitectura de despliegue cuando sea aplicable.

---

### Unitree RL MJLab

https://github.com/unitreerobotics/unitree_rl_mjlab

Analizar este repositorio como referencia adicional para:

* políticas entrenadas específicamente en MuJoCo;
* deployment;
* G1;
* low-level control;
* sim2sim;
* integración con SDK2.

No copiar el repositorio entero.

Determinar qué partes son útiles para mi proyecto.

---

### MuJoCo Menagerie

https://github.com/google-deepmind/mujoco_menagerie

Utilizar como referencia para el modelo Unitree G1 si mi implementación actual utiliza o puede beneficiarse de este modelo.

---

==================================================
2. REFERENCIAS DE LOCOMOCIÓN
============================

NO quiero entrenar locomoción.

Quiero utilizar una política preentrenada.

Estudiar las siguientes implementaciones/referencias:

### Unitree RL Lab

https://github.com/unitreerobotics/unitree_rl_lab

Esta debe ser una de las principales referencias para la política y deployment de locomoción G1.

Analizar:

* qué observaciones recibe la política;
* qué acciones produce;
* frecuencia de inferencia;
* normalización;
* escalado de acciones;
* joint ordering;
* joint limits;
* cómo se transforma la salida de la policy en comandos;
* cómo se integra con SDK2;
* cómo se realiza sim2sim en MuJoCo.

---

### Unitree RL MJLab

https://github.com/unitreerobotics/unitree_rl_mjlab

Analizar específicamente si existe una política/modelo de locomoción que pueda utilizarse directamente o cuya estructura sea apropiada para mi workflow MuJoCo.

---

### G1 Locomotion RL

https://github.com/adamson-gh/g1-locomotion-rl

Es una referencia secundaria.

El repositorio contiene una política PPO de locomoción G1 en MuJoCo y un workflow de evaluación de una política entrenada.

No asumir que esta política es automáticamente compatible con SDK2.

Determinar qué formato tiene el checkpoint y qué adaptaciones serían necesarias.

---

### Unitree-G1-RL

https://github.com/idan0405/Unitree-G1-RL

Referencia adicional para locomoción G1 en MuJoCo.

Su carpeta `locomotion/` utiliza MuJoCo + Gymnasium + Stable-Baselines3 PPO y contiene un checkpoint preentrenado.

Estudiar únicamente como referencia para:

* observaciones;
* acciones;
* evaluación;
* estructura de política.

No utilizar su política si otra opción oficial/compatible con SDK2 resulta mejor.

---

### Criterio para elegir locomotion policy

NO elegir simplemente la política que sea más fácil de cargar.

Priorizar:

1. compatibilidad con G1 29 DoF;
2. compatibilidad con MuJoCo;
3. compatibilidad con Unitree SDK2;
4. posibilidad de sim2real;
5. documentación;
6. estabilidad;
7. facilidad de integración;
8. reproducibilidad.

Antes de implementar, explicar cuál política se eligió y por qué.

==================================================
3. REFERENCIAS DE MANIPULACIÓN
==============================

NO quiero entrenar una política de manipulación.

Quiero integrar una política preentrenada.

La referencia principal para analizar manipulación será:

### humanoid_vla

https://github.com/ozkannceylan/humanoid_vla

Este repositorio es especialmente relevante porque utiliza:

* Unitree G1;
* MuJoCo;
* ACT;
* cámara egocéntrica;
* manipulación;
* bimanual manipulation;
* física de contactos.

Analizar:

* arquitectura ACT;
* checkpoint/model format;
* observaciones;
* imágenes;
* robot state;
* action space;
* frecuencia de control;
* joint ordering;
* grippers;
* inferencia;
* cómo se conecta ACT con G1.

NO copiar el proyecto entero.

Quiero determinar qué partes necesito para integrar una política de manipulación preentrenada en MI repositorio.

---

### LeRobot Unitree G1

https://github.com/unitreerobotics/unitree_lerobot

Utilizar como referencia para:

* Unitree G1;
* ACT;
* datasets;
* inferencia;
* formato de políticas;
* integración con LeRobot;
* control del robot.

---

### Unitree G1 manipulation checkpoint

También analizar checkpoints disponibles para G1 + ACT, por ejemplo:

https://huggingface.co/sotata/act-okura-pick-06102026

El checkpoint debe considerarse como una opción experimental.

Antes de usarlo, comprobar:

* embodiment;
* número de DoF;
* manos utilizadas;
* formato de observación;
* cámara;
* state vector;
* action vector;
* compatibilidad con mi modelo G1;
* compatibilidad con mi configuración de manos;
* frecuencia de control.

NO asumir compatibilidad solamente porque diga "G1".

---

==================================================
4. ARQUITECTURA OBJETIVO
========================

Quiero terminar con algo conceptualmente similar a:

```
                TASK
                 │
                 ▼
           Task Executor
                 │
          ┌──────┴──────┐
          ▼             ▼
        GoTo          Pick/Place
          │             │
          ▼             ▼
   Locomotion       Manipulation
    Policy            Policy
   pretrained        pretrained
          │             │
          └──────┬──────┘
                 ▼
           SDK2 / DDS
                 ▼
           unitree_mujoco
                 ▼
               MuJoCo
                 ▼
                G1
```

Las políticas NO deben conocer detalles innecesarios de MuJoCo.

El simulador NO debe conocer detalles internos del modelo de política.

SDK2/DDS debe ser la interfaz de comunicación entre controlador/política y robot simulado siempre que la arquitectura lo permita.

==================================================
5. FASE 0 — AUDITORÍA DEL REPOSITORIO ACTUAL
============================================

Antes de cambiar código:

Analizar:



Determinar:

* estructura;
* dependencias;
* modelo G1;
* versión de MuJoCo;
* cómo arranca;
* cómo funciona SDK2;
* DDS;
* ROS2 si existe;
* cámaras;
* políticas existentes;
* checkpoints;
* control loops;
* frecuencia de simulación;
* frecuencia de control;
* scripts actuales;
* qué ya funciona.

Crear un documento:

docs/current_state.md

con:

* componentes existentes;
* componentes reutilizables;
* componentes incompletos;
* problemas;
* dependencias externas.

NO reestructurar todavía.

==================================================
1. FASE 1 — G1 + MUJOCO + SDK2/DDS
==================================

Dejar estable:

G1
↓
MuJoCo
↓
unitree_mujoco
↓
DDS
↓
SDK2
↓
controller

Verificar:

* LowState;
* LowCmd;
* IMU;
* joint positions;
* joint velocities;
* motor states;
* control frequency;
* reset;
* emergency stop;
* domain ID;
* network/interface configuration.

Crear un test mínimo de comunicación.

El resultado debe ser:

"una aplicación externa puede comunicarse con el G1 simulado mediante la misma arquitectura de comunicación que posteriormente queremos utilizar con el robot real."

==================================================
7. FASE 2 — INTEGRAR LOCOMOTION POLICY
======================================

Seleccionar una política preentrenada.

Primera opción a estudiar:

https://github.com/unitreerobotics/unitree_rl_lab

Segunda:

https://github.com/unitreerobotics/unitree_rl_mjlab

Alternativas:

https://github.com/adamson-gh/g1-locomotion-rl

https://github.com/idan0405/Unitree-G1-RL

No entrenar.

Crear una interfaz:

LocomotionPolicy

Conceptualmente:

```
reset()
get_observation(robot_state)
infer(observation)
get_action()
```

Documentar exactamente:

* observation vector;
* action vector;
* joint order;
* normalization;
* action scale;
* control frequency;
* policy frequency.

El primer test será:

Stand
↓
Walk
↓
Stop

Después:

GoTo(target)

==================================================
8. FASE 3 — INTEGRAR MANIPULATION POLICY
========================================

Seleccionar una política preentrenada.

Primera referencia:

https://github.com/ozkannceylan/humanoid_vla

Analizar su ACT y determinar si el checkpoint/pipeline puede reutilizarse directamente.

Segunda referencia:

https://github.com/unitreerobotics/unitree_lerobot

También estudiar:

https://huggingface.co/sotata/act-okura-pick-06102026

NO asumir compatibilidad.

Primero verificar:

* G1 embodiment;
* hands;
* camera;
* state vector;
* action vector;
* joint order;
* image size;
* normalization;
* temporal horizon;
* control frequency.

Crear una interfaz:

ManipulationPolicy

Conceptualmente:

```
reset()
observe(image, robot_state)
infer(...)
get_action()
is_done()
```

==================================================
9. FASE 4 — CÁMARA
==================

NO crear el escenario.

Solamente implementar/preparar la infraestructura de sensores necesaria para la política.

La cámara debe ser parametrizable:

* RGB;
* depth;
* resolution;
* FOV;
* FPS;
* position;
* orientation.

Primero usar exactamente las entradas que necesita la política preentrenada.

No agregar RGB-D si la política solamente utiliza RGB.

Guardar la configuración de cámara de manera reproducible.

==================================================
10. FASE 5 — TEST DE LOCOMOCIÓN AISLADA
=======================================

Crear un benchmark:

```
start
  ↓
stand
  ↓
walk to target
  ↓
stop
```

Variar:

* posición inicial;
* orientación;
* velocidad;
* distancia.

Medir:

* success;
* final position error;
* orientation error;
* completion time;
* falls;
* collisions.

==================================================
11. FASE 6 — TEST DE MANIPULACIÓN AISLADA
=========================================

No utilizar locomoción todavía.

Colocar G1 en una posición fija frente al objeto.

Ejecutar:

```
observation
    ↓
manipulation policy
    ↓
   grasp
    ↓
   lift
    ↓
  release/place
```

Medir:

* grasp success;
* lift success;
* place success;
* object drops;
* placement error;
* completion time;
* collisions.

==================================================
12. FASE 7 — SKILLS
===================

Crear una capa pequeña:

skills/

```
goto
pick
place
```

No debe contener la implementación de las políticas.

Ejemplo conceptual:

```
GoTo(target)
    ↓
LocomotionPolicy

Pick(object)
    ↓
ManipulationPolicy

Place(target)
    ↓
ManipulationPolicy
```

==================================================
13. FASE 8 — TASK EXECUTOR
==========================

Crear un ejecutor sencillo.

Ejemplo:

```
task:
  - goto(table_a)
  - pick(box)
  - goto(table_b)
  - place(table_b)
```

La primera implementación puede ser determinista.

NO implementar todavía:

* VLM;
* Qwen;
* lenguaje natural;
* LLM planner;
* SLAM;
* navegación autónoma compleja.

Eso pertenece a una fase posterior.

==================================================
14. FASE 9 — PICK AND PLACE COMPLETO
====================================

Una vez validadas las dos políticas:

```
GoTo(Table A)
      ↓
    Pick
      ↓
GoTo(Table B)
      ↓
    Place
```

Este será el primer benchmark integrado.

Las políticas siguen siendo preentrenadas.

No entrenar ninguna política para esta fase.

==================================================
15. FASE 10 — EVALUACIÓN
========================

Crear infraestructura de evaluación.

Locomoción:

* navigation success;
* position error;
* orientation error;
* walking time;
* falls;
* collisions.

Manipulación:

* grasp success;
* lift success;
* place success;
* placement error;
* manipulation time;
* object drops.

Task:

* overall success;
* total completion time;
* failed skill;
* retries;
* collisions.

Simulation:

* simulation FPS;
* steps/sec;
* CPU;
* GPU;
* VRAM;
* RAM.

Guardar resultados en formato estructurado, por ejemplo:

results/
experiment_001/
config.yaml
metrics.json
trajectory/
video.mp4

==================================================
16. FASE 11 — REPRODUCIBILIDAD
==============================

Crear configuraciones:

configs/
robot.yaml
locomotion.yaml
manipulation.yaml
camera.yaml
task.yaml

y:

configs/experiments/
locomotion.yaml
manipulation.yaml
pick_and_place.yaml

Ejemplo:

python scripts/run_experiment.py 
--config configs/experiments/pick_and_place.yaml

El experimento debe registrar automáticamente:

* git commit;
* configuración;
* policy checkpoint;
* seed;
* simulator version;
* MuJoCo version;
* métricas;
* timestamp.

==================================================
17. FASE 12 — API PARA EL FUTURO REPO GRANDE
============================================

El repositorio MuJoCo debe seguir funcionando independientemente.

Pero posteriormente otro repositorio lo utilizará como backend.

Crear una interfaz externa simple, por ejemplo:

```
run_experiment(config)
```

El repositorio principal debería poder decir:

```
MujocoBackend.run(config)
```

sin conocer:

* joints;
* MuJoCo internals;
* SDK2 internals;
* checkpoint internals;
* detalles de ACT.

La estructura final conceptual será:

```
        G1-SIM2REAL
             │
    ┌────────┴────────┐
    ▼                 ▼
```

MUJOCO REPO        ISAAC REPO
│                 │
pretrained         pretrained
policies           policies
│                 │
SDK2/DDS           DDS
│                 │
MuJoCo             Isaac

```

==================================================
18. REGLAS IMPORTANTES
==================================================

NO entrenar:

- locomoción;
- ACT;
- manipulación.

NO crear:

- VLM;
- VLA;
- SLAM;
- WBC desde cero;
- planner LLM;
- escenario desde cero.

El escenario será proporcionado/seleccionado posteriormente.

NO copiar repositorios completos.

Utilizar:

- Unitree MuJoCo como backend;
- Unitree SDK2 como comunicación;
- políticas preentrenadas;
- repositorios de referencia solamente donde sea necesario.

Priorizar siempre una solución que preserve el camino:

    pretrained policy
          ↓
      SDK2 / DDS
          ↓
     unitree_mujoco
          ↓
        MuJoCo
          ↓
       real G1

==================================================
19. ORDEN DE TRABAJO
==================================================

Trabajar estrictamente en este orden:

1. Auditar mi repo.
2. Auditar las referencias externas.
3. Elegir locomotion policy.
4. Elegir manipulation policy.
5. Documentar interfaces de ambas.
6. Estabilizar SDK2/DDS.
7. Integrar locomotion.
8. Integrar manipulation.
9. Integrar cámara.
10. Validar locomoción.
11. Validar manipulación.
12. Crear skills.
13. Crear Task Executor.
14. Ejecutar Pick & Place.
15. Crear benchmark.
16. Crear API externa.
17. Documentar.

Antes de modificar código, mostrarme:

A. qué ya tengo;
B. qué voy a reutilizar;
C. qué dependencias externas voy a utilizar;
D. qué política concreta se propone para locomoción;
E. qué política concreta se propone para manipulación;
F. qué adaptadores son necesarios;
G. qué archivos nuevos/modificados se crearán.

No crear una arquitectura gigantesca si mi repositorio actual ya tiene una solución funcional.

El objetivo es construir sobre lo existente, no empezar de cero.

**Una aclaración importante:** `unitree_rl_lab` está basado en Isaac Lab, por lo que lo usaría principalmente como **referencia de deployment/sim2sim y como fuente de políticas compatibles**, no como si fuera tu simulador MuJoCo. En cambio, `unitree_rl_mjlab` es una referencia más directamente alineada con un pipeline de RL en MuJoCo.

Para manipulación, `humanoid_vla` es particularmente útil porque ya demuestra G1 + MuJoCo + ACT y manipulación bimanual con física de contactos.

Y dejaría **`Unitree-G1-RL` y `g1-locomotion-rl` como alternativas/benchmarks**, no como primera elección: el primero tiene una política MuJoCo preentrenada muy sencilla para locomoción, mientras que el segundo es principalmente un proyecto de entrenamiento PPO.
```
