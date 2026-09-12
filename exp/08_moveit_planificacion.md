# Planificación con MoveIt 2

## Qué agrega esta fase

Hasta la fase 4 el repositorio sabía reproducir dos listas de ángulos escritas a
mano. Ahora se le puede pedir una posición y orientación para una mano. MoveIt
resuelve los ángulos, comprueba colisiones, busca un camino y lo entrega al
mismo controlador que ya mueve MuJoCo.

```text
pose de la mano
      ↓
IK: calcula joints candidatos
      ↓
escena: descarta estados en colisión
      ↓
OMPL: conecta inicio y objetivo
      ↓
FollowJointTrajectory
      ↓
whole_body_controller → MuJoCo
```

Planificar y ejecutar son acciones distintas. `--plan-only` permite mirar el
resultado en RViz sin mover el robot; es el modo apropiado para probar un
objetivo nuevo por primera vez.

## Por qué hay dos paquetes

`g1_moveit_config` contiene datos del robot y del planificador. No contiene una
tarea concreta:

- `g1.srdf`: grupos `left_arm`, `right_arm`, `dual_arms`, pose `home` y matriz
  de colisiones permitidas.
- `kinematics.yaml`: KDL como solver de cinemática inversa para cada brazo.
- `ompl_planning.yaml`: RRTConnect rápido por defecto y RRTstar disponible.
- `joint_limits.yaml`: velocidades y aceleraciones deliberadamente moderadas.
- `moveit_controllers.yaml`: traduce MoveIt al action
  `/whole_body_controller/follow_joint_trajectory`.
- `moveit.rviz`: abre el panel MotionPlanning, grupo izquierdo y frame `world`.

`g1_manipulation` contiene una tarea reusable: recibir una pose, crear una mesa,
planificar y opcionalmente ejecutar. Esta división es habitual porque permite
cambiar la aplicación sin duplicar la configuración del robot.

## Decisión de seguridad: solo brazos

Cada brazo tiene siete joints desde `torso_link` hasta `*_rubber_hand`. Piernas
y cintura no forman parte de esos grupos. El controlador general posee los 29
joints, pero acepta objetivos parciales; por eso una trayectoria de MoveIt solo
manda los siete joints seleccionados y los demás conservan su referencia.

La base continúa fijada. Planificar con base flotante requeriría añadir balance
y locomoción, algo que corresponde a fases posteriores.

## Instalación y compilación

MoveIt es una dependencia ROS del sistema, no una dependencia Python de Conda.
Se instala una sola vez:

```bash
./scripts/install_ros_dependencies.sh
./scripts/build.sh
```

El camino más corto hace instalación, compilación, tests y una planificación
headless automática en un solo comando:

```bash
./scripts/setup_phase5.sh
```

`sudo` pedirá tu contraseña en la terminal. Después, cada terminal nueva solo
necesita `source scripts/activate.sh`; ese script activa tanto Conda
`mujoco-unitree` como ROS y el workspace.

## Primera prueba, paso a paso

Terminal 1:

```bash
./scripts/sim.sh moveit:=true rviz:=true
```

Espera hasta que RViz muestre el G1 y el panel MotionPlanning indique que el
grupo es `left_arm`. El Fixed Frame ya queda configurado en `world`.

Terminal 2:

```bash
source scripts/activate.sh
ros2 run g1_manipulation move_to_pose --group left_arm --named-home --plan-only
```

Si el plan aparece correctamente, ejecútalo:

```bash
ros2 run g1_manipulation move_to_pose --group left_arm --named-home
```

Luego prueba una pose cartesiana:

```bash
ros2 run g1_manipulation move_to_pose --group left_arm \
  --pose 0.35 0.25 1.05 0.0 1.5708 0.0
```

Los primeros tres números son `x y z` en metros. Los últimos son
`roll pitch yaw` en radianes. Para el brazo derecho cambia el grupo y usa una
coordenada `y` negativa.

## Mesa y colisiones

El ejecutable agrega `work_table`, una caja de 0.70 × 1.20 × 0.05 m, a la
Planning Scene. MuJoCo no recibe esa caja: por ahora es un obstáculo lógico para
el planificador. Esto sirve para aprender la separación entre el mundo de
planificación y el mundo físico; en una fase posterior ambos se generarán desde
una misma descripción de escena.

Usa `--no-table` para desactivarla. Si una pose está dentro de la mesa o fuera
del alcance, que el plan falle es el comportamiento correcto.

## Cómo depurar

Comprueba en este orden:

```bash
ros2 node list | grep move_group
ros2 action list | grep whole_body_controller
ros2 topic echo /joint_states --once
ros2 control list_controllers
```

`move_group` ausente suele indicar que MoveIt no está instalado o que su
configuración no cargó. Un action ausente apunta al controlador. Sin
`/joint_states`, MoveIt no conoce el estado inicial. Un error de IK suele
significar que la pose está fuera del workspace o que su orientación es
demasiado restrictiva.

## Qué verifican los tests

`tests/test_moveit_config.py` exige que:

- los dos grupos sean cadenas correctas de siete DOF;
- MoveIt mande exactamente los 14 joints de brazos conocidos por ros2_control;
- ambos brazos tengan IK, planners y límites;
- la pose `home` cubra todos sus joints;
- cada par de links adyacentes esté deshabilitado en la autocolisión.

Esos contratos detectan errores de nombres y configuración sin arrancar la
simulación. La prueba runtime sigue siendo necesaria para validar IK, plugins y
ejecución real.
