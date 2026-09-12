# Recorrido de control y datos

Al ejecutar:

```bash
ros2 run g1_control arm_pose.py reach_forward
```

ocurre esta secuencia:

```text
arm_pose.py
  → acción FollowJointTrajectory
  → whole_body_controller
  → PID posición/velocidad a esfuerzo
  → interfaz MujocoSystemInterface
  → actuadores MJCF
  → dinámica y contactos
  → estados position/velocity/effort
  → /joint_states y robot_state_publisher
  → TF
```

El cliente envía los 14 joints de brazos. `allow_partial_joints_goal` permite
que el mismo controlador mantenga sin cambios piernas y cintura. Cada objetivo
tiene duración positiva, se valida por la acción y devuelve éxito o error.

Los límites de fuerza siguen en el MJCF mediante `actuatorfrcrange`. El PID no
puede pedir al modelo fuerzas ilimitadas. Las ganancias actuales son un punto
de partida conservador y deben ajustarse midiendo tracking, oscilación y caída,
no por intuición.

La pelvis está soldada al mundo por defecto durante esta fase. Mantener una pose
articular no equivale a controlar el centro de masa de un bípedo. El argumento
`fixed_base:=false` existe para ensayos futuros, pero requiere balance activo.
