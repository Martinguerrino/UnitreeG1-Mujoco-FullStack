# Contratos de políticas

## Estado y acción comunes

`g1_backend/contracts.py` define tipos propios sin dependencia de MuJoCo ni SDK.
`RobotState`: q/dq de 29 motores en orden SDK, cuaternión IMU wxyz, velocidad
angular local rad/s, timestamp monotónico de recepción y mode_machine.
`JointCommand`: 29 posiciones absolutas en radianes y ganancias kp/kd.
El transporte utiliza PR, velocidad deseada y feedforward cero, habilita motores,
aplica límites del modelo y calcula CRC con la biblioteca oficial.

Orden SDK: pierna izquierda (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch,
ankle_roll), pierna derecha igual, waist_yaw/roll/pitch, brazo izquierdo
(shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw), brazo derecho igual.

## Unitree RL Lab velocity/v0

Interfaz: `reset`, `get_observation(state, velocity)`, `infer`, `get_action`.
Se valida SHA-256 de ONNX y deploy.yaml antes de cargar. Proveedor alternativo:
`module:factory`, recibiendo el bloque YAML de locomoción.

Mapa **índice policy → índice SDK**:

```
[0,6,12,1,7,13,2,8,14,3,9,15,22,4,10,16,23,5,11,17,24,18,25,19,26,20,27,21,28]
```

Observaciones concatenadas por grupo, con cinco muestras antiguas→recientes
para cada grupo (no cinco vectores completos intercalados):

| Grupo | Valores por muestra | Escala |
|---|---:|---:|
| base_ang_vel | 3 | 0.2 |
| projected_gravity | 3 | 1 |
| velocity_commands | 3 | 1 |
| joint_pos_rel | 29 | 1 |
| joint_vel_rel | 29 | 0.05 |
| last_action | 29 | 1 |

Total: 5×96=480 float32. La gravedad es R(q)ᵀ·[0,0,-1]. Posiciones y velocidades
se reordenan a policy; se resta default_joint_pos en las posiciones. Al reset
se repite la primera observación para llenar el historial, como upstream.
Última acción inicial cero. No se introduce una normalización estadística nueva.

Comandos locales vx∈[-0.5,1], vy∈[-0.3,0.3], yaw_rate∈[-0.2,0.2].
29 acciones residuales: `q_policy = 0.25 * action + default_joint_pos`.
Después se mapean a SDK. La postura por defecto en orden policy es:

```
[-.1,-.1,0,0,0,0,0,0,0,.3,.3,.3,.3,-.2,-.2,.25,-.25,0,0,0,0,.97,.97,.15,-.15,0,0,0,0]
```

Kp/kd son los del deploy.yaml original **en orden SDK**, sin reordenarlos otra
vez. La inferencia apunta a 50 Hz; publicación LowCmd y física a 500 Hz.
Son frecuencias nominales de host, sin garantía de tiempo real duro. Se registra
el rendimiento observado; nunca se presenta steps/s como FPS de render.
El transporte recorta destinos fuera de límites físicos y registra el número
de valores recortados. No es un controlador habilitado para hardware.

## Proveedor futuro de manipulación

`provider: null` produce un resultado `blocked` antes de crear DDS/simulador.
Una implementación futura configura `provider: my_package.policy:create` y
`checkpoint`, más sus propios parámetros. La factory recibe el dict completo.

Contrato:

```
period: float
reset()
observe(sensors, robot_state) -> observation
infer(observation)
get_action() -> JointCommand
is_done() -> bool
```

El argumento sensors de observe contiene `images` (dict RGB/depth o None) y
`request` (skill, objeto o destino). El proveedor administra su normalización,
horizonte temporal y formato de entrada. Pick/place comparten este contrato.
Debe producir un comando corporal completo y resolver explícitamente balance
y transporte de manos: no se mezclan dos comandos de cuerpo completo ni se
sobrescriben piernas de locomoción con acciones parciales de brazo.

`is_done` significa fin de la política. Las métricas grasp/lift/place permanecen
null hasta incorporar observadores físicos del escenario seleccionado. El
executor no convierte ese booleano en evidencia de agarre. La nueva prueba
[Dex1 + ACT](dex1_act.md) ejecuta manipulación física aislada con una factory
de chunks independiente; todavía no implementa un proveedor pick/place validado
para esta ruta general.
