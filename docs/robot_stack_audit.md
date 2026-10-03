# Auditoría del estado corporal, observaciones y actuadores

Revisión de código del 2026-10-03. Alcance: política predeterminada
`UnitreeVelocityPolicy`, transporte SDK2, puente MuJoCo y perfil `--real`.
Este informe no sustituye una calibración ni prueba física del G1.

## Estado corporal: qué existe

`g1_backend/transport.py` convierte LowState en posiciones y velocidades de
29 articulaciones, quaternion y giróscopo. Guarda la hora de recepción local,
valida valores finitos y rechaza estados demasiado antiguos. El mensaje SDK
incluye acelerómetro y torque estimado, pero `RobotState` no los conserva.

FAST-LIO2 estima movimiento usando LiDAR e IMU de torso; RGB-D estima movimiento
visual. Ninguno constituye un estimador corporal con cinemática y apoyos de pies
para la locomoción. El módulo `sensor_odometry.py` registra nubes y filtra poses;
no es un estimador de contacto. El simulador cuenta contactos para evaluación,
pero no entrega una estimación de apoyo/deslizamiento al controlador.

La orientación entregada al balance proviene de sensores MuJoCo, incluyendo
framequat; no se calcula aquí mediante una fusión implementada de giróscopo y
acelerómetro. En hardware hay que verificar qué estimación entrega el SDK y su
frame, calibración y calidad. No hace falta duplicar una fusión de orientación
válida sin antes caracterizarla.

### Mejoras propuestas, en orden

1. Extender el contrato de medición con acelerómetro, tau_est, identificación
   de muestra y tiempo de captura cuando el SDK lo permita. Conservar también
   hora de recepción y edad; distinguir muestra nueva de repetida. Verificar
   unidades, orden de articulaciones y extrínsecos de ambas IMU.
2. Incorporar cinemática directa y Jacobianos de pies con encoders medidos,
   evitando qpos/qvel de la base perfecta. Estimar apoyo con confianza e
   histéresis; tau_est puede aportar evidencia, pero no equivale por sí solo
   a una medición precisa de fuerza de contacto. No asumir sensores de fuerza
   plantares disponibles sin verificar hardware.
3. Implementar fusión IMU/cinemática/contactos con estado de orientación,
   velocidad, posición local y sesgos, más incertidumbre. Un pie estable puede
   aportar una restricción de velocidad; un pie que desliza debe dejar de fijar
   la base. El estimador debe reconocer apoyo doble, simple y ausencia de apoyo.
4. Integrar SLAM como corrección externa con incertidumbre y tiempos coherentes.
   Separar la trayectoria local continua de las correcciones globales del mapa;
   un cierre de bucle no debe producir un salto artificial en el balance.
5. Exponer salud del estimador: innovación, sesgos, confianza de apoyo, edad de
   datos y covarianza. No tratar una estimación degradada como estado preciso.

Validación necesaria: robot quieto, apoyo simple/doble, marcha recta/lateral,
giro y perturbaciones; medir deriva, velocidad espuria estando quieto,
recuperación tras pérdida de apoyo y comportamiento ante deslizamiento. La
verdad MuJoCo puede evaluar errores, nunca entrar al estimador.

## Observaciones de la política predeterminada

`artifacts/locomotion/deploy.yaml` y `g1_backend/policies/locomotion.py` definen
96 valores por instante y cinco muestras de historia: entrada ONNX de 480.
Los términos son velocidad angular, gravedad proyectada, comando vx/vy/wz,
posiciones articulares relativas, velocidades articulares y acción anterior.
Se conservan escalas, mapeo de articulaciones y artefactos con checksum.
No recibe velocidad lineal medida, contactos, acelerómetro directo ni alturas
del terreno. Su ausencia no demuestra un error: es el contrato del checkpoint.

Hallazgo importante: el export declara rango angular de ±0.2 rad/s, pero
`configs/locomotion.yaml` lo amplía a ±1 rad/s. Esto sale del rango declarado del
export; no permite concluir cuál fue toda la distribución de entrenamiento.
La asistencia de giro modifica adicionalmente las consignas. Antes de ampliar
más ganancias hay que medir la respuesta efectiva de marcha.

La historia se actualiza cuando se ejecuta la inferencia; no lleva tiempos por
muestra ni comprueba que cada LowState sea nuevo. El período objetivo es 20 ms,
pero saltos del scheduler alteran el espaciado real de las cinco observaciones.
La acción anterior guardada es la salida de la red; el transporte puede recortar
su objetivo articular. Conviene registrar ambas cosas sin cambiar a ciegas
la semántica de last_action del modelo.

### Mejoras propuestas

- Auditar el contrato contra el despliegue de referencia: frames y signos del
  giróscopo/gravedad, mapeo SDK, escalas, orden temporal, posición por defecto y
  composición de los 480 valores. Extender las comprobaciones existentes con
  movimientos conocidos y trazas de observaciones completas.
- Registrar frecuencia real de inferencia, intervalos de muestra, repeticiones,
  pérdidas y distribución de cada término. Comprobar que no existen saturaciones
  silenciosas y que la historia mantiene tiempos compatibles con el export.
- Hacer que un perfil de realismo opcional alcance encoders e IMU de balance.
  Hoy `--real` afecta SLAM/percepción, no toda la cadena que controla la marcha.
  Introducir ruido, bias, cuantización y retardo gradualmente, con pruebas de
  estabilidad y sin mezclar datos desalineados.
- Medir seguimiento de vx, vy y wz dentro del rango declarado primero: error
  estacionario, demora, deriva transversal y acoplamiento entre giro y avance.
  Después estudiar el rango ampliado y si la asistencia puede retirarse.
- Si queremos velocidad estimada, contactos o mapa de alturas como nuevas
  entradas, entrenar/exportar otra política o elegir una que las espere.
  Agregarlas directamente al ONNX actual rompe su contrato; no son 480 valores
  arbitrarios que puedan reemplazarse por otros.

Holosoma es un proveedor alternativo con otro contrato de 100 observaciones;
además fija observaciones de brazos para separar su control. No debe mezclarse
su preprocesamiento con el de la política RL Lab predeterminada.

## Actuadores: qué existe

La red entrega posiciones articulares objetivo. `DdsRobot.send` recorta esos
objetivos a límites articulares y manda dq deseada y torque feedforward cero,
con kp/kd del despliegue. El puente calcula
`tau = kp*(q_obj-q) + kd*(dq_obj-dq) + tau_feedforward` en cada paso de física.
El modelo tiene rangos de fuerza por articulación (por ejemplo 88, 139 y 50 Nm
según articulación), fricción, amortiguamiento y armature. No es correcto decir
que todos los motores son ilimitados o que no hay dinámica mecánica.

No encontré en esta ruta un modelo explícito de respuesta eléctrica del motor,
retardo configurable de actuación, curva torque-velocidad, batería/temperatura
ni backlash. Armature y fricción son aproximaciones mecánicas, no validación de
un motor real. El watchdog sí cambia a damping si falta la orden.

### Mejoras propuestas, en orden

1. Registrar torque PD solicitado y esfuerzo efectivo después de límites,
   saturación por articulación, seguimiento q/dq, kp/kd y comando realmente
   aplicado. Hoy contar objetivos recortados no revela saturación de torque.
2. Comprobar en el MjModel efectivo los límites y fuerzas, además del XML;
   comparar puente, modo de control SDK y despliegue de referencia. Verificar
   que ganancias y mapeos coinciden y que sensores de torque representan lo
   que creemos medir.
3. Medir respuestas a escalones pequeños y barridos de frecuencia, primero
   con soporte y después en equilibrio: tiempo de respuesta, sobreimpulso,
   amortiguamiento y seguimiento. No usar movimientos grandes como primer
   procedimiento de identificación.
4. Añadir opcionalmente retardo y dinámica de primer orden de torque, con
   parámetros por grupo motor. Después introducir curvas torque-velocidad y
   variación de fricción si hay datos que permitan identificarlas.
5. Incorporar variaciones de parámetros al entrenamiento/evaluación. Un modelo
   más realista puede hacer fallar un checkpoint entrenado con otros motores;
   eso requiere adaptación de política, no sólo aumentar kp para compensarlo.

## Secuencia recomendada

Primero adquisición y observabilidad: completar RobotState, registrar tiempos,
observaciones y esfuerzos efectivos. Después comparar consignas y respuesta
medida, incluyendo el giro que hoy requiere ayuda. En paralelo desarrollar el
estimador corporal como módulo independiente y contrastarlo contra verdad sólo
para evaluación. Finalmente aplicar imperfecciones al balance/actuadores y
considerar otra política o reentrenamiento si el contrato actual limita el
resultado. Así cada cambio tiene una medición que demuestra qué mejora.
