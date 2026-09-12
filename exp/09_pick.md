# Fase 6: acción de Pick y límite físico actual

Esta entrega implementa la coordinación de Pick, pero **todavía no permite
recoger un objeto físicamente con el G1 actual**. El modelo de 29 DOF tiene
manos rígidas. Falta un mecanismo de agarre y su backend en MuJoCo. La acción
aborta antes de mover el brazo si ese backend no existe.

## Qué se agregó

`g1_interfaces` define `/g1/pick` como acción ROS: recibe un objetivo, publica
la etapa actual y devuelve el resultado. `g1_manipulation/pick_server` coordina:

1. Comprobar backend físico y objeto en Planning Scene.
2. Planificar y ejecutar preagarre.
3. Planificar y ejecutar aproximación.
4. Solicitar y confirmar agarre físico mediante `/g1/grasp`.
5. Adjuntar el objeto en MoveIt y esperar confirmación de la escena.
6. Planificar y ejecutar elevación.

Solo acepta una petición a la vez. Usa velocidad y aceleración al 10% de los
límites de MoveIt. La cancelación es cooperativa entre etapas: una trayectoria
que ya empezó puede terminar antes de cancelar. No es una parada de emergencia.

## Contrato del objetivo

El objeto debe existir previamente en MuJoCo y en MoveIt con el mismo ID.
`grasp_pose` es la pose deseada de la mano, **no el centro del objeto**. La capa
que conozca su geometría deberá calcular ese desplazamiento. Por ahora solo se
acepta el frame `pelvis`, para no depender de la alineación pendiente de `world`
en el modelo de planificación. Las distancias admitidas son mayores que cero y
hasta 0.25 m. La preaproximación resta X y la elevación suma Z en ese frame.
Los movimientos usan planificación libre: no se promete aproximación rectilínea.

El servidor requiere base fija y uso exclusivo del controlador: no envíes en
paralelo movimientos desde RViz, `arm_pose.py` u otros clientes. El bloqueo del
servidor solo arbitra sus propias peticiones.

## Probar el rechazo esperado sin backend

Compila con `./scripts/build.sh`. En una terminal:

```bash
source scripts/activate.sh
ros2 run g1_manipulation pick_server
```

En otra:

```bash
source scripts/activate.sh
ros2 action send_goal /g1/pick g1_interfaces/action/Pick \
  "{object_id: cube, arm: left_arm, grasp_pose: {header: {frame_id: pelvis}, pose: {position: {x: 0.3, y: 0.2, z: 0.2}, orientation: {w: 1.0}}}, approach_distance: 0.08, lift_distance: 0.10}" \
  --feedback
```

Resultado esperado con las manos actuales: `success: false` y mensaje de backend
físico ausente. No requiere arrancar MuJoCo para comprobar este rechazo.

## Qué falta para cerrar la fase

Implementar `/g1/grasp` requiere integrar una pinza/mano y detectar captura real
en MuJoCo. Responder `success: true` solo por cerrar dedos o dibujar un objeto
pegado en RViz viola el contrato. El backend debe ser idempotente y terminar en
menos de 10 segundos. Un timeout deja estado físico desconocido y bloquea nuevos
picks; requiere inspección y recuperación antes de reiniciar el servidor.

Después de un pick exitoso también queda bloqueado otro pick: sostener un objeto
necesita primero Place o recuperación. Todavía no hay un servicio de recuperación
ni persistencia del estado entre reinicios. Nunca reinicies para eludir esa
comprobación mientras el robot sostiene un objeto.

Queda pendiente validar colisiones completas (el URDF tiene links sin geometría
de colisión), coherencia temporal de MoveIt y ejecución física de principio a fin.
Por eso la fase 6 se considera **infraestructura implementada, demo física pendiente**.
