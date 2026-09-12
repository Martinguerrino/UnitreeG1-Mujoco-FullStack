# Pruebas y depuración

Empieza siempre por la capa más baja que pueda demostrar el problema.

```bash
source scripts/activate.sh
./scripts/check.sh
```

Si MuJoCo no carga, revisa el error XML o el mesh mencionado. Si MuJoCo carga
pero no aparecen controladores:

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

Debe haber interfaces `effort` disponibles y dos controladores `active`.

Si el controlador está activo pero no se mueve:

```bash
ros2 action list
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
```

Si se mueve mal, grafica error de posición y esfuerzo antes de tocar ganancias.
Una mejora que “se ve mejor” pero no tiene métricas no es todavía una mejora de
control.

El modo `headless:=true` elimina problemas de OpenGL y sirve para integración
continua. El viewer debe probarse aparte cuando el ciclo de control ya funciona.

## RViz falla con `GLIBC_PRIVATE` o `libpthread`

Si VS Code fue instalado mediante Snap, su terminal integrada puede heredar
rutas GTK de `/snap/core20`. RViz es una aplicación nativa del sistema y mezclar
esas bibliotecas con las de Ubuntu produce errores como:

```text
symbol lookup error: .../libpthread.so.0: undefined symbol: __libc_pthread_init
```

`scripts/activate.sh` elimina incondicionalmente esas rutas GTK antes de iniciar
ROS y el launch vuelve a aislar el entorno del proceso RViz. Esto también cubre
terminales que heredan rutas `/snap/...` pero no exportan la variable `SNAP`.
Usa `./scripts/sim.sh rviz:=true`; no hace falta desinstalar VS Code ni modificar
el sistema.

Cuando `rviz:=true`, el launch espera brevemente a que comience MuJoCo, activa
`use_sim_time`, carga `/robot_description`, selecciona `world` como marco fijo y
centra la cámara en `pelvis`. No hay que agregar `RobotModel` ni configurar la
cámara manualmente.

## `Controller already loaded` y el tiempo retrocede

Estos mensajes suelen indicar que ya existe otro launch del mismo robot en el
mismo dominio ROS:

```text
Controller already loaded
Moved backwards in time
```

Cada simulador publica `/clock` y ofrece `/controller_manager`. Si se ejecutan
dos instancias, sus tiempos y servicios se mezclan. Vuelve a la terminal de la
primera simulación, ciérrala con `Ctrl+C` y luego inicia una sola instancia. Para
trabajar deliberadamente con varias simulaciones hay que darles namespaces o
valores `ROS_DOMAIN_ID` diferentes.

Los avisos sobre planificación FIFO/realtime y la inercia del link raíz no
impiden que la simulación funcione; son advertencias, no fallos de RViz.
