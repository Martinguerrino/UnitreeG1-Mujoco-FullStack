# Backend MuJoCo independiente

El backend usa el modelo y la escena existentes. La ruta de control es:

```
LocomotionPolicy / ManipulationPolicy
          → DdsRobot (SDK2 oficial)
          → rt/lowcmd
          → bridge Python oficial unitree_mujoco
          → MuJoCo/G1
          → rt/lowstate → RobotState
```

ROS/MoveIt sigue disponible mediante sus scripts originales. No se arranca ROS
para estas pruebas. No se entrena ni se crea otro escenario para esta ruta.
La prueba aislada [Dex1 + ACT](dex1_act.md) reutiliza la escena experimental Okura;
su integración al executor general sigue pendiente de validar el agarre.

## Instalación y ejecución

Desde la raíz, con el entorno de `environment.yml` activado y CMake/compilador C:

```bash
python scripts/setup_dds.py
python scripts/run_experiment.py --config configs/experiments/communication.yaml
python scripts/run_experiment.py --config configs/experiments/locomotion.yaml
python scripts/benchmark_locomotion.py
```

El instalador descarga revisiones y verifica SHA-256 de `configs/sources.lock.json`.
Compila CycloneDDS nativo 0.10.5 (binding Python 0.10.2) en `artifacts/cyclonedds` y coloca los paquetes Python
en `artifacts/dds_runtime`, sin actualizar ROS/ACT. Instala las bibliotecas CRC
originales que el wheel SDK2 omite. En equipos donde pip no disponga de wheel,
la compilación de CycloneDDS Python usa ese mismo prefijo local. El binding
se compila sin usar wheels de caché de otros entornos. La versión nativa 0.10.2
abortó en snprintf al crear el dominio con el compilador de Ubuntu; 0.10.5
contiene la corrección upstream y está fijada por revisión y SHA-256.
`--download-only` permite preparar artefactos sin instalar paquetes.
No se requieren Isaac Lab, Isaac Sim ni GPU para locomoción.

DDS requiere sockets locales; un sandbox que bloquee interfaces/red impedirá
la prueba aunque no se contacte ningún robot físico. El launcher acepta solo
`lo` y dominios 1..232; predeterminado 42. Un bloqueo por dominio evita ejecutar
dos instancias propias en el mismo dominio. Otros programas DDS deben usar otro
dominio. SDK2 fija el dominio para toda la vida del proceso: cambiarlo requiere
un proceso nuevo; el benchmark hace esto por episodio.

## Configuración y reset

Las referencias YAML a componentes se resuelven desde la raíz del repositorio.
Las rutas de checkpoint/modelo también. La API acepta un dict ya resuelto o
una ruta a YAML. `robot.yaml` configura timestep, interfaz/domain, timeouts,
posición/orientación inicial y modelo; `locomotion.yaml` selecciona proveedor;
`manipulation.yaml` reserva la política futura; `camera.yaml` configura sensores.

Cada `run` crea un proceso MuJoCo y MjData nuevos: ese es el reset reproducible.
La postura inicial se aplica solo al inicializar el episodio. La base conserva
el fixture existente durante warmup; se libera al iniciar locomoción. El test
de comunicación y la manipulación aislada mantienen base fija.

El wrapper sincroniza callbacks y física, verifica CRC/valores de LowCmd y
activa amortiguación si cesan los comandos durante 0.25 s. El controlador aborta
si LowState queda obsoleto, la telemetría deja de avanzar o se detecta caída.
Ctrl+C ejecuta limpieza y detiene el proceso. La parada por amortiguación y el
watchdog se ensayan con una interrupción deliberada de publicación en el test
de comunicación. No representan certificación de parada de hardware.

## Cámara y navegación

Cámara configurable por body, posición y orientación wxyz, FOV vertical,
resolución, FPS, RGB/depth. Por defecto está desactivada; RGB es la única
modalidad predeterminada. Al activarla se agrega únicamente un sensor al modelo
existente; no se agregan mesas/objetos ni se usa la escena experimental ACT.
La pose inicial es un valor de infraestructura, no calibración de un checkpoint.
El último frame real se guarda en `camera_latest.npz` con timestamp de simulación.
No se genera video cuando la cámara está desactivada.

GoTo usa pose privilegiada del simulador, recibida por el canal de telemetría.
Transforma el desplazamiento mundial a velocidad en el marco corporal y llama
la política de locomoción. Es navegación a un punto libre, sin obstáculos,
SLAM ni localización sim2real. Usa la velocidad solicitada hasta entrar en la
tolerancia: reducirla proporcionalmente a la distancia producía bloqueo por
la zona muerta de velocidad de la política.

Stand → Walk → Stop requiere mantener la pose dentro de tolerancias durante
`stop_duration` con comando de velocidad cero. El benchmark varía posición,
yaw, velocidad y distancia; registra fallos sin reemplazar la política.

## Resultados y API

```python
from g1_backend import MujocoBackend, run_experiment

report = MujocoBackend.run('configs/experiments/locomotion.yaml')
# Equivalente: run_experiment(config_dict)
```

Usar el guard `if __name__ == '__main__':` en scripts que llaman esta API, porque
el proceso MuJoCo se crea con multiprocessing spawn. No hacen falta joints,
SDK2 ni internals ONNX en el repositorio que la consume.

Cada episodio guarda en `results/<timestamp>_<id>/`:

- `config.yaml`: configuración resuelta.
- `metadata.json`: commit, árbol sucio, hashes del backend, versiones, seed,
  timestamp, checkpoints y hashes, y revisiones externas.
- `metrics.json`: resultado, motivo de fallo/bloqueo, errores, caídas, contactos,
  tiempos, frecuencia física, CPU y RAM máxima de ambos procesos.
- `trajectory.jsonl`: estado DDS, destinos, fase, comando y telemetría física.
- `camera_latest.npz`, si se habilitan sensores visuales.

GPU/VRAM y métricas de manipulación sin observador permanecen null, no cero.
La inferencia usa CPU. `render_fps` mide frames renderizados, mientras que
`steps_per_second` mide física. Colisiones cuentan comienzos de pares de
contacto, excluyendo apoyo ankle_roll–suelo; no es una clasificación semántica
de colisión válida para cualquier escenario.

Los seeds permiten repetir configuración e inferencia, pero DDS y planificación
de hilos de host no garantizan trayectorias idénticas bit a bit.
La salida CLI vale 0 solo para `completed`; fallo o bloqueo devuelve 2.

## Manipulación y tareas pendientes

`pick_and_place.yaml` expresa goto/pick/goto/place. El executor es secuencial y
carga políticas por proveedor. Con `provider: null` informa `blocked` sin
arrancar física. La interfaz está preparada, pero no se ha validado agarre,
colocación ni ejecución de tareas con una política de manipulación.
Las métricas físicas de objetos necesitan un escenario y observadores concretos.
No se declara terminada esa parte del plan.

Ver [auditoría inicial](current_state.md), [referencias](reference_audit.md),
[contratos](policy_interfaces.md) y [validación](validation.md).

## Ampliación: tareas, repeticiones y video

`configs/experiments/waypoints.yaml` ejecuta dos GoTo sin manipulación. Cada
paso admite `timeout`, `max_retries`, `settle_duration`, `max_speed`, tolerancias
de posición/orientación y `stop_speed_tolerance`. Los valores comunes van en
`task.defaults`; los del paso tienen prioridad. La espera debe ser continua:
salir de tolerancia reinicia el contador. Para pasar al siguiente GoTo, además
se comprueba velocidad horizontal de base ≤0.1 m/s por defecto.

Un timeout puede reintentar la skill hasta el máximo configurado. Reinicia la
skill; GoTo conserva el historial de locomoción y manipulación llama reset en
su proveedor. La física continúa desde el estado actual. Al agotar los intentos se detiene
la tarea y quedan `failed_skill`, índice y `retries`. `skills.json` conserva
inicio, timeout y terminación con tiempos, intentos y errores. Las caídas y
fallos DDS abortan; no disparan reintentos físicos automáticos.

El benchmark ahora lee casos y repeticiones de
`configs/benchmarks/locomotion.yaml`. Cada corrida crea un directorio propio,
con especificación, configuración/seed de cada episodio, reporte y log separado
del proceso. `summary.json` se actualiza después de cada intento. Incluye
agregados globales y por caso: tasa de éxito, estados, cantidad de mediciones,
media, mediana y extremos. Los valores ausentes no se convierten en ceros;
los bloqueos y errores de proceso cuentan como intentos no exitosos.

```python
from g1_backend import MujocoBackend
summary = MujocoBackend.benchmark('configs/benchmarks/locomotion.yaml')
```

Cada ensayo utiliza un intérprete nuevo. En un timeout externo se detiene su
grupo de procesos para cerrar también el simulador. El CLI de experimentos
acepta `--report archivo.json`: el consumidor ya no tiene que extraer JSON de
la salida mezclada con logs del SDK.

Para conservar muestras visuales, habilitar `camera.enabled` y `camera.record`.
`max_recorded_frames` limita el almacenamiento (predeterminado 10000). Se guarda
cada muestra nueva recibida en `trajectory/camera/*.npz`; `index.jsonl` registra
su timestamp de simulación e ID. `recording.json` distingue muestras guardadas,
observadas, saltos de IDs y truncamiento por el límite. Al cerrar el episodio
se conserva también `camera_latest.npz`. Esto no garantiza grabar todo frame
renderizado si el consumidor no alcanza la frecuencia del productor.

El video se codifica **después** de la simulación, sin bloquear el controlador:

```bash
# Solo si falta PyAV en el entorno; no requiere ACT:
python -m pip install -r requirements-video.txt
python scripts/export_video.py --experiment results/<episodio> --fps 10
```

El exportador usa las muestras RGB reales y sus tiempos de simulación, manteniendo
la imagen previa cuando faltan tiempos intermedios. No interpola movimientos,
no convierte depth en RGB y no vuelve a ejecutar física. `video.json` registra
la duración y cantidad de frames; la duración MP4 se redondea a la grilla de FPS
e incluye la presentación del último frame. La codificación usa la API de
[PyAV para imágenes NumPy](https://pyav.org/docs/stable/cookbook/numpy.html).

Los reportes nuevos incluyen `schema_version: 1`, hashes de modelos/meshes y
trayectoria con q, dq, IMU y antigüedad de LowState. `total_completion_time`
incluye inicialización y limpieza; `control_elapsed` mide solo el lazo. Las
fases se conservan también cuando el episodio termina por una excepción.

Estado completo y pruebas: [avance del plan](plan_progress.md).
