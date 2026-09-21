# Validación del backend DDS

Fecha: 2026-09-12. Ejecución local en Python 3.12, MuJoCo 3.3.6,
ONNX Runtime 1.22.1, SDK2 Python 1.0.1, CycloneDDS Python 0.10.2 y
CycloneDDS nativo 0.10.5. Inferencia CPU. Datos estructurados del benchmark
conservados en `docs/validation_metrics.json`; las trayectorias completas están
en los directorios `results/` referenciados allí y no se versionan.

## Contratos y regresiones

**26 tests pasaron en 7.11 s**, incluidos los 18 existentes. Se añadieron pruebas
de orden de sensores/actuadores, cámara compilable, proveedor intercambiable,
manipulación pendiente, validación de comandos, transformación de marcos,
secuencia de tareas e inferencia ONNX real con historial y reset.
Ruff check/format y sintaxis de scripts también pasaron.

No se reejecutó el lanzamiento ROS/MoveIt completo. Su código no fue sustituido.

## Locomoción: matriz real

`python scripts/benchmark_locomotion.py` ejecutó tres procesos independientes,
con posición/yaw inicial, velocidad y distancia diferentes. El criterio exige
posición ≤0.12 m y yaw ≤0.15 rad, sostenidos dos segundos con velocidad cero.

| Inicio x,y,yaw | Velocidad solicitada | Distancia x | Resultado | Error final | Error yaw | Tiempo de control |
|---|---:|---:|---|---:|---:|---:|
| 0,0,0 | 0.2 m/s | 0.4 m | Timeout | 0.2492 m | 0.0565 rad | 17.00 s |
| 0.2,-0.2,0.4 | 0.3 m/s | 0.6 m | Éxito | 0.0742 m | 0.0218 rad | 8.16 s |
| -0.2,0.2,-0.4 | 0.4 m/s | 0.8 m | Éxito | 0.0529 m | 0.0306 rad | 8.49 s |

Éxito: **2/3**. Ninguna caída ni colisión detectada en esos tres episodios.
Física observada: aproximadamente 474–475 pasos/s, inferior a los 500 nominales.
El tiempo de control incluye warmup; los resultados nuevos también desglosan
las fases. No se presenta esta pequeña muestra como tasa de éxito general.

El comando principal `configs/experiments/locomotion.yaml` también se repitió
con la instalación final: episodio `20260912T232242_3f233d81`, **éxito**, error
0.0641 m y 0.0139 rad, sin caídas/colisiones ni watchdog. Tiempo de control
8.13 s; caminata 2.11 s. Los ticks de política incluyen preparación de
observaciones y fases sin inferencia; el campo de tiempo medio se renombró
después a `mean_policy_tick_seconds` para no presentarlo como latencia pura ONNX.

El fallo a 0.2 m/s permanece visible. Se observó poco avance con ese comando;
no se cambia la velocidad solicitada ni se sustituye la política para forzar
éxito. La lógica GoTo inicial reducía además la velocidad al acercarse y dejó
ensayos detenidos; se corrigió manteniendo la velocidad solicitada hasta la
tolerancia. El resultado anterior exploratorio queda en
`results/locomotion_matrix_initial.json`.

## Comunicación, watchdog y cámara

Episodio `20260912T232140_97f19ec0`, dominio 90, cámara RGB 320×240 a 10 FPS
nominales: **completado**. Imagen real uint8 de forma (240,320,3), inspeccionada:
suelo y horizonte del escenario existente, sin crear objetos.

- Desplazamiento de muñeca medido por LowState: 0.0906 rad, tras LowCmd.
- 2063 mensajes LowState en cuatro segundos de control (~515.7 Hz).
- Interrupción deliberada de comandos: watchdog observado, un evento.
- Comando de amortiguación explícito: observado en el bridge.
- Cero CRC/comandos inválidos detectados.
- Física con cámara: 417 pasos/s; render medido: 8.28 FPS.
- Tres comienzos de contacto no clasificados como apoyo durante el ensayo de
  amortiguación con base fija; no se ocultan ni se cuentan como agarres.

El primer render se precalienta antes de anunciar que DDS está listo: la
compilación inicial de shaders EGL excedía el timeout del estado. Las pruebas
no certifican una cámara de entrada para una política de manipulación concreta.

## Instalación reproducible

El script se ejecutó de extremo a extremo. Se detectaron y resolvieron:

1. CRC nativo omitido por el wheel de SDK2: se instala desde el archivo oficial.
2. Binding CycloneDDS en caché enlazado a otro proyecto: se recompila sin caché
   contra `artifacts/cyclonedds`.
3. Aborto nativo de CycloneDDS 0.10.2 en `do_print_uint32_bitset`/snprintf bajo
   fortify de Ubuntu: se usa la corrección upstream de 0.10.5 fijada por commit.
4. Posible interferencia entre ensayos propios: exclusión por dominio DDS.

## Qué sigue pendiente

ACT/manipulación y validación de manos, agarre, lift, place y traslado quedan
aplazados por instrucción del usuario. La tarea pick-and-place devuelve
`status: blocked`, `failed_skill: manipulation` cuando no hay proveedor.
No ejecuta movimientos sustitutos. Los observadores de objetos y benchmarks
físicos de manipulación dependen de la política y del escenario seleccionados.

La API, configuraciones, despacho de tareas y contrato de proveedor están
implementados; no se declara validación física de pick-and-place. GoTo usa pose
privilegiada MuJoCo, sin navegación con obstáculos ni localización para hardware.

## Ampliación del 2026-09-13

La validación de tareas con varios destinos, reintentos, benchmark repetido y
video está en [plan_progress.md](plan_progress.md) y sus datos estructurados
en `workflow_validation.json`. La suite pasó a 40 tests. Los resultados de
esta página se conservan como evidencia de la integración inicial.

## Validación posterior: pinzas Dex1 y ACT

La ampliación se documenta en [Dex1 + ACT](dex1_act.md), con resultados persistentes
en [dex1_act_validation.json](dex1_act_validation.json). Pasan 44 tests y la prueba
mecánica de pinzas; los dos ensayos ACT finales no consiguen agarrar la caja.
