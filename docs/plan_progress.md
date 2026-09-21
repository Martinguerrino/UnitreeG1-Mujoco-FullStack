# Avance del plan — 2026-09-13

## Ampliación de navegación — 2026-09-19

Se inició la integración de Nav2 + SLAM solicitada posteriormente. El puente
ROS/DDS y el LiDAR virtual tienen una prueba física corta aprobada; Nav2, SLAM
y AMCL cuentan con launch/configuración, pero su validación conjunta está
pendiente de instalar los paquetes ROS. Ver [navegación](navigation.md).
La manipulación queda aplazada. El estado histórico de abajo no acredita
estabilidad en el recorrido largo ni en el visor, que siguen pendientes.

Se aplica `exp/plan.md`. La instrucción más reciente habilita probar Dex1 y ACT,
manteniendo la selección de políticas modular. Véase [Dex1 + ACT](dex1_act.md).

| Fase | Estado | Evidencia o límite |
|---|---|---|
| 0. Auditoría | Hecha | `current_state.md`, `reference_audit.md` |
| 1. G1 + SDK2/DDS | Validado en simulación local | LowCmd/LowState/IMU, CRC, watchdog, amortiguación, dominios y episodios nuevos para reset |
| 2. Locomoción preentrenada | Integrada | RL Lab velocity/v0, ONNX original, control exclusivamente DDS; baja velocidad todavía no robusta |
| 3. Manipulación | Ejecución aislada integrada | Dex1 + ACT por DDS; falta agarre válido e integración al executor |
| 4. Cámara | Infraestructura implementada | RGB/depth configurables; RGB grabado y exportado a video; falta calibración para la política futura |
| 5. Locomoción aislada | Benchmark ejecutable | Matriz YAML, seeds/repeticiones, errores, caídas, contactos y tiempos; 4/6 éxitos en la última corrida |
| 6. Manipulación aislada | Probada, agarre fallido | Pinzas abren/cierran; ACT mueve brazos y manos, sin levantar la caja |
| 7. Skills | Implementadas a nivel de despacho | GoTo validado; pick/place son adaptadores al proveedor pendiente |
| 8. Task Executor | Validado con locomoción | Dos GoTo con parada estable, deadlines, reintentos acotados y eventos por skill |
| 9. Pick-and-place | Pendiente | Configuración disponible, ejecución bloqueada explícitamente sin proveedor |
| 10. Evaluación | Implementada para lo medible | Agregados y reportes por caso, CPU/RAM/física, trayectoria, muestras visuales/video; GPU/VRAM y objetos sin observador siguen null |
| 11. Reproducibilidad | Extendida | Revisiones/hashes, configuración resuelta, seed, hashes de modelo/meshes, logs de procesos y reportes atómicos |
| 12. API externa | Implementada | `MujocoBackend.run(config)` y `MujocoBackend.benchmark(config)` |

Dex1 reutiliza la escena experimental Okura existente. No se añaden entrenamiento,
VLM, SLAM, WBC ni planner LLM.
La interfaz preparada de manipulación no es evidencia de éxito físico.
GoTo sigue utilizando pose privilegiada del simulador y no evita obstáculos.

## Validación de esta ampliación

- **40 tests pasan** (24.52 s), incluyendo los existentes.
- Tarea `waypoints.yaml`: dos skills completas, error final 0.0695 m,
  cero caídas/colisiones. Reporte de cada skill en `skills.json`.
- Prueba intencional de timeout: destino x=100 m, 0.4 s por intento y un
  reintento. Termina con `failed_skill=goto`, `failed_skill_index=0`, `retries=1`
  y ninguna skill completada. No reinicia ni teletransporta el robot al reintentar.
- Matriz de seis episodios (dos repeticiones por caso):

| Caso | Repetición 0 | Repetición 1 | Error final por repetición |
|---|---|---|---|
| 0.2 m/s | Timeout | Timeout | 0.3400 / 0.1158 m |
| 0.3 m/s, inicio desplazado | Éxito | Éxito | 0.0834 / 0.0875 m |
| 0.4 m/s, inicio desplazado | Éxito | Éxito | 0.0422 / 0.0741 m |

El segundo timeout acaba dentro de tolerancia de posición, pero no cumple la
permanencia de dos segundos del criterio de éxito antes del límite temporal.
No se convierte un error final pequeño en éxito retrospectivo. Cero caídas y
colisiones en los seis ensayos. Son pocas muestras, no una tasa general.

- Cámara: 31 muestras RGB reales, ninguna pérdida observada entre IDs recibidos
  y sin truncamiento. Exportación y decodificación verificadas: 32 frames de
  320×240, 10 FPS. El video dura 3.2 s y usa tiempo de simulación. El ensayo
  completo incluye cuatro segundos de control de host; no son el mismo reloj.
- La prueba de cámara/comunicación detecta watchdog y amortiguación; registra
  tres comienzos de contacto durante la interrupción deliberada de control.
- Ruff y `git diff --check` pasan. No se volvió a lanzar ROS/MoveIt.

Los datos resumidos quedan en `workflow_validation.json`. Los episodios y sus
archivos completos se conservan en los directorios `results/` indicados allí.

## Comandos

```bash
python scripts/run_experiment.py --config configs/experiments/waypoints.yaml
python scripts/benchmark_locomotion.py --matrix configs/benchmarks/locomotion.yaml --repetitions 2
python scripts/run_experiment.py --config configs/experiments/recording.yaml
python scripts/export_video.py --experiment results/<episodio> --fps 10
```

## Ampliación Dex1/ACT solicitada posteriormente

Se habilitó la prueba aislada con dos pinzas oficiales convertidas a MJCF y ACT
original por DDS. **44 tests pasan**, incluyendo retención/liberación física y
compatibilidad de sensores del bridge. Se corrigieron sentido de apertura,
fricción del solver y cuantización de encoder compatible con el normalizador.
El Task Executor conserva pick/place pendiente de agarre autónomo validado.
Ver [detalle y reproducción](dex1_act.md) y [métricas](dex1_act_validation.json).
