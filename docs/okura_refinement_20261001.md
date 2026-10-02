# Okura: ajustes y verificación física por fases — 2026-10-01

La política ACT sigue sin agarrar el objeto. Se completaron 16 intentos físicos de política, 13 con comandos sin eventos de watchdog ni estados de pinza vencidos; hubo **0 agarres**. Los escenarios incluyen mesa simple, una aproximación visual a la demostración y transferencia completa solicitada. En transferencia se agotó el límite de 25 s de `pick`, por lo que las fases posteriores no se ejecutaron con un agarre producido por ACT.

Los ajustes mejoraron problemas medibles del controlador, pero no resolvieron la manipulación autónoma. No se modificaron los pesos del checkpoint, el backend compartido, los assets originales ni el contrato SDK. Las nuevas opciones conservan los valores anteriores como defaults. Las escenas experimentales y configuraciones están separadas.

## Ajustes implementados

- `--arm-gain-scale`: ganancia PD de brazos y muñecas; amortiguamiento escalado por la raíz de la ganancia. Con escala 2, el error absoluto de seguimiento P95 bajó de 0,0556 a 0,0305 rad en episodios con comandos limpios.
- `--arm-speed`: límite de velocidad de consigna; se probó 3 rad/s frente al original 1,5 rad/s.
- `--waist-gain-scale`: estabilización opcional de cintura. En la escena de referencia, escala 4 redujo el error P95 de pitch de 0,1984 a 0,0543 rad (11,4° a 3,1°). El objeto siguió sin contacto.
- `--action-timing compensate|ensemble`: selecciona la acción correspondiente al tiempo simulado de observación o mezcla predicciones vigentes. Se observó una demora mediana de aproximadamente 0,28–0,30 s simulados entre observación y disponibilidad del chunk, equivalente a 8–9 acciones a 30 Hz. `legacy` sigue siendo el default.
- `--async-commands`: publicación independiente de renderizado/inferencia, conservando el watchdog y la interrupción deliberada de comandos. Los episodios finales quedaron sin cortes involuntarios.
- Cámara configurable, escala/posición de cesta, resets alternativos y `--visual-style reference`: aproximación visual opcional con mesa gris, carcasa oscura y rejilla cruzada.
- `--placement-speed-window 0.1`: velocidad estimada por desplazamiento en una ventana física de 100 ms. Evita bloquear la condición de reposo por oscilaciones del solver; conserva contacto con mesa, ausencia de contacto con mano, pinza abierta y tiempo de estabilidad. Por defecto sigue usando la velocidad instantánea.
- Registro de tiempos de inferencia, estados, fuerzas, contactos, configuración y procedencia; herramientas de análisis y pruebas físicas aisladas de fases.

## Intentos de política

Cada episodio conserva `report.json`, `trajectory.npz`, entradas de inferencia, imágenes y video en `results/okura_refinement_20261001/<nombre>/`.

| Intento | Comandos limpios | Agarre |
|---|---|---|
| baseline | No | No |
| gain2 | Sí | No |
| speed3 | Sí | No |
| compensate | Sí | No |
| ensemble | No | No |
| mean_reset | Sí | No |
| aligned | Sí | No |
| aligned_park | Sí | No |
| combined | Sí | No |
| transfer | No | No; pick_timeout |
| async_baseline | Sí | No |
| async_gain2 | Sí | No |
| async_ensemble | Sí | No |
| async_transfer | Sí | No; pick_timeout |
| reference_transfer | Sí | No; pick_timeout |
| reference_waist4 | Sí | No; pick_timeout |

Los tres episodios con cortes están registrados pero excluidos de la comparación de comandos limpios. Algún intento tuvo contacto de un dedo y movimiento submilimétrico del objeto: no satisface agarre bilateral sostenido ni levantamiento. La última combinación no tuvo contacto alguno. Todas las trayectorias numéricas de estos 16 intentos son finitas.

## Verificación de todas las fases

Para alcanzar las fases bloqueadas por `pick` se crearon **fixtures diagnósticos con agarre preparado**, sin ACT: se coloca el objeto inicialmente en la pinza, se cierra con gravedad deshabilitada durante 1 s y después se exige retención bilateral con gravedad durante 0,5 s. No se suelda el objeto ni se modifica su posición durante la fase. Se conserva explícitamente esta preparación en cada reporte.

| Fase | Resultado físico | Alcance |
|---|---|---|
| pick | Falla en todos los intentos ACT | Sin agarre autónomo |
| hold | Pasa a stand | Retiene el agarre preparado |
| stand | Pasa a carry | Libera la fijación de pelvis y sostiene el objeto durante la transición |
| carry | Falla: object_lost | El robot avanza, pero pierde el objeto antes de llegar |
| lower | Pasa a release | Fixture de colocación con pelvis fija y altura inicial de 1 m |
| release, observador original | Falla: release_timeout | Mesa, pinza abierta y mano sin contacto; velocidad instantánea cercana al umbral de 0,05 m/s |
| release, timestep 1 ms | Falla: release_timeout | Reducir el paso de física no resolvió la condición |
| release, ventana 100 ms | Pasa a done | Objeto apoyado y libre; velocidad por ventana 0,00432 m/s al final |

Las primeras fixtures de colocación con altura 0,8 m no conservaron agarre bilateral tras habilitar gravedad y se excluyen de la evaluación de esas fases. La prueba válida se repitió a 1 m. `lower` y `release` prueban colocación bajo soporte de pelvis; no prueban colocación mientras el robot equilibra libremente. No hay éxito de transferencia completa ni una única ejecución exitosa que encadene todas las fases.

## Diagnóstico de la política

La evidencia más fuerte sigue apuntando a discrepancia visual y de estado respecto del entrenamiento. Comparando el frame público inicial real con la imagen inicial simulada y manteniendo el mismo estado derecho, el error de predicción frente a las acciones públicas fue:

| Imagen inicial | RMSE brazo derecho (rad) | RMSE pinza derecha (rad motor) |
|---|---:|---:|
| Real | 0,0476 | 0,2026 |
| Simulada original | 0,1365 | 3,4427 |
| Simulada con estilo de referencia | 0,1039 | 3,5473 |

El estilo de referencia mejora el brazo en este diagnóstico, pero no la pinza. Estas mediciones **no reproducen el entrenamiento completo**: el dataset público disponible tiene 8 dimensiones derechas, falta el estado izquierdo original de 16 dimensiones y se usó la media del checkpoint para ese lado. Las comparaciones de frames posteriores usan una imagen simulada inicial fija y sólo muestran sensibilidad, no un replay válido. No se realizó fine-tuning con estados izquierdos inventados.

El ajuste de cámara también es aproximado: encaja dos landmarks de mano derecha y el borde de mesa; el residual de 6,3 px corresponde sólo a la mano. Algunos parámetros llegaron a sus límites de búsqueda. La cesta fue recolocada después del ajuste. Esto no constituye una calibración completa del sensor ni de la escena. El reset `park-left` tampoco reproduce una postura izquierda medida y puede salir de la distribución del checkpoint.

La cintura cambiaba la vista después del reset; su corrección redujo ese cambio sin lograr agarre. Aumentar ganancias y compensar latencia mejoró el control, pero no basta para atribuir la falla exclusivamente a seguimiento o temporización. La pérdida en `carry` es un problema adicional y separado de la ausencia de agarre ACT.

## Qué cambiar después, en orden

1. Obtener las demostraciones originales de 16 dimensiones, extrínsecos/intrínsecos y transformaciones de pinza; reconstruir el reset y verificar la dirección/apertura de cada canal con un replay físico de acciones registradas. Primero debe lograrse una demostración reproducible sin inferencia.
2. Calibrar cámara y transformación muñeca–pinza con varios landmarks y posturas bajo carga; medir cintura estable antes de tomar la primera imagen. Mantener escala física del objeto y contactos comprobados. No seguir ajustando sólo ganancias para corregir un error perceptivo.
3. Recoger demostraciones en esta escena con las dos pinzas y el estado de 16 dimensiones real. Fine-tuning de ACT sobre datos compatibles y evaluación en posiciones/episodios reservados. Mantener el checkpoint actual como baseline.
4. Para transporte, probar primero retención quieta prolongada y marcha lenta con agarre preparado; identificar apertura, deslizamiento y oscilación, y ajustar postura de transporte, cierre/fricción y aceleración usando contactos medidos. Sólo después integrar con el agarre aprendido.
5. Repetir colocación sin fijación de pelvis y, finalmente, el recorrido completo. El observador de ventana debe seguir siendo experimental hasta pasar escenas con objetos aún moviéndose; su prueba unitaria ya verifica que una traslación de 0,1 m/s no se considera estable.

## Reproducción y validación

Desde la raíz del repositorio, en el entorno `mujoco-unitree` con `MUJOCO_GL=egl`:

```bash
python results/okura_refinement_20261001/run_matrix.py
python results/okura_refinement_20261001/run_async_matrix.py
python results/okura_refinement_20261001/run_reference_transfer.py
python results/okura_refinement_20261001/run_reference_waist4.py
python results/okura_refinement_20261001/run_phases.py
```

Estos scripts guardan parámetros explícitos y ejecutan episodios secuencialmente con dominios DDS locales. `run_phases.py` reproduce la matriz inicial; las pruebas de colocación refinadas se reproducen mediante `experiments.okura.phase_probe` con `--phase lower|release --placement-height 1.0`, y para el caso exitoso de release `--speed-window 0.1`. Por ejemplo: `python -m experiments.okura.phase_probe --phase release --placement-height 1.0 --speed-window 0.1 --output results/okura_release_repeat`. Consultar `--help` para las restantes opciones. No superponer procesos del mismo dominio DDS. Los tiempos reportados son simulados; la duración de 70 s solicitada en transferencia termina antes por el timeout de pick.

Validación: suite completa **93 passed, 3 skipped**; prueba focalizada de las nuevas opciones y utilidades **7 passed**; Ruff y `git diff --check` pasan. Las dos pruebas mecánicas detectaron correctamente la pérdida deliberada de comandos. Los tests validan infraestructura y regresiones, no éxito de política.

Resultados estructurados: `docs/okura_refinement_20261001.json`. Gráfico: `results/okura_refinement_20261001/comparison.png`. Video del último intento: `results/okura_refinement_20261001/reference_waist4/camera.mp4`. Video de liberación preparada: `results/okura_refinement_20261001/phase_release_window/camera.mp4`.
