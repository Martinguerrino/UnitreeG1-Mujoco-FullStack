# Mejoras de percepción y comparación — 2026-10-09

Se implementaron espera no bloqueante de TF, asociación global de objetos por
plano de apoyo, validación de candidatos y parches de superficie libre observada.
No se incorporaron agarres ni órdenes a los brazos.

## Matriz estática de Maxwell

18 configuraciones, cinco capturas por configuración: 0.6/1.0/1.5 m,
-20/0/20 grados, ideal y ruido de `sensors_real.yaml`. Se conservó el modelo
YOLO distribuido, con hash en `comparison.json`; sólo cambió el refinamiento.
La referencia y segmentación de MuJoCo son entradas exclusivas del evaluador.

| Cajas visibles | Antes | Después |
|---|---:|---:|
| Asociaciones | 90/90 | 89/90 |
| Omisiones visibles | 0 | 1 |
| Candidatos extra | 30 | 20 |
| Precisión de candidatos | 75.0% | 81.7% |

Se considera visible una referencia con al menos 20 píxeles y se permiten
asociaciones hasta 20 cm; estas cifras no garantizan exactitud general ni
geometría completa. El total de referencias incluye otras cajas fuera de vista,
por lo que se presenta aquí la métrica de visibilidad comparable.

La validación de una tapa completa se aplica cuando hay suficiente evidencia.
Una caja parcialmente observada puede mantenerse con confianza visual mínima
0.55 y evidencia geométrica parcial; se identifica explícitamente como `limited`
y `box_shape_incomplete`. Esto conserva la caja amarilla en vistas difíciles,
pero también permite algunas confusiones visuales persistentes. No se interpretan
los scores de YOLO como probabilidades calibradas. Un ensayo estricto guardado en
`static_strict_trial` eliminaba los extras a costa de omitir 26 asociaciones
visibles, por eso no se adoptó.

La evaluación de mesas conserva 60/90 asociaciones y 24 candidatos extra.
Las superficies geométricas de apoyo siguen identificadas como clase visual
sin confirmar. Estos límites no quedan resueltos por el filtro de cajas.

## Seguimiento dinámico

118 capturas renderizadas de dos perfiles, con poses prescritas y movimiento de
la caja roja. Las seis comprobaciones de estabilidad, pérdidas y recuperación
pasaron en ambos perfiles. No son una prueba de locomoción.

Durante la aproximación ideal el ID de la referencia pasó de dos IDs a uno,
y los candidatos extra de tres a cero. Con ruido se mantuvo un ID y los extras
pasaron de dos a cero. La asociación usa la última observación sin filtrar y
se realiza una sola vez para todos los objetos, preservando el plano de apoyo
cuando cambia la identidad visual de una mesa. No evita todos los intercambios
entre objetos parecidos ni cambios de categoría de YOLO.

## Marcha física con ROS

Se repitió el comando corto de avance, giro y retroceso frente a la mesa, sin
Nav2 ni interfaz gráfica. La trayectoria y los tiempos no son idénticos entre
sesiones: los resultados son evidencia de integración, no una comparación
controlada de rendimiento.

| Métrica | Antes | Después |
|---|---:|---:|
| Tiempo simulado | 18.034 s | 18.012 s |
| Desplazamiento máximo XY | 1.076 m | 1.074 m |
| Mensajes de percepción | 36 | 36 |
| Capturas con mesa (`ok`) | 14 | 19 |
| Observaciones de cajas | 28 | 28 |
| Observaciones estables | 3 | 10 |
| Salidas `unavailable` | 7 | 0 |

Después: una espera inicial de TF, una de RGB-D y 15 capturas sin mesa.
El simulador terminó con cero caídas, cero watchdogs y cuatro contactos.
Esto no acredita navegación sin colisiones ni detección continua en toda la marcha.
Los IDs de esta prueba no se asocian a referencia, porque se ven varias cajas.

La cola final informó cero descartes por capacidad, 17 por antigüedad,
9 por espera TF vencida y 114 capturas reemplazadas por otras más recientes.
La inferencia sigue a 2 Hz y la cámara aproximadamente a 10 Hz; no se intenta
procesar cada imagen. La cola se mantiene cada 20 ms y distingue TF ya disponible
de la espera de turno de inferencia. Los plazos no se eliminan: una captura sin
TF a tiempo o demasiado antigua se descarta, sin usar transformaciones actuales
como sustituto de las del instante de captura.

## Superficie libre y panel

Las 19 capturas con mesa publicaron parches libres, con 59 rectángulos en total.
Son regiones de la superficie observada, no una reconstrucción de todo el tablero.
Se excluyen puntos elevados, huellas de objetos y celdas sin superficie observada;
se aplican márgenes de 4 cm más protección por discretización. La rejilla tiene
2.5 cm de resolución, por lo que no garantiza ausencia de obstáculos menores a
esa resolución, muy bajos, transparentes o sin profundidad fiable.

RViz muestra los parches cian en **RGB-D perception**. El panel agrega
**Libre visible (m²)** y la advertencia de geometría parcial. Los parches vencidos
se eliminan y el área vuelve a cero. No se calcula colocación ni alcanzabilidad.

## Ejemplos difíciles y verificaciones

Se generaron 120 escenas sintéticas nuevas con semilla 193, incluyendo 60 casos
sin cajas objetivo (fondos/muebles y mesas con objetos redondos). En esos 60
casos el YOLO actual no propuso cajas, y tampoco hubo candidatos 3D de caja.
Esto confirma esos ejemplos; no demuestra por sí solo que el filtro reduzca
falsos positivos. Las pruebas con máscaras deliberadamente mal clasificadas
verifican rechazo de tapas redondas y duplicados entre categorías.

Los ejemplos se incorporaron al generador de entrenamiento, con proporción
configurable y manifiesto. Los pesos incluidos no se reentrenaron ni sustituyeron.

Pasaron 83 pruebas Python, las pruebas de ambos paneles Qt, Ruff y
`git diff --check`. Las pruebas cubren TF retrasado con timestamp exacto, selección de un par anterior
listo, cadencia de inferencia, reinicio de reloj, pérdida de planos de apoyo,
asignación uno a uno, bordes, regiones ocultas, obstáculos sin detectar,
huellas giradas, geometría parcial y panel Qt. Ambos paneles se compilaron,
cargaron con pluginlib y pasaron sus pruebas.

## Repetir

```bash
source scripts/python_env.sh
"$G1_PYTHON" scripts/evaluate_perception.py --backend yolo \
  --model assets/perception/g1_objects_yolo11n_seg.onnx \
  --output results/perception/improvements_20261009/static_after
"$G1_PYTHON" scripts/evaluate_perception_dynamic.py \
  --output results/perception/improvements_20261009/dynamic_after
"$G1_PYTHON" scripts/evaluate_perception_hard_negatives.py

source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
/usr/bin/python3 scripts/check_perception_motion_ros.py \
  --output results/perception/improvements_20261009/motion_after
```

La prueba de marcha requiere que no haya otra navegación activa, porque comparte
sus puertos. Datos, logs, manifiestos y comparación:
`results/perception/improvements_20261009/`. Los resultados históricos anteriores
se conservan allí como `dynamic_evaluation_before.json` y `motion_ros_before.json`.
