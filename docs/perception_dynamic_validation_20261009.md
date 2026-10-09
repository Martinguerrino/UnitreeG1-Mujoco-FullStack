# Evaluación dinámica de percepción — 2026-10-09

## Secuencias renderizadas de Maxwell

Se ejecutó `scripts/evaluate_perception_dynamic.py` con el modelo YOLO ONNX
incluido en `assets/perception`, a 424×240. Son 59 capturas por perfil, ideal y
ruido de `sensors_real.yaml`: 118 capturas en total. La referencia de MuJoCo y
su segmentación sólo se usan después de la inferencia para puntuar.

Las poses del robot se prescriben: aproximación de 0.9 a 0.6 m, cambio de vista
entre -15° y 15°, pérdidas breves/largas y desplazamiento de la caja roja de
12.5 cm. No representan una política de locomoción ni errores de sincronización.

Las seis comprobaciones se cumplieron en ambos perfiles:

- La caja estacionaria alcanza estabilidad.
- Al perder la vista no se publican cajas fantasma asociadas a las referencias.
- Una pérdida breve conserva el ID, pero reinicia la historia de estabilidad.
- Una pérdida larga supera el timeout y genera un ID nuevo.
- En las cinco capturas de movimiento la caja roja se marca inestable.
- Al detenerse recupera estabilidad (últimas cuatro de siete capturas).

Se observaron límites: en la aproximación ideal la asociación de referencia pasó
por dos IDs y hubo tres candidatos extra; con ruido hubo dos extras al acercarse
y uno al girar. En cada perfil la caja roja se omitió en una de nueve vistas de
giro. El RMSE de centro de todas las asociaciones aceptadas fue 9.40 mm ideal
y 11.61 mm con ruido; incluye filtrado y retraso al mover el objeto, pero excluye
omisiones y extras, y permite asociaciones hasta 20 cm. No es precisión general.

La purga de geometría acumulada por movimiento se verifica además mediante la
prueba `test_slow_cumulative_object_motion_resets_geometry`; en la secuencia
renderizada el movimiento sucede después de una pérdida larga y las vistas de
esa caja ya habían expirado, por lo que esa secuencia no acredita por sí sola
la purga de una reconstrucción con múltiples vistas.

Resultado completo: `results/perception/dynamic_evaluation/evaluation.json`.

## Marcha física y percepción ROS

Se ejecutó `scripts/check_perception_motion_ros.py`, sin visor ni RViz,
con cámara en GPU, ruido de sensores y estimación de odometría. El robot
comenzó frente a la mesa, recibió órdenes de avance, giro y retroceso y terminó
por duración. No se enviaron metas de Nav2.

- 18.034 s simulados, desplazamiento máximo en XY de 1.076 m.
- 0 caídas, 0 watchdogs; 4 contactos nuevos contabilizados por el simulador.
- 36 mensajes de percepción: 14 `ok`, 14 `no_table`, 1 espera inicial y
  7 salidas no disponibles (conexión TF inicial, extrapolación o captura antigua).
- 28 observaciones de cajas, de las cuales 3 estables, y cinco IDs de candidatos.
  Los IDs aquí no están asociados a referencia: también se ven diferentes cajas.

Confirma que marcha física, cámara y percepción funcionan juntos. Los contactos,
pérdidas de mesa y fallos transitorios de TF impiden presentarla como una prueba
de robustez satisfactoria durante toda la marcha. La referencia de posición sólo
puntúa desplazamiento; no mide el error de pose de objetos en esta prueba.

Resultado completo y logs: `results/perception/motion_ros/`.

## Profundidad con la cámara levantada

Se renderizaron seis inclinaciones (-30°, -15°, 0°, 15°, 30°, 45°) frente a la
mesa y en el origen. Negativo significa mirar hacia arriba respecto del montaje.
A -30° quedaron lecturas válidas en el 87.5% de los píxeles frente a la mesa y
61.3% en el origen con ruido. No se reprodujo una imagen métrica totalmente vacía.

Frente a la mesa el fondo ideal alcanzó aproximadamente 400 m (plano lejano de
renderizado), mientras las superficies cercanas comenzaban a 0.61 m. Normalizar
esa imagen completa puede comprimir el contraste de las distancias útiles.
Esta es una explicación compatible con el aspecto negro observado, no una
confirmación de su causa exacta en la sesión del usuario.

La vista `D435i Depth` ahora muestra una previsualización RGB con escala fija
0.2–6 m y píxeles violetas para valores inválidos/fuera de rango. Incluye el
porcentaje sin lectura. El tópico de profundidad métrica no se modifica. Si
se apunta hacia superficies fuera del rango real, la visualización lo muestra
sin inventar profundidad. Los NPZ y RGB del barrido se guardan junto al informe.

## Paneles y verificaciones

El plugin `g1_rviz_plugins/PerceptionPanel` muestra IDs, confirmación, estabilidad,
edad, dimensiones, soporte de profundidad y razones. No publica órdenes. Las
pruebas cargan ambos paneles con pluginlib, ejercitan los controles Qt y comprueban
que mensajes vacíos, inválidos, antiguos o vencidos no queden como detecciones
estables. Se compilaron e instalaron ambos paneles para ROS 2 Jazzy.
