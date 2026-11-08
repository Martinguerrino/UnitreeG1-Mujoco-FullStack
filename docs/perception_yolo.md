# Segmentación YOLO + profundidad + múltiples vistas

La percepción usa YOLO11n-seg con cuatro clases: `table`, `box`, `cylinder` y
`sphere`. RGB determina máscaras, categorías y puntuaciones; profundidad y TF
permiten estimar geometría métrica. El entrenamiento usa escenas sintéticas
independientes de las vistas de Maxwell utilizadas para evaluar.

## Ejecutar

El entorno local preparado conserva el checkpoint y el modelo ONNX en
`artifacts/perception/`. El nodo ROS usa ONNX en CPU, sin cargar PyTorch ni
consultar servicios externos:

```bash
bash scripts/navigation.sh perception:=true
```

RViz incorpora **YOLO segmentation**, con la imagen RGB y máscaras coloreadas,
y **RGB-D perception**, con geometría, IDs, dimensiones, ejes, antigüedad,
confianza del segmentador, porcentaje de profundidad válida y número de vistas.
Mesa verde, caja roja, cilindro naranja y esfera azul. Los volúmenes representan
límites geométricos observados; no son modelos CAD completos.

Para preparar el entorno desde cero:

```bash
bash scripts/setup_perception_yolo.sh
```

El script instala versiones fijadas de las dependencias en
`artifacts/perception/python` y prepara el ONNX entrenado incluido en
`assets/perception/`, verificando su SHA-256. Usa el Python configurado por
`setup.sh`; no requiere entrenar ni cargar PyTorch para inferencia.
Para volver a generar 240 imágenes sintéticas y entrenar 35 épocas sobre CPU,
usar `--train`; esa opción instala PyTorch/Torchvision en el entorno elegido
y descarga el checkpoint inicial oficial. Los resultados se guardan en
`artifacts/perception/training/synthetic`.

```bash
bash scripts/setup_perception_yolo.sh --train --reuse-dataset --epochs 35
```

Las imágenes se dividen en entrenamiento y validación, con semilla reproducible.
Se varían tamaños, orientación, color, iluminación, posición de objetos y cámara.
Las máscaras de geoms de MuJoCo se usan **sólo como etiquetas durante la
preparación del modelo**. No se usan durante inferencia para corregir una
máscara, nombrar un objeto o forzar una detección.

El modelo se exporta a ONNX de entrada estática 640 × 640. El manifiesto registra
clases, versiones, semilla y hashes de los pesos y del ONNX. La segmentación
sigue el formato de [Ultralytics YOLO](https://docs.ultralytics.com/tasks/segment/).
Los pesos COCO originales no incluyen una clase `box`; por eso se ajustaron
las clases del escenario y no se renombra otra clase como si fuera una caja.

## Cómo funciona

1. YOLO recibe RGB, con letterbox que conserva proporciones. El decodificador
   reconstruye cada máscara a la resolución original y aplica supresión de
   detecciones superpuestas. Descarta además fragmentos de la misma clase
   contenidos en una máscara de mayor puntuación. Mantiene clase y puntuación
   del modelo.
2. Cada máscara se cruza con profundidad válida registrada. Se retiran bordes
   cuando la máscara es suficientemente grande y se rechazan regiones sin
   suficientes píxeles/puntos válidos.
3. En las máscaras `table`, RANSAC busca superficies horizontales compatibles
   con las alturas del stack. El detector geométrico sigue calculando la normal
   y el contorno de la superficie, pero sólo dentro de máscaras predichas.
   `refinement.geometric_support` agrega planos obtenidos de profundidad cuando
   el tablero recortado no tiene máscara visual. Esas superficies se publican
   como `semantic_class: support_surface`, `source: geometry_support`, confianza
   de segmentación nula y advertencia `semantic_table_unconfirmed`. No se
   presentan como categorías aprendidas. Superficies pequeñas elevadas y
   totalmente contenidas en un tablero mayor se descartan como mesas adicionales.
4. Dentro de máscaras de objetos se descartan puntos aislados, puntos de la
   mesa y puntos fuera de su contorno/altura. Se conserva el componente 3D
   principal y se ajusta un volumen orientado apoyado sobre la superficie.
   No hace falta exigir una tapa rectangular para identificar una caja:
   la categoría proviene del segmentador.
5. El tracker asocia observaciones en `odom`, mantiene separación por categoría
   y filtra las poses. `box_poses` contiene únicamente cajas; `object_poses`
   contiene también cilindros y esferas.
6. La acumulación conserva puntos observados desde poses de cámara distintas.
   Repetir una vista actualiza sus puntos en vez de incrementar artificialmente
   el número de vistas. Con varias vistas, reajusta los límites del volumen.
   Usa hasta 6 vistas, 15 000 puntos por objeto y 4 s de historia. Reinicia la
   geometría de un objeto cuando su centro se desplaza más de 8 cm respecto
   del centro con el que comenzó la acumulación. Los reinicios del reloj y la
   antigüedad también descartan historia.
7. Se transforma el resultado a `pelvis` al tiempo de captura, y se publican
   las detecciones y visualización.

No hay fallback de categorías de objetos si YOLO falla. El soporte de planos
es una opción explícita y nunca crea una caja sin máscara YOLO. Un modelo
faltante produce un error con instrucciones de preparación. Para comparar
explícitamente con el detector anterior:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
/usr/bin/python3 scripts/perception_rgbd.py --ros-args \
  --params-file configs/perception.yaml -p segmentation_backend:=geometry
```

## Salidas e indicadores de calidad

- `/perception/segmentation/image`: RGB `sensor_msgs/Image` con máscaras y
  puntuaciones **validadas por RGB-D**. Sólo dibuja instancias aceptadas en 3D
  y recorta las máscaras/bounding boxes a sus puntos de profundidad compatibles.
  Una propuesta `table` sobre el piso no aparece como una mesa válida, aunque
  YOLO le asigne una puntuación alta. Conserva timestamp y frame óptico.
- `/perception/segmentation/raw_image`: propuestas originales de YOLO, sin
  validación geométrica, para diagnóstico. Se publica sólo si tiene suscriptores.
  Su puntuación no confirma clase, altura ni presencia de una mesa real.
- `/perception/detections`: JSON de mesas y objetos, con `semantic_class`,
  `source: yolo_rgbd`, IDs y poses métricas en el frame de salida.
- `/perception/object_poses`: todas las poses de objetos observados.
- `/perception/box_poses`: solamente cajas. Ambos son `PoseArray` sin IDs;
  para consumir IDs/categorías/dimensiones se debe usar el JSON.

`quality` contiene puntuación de YOLO, cantidad de píxeles, fracción de
profundidad válida, fracción de inliers, residual del plano en metros, vistas,
puntos acumulados, distancia entre vistas y advertencias. La puntuación de
YOLO **no es una probabilidad calibrada de pose correcta**. Los estados
`observed`/`limited` describen soporte de medición, no alcanzabilidad ni éxito
de agarre. `confirmed` sigue significando repetición de una asociación.
`depth_support_scope` distingue soporte dentro de una máscara del porcentaje
global de profundidad válida usado por un plano geométrico sin máscara.

Las advertencias identifican pocas muestras, recorte en el borde de imagen,
profundidad incompleta, simetrías y falta de diversidad de vistas.
`geometry_complete` se mantiene falso: acumular varias vistas no demuestra
que se hayan visto todas las caras. La máscara también puede estar oculta
sin tocar el borde de la imagen; no se garantiza detectar toda oclusión.

Los parámetros `yolo.*`, `refinement.*`, `multiview.*` y `tracking.*` están en
`configs/perception.yaml`. `multiview.enabled:=false` desactiva la acumulación.
`inference_ms` registra el tiempo de segmentación/refinamiento por publicación.
`processing_ms` registra el procesamiento completo antes de publicar;
`inference_backend` y `processing_period_sec` permiten comprobar el backend
activo y su intervalo. Con YOLO CUDA, `gpu.update_period: 0.2` permite hasta
5 actualizaciones por segundo de tiempo simulado; CPU mantiene
`update_period: 0.5` (2 Hz). La frecuencia efectiva depende también del render,
la disponibilidad de RGB-D/TF y el tiempo de procesamiento.

La recepción RGB-D usa un grupo de callbacks separado del procesamiento y
un executor de dos hilos. Las consultas de vecinos grandes conservan el
árbol espacial CPU, evitando calibraciones CUDA con coste cuadrático; las
calibraciones rechazadas se reutilizan para tamaños similares.
La edad de captura sigue publicada; un consumidor debe comprobarla antes de
usar una pose, especialmente si CPU, render o SLAM están sobrecargados.

## Evaluar

```bash
"$G1_PYTHON" scripts/evaluate_perception.py \
  --backend yolo --output results/perception/maxwell_yolo_evaluation

"$G1_PYTHON" scripts/evaluate_perception.py \
  --backend geometry --output results/perception/maxwell_geometry_comparison
```

La evaluación mantiene distancias, ángulos y ruido de la batería anterior,
registra fallos y falsos positivos, guarda overlays y compara contra geoms
sólo después de inferir. También reproduce las tres vistas cercanas con un
tracker persistente, para medir acumulación de geometría del tablero. Cada
batería guarda `evaluation.json`, RGB-D y las transformaciones de cámara.

El modelo es un primer ajuste sobre simulación: no acredita reconocimiento
universal ni transferencia a una cámara física. Los resultados de Maxwell,
que no alimentó las etiquetas del entrenamiento, deben usarse para valorar
si mejora el detector anterior antes de conectar manipulación.

Resultados de la ejecución local y comparación con el detector anterior:
[validación YOLO](perception_yolo_validation_20261007.md).
