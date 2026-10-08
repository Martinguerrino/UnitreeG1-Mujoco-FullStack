# Validación YOLO y RGB-D — 2026-10-07

Se preparó YOLO11n-seg desde pesos oficiales, ajustado a `table`, `box`,
`cylinder`, `sphere`: 240 renders sintéticos aleatorios, 192 para entrenamiento,
48 para validación, semilla 73 y 35 épocas en CPU. Las escenas de preparación
son independientes del escenario Maxwell usado para evaluar. El checkpoint y
su exportación ONNX se conservan en `artifacts/perception/`, con versiones y
hashes en `model_manifest.json`.

## Batería Maxwell

Se mantuvieron los 18 casos anteriores: 0.6/1.0/1.5 m, ángulos -20/0/20 grados,
imágenes ideales y perfil de ruido de cámara, cinco capturas por caso. Son 90
capturas estáticas con la D435i montada en el robot; no prueban locomoción,
latencia ni errores de calibración/TF. Geometría real y segmentación del
simulador se usan exclusivamente para puntuar, nunca para inferir.

| Métrica de cajas | Geometría anterior | YOLO + RGB-D |
|---|---:|---:|
| Asociaciones a referencias visibles | 36 / 90 | 90 / 90 |
| Candidatos extra | 23 | 30 |
| Precisión de candidatos 3D | 61.0 % | 75.0 % |
| RMSE de centro en asociaciones aceptadas | 7.32 mm | 9.70 mm |

La visibilidad exige al menos 20 píxeles de la referencia; no demuestra que
se vea la tapa ni todas sus caras. Se consideran asociaciones hasta 20 cm del
centro real. Los errores no incluyen objetos omitidos: no deben interpretarse
como precisión general del sistema. YOLO recupera más cajas, incluida la
amarilla, pero el error de las asociaciones aceptadas no mejora automáticamente.

Se siguieron detectando confusiones visuales y candidatos extra, especialmente
a 1 m. Las máscaras duplicadas de una misma clase se suprimen mediante
contención antes de reconstruirlas en 3D. La segmentación de algunas mesas
recortadas falla; la opción de soporte geométrico mantiene esas superficies
con clase visual no confirmada y confianza YOLO nula.

Las superficies de mesa se asociaron en 60 de 90 capturas, con 24 candidatos
extra. La percepción individual todavía no acredita límites completos del
tablero. Los planos pequeños elevados dentro de una superficie mayor se
rechazan como mesas adicionales, evitando parte de las confusiones con la
bandeja de la escena.

## Acumulación de vistas

Se reprodujeron las tres vistas cercanas con un tracker persistente. Se
conservaron IDs de los objetos entre vistas y se acumularon tres posiciones de
cámara, con separación máxima de 0.374 m. Comparación de RMSE del centro del
tablero dentro de esa secuencia:

| Perfil | Pose filtrada de vista individual | Geometría acumulada |
|---|---:|---:|
| Ideal | 32.1 mm | 11.3 mm |
| Ruido | 27.0 mm | 7.0 mm |

Son tres puntos de observación de una escena estática, no una garantía de
mejora general. El filtro y la acumulación pueden retrasar cambios reales;
la geometría se descarta por antigüedad, reinicio del reloj o desplazamiento
acumulado del objeto. `geometry_complete` continúa falso.

## Verificación de software

- 30 pruebas de percepción/core/ROS satisfactorias; la comparación nativa de
  PyTorch se omite en Python de ROS porque ese intérprete no tiene PyTorch.
- La comparación nativa se ejecutó por separado en Conda: clases, puntuaciones
  y máscaras ONNX coinciden con YOLO antes de la supresión adicional de fragmentos.
- Prueba ROS con RGB-D de Maxwell, modelo ONNX real, TF, máscaras, categorías,
  calidad y confirmación de IDs; sin usar objetos reales como entrada del nodo.
- Cobertura de profundidad inválida, máscaras mal registradas, apoyo geométrico,
  duplicados, superficies elevadas anidadas, límites de memoria, diversidad de
  vistas, expiración y movimiento acumulado de objetos.
- Ruff y `git diff --check` satisfactorios.

Resultados por caso, máscaras y secuencias:
`results/perception/maxwell_yolo_evaluation/evaluation.json` y sus NPZ/PNG.
La batería geométrica histórica se conserva en
`results/perception/maxwell_evaluation/evaluation.json`.

```bash
"$G1_PYTHON" scripts/evaluate_perception.py \
  --backend yolo --output results/perception/maxwell_yolo_evaluation
```
