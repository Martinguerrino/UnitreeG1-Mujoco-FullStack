# Modelo de percepción

YOLO11n-seg ajustado con 240 escenas sintéticas MuJoCo, 35 épocas y semilla 73.
Clases: mesa, caja, cilindro y esfera. No validado en hardware real.
El manifiesto fija el SHA-256 y las versiones de entrenamiento.

`setup.sh` copia el ONNX a `artifacts/perception/` después de verificarlo.
No necesita entrenarlo ni descargar pesos de un servidor externo.
Para entrenar nuevamente: `bash scripts/setup_perception_yolo.sh --train`.

Derivado de Ultralytics YOLO11n-seg, publicado por
[Ultralytics](https://github.com/ultralytics/ultralytics).
Este modelo se distribuye bajo AGPL-3.0; ver [LICENSE](LICENSE).
El procedimiento de generación y entrenamiento está en
`scripts/train_perception_yolo.py`. Esta licencia es independiente de la
licencia Apache-2.0 del código propio del proyecto.
