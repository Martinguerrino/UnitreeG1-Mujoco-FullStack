# Okura ACT: qué funciona y qué todavía no

## Resultado de esta etapa

Se descargó y ejecutó **el checkpoint original**, no una política inventada ni
una animación. Produce bloques de 100 acciones de 16 valores. La primera prueba
con una imagen RGB renderizada de MuJoCo tardó aproximadamente 0,25 s en CPU.
Esto mide una inferencia, no el rendimiento de un controlador completo.

Validación local del 12 de septiembre de 2026: **18 tests pasaron en 7,02 s**,
incluidos los tests opcionales que cargan los pesos originales. Ruff y la
comprobación de sintaxis de los scripts modificados también pasaron.

**Todavía no hay una prueba de agarre ni de traslado con Dex1.** La escena de
entrada conserva las manos rígidas originales. No se enviaron las acciones a
los motores. El reporte lo indica explícitamente para no confundir «la red
funciona» con «el robot agarra».

## Cómo repetirlo

Desde la raíz del repositorio:

```bash
conda activate mujoco-unitree
# Solo para instalar en otra máquina o descargar archivos que falten:
bash scripts/setup_okura.sh

# Verificar pesos y estructura:
python -m experiments.okura.checkpoint

# Inferencia con la imagen de la escena MuJoCo, sin ROS ni ventana:
MUJOCO_GL=egl python -m experiments.okura.infer

# Tests, sin cargar los plugins opcionales de ROS:
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests
```

EGL permite renderizar sin ventana, pero necesita un backend OpenGL funcional.
La cámara se inspeccionó visualmente: se ven la mesa, la caja roja, los brazos
y la marca verde de destino. Esa imagen **no muestra un agarre**.

Los resultados quedan en `artifacts/okura/inference/`:

- `head_camera.png`: imagen realmente entregada al modelo.
- `observation.npz`: imagen y estado utilizados; permite repetir la inferencia
  con `--observation artifacts/okura/inference/observation.npz`.
- `actions.npy`: las 100 acciones desnormalizadas. **No son una trayectoria
  aprobada para ejecución**: no se han validado colisiones ni límites dinámicos.
- `report.json`: tiempo, procedencia, dimensiones y qué se probó realmente.

`artifacts/` no se versiona: contiene pesos grandes y salidas regenerables.

## Qué hace cada archivo

- `experiments/okura/checkpoint.py`: descarga una revisión fija, comprueba el
  SHA-256 de los pesos, dimensiones y que los safetensors no estén truncados.
- `experiments/okura/infer.py`: usa ACT y los procesadores originales de LeRobot.
  Conserva la normalización de entrenamiento; cambia el dispositivo a CPU y
  evita una descarga redundante de pesos ImageNet. No tiene conexión ROS/DDS.
- `experiments/okura/scene.py`: genera una escena temporal en memoria con G1,
  mesa, caja libre de 6 cm y destino. La cámara está a altura de cabeza, con
  una pose aproximada que **no está calibrada con la cámara del dataset**.
- `tests/test_okura_contract.py`: contrato de articulaciones, detección de pesos
  truncados y caja libre que cae por gravedad, sin soldadura a la mano.
- `tests/test_okura_inference.py`: carga estricta de pesos originales, inferencia
  repetible y rechazo de entradas incompatibles. Se omite si no están instaladas
  las dependencias o no se descargaron los pesos; nunca descarga durante tests.

El estado inicial de brazos es la media del checkpoint, asignada solo al
inicializar la escena. Los valores de pinza son también la media del checkpoint,
**no mediciones de una Dex1 simulada**, porque todavía no está instalada. Es una
prueba de entrada/salida; no un episodio de control.

## Compatibilidad comprobada y advertencias

Checkpoint: [sotata/act-okura-pick-06102026](https://huggingface.co/sotata/act-okura-pick-06102026),
revisión `220fc42eeff7585136ef29cf2bd5a82de8239092`.

Su configuración, pesos y normalizadores coinciden en 16 valores:
7 articulaciones del brazo izquierdo, 7 del derecho y una salida por pinza.
El orden semántico procede de la ficha del autor. Las salidas de pinza no son
metros de apertura: las estadísticas de la derecha llegan aproximadamente a
5,4. Hace falta verificar su conversión mecánica antes de ejecutar.

El [dataset público asociado](https://huggingface.co/datasets/sotata/okura-pick-06102026/blob/main/meta/info.json)
declara actualmente 8 valores del brazo derecho, aunque el checkpoint conserva
16. No se rellenaron arbitrariamente esos datos para entrenar o reproducirlos.
La discrepancia debe resolverse antes de usar ese dataset como referencia.

La instalación opcional usa Python 3.12 del entorno `mujoco-unitree`, PyTorch
2.7.1 CPU, torchvision 0.22.1 y LeRobot 0.4.1. El resolvedor instaló NumPy 2.2.6
para compatibilidad con OpenCV; no cambió el Python del entorno ni MuJoCo 3.3.6.
`pip check` detecta además `typeguard` ausente para un paquete ROS del sistema;
es una dependencia ajena a esta prueba y no impidió la inferencia. No se ha
revalidado aquí el lanzamiento completo de ROS/MoveIt tras instalar LeRobot.

## Qué falta para mover la caja

1. Elegir el modelo de mano. Unitree publica Dex1 en sus
   [assets de Isaac Lab](https://github.com/unitreerobotics/unitree_sim_isaaclab),
   distribuidos en un ZIP de aproximadamente 1,3 GB. No se localizó un MJCF Dex1
   listo para usar en las fuentes consultadas. Convertirlo y validar geometría,
   masas, articulaciones, montaje y unidades es distinto de usar una pinza
   paramétrica aproximada; esta última requiere aceptar esa aproximación.
2. Calibrar cámara, estado inicial, mesa y conversión de la pinza. Un modelo que
   aprendió con imágenes reales puede fallar con nuestra escena simulada.
3. Ejecutar acciones mediante control físico y límites de posición, velocidad
   y esfuerzo. La caja debe permanecer libre; nunca actualizar su pose para
   seguir a la mano ni activar una soldadura de agarre.
4. Medir contacto de ambos dedos, elevación sostenida y caída al abrir. Después,
   medir traslado hasta el destino, liberación y reposo dentro de la zona.
5. Evaluar varios episodios. El checkpoint es de **pick** y no recibe coordenadas
   de destino: no hay evidencia de que sepa colocar una caja en nuestra marca
   verde. Puede necesitar demostraciones nuevas de pick-and-place o un
   controlador separado de colocación, identificado como tal.

No se debe sustituir un fallo de ACT por una secuencia programada y reportarla
como éxito de la política. Primero se publicará el resultado real, sea éxito
o fallo.
