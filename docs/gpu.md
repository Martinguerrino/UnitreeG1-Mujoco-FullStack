# Backend CUDA opcional

El backend CPU sigue disponible. La física permanece en MuJoCo CPU; los módulos
nuevos de `g1_backend/cuda/` aceleran procesamiento y sensores. No se descarga
ninguna dependencia al iniciar una simulación.

## Instalar en el entorno del proyecto

Después del setup normal, en Ubuntu 24.04/Python 3.12 con NVIDIA compatible:

```bash
bash scripts/setup_gpu.sh
"$G1_PYTHON" scripts/check_gpu.py
bash scripts/navigation.sh perception:=true
```

El instalador usa el Python configurado para el proyecto (`.g1-runtime.json` o
`G1_PYTHON`). Si es tu entorno Conda, instala ahí las variantes CUDA de PyTorch y
torchvision, ONNX Runtime GPU, CuPy y Open3D. No crea otro virtualenv ni cambia
`.g1-runtime.json`. Las versiones CUDA también ejecutan operaciones CPU cuando
`G1_GPU=off` o no hay GPU compatible. No instala drivers del sistema.

El lanzador de navegación propaga automáticamente el site-packages del entorno
del proyecto a sus workers Python de percepción y SLAM, si las versiones de
Python son compatibles. No requiere `source scripts/gpu_env.sh` manual. Es necesario haber instalado las
dependencias funcionales normales (DDS, ROS, percepción y, para ACT, LeRobot y
su checkpoint).

También se puede usar otro entorno que ya tenga dependencias GPU compatibles.
Para launchers ROS que usan un Python diferente, exportar
`G1_GPU_PYTHON_PATH` con el directorio site-packages de ese entorno.

## Selección y recuperación

`G1_GPU=auto` es el valor predeterminado. Cada componente comprueba su propia
capacidad: tener una NVIDIA no demuestra que una biblioteca tenga soporte CUDA.
No se exige que todas las bibliotecas GPU estén presentes para usar una de ellas.

```bash
G1_GPU=off bash scripts/navigation.sh perception:=true
G1_GPU=off "$G1_PYTHON" -m experiments.okura.infer
```

Sin dependencias GPU o sin dispositivo compatible se ejecuta el camino CPU.
Las asignaciones y ejecuciones CUDA tienen respaldo CPU; los fallos se registran
una vez por componente. Una falla CPU sigue siendo un error, no se oculta.

- ONNX: recrea la sesión CPU y repite la inferencia.
- ACT: mueve pesos y preprocesador a CPU, resetea la política y repite el chunk.
- Entrenamiento: ante error CUDA reinicia desde los pesos fuente en CPU; no
  continúa desde pesos parcialmente actualizados.
- TSDF: descarta el volumen incompleto y reintegra todos los frames retenidos en
  CPU con las poses optimizadas actuales.
- Sensores: restaura el estado RNG antes de repetir en CPU, evitando consumir dos
  veces las muestras de ruido.
- Video: prueba NVENC con un frame; si falla la exportación, la reinicia con x264.

La GPU no se selecciona para operaciones pequeñas donde los costes de copia
pueden dominar. LiDAR y redes pequeñas de locomoción comparan resultados y
latencia antes de seleccionar el camino CUDA. Las transformaciones aisladas
siguen en CPU; se fusionan en GPU cuando la nube ya está en un workspace/mapa.

## Componentes

| Área | Implementación opcional | Interfaz CPU conservada |
|---|---|---|
| YOLO | Buffers ONNX residentes, normalización, prototipos y resize/umbral de máscaras CUDA | `YoloSegmenter.predict()` devuelve las mismas instancias/máscaras NumPy |
| ACT | Modelo y preprocesamiento PyTorch CUDA | Chunks NumPy de 100 × 16 |
| Entrenamiento YOLO | PyTorch CUDA, conservando argumentos y FP32 originales | Mismos dataset, pesos fuente y exportación ONNX |
| Geometría RGB-D | Workspace por frame, proyección y transformación fusionadas, RANSAC por lotes | Mismos frames, unidades, hipótesis y estructuras de detección |
| Componentes y vecinos | Hashing espacial GPU, unión de componentes y conteos por radio | Índices de puntos originales; SciPy como respaldo |
| ICP | Referencia residente y consultas CUDA por lotes con comparación de velocidad | Resolver robusto, covarianzas y criterios de aceptación actuales |
| Mapas LiDAR | Transformación de scans, concatenación y voxelización en GPU | Nube optimizada NumPy y límites actuales |
| Raycasting LiDAR | Kernels para plano, esfera, caja, cápsula, cilindro, elipsoide y malla | Mismas distancias e IDs de MuJoCo, incluyendo oclusión propia |
| TSDF/malla RGB-D | Open3D VoxelBlockGrid CUDA, extracción y colores de voxels observados | Malla legacy Open3D, corrección de poses y evicción |
| Imágenes | Ruido de sensores, overlays, preview y EDT opcionales | RGB uint8, profundidad métrica y RNG original |
| Odometría visual | ORB CUDA si OpenCV fue compilado con soporte | PnP y criterios métricos originales |
| Cámaras offline | EGL acelerado con alternativa software | MuJoCo Renderer y física originales |
| Video | NVENC cuando PyAV/FFmpeg y el driver lo soportan | Timestamps de simulación y muestreo originales |

Resize de entrada y NMS de YOLO mantienen OpenCV. Las salidas que consumen ROS,
SciPy, tracking y planificación siguen en memoria CPU. Los workspaces mantienen
RGB, depth y grillas en GPU durante el refinamiento de un frame; el mapa LiDAR y
TSDF también agrupan varias operaciones antes de devolver resultados.

## Compatibilidad y límites

- El raycaster usa poses de MuJoCo y conserva grupos y transparencia. No altera
  contactos ni estados. Heightfields y SDF usan MuJoCo CPU. Antes de seleccionarlo
  compara todos los rayos e IDs y luego controla muestras en cada scan. Los
  modelos y mallas compilados se consideran inmutables; para cambiar topología,
  recompilar el modelo, como en los launchers actuales.
- CUDA TSDF reserva hasta 2.048 bloques de 16³ voxels; al superar el presupuesto
  reconstruye en CPU. El muestreo y la extracción de la API tensorial pueden
  producir diferente triangulación. Los colores se toman del voxel observado
  más próximo en un radio local, evitando colores negros de voxels no observados
  en bordes. No se promete igualdad de vértices ni textura interpolada idéntica.
- La triangulación GP3/PCL existente sigue disponible en CPU. TSDF es el camino
  de reconstrucción de superficies GPU añadido; no se reemplaza GP3 por un
  algoritmo diferente sin que el consumidor lo solicite.
- Los wheels normales de OpenCV no incluyen CUDA: ORB conserva CPU en ese caso.
  EDT de CuPy puede requerir cuCIM; si no está instalado, conserva SciPy.
- Kalman, control, tracking de pocos objetos, DP de rectángulos libres, ROS/DDS,
  Nav2, MoveIt, FAST-LIO y el grafo RTAB-Map conservan sus implementaciones CPU.
  Instalar CUDA no convierte esos motores externos en implementaciones GPU.
- CUDA tiene warmup y compilación de kernels. No interpretar el primer frame
  como latencia estable; no se activa FP16 automáticamente.
- La migración de física está detallada por separado en
  [MuJoCo Warp futuro](mujoco_warp_future.md).

## Pruebas y mediciones

```bash
# Suite completa CPU, con bibliotecas SLAM configuradas y EGL headless.
source scripts/slam_env.sh
G1_GPU=off MUJOCO_GL=egl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 "$G1_PYTHON" -m pytest -q tests

# Pruebas reales GPU (se omiten las capacidades opcionales ausentes).
MUJOCO_GL=egl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 "$G1_PYTHON" -m pytest -q \
  tests/test_gpu_backend.py tests/test_gpu_tsdf.py tests/test_okura_inference.py

# Las mediciones incluyen transferencias de entrada/salida.
"$G1_PYTHON" scripts/benchmark_gpu.py --output results/gpu/benchmark.json
```

Resultados de validación local y sus condiciones: [gpu_validation.json](gpu_validation.json).
Son mediciones de componentes, no una promesa de aumento de FPS de todo el robot.
Una reconstrucción GPU puede cambiar el número de triángulos manteniendo la
superficie métrica correcta. Comparar también detecciones, calidad del mapa,
frames descartados y latencias p95/p99 en la escena real.
