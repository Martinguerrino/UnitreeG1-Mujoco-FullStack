# Percepción RGB-D de mesa y objetos

El modo predeterminado ahora usa **YOLO11n-seg**, refinamiento RGB-D y
acumulación de vistas. Preparación, clases, máscaras, indicadores de calidad
y evaluación están en [segmentación YOLO](perception_yolo.md).
Los detalles geométricos y la validación histórica de abajo describen el
backend de referencia `segmentation_backend:=geometry`.

Durante inferencia, el nodo `scripts/perception_rgbd.py` usa exclusivamente imágenes RGB-D,
calibración y TF. No recibe posiciones, nombres ni geometrías de objetos del
simulador. Publica percepción; no mueve brazos ni pinzas y no modifica la
Planning Scene de MoveIt.

## Ejecutar

Desde la raíz del repositorio:

```bash
bash scripts/navigation.sh perception:=true
```

Es compatible con `--slam lidar3d`, `--slam rgb-d`, `--real` y localización.
No requiere SLAM 3D para detectar objetos; sí requiere cámara y TF. `--ideal`
desactiva la cámara y se rechaza si se solicita percepción.
RViz agrega automáticamente **RGB-D perception**: superficie de mesa verde y
cajas candidatas rojas. Cada detección muestra ID, dimensiones, antigüedad y
estado provisional/confirmado, junto con ejes X rojo, Y verde y Z azul.
La mesa debe estar dentro del campo de visión, a menos
de 3.5 m; la cámara debe ver su superficie y la tapa de la caja. El arranque
habitual en el centro de Maxwell no garantiza ver la mesa de trabajo.

Para agregar percepción a una simulación de navegación que ya está ejecutándose,
usar otra terminal, sin activar una segunda instancia del nodo:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
/usr/bin/python3 scripts/perception_rgbd.py --ros-args --params-file configs/perception.yaml
```

Dependencias ROS/Python utilizadas: `rclpy`, `message_filters`, `tf2_ros`,
`sensor_msgs`, `geometry_msgs`, `std_msgs`, `visualization_msgs`, NumPy y SciPy.
Si falta alguna dependencia:

```bash
sudo apt install python3-numpy python3-scipy ros-jazzy-message-filters \
  ros-jazzy-tf2-ros ros-jazzy-visualization-msgs
```

## Procesamiento

Procesamiento del backend geométrico de referencia:

1. Sincroniza RGB y profundidad con tolerancia de 20 ms. Usa únicamente el par
   más reciente, con antigüedad máxima de 0.5 s. Procesa cada 0.5 s.
2. Reconstruye puntos ópticos con `X=(u-cx)*Z/fx`, `Y=(v-cy)*Z/fy` y `Z=depth`.
   Acepta profundidad `32FC1` en metros o `16UC1` en milímetros, RGB `rgb8`,
   padding de filas y ambos órdenes de bytes. Descarta profundidad inválida.
3. Usa TF **al instante de captura de profundidad** para pasar al frame
   `base_link`. En este stack ese frame tiene Z hacia arriba y origen a nivel
   del suelo; así la altura no depende de la inclinación de pelvis/cámara.
4. Reduce puntos repetidos con voxeles de 5 mm. RANSAC encuentra planos
   aproximadamente horizontales entre 0.45 y 1.15 m. Separa superficies
   coplanares desconectadas y exige lados de al menos 25 cm. Ajusta normal y
   rectángulo de área mínima a cada superficie observada.
5. Agrupa puntos de 1.8 a 40 cm sobre cada superficie y dentro de su contorno.
   Estima un rectángulo orientado, altura sobre la mesa y centro del volumen.
   Exige una tapa aproximadamente plana y rectangular para aceptar una
   `box_candidate`; un contorno redondo se descarta. RGB aporta color medio,
   no determina la clase.
6. Transforma las observaciones a `odom`, un frame fijo, para asociarlas entre
   capturas. Un algoritmo de asignación uno a uno combina distancia, dimensiones
   y color; la mesa admite cambios de contorno visible, comprobando además que
   siga en el mismo plano. Aplica un filtro exponencial a centro/dimensiones y
   una interpolación de rotaciones que respeta las simetrías del rectángulo.
7. Transforma la pose filtrada a `pelvis`, usando TF al mismo instante. Es el
   frame de referencia de brazos admitido actualmente por Pick; se puede
   seleccionar otro con `target_frame`. Las dimensiones permanecen expresadas
   en los ejes propios del objeto.

La implementación numérica está en `g1_backend/perception.py` y los parámetros
en `configs/perception.yaml`. `processing_frame` debe tener origen a nivel del
suelo y Z hacia arriba. Cambiarlo por un frame inclinado invalida los filtros de
altura y horizontalidad. Las tolerancias y el espaciado de agrupación deben
ajustarse a distancia, resolución y ruido del sensor.

## Salidas

| Tópico | Tipo | Contenido |
|---|---|---|
| `/perception/detections` | `std_msgs/String` | JSON con estado, frame, tiempo de captura, mesas y cajas, dimensiones y calidad geométrica |
| `/perception/table_pose` | `geometry_msgs/PoseStamped` | Centro de la primera superficie detectada y orientación |
| `/perception/box_poses` | `geometry_msgs/PoseArray` | Centros y orientaciones de todas las cajas candidatas |
| `/perception/object_poses` | `geometry_msgs/PoseArray` | Todas las categorías de objetos detectadas por YOLO |
| `/perception/segmentation/image` | `sensor_msgs/Image` | RGB con máscaras y categorías de YOLO |
| `/perception/markers` | `visualization_msgs/MarkerArray` | Volúmenes orientados para RViz |

Para inspeccionar las estimaciones:

```bash
ros2 topic echo /perception/detections --once
```

`center` contiene XYZ en metros, `orientation_xyzw` es un quaternion y
`dimensions` son largo, ancho y alto en los ejes de la pose. El eje X representa
el lado largo del rectángulo. Para mesa se publica **centro de superficie** y
alto 0, porque no se observa el espesor; RViz dibuja 5 mm sólo para visualizar.
Para caja, el centro Z se infiere como la mitad de la altura sobre la mesa.

`id` y `track_id` identifican una detección durante esta sesión. `hits` cuenta
observaciones asociadas y `confirmed` se activa al observarla dos veces.
`raw_pose` conserva centro, orientación y dimensiones sin filtrar, expresados
también en el frame de salida. `last_seen` y `first_seen` usan segundos del reloj
ROS; `age_sec` mide tiempo desde la captura al publicar. La etiqueta de RViz
actualiza esa edad con cada publicación; no es un cronómetro independiente.

El tracker conserva internamente una detección perdida hasta 2 s para recuperar
su ID, pero **no publica objetos ausentes como si siguieran observados**. Tras
el timeout recibe un ID nuevo. Un reinicio del reloj borra las asociaciones sin
reciclar IDs. Reiniciar el nodo comienza una sesión nueva.

Los parámetros `tracking.alpha` (0.35), `tracking.match_distance` (0.18 m),
`tracking.dimension_gate` (0.08 m), `tracking.timeout` y
`tracking.confirmation_hits` ajustan el seguimiento. `tracking_frame` debe ser
un frame fijo; `odom` evita que la marcha del robot se convierta en movimiento
aparente de la caja. Si falta ese TF, la salida queda inválida. El filtrado
reduce variación temporal pero introduce retraso ante movimientos reales y no
corrige sesgos ni completa superficies ocultas.

El JSON es la fuente de validez: `ok`, `no_table`, `waiting_for_rgbd`,
`waiting_for_calibration` o `unavailable: ...`. Una salida inválida contiene
`tables: []`, vacía `box_poses` y borra los markers. Los markers también vencen
solos. `table_pose` sólo publica detecciones válidas; un consumidor debe comprobar
su timestamp y el estado del JSON antes de reutilizar la última pose.

## Límites

Estos límites de clasificación describen el backend geométrico. Para el
modelo aprendido y la acumulación de vistas consultar [YOLO](perception_yolo.md).

- Detecta **superficies compatibles con mesas y candidatos geométricos a cajas**,
  no reconocimiento semántico general. Un estante o un objeto rectangular puede
  satisfacer los mismos filtros. No se usa un modelo entrenado.
- Una imagen no revela caras ocultas. `geometry_complete: false` indica que
  centro y dimensiones se basan en lo observado y en el supuesto de que el
  objeto descansa sobre la mesa. No son dimensiones completas garantizadas.
- La orientación del rectángulo es equivalente tras un giro de 180 grados.
  Si largo y ancho son similares, `yaw_ambiguous: true` advierte que tampoco
  se puede distinguir de forma fiable un giro de 90 grados.
- Necesita ver la tapa de la caja. Una caja vista sólo de lado, muy pequeña,
  tapada o con pocos puntos puede no detectarse. Objetos en contacto pueden
  agruparse juntos. La asociación geométrica puede intercambiar IDs si objetos
  parecidos se cruzan; los IDs no persisten tras reiniciar el nodo.
- La pose de la caja **no es una pose de agarre**: falta calcular el
  desplazamiento y la orientación apropiados para la herramienta.

## Verificación

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/martin/miniforge3/envs/mujoco-unitree/bin/python \
  -m pytest -q tests/test_perception.py tests/test_perception_tracking.py tests/test_perception_evaluation.py

source /opt/ros/jazzy/setup.bash
ROS_LOG_DIR=/tmp/g1-perception-ros-log ROS_DOMAIN_ID=93 \
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /usr/bin/python3 -m pytest -q tests/test_perception.py tests/test_perception_tracking.py \
    tests/test_perception_evaluation.py tests/test_perception_ros.py
```

La prueba ROS publica imágenes registradas y TF en un dominio separado,
verifica la pose métrica en `pelvis` y comprueba que las detecciones se limpian
cuando deja de llegar la cámara. Las pruebas geométricas cubren ruido,
orientación de caja, transformación con roll/pitch/yaw, mesas separadas,
rechazo de suelo/paredes y objetos redondos. Es validación sintética reproducible;
no acredita precisión general sobre todos los objetos de Maxwell.

También se validó una escena controlada con RGB-D **renderizados por MuJoCo**
(cámara oblicua, piso, mesa y caja girada): error del centro de 0.34 mm, error
máximo de dimensión de 0.42 mm. Es una escena ideal, sin ruido ni oclusiones;
esos errores no representan precisión esperable en hardware o escenas generales.
Para repetir esa prueba sin ROS, usando Mesa/EGL por software:

```bash
/home/martin/miniforge3/envs/mujoco-unitree/bin/python scripts/check_perception_render.py
```

Guarda imágenes y calibración en `results/perception/render_rgbd.npz` y las
estimaciones/métricas en `results/perception/render_validation.json`. La pose
real del objeto sólo se usa en ese comprobador para evaluar el error.

## Evaluación de precisión en Maxwell

```bash
/home/martin/miniforge3/envs/mujoco-unitree/bin/python scripts/evaluate_perception.py --backend geometry
```

Carga el escenario real de navegación y el G1 con su cámara D435i montada,
usando la postura nominal de la política. Evalúa 0.6, 1.0 y 1.5 m desde el cubo
rojo, con ángulos de -20, 0 y 20 grados, imágenes ideales y el perfil de ruido
de cámara de `sensors_real.yaml`: 18 configuraciones de cinco capturas cada una.
Son poses estáticas del robot; no ejecuta locomoción ni prueba latencias.

En `results/perception/maxwell_evaluation/evaluation.json` registra:

- Error de centro, dimensiones y orientación, con resultados crudos y filtrados.
- Referencias no detectadas y falsos positivos, sin descartarlos del resumen.
- Cuántos píxeles de cada referencia son visibles, obtenidos por segmentación
  exclusiva del evaluador. Un objeto con al menos 20 píxeles visibles cuenta
  en la métrica adicional `visible_boxes`; eso no garantiza que se vea su tapa.
- Variación de posición estacionaria, IDs observados y cambios de ID.
- Parámetros, semilla, resolución, perfil de ruido y hash del escenario.

La referencia proviene de posiciones y tamaños de geoms de MuJoCo, **sólo para
puntuar resultados después de detectar**. El detector y el tracker nunca
reciben esa referencia ni la imagen de segmentación. Para la mesa se compara
el contorno visible estimado contra el tablero completo; el error puede ser
grande por recorte del campo de visión. Las orientaciones se comparan módulo
180 grados para rectángulos y módulo 90 para cuadrados.

Cada configuración guarda el último RGB-D y la transformación de cámara en
un NPZ. Se pueden cambiar `--distances`, `--bearings`, `--frames`, `--seed`,
`--width`, `--height` y `--output`. Resultados y limitaciones de la ejecución
local: [validación de percepción](perception_validation_20261007.md).
