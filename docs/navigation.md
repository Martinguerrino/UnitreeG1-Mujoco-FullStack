# Nav2 y SLAM en el escenario Maxwell

Nav2 y SLAM están instalados en el entorno local. El 2026-10-01 se validó
una meta corta `(0.7, 0)` en modo normal y en `--ideal`, con MuJoCo, puente ROS,
robot_state_publisher, SLAM y Nav2 sin ventanas. Esto no valida todavía una
batería completa de recorridos ni la estabilidad de locomoción prolongada.

Validación local del 2026-09-19: 25 tests existentes y 3 tests nuevos pasan.
`scripts/check_navigation_bridge.py` verificó MuJoCo + DDS + ROS con un comando
real de `/cmd_vel`: desplazamiento de 0.2796 m, cero caídas/watchdogs y recepción
de 69 escaneos y 328 mensajes de odometría/reloj. Reporte físico:
`results/navigation/20260919T203402_820d64a7/metrics.json`.
Esa prueba histórica no incluía planificación Nav2 ni mapeo SLAM.
La instalación y la prueba integrada posteriores se describen arriba.

El flujo es RViz → NavigateToPose → planificador Nav2 → controlador Nav2 →
cmd_vel → política RL Lab original → LowCmd DDS → MuJoCo. Nav2 calcula los caminos;
la política produce las consignas articulares. No se usan los waypoints del
test mesa-a-mesa para conducir esta modalidad.

La simulación de navegación equipa al G1 con una D435i RGB-D y un MID-360 3D
aproximado, ambos montados en `torso_link` usando las posiciones del URDF
oficial. El control SDK2/DDS no se modifica: percepción se transporta por un
canal local independiente y el puente ROS publica:

- `/mid360/points` (`sensor_msgs/PointCloud2`, frame `mid360_link`);
- `/camera/color/image_raw` (`rgb8`);
- `/camera/depth/image_rect_raw` (`32FC1`, profundidad en metros);
- `/camera/color/camera_info`;
- `/scan`, proyección horizontal del MID-360 usada por SLAM Toolbox y Nav2.

En RViz se pueden agregar displays `PointCloud2`, `Image` y `LaserScan` con
esos tópicos. La configuración reproducible de resolución, frecuencia, montaje,
capas verticales y alcance está en `configs/experiments/navigation.yaml`.
El arranque normal abre los paneles `D435i RGB` y `D435i Depth` a la derecha
de la vista 3D. La distribución está en `configs/navigation_rviz_layout.yaml`;
`Navigation 2` inicia oculto para dejar espacio a las imágenes y se puede
abrir desde el menú Panels. La herramienta Nav2 Goal sigue en la barra superior.
Para cargarla en un RViz ya abierto, usar
File → Open Config y seleccionar `results/navigation/g1_navigation.rviz`.
La opción adicional `D435i Projection 3D` en Displays empieza desactivada.
Al marcarla, dibuja el campo de visión desde el frame óptico de la D435i y
la imagen RGB sobre un plano a 0.8 m, con la calibración y el tiempo de captura.
Es una representación visual de la imagen, no una reconstrucción de obstáculos:
no usa profundidad ni alimenta SLAM/Nav2. Los paneles RGB y Depth siguen disponibles.
El proceso `navigation_camera_plane.py` publica `/camera/color/image_plane`
solo cuando hay suscriptores, usando una muestra cada ocho píxeles por eje.
La imagen se dibuja con triángulos coplanares, perpendiculares al eje óptico;
mantiene la orientación del torso y no gira para mirar al observador de RViz.
El visualizador CameraInfo usa `/camera/color/camera_info` para las aristas.

SLAM Toolbox construye `/map` y publica `map → odom`. El puente publica
`odom → base_link → pelvis`, `/joint_states`, `/odom`, `/clock` y `/scan`.
`robot_state_publisher` resuelve pelvis → torso → sensores con el URDF.
`base_link → laser` es un frame horizontal virtual a 0.78 m sobre el suelo;
la nube conserva su frame físico `mid360_link`. Todos los nodos Nav2
y SLAM usan tiempo simulado. ROS usa Python del sistema y FastDDS en dominio
43; SDK2 permanece en el proceso Conda y dominio 44. Los dos procesos se
comunican por UDP exclusivamente en 127.0.0.1 (19420/19421 y 19422 para sensores). Ejecutar una sola
instancia; no ejecutar otro test DDS simultáneamente.

## Instalación y arranque

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
bash scripts/navigation.sh
```

Abre MuJoCo y RViz. Esperar que termine el calentamiento y aparezca el mapa.
RViz se abre a los 20 segundos para no coincidir con la activación de Nav2.
Por defecto usa aceleración gráfica GPU. Si aparecen errores Mesa/DRI3 en
XWayland, añadir `rviz_software_rendering:=true` para usar renderizado por
software con dos hilos únicamente en RViz; MuJoCo y la cámara mantienen su
configuración GPU.
Usar **Nav2 Goal** en RViz para indicar un punto y su orientación; empezar con
destinos cercanos dentro de la zona ya observada. El panel Navigation2 permite
cancelar una meta. La exploración automática no está implementada. El mapa se
amplía a medida que se navega; no se puede asumir que SLAM conoce las salas
ocultas al inicio.

Sin ventanas: `bash scripts/navigation.sh viewer:=false rviz:=false`.
Cerrar MuJoCo detiene el launch; Ctrl+C detiene el conjunto. Sin comandos nuevos
durante 0.5 segundos de pared, la consigna de velocidad vuelve a cero y la
política sigue sosteniendo al robot. Velocidades máximas: 0.45 m/s lineal y
1.0 rad/s angular; la aceleración lineal de Nav2 está limitada a 0.8 m/s².
La navegación vuelve a usar el checkpoint RL Lab original. Como esta política
gira muy lentamente sin trasladarse, las órdenes de rotación pura de Nav2 se
acompañan con pulsos cosenoidales: primero retrocede, llega suavemente a cero
y luego avanza. Se admite una orden lineal residual de hasta 0.05 m/s;
por encima de ese umbral se conserva el avance de Nav2.
La ayuda de giro de Nav2 se aplica antes del monitor de colisiones:
`cmd_vel_smoothed → cmd_vel_assisted → collision_monitor → cmd_vel`.
El backend recibe la orden comprobada sin añadir movimiento. Los pulsos tienen
un máximo de 0.30 m/s, período de 1.5 s y requieren 0.5 m libres según LiDAR.
Además se comprueba el rectángulo barrido por el ancho de la huella acolchada
(0.66 × 0.70 m), con margen adicional de 0.10 m, para incluir obstáculos
diagonales que quedan fuera de los conos frontal y trasero.
La recuperación de metas individuales intenta retroceder 0.30 m a 0.10 m/s
antes de girar; el retroceso y el giro conservan las comprobaciones de colisión
de Nav2. Si no hay espacio seguro, pueden seguir siendo rechazados.
La ganancia angular es 4.0 al alinear y 1.5 durante seguimiento, limitada a
1.0 rad/s con rampa de 2.0 rad/s². La alineación inicial termina bajo 0.6 rad.
Una orden cero cancela el pulso. Sin comandos recientes (0.25 s reales) o
escaneos recientes (0.5 s de simulación), el filtro publica cero.
Los parámetros compartidos están en `configs/navigation/nav2_tuning.yaml`.

Las mediciones siguientes corresponden a versiones anteriores del asistente:
La validación integrada a 0.48 rad/s produjo 1.786 rad (102.3 grados) de giro,
sin caídas, watchdogs ni targets articulares recortados. Reporte:
`results/navigation/20260921T022754_6825f60c/metrics.json`.
La validación posterior a 0.55 rad/s produjo 1.837 rad (105.3 grados), también
sin caídas, watchdogs ni targets recortados. Reporte:
`results/navigation/20260921T024429_38f09827/metrics.json`.
La validación a 0.65 rad/s produjo 2.630 rad (150.7 grados), sin caídas ni
watchdogs y con 10 targets articulares recortados. Reporte:
`results/navigation/20260921T025927_0512f0fb/metrics.json`.
La validación corta a 0.75 rad/s produjo 3.136 rad (179.7 grados), sin caídas ni
watchdogs y con 73 targets articulares recortados. Reporte:
`results/navigation/20260921T030313_093c4276/metrics.json`. Este resultado aún
no sustituye una prueba prolongada de estabilidad.
Con la asistencia reforzada a 0.85 rad/s y 0.50 m/s, la prueba produjo 3.777 rad
(216.4 grados): 107.5 grados durante el retroceso y 97.7 durante el avance.
Terminó sin caídas ni watchdogs, con 296 targets articulares recortados. Reporte:
`results/navigation/20260921T031027_2549ede8/metrics.json`. Ambas semiondas ya
contribuyen al alineamiento, aunque esta configuración opera frecuentemente en
los límites articulares simulados.
Con pulsos cosenoidales simétricos y giro de hasta 1.0 rad/s, la prueba produjo
4.016 rad (230.1 grados): 84.9 grados retrocediendo, 98.0 avanzando y 47.1 en
las transiciones suaves. Terminó sin caídas ni watchdogs y con 102 targets
articulares recortados. Reporte:
`results/navigation/20260921T031806_85a6b683/metrics.json`.
En la prueba integrada del 2026-09-20, una orden pura de 0.35 rad/s produjo
1.754 rad (100.5 grados) de giro y 0.420 m de deriva en unos 11.6 s de comando,
con cero caídas, cero watchdogs y cero targets articulares recortados. La
trayectoria confirmó que la primera semionda fue hacia atrás. Reporte:
`results/navigation/20260921T014354_6d18de32/metrics.json`.

## Guardar un mapa y reutilizarlo

En otra terminal con ROS Jazzy, mientras corre mapping:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/facility" --ros-args -p use_sim_time:=true
```

Después de detener el launch anterior:

```bash
bash scripts/navigation.sh mode:=localization map:="$PWD/maps/facility.yaml"
```

En este modo AMCL estima `map → odom` usando el mapa guardado y el láser.
Marcar **2D Pose Estimate** en RViz en la posición inicial (centro del escenario,
orientado hacia +X) antes de enviar **Nav2 Goal**. No ejecutar SLAM y AMCL
simultáneamente. Nav2 es quien planifica y sigue caminos; la localización la
aporta SLAM durante mapping o AMCL con mapa guardado.

## Alcance y límites

- La odometría de entrada es ideal y procede de MuJoCo: falta un estimador
  propio para un robot físico. La corrección global sí corresponde a SLAM/AMCL.
- El MID-360 simulado usa ray casting físico, 360 muestras horizontales, nueve
  capas verticales, 10 Hz y 20 m de alcance. No reproduce todavía el patrón
  no repetitivo, reflectividad, retorno múltiple, ruido ni latencia del Livox
  real. `/scan` usa su capa más cercana a la horizontal; puede omitir obstáculos
  bajos y desniveles, por lo que no debe usarse para validar rampas.
- La D435i produce RGB y profundidad geométrica de MuJoCo a 424×240 y 10 Hz.
  Sus frames e intrínsecos ROS están disponibles, pero aún no se modelan ruido
  estéreo, agujeros de profundidad, exposición, rolling shutter ni distorsión.
- Se utiliza una base planar para navegación; la política controla el balance
  completo del humanoide. La huella rectangular con margen, de 0.66 × 0.70 m, aproxima el volumen durante la marcha.
- Los parámetros efectivos de Nav2 se generan desde los defaults de la versión
  Jazzy instalada y se guardan en `results/navigation/nav2_*.yaml`.
- Falta validar formación del mapa, planificación, cancelación, llegada y
  estabilidad con el stack completo. Los tests del sensor/puente por sí solos
  no demuestran navegación autónoma exitosa.

Referencia: https://docs.nav2.org/jazzy/tutorials/general_tutorials/navigation2_with_slam/navigation2_with_slam/

## Reparación de sensores 3D y evidencia (2026-10-01)

Las posiciones y rotaciones anteriores del MID-360 y la D435i diferían del
URDF cargado por ROS. El MID-360 estaba invertido 180° respecto de ese URDF.
Además, se publicaba una fila inclinada de rayos como `LaserScan` planar,
con distancias oblicuas y un frame ligado al sensor. Eso alteraba los lados,
la altura y las posiciones que SLAM/Nav2 atribuían a los obstáculos.
El enlace base/pelvis también omitía altura y balanceo del cuerpo.

Ahora el montaje coincide con el URDF del repositorio. La cámara compone
la inclinación `d435_joint` con la convención óptica ROS y MuJoCo; RGB y
profundidad comparten pose, dimensiones, calibración y tiempo de adquisición.
La pose del pelvis conserva altura, roll y pitch; las articulaciones llevan
el mismo timestamp que la pose. Los poses cartesianos de MuJoCo se actualizan
antes de muestrear para evitar combinar qpos nuevo con xmat/xpos anteriores.

El scan se construye con todos los retornos reales del LiDAR: transformación
al mundo, selección de altura 0.15–1.50 m, distancia XY desde el centro del
robot y mínimo por sector angular en un frame horizontal. Las capas hacia
abajo llegan a -45° para observar muebles por debajo del montaje. La nube
publicada conserva también suelo y otros retornos fuera de la banda del scan.
Los meshes de colisión del robot ocluyen rayos; esos impactos propios se
filtran sin buscar artificialmente otro objeto detrás del cuerpo.
Los objetos que no reciben un rayo no se consideran detectados: esto sigue
siendo un sensor aproximado, sin el patrón físico completo del MID-360.

La nube se publica a su propia frecuencia incluso con cámara deshabilitada.
Cada modalidad conserva su tiempo de adquisición. El receptor UDP ya no
elimina una imagen parcial al completar una nube más nueva y acota los frames
incompletos (32 secuencias, expiración 0.5 s). Imágenes y nube usan QoS de
sensores; scan mantiene QoS compatible con Nav2. Se procesan pose/articulaciones
antes de imágenes/nube. Durante ray casting/renderizado la física está pausada,
pero se permite publicar LowState DDS para evitar interrupciones por timeout.

Auditorías de 70 s de pared, seed 0, sin MuJoCo viewer ni RViz:

| Evidencia | Normal | Ideal |
|---|---:|---:|
| Scans recibidos | 372 | 531 |
| Frecuencia scan en tiempo simulado | 9.885 Hz | 9.919 Hz |
| Nubes / RGB / profundidad recibidos | 372 / 372 / 372 | deshabilitados |
| Pares RGB-D con igual timestamp, frame y dimensiones | 372 | — |
| Mensajes sin TF al tiempo de adquisición | 0 | 0 |
| Meta Nav2 `(0.7, 0)` | SUCCEEDED | SUCCEEDED |
| Obstáculos observados en mapa y costmaps | sí | sí |

Reportes: `results/navigation/sensor_audit_normal.json` y
`results/navigation/sensor_audit_ideal.json`. El ensayo MuJoCo–DDS–ROS de 6 s
registró 0.361 m de avance, cero caídas y cero watchdogs en
`results/navigation/20261001T165625_9ac4442d/metrics.json`.
Las pruebas de regresión verifican montaje, cadena URDF con cintura articulada
y pelvis inclinado, proyección, autooclusiones, RGB-D renderizado, TF y
reensamblado UDP. No hubo avisos de cola de sensores llena en las auditorías.
Los conteos iguales no garantizan ausencia de pérdidas bajo toda carga.

Para repetir la auditoría sobre una sesión arrancada:

```bash
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=43 /usr/bin/python3 scripts/check_navigation_sensors.py \
  --duration 70 --goal 0.7 0 --output results/navigation/sensor_audit_normal.json
# Con navigation.sh --ideal, agregar --ideal al auditor.
```

Esta comparación valida una meta y la integración de sensores, TF, SLAM y
costmaps. No prueba mapas idénticos (las alturas observadas difieren),
recorridos largos, orientación final arbitraria, obstáculos muy bajos,
AMCL ni rendimiento con ambas ventanas abiertas. El modo normal sigue siendo
más lento: se recibieron menos scans en el mismo intervalo de pared.
El error de cierre `Cannot shutdown a ROS adapter that is not running`
también volvió a aparecer al interrumpir el launch; corresponde al pendiente
separado de arranque/cierre.

### Sensores en procesos separados

La física y el puente DDS ejecutan el simulador; la cámara RGB-D y el LiDAR
usan dos procesos adicionales con sus propias instancias de `MjModel`/`MjData`.
Reciben copias del estado completo de integración por memoria compartida.
Cada entrada y salida conserva únicamente el último cuadro pendiente: cuando
un sensor se retrasa, se reemplazan los pendientes sin acumular una cola.
Los tiempos `sensor_times` y `scan_time` corresponden al estado capturado,
no al momento en que termina el cálculo. El grabador usa `camera_time` y
solo guarda RGB/profundidad; los resultados tardíos del LiDAR no modifican
la cronología de la cámara.

`camera.backend` admite `auto` (predeterminado), `egl`, `osmesa` y `cpu`.
En `auto` se intenta EGL primero y, si falla la inicialización, OSMesa y luego
Mesa/EGL sin superficie con llvmpipe. `cpu` fuerza las alternativas de software.
OSMesa requiere `libosmesa6`; la alternativa Mesa/EGL usa el controlador Mesa
instalado y no necesita OSMesa. Si ninguna alternativa puede inicializarse,
el experimento informa el error. La telemetría `camera_renderer` identifica el
backend, el dispositivo y la aceleración efectiva (`gpu` o `cpu`). La cámara
selecciona su backend independientemente del visor interactivo GLFW.

La separación evita esperar rayos o renders dentro del bucle físico. No garantiza
por sí sola eliminar todos los timeouts: la transferencia de cuadros, el visor
y la carga general del equipo siguen consumiendo tiempo. El FPS reportado cuenta
los cuadros recibidos; los cuadros reemplazados no se cuentan.

El arranque también libera los puertos UDP 19420/19421/19422 si pertenecen a
una ejecución anterior de `navigation_bridge.py` o `navigation_sim.py` de este
workspace y usuario. Si pertenecen a otro proceso, cancela el arranque sin
cerrarlo. La salida del puente o del simulador solicita un único cierre del
stack para evitar continuar sin uno de los procesos esenciales.

### Odometría LiDAR + RGB-D con Kalman (`--real-odom`)

```bash
bash scripts/navigation.sh --real-odom
# Sin ventanas:
bash scripts/navigation.sh --real-odom viewer:=false rviz:=false
```

El arranque habitual conserva la odometría ideal. `--real-odom` usa las nubes
`/mid360/points` y los pares sincronizados de RGB, profundidad métrica y
calibración. No consume `/imu/data`, posición/velocidad de MuJoCo ni comandos
para estimar movimiento. La IMU del SDK sigue usándose para controlar el
balance y la marcha, independientemente de esta odometría.

Cada sensor mantiene una trayectoria y referencias propias:

- LiDAR: registro ICP punto a plano en seis grados de libertad. Conserva el
  suelo para observar altura, roll y pitch, y rechaza geometría degenerada.
- RGB-D: características ORB, profundidad de la referencia y PnP con RANSAC
  para obtener movimiento métrico. Verifica profundidad actual y error de
  reproyección. Si faltan características, intenta registro de profundidad
  con correspondencias filtradas por color. En escenas con poca geometría,
  asigna incertidumbre alta a los ejes que no puede determinar.

Un filtro de Kalman de 12 estados estima XYZ, roll/pitch/yaw y sus velocidades.
Predice con un modelo de velocidad constante y corrige con las poses de ambos
sensores, ponderadas por covarianza. Envuelve las innovaciones angulares y
rechaza saltos mediante la distancia de Mahalanobis. Las covarianzas son
aproximaciones conservadoras derivadas de calidad/geometría y errores del
registro; no son garantías estadísticas calibradas en hardware.

Se mantienen referencias hasta 0.25 m o 0.15 rad para evitar integrar ruido
entre cada par de cuadros. Si un sensor pierde seguimiento durante cinco
intentos y el otro sigue actualizado, se reinicializa su referencia en la
pose fusionada con incertidumbre inflada. Esta recuperación comparte un
origen con el otro sensor; no representa una localización independiente ni
un cierre de bucle. El filtro aproxima las correlaciones entre referencias.

El estimador publica `/odom` y `odom → base_link` para Nav2, y
`base_link → pelvis` con altura e inclinación estimadas. El puente no publica
ninguna de esas transformaciones en este modo, ni usa IMU para ellas.
El origen empieza con XY=0, orientación inicial cero y altura nominal
0.793 m, suponiendo que el robot empieza de pie sobre suelo plano; los cambios
posteriores de altura e inclinación sí se estiman desde los sensores.
Las transformaciones internas pelvis–sensor provienen del montaje URDF y
las posiciones articulares. `/scan` se proyecta desde los retornos reales
del LiDAR usando la pose fusionada, banda de altura 0.15–1.50 m.

Tópicos de diagnóstico:

| Tópico | Contenido |
|---|---|
| `/odometry/lidar` | Pose 3D aceptada del LiDAR y su covarianza |
| `/odometry/rgbd` | Pose 3D aceptada de RGB-D y su covarianza |
| `/odometry/fused_3d` | Pose 3D del filtro |
| `/odometry/status` | Fuente, método, aceptaciones, rechazos y último tiempo válido |
| `/ground_truth/odom` | Referencia aislada de MuJoCo para evaluación |

Una inicialización o una predicción sin medición no cuenta como odometría
válida. Si fallan ambos sensores, no se refresca `/odom`; el puente inhibe
consignas tras 0.3 s simulados sin odometría medida. Si falla solo uno, el
filtro sigue con el otro. Nav2 sigue necesitando el LiDAR para su escaneo de
obstáculos. `--ideal` es incompatible, porque desactiva estos sensores.

Dependencias en `/usr/bin/python3`: ROS Jazzy, NumPy, SciPy y OpenCV
(`python3-opencv` si falta). El proceso limita BLAS/OpenMP/OpenCV a un hilo
para no competir con el controlador DDS. No necesita RTAB-Map ni GPU para
la estimación; el renderizado de RGB-D conserva su backend habitual.

La fusión estima odometría local; SLAM Toolbox sigue formando el mapa 2D.
La cámara no agrega imágenes al mapa. Persisten limitaciones de textura,
oclusiones, registro local, sesgos/correlaciones y pérdida prolongada de
ambos sensores. No se replica aún ruido o distorsión temporal de sensores
reales ni se valida navegación sobre rampas.

Para comparar con la referencia, sin que esta entre en el estimador:

```bash
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  /usr/bin/python3 scripts/check_navigation_odometry.py --duration 65 \
  --output results/navigation/real_odom_rgbd_lidar_comparison.json
```

La auditoría alinea el origen XY/yaw inicial, mide error de trayectoria y
cuenta mediciones aceptadas de cada sensor. También lista las suscripciones
del estimador para comprobar que no consume IMU ni ground truth.

Validación sin ventanas (2026-10-01):

- Meta Nav2 `(0.7, 0)` alcanzada; 462 scans verificados por TF, sin errores en
  `results/navigation/sensor_audit_rgbd_lidar.json`. La comparación simultánea
  aceptó 459 poses LiDAR y 434 RGB-D; error XY máximo 0.0215 m.
- Ensayo acotado de avance y giro con ambos sensores: sesión completada,
  cero caídas y cero watchdogs. Desplazamiento neto 1.245 m y giro físico de
  aproximadamente 54.5°. Aceptó 312 poses LiDAR y 309 RGB-D; en el intervalo
  auditado el error XY máximo fue 0.0215 m, final 0.0165 m, y el error de yaw
  máximo 0.0094 rad (0.54°). Reportes:
  `results/navigation/real_odom_rgbd_lidar_turn_comparison.json` y
  `results/navigation/20261001T234025_24dbe56e/metrics.json`.

Estos ensayos cortos no establecen precisión en recorridos largos ni
rendimiento con MuJoCo viewer y RViz abiertos. Una prueba previa bajo carga
concurrente terminó por `LowState missing or stale`; la ejecución acotada
final, sin pruebas gráficas paralelas, completó sin ese error.

### SLAM 3D: FAST-LIO2 y RTAB-Map

```bash
# Instalación local, sin sudo (ROS Jazzy y herramientas de compilación requeridas).
bash scripts/install_navigation_slam.sh

# Nube geométrica XYZ: FAST-LIO2 + IMU, con cierre de bucle LiDAR de RTAB-Map.
bash scripts/navigation.sh --slam lidar3d

# Nube coloreada XYZRGB: odometría RGB-D y SLAM de RTAB-Map.
bash scripts/navigation.sh --slam rgb-d

# Carpeta de salida explícita; para pruebas sin interfaces gráficas:
bash scripts/navigation.sh --slam lidar3d viewer:=false rviz:=false \
  duration:=60 slam_output:=results/navigation/my_lidar_map
```

Sin `--slam` se conserva SLAM Toolbox 2D. Los dos modos nuevos activan
odometría por sensores automáticamente, sin necesitar `--real-odom`, y
reemplazan el estimador LiDAR/RGB-D propio y SLAM Toolbox. `--ideal` es
incompatible porque desactiva los sensores montados. Actualmente estos modos
implementan `mode:=mapping`; la localización desde un mapa 3D guardado queda
fuera de esta integración. `nav2:=false` permite ejecutar únicamente sensores,
odometría, mapeo y RViz para comparar los motores.

Los resultados se guardan en una carpeta única bajo `results/navigation/`, o en
`slam_output`. Ambos escriben **`map.pcd`** y exportan al cerrar normalmente.
RGB-D lo guarda cada 30 segundos de simulación; LiDAR lo actualiza con cada
reconstrucción del grafo (hasta una cada 2 segundos de reloj real). Ambos
conservan **`rtabmap.db`**: RGB-D guarda imágenes y profundidad; LiDAR guarda
escaneos 3D. Las bases incluyen trayectoria y grafo para las herramientas de RTAB-Map. El PCD visual conserva el campo RGB. El modo RGB-D
también reconstruye una malla coloreada y guarda `map_mesh.ply`; no utiliza
texturas UV de las imágenes. Al reutilizar explícitamente una carpeta existente RTAB-Map puede
abrir su base de datos anterior; usar una carpeta nueva para una sesión nueva.

En RViz, `SLAM 3D map` muestra `/slam/map_points`. Ambos motores proporcionan
`/odom` y las transformaciones estimadas para Nav2. `/map` es una proyección 2D
para planificación, adicional al mapa 3D. RTAB-Map reconstruye esa ocupación
con los escaneos LiDAR o la profundidad RGB-D y sus poses optimizadas.

En `--slam rgb-d`, RViz permite alternar las casillas:

- **SLAM 3D map:** nube de puntos coloreados, activada inicialmente.
- **SLAM 3D mesh:** superficies trianguladas coloreadas, desactivada inicialmente.

Para ver solo la malla, desactivar `SLAM 3D map` y activar `SLAM 3D mesh`.
También pueden mostrarse ambas a la vez. Las visualizaciones de sensores, como
`MID-360 3D` y `LaserScan`, tienen casillas independientes.

La malla se publica como un `Marker/TRIANGLE_LIST` en `/slam/map_mesh`, en el
frame `map`, con el último resultado disponible al activar la casilla de RViz.
Ahora se genera mediante [TSDF de Open3D](https://www.open3d.org/docs/release/tutorial/pipelines/rgbd_integration.html),
fusionando las imágenes RGB-D originales de los nodos de RTAB-Map. Usa la
calibración de cámara, su transformación local y las poses optimizadas del grafo,
en lugar de la nube simplificada de ocupación. Los cierres de bucle reconstruyen
el volumen con las poses corregidas para eliminar las superficies anteriores.

La resolución por defecto es **3 cm**, con una banda TSDF de **9 cm** y profundidad
entre **0.5 y 6 m**. Se actualiza como máximo cada **5 segundos de reloj real**, en
un trabajador separado, y limita la visualización/exportación a **100000 triángulos**.
Se conservan hasta **400 imágenes clave admitidas**; al superar ese límite se
retiran las más antiguas y se reconstruye el volumen. Por ello la malla está
limitada a esas observaciones, mientras que `rtabmap.db` conserva el mapa completo.
RTAB-Map admite observaciones quieto a 2 Hz, y TSDF conserva las tres vistas más recientes
casi idénticas (menos de 2 cm y 0.02 rad) para limitar su peso y consumo.

`map_mesh.ply` guarda geometría y colores por vértice, sin texturas UV.
`map_mesh.json` registra resolución, número de imágenes, triángulos y
reconstrucciones. Al cerrar normalmente se exporta el último grafo recibido.
La integración reduce ruido si las poses son buenas; no corrige una odometría
incorrecta ni completa superficies ocultas. Nav2 conserva su mapa de ocupación
independiente de 10 cm.

El instalador prepara Open3D 0.19.0 en un entorno Python local con acceso a ROS
y compila el decodificador que utiliza el codec oficial de RTAB-Map. Para actualizar
una instalación existente ejecutar `bash scripts/install_navigation_tsdf.sh`.
El modo `--slam rgb-d` utiliza TSDF automáticamente y mantiene las mismas casillas
de RViz. La triangulación PCL anterior ya no se usa para la malla RGB-D.

Para ajustar resolución, frecuencia y cantidad de imágenes:

```bash
bash scripts/navigation.sh --slam rgb-d mesh_voxel_size:=0.02 mesh_update_period:=3.0 mesh_max_frames:=400
```

Vóxeles menores o más imágenes aumentan el consumo de memoria y CPU. La resolución
del vóxel es un ajuste de reconstrucción, no una garantía de precisión de posición.

Nav2 arranca cuando recibe un mapa, odometría y un `/scan` reciente, después de la
estabilización inicial del robot. La espera de arranque admite hasta 180
segundos de reloj real para cargar sensores y modelos.
La configuración de navegación del simulador admite hasta 1 segundo sin un
`LowState` nuevo para tolerar pausas de carga del equipo. La frescura de comandos
DDS sigue limitada a 250 ms y el bridge exige odometría de menos de 300 ms.

El modo `rgb-d` captura a 20 Hz (`camera_fps:=...` permite ajustar la frecuencia).
Usa flujo óptico entre cuadros consecutivos y correspondencias 3D–3D de la
profundidad, sin extrapolar movimiento. Renueva el cuadro de referencia en cada
captura y descarta puntos a menos de 0.5 m para reducir observaciones del propio
robot. Estos ajustes evitan que la textura repetitiva del suelo simulado mantenga
una predicción incorrecta. La odometría y el mapa visual usan únicamente RGB-D;
el LiDAR sigue proporcionando obstáculos a Nav2.

```bash
# Guardar la última nube sin detener la sesión (en un terminal ROS del dominio 43).
source /opt/ros/jazzy/setup.bash
source scripts/slam_env.sh
export ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 service call /slam/save_map std_srvs/srv/Trigger '{}'

# Prueba observable; --drive avanza y gira lentamente el robot simulado.
/usr/bin/python3 scripts/check_navigation_slam.py --duration 35 --drive \
  --output results/navigation/slam_check.json
```

**FAST-LIO2:** se usa el port ROS 2 de
[Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2), fijado en
`2fffc570a25d0df172720bac034fbdb6a13d2162`, derivado de
[hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO). La instalación compila
las interfaces oficiales Livox sin ejecutar el driver de hardware. Los ajustes
reproducibles en `prepare_fast_lio.py` fuerzan C++17 y permiten capturas
instantáneas: el LiDAR de MuJoCo no tiene un barrido físico ni tiempos por punto,
por lo que no se inventan y no se aplica deskew a esas capturas. Este ajuste es
para el simulador; con un MID360 físico hay que utilizar sus mensajes y tiempos
reales, extrínsecos calibrados y la configuración normal de FAST-LIO.

`/imu/torso` publica aceleración específica y velocidad angular de
`secondary_imu` en cada paso de física (500 Hz con timestep 0.002), con sus
marcas temporales originales. No publica orientación real del simulador como
medición. La IMU está en `waist_roll_link`, unido rígidamente al torso mediante
el desplazamiento de 0.019 m; su frame ROS y el extrínseco del LiDAR incluyen
ese desplazamiento. Conservar todas las muestras evita aliasing de las
aceleraciones impulsivas de los contactos del humanoide.

El adaptador obtiene la pose de pelvis a partir de la pose estimada y las
articulaciones publicadas, y separa la pose planar de la altura/roll/pitch. El
origen vertical usa la altura nominal de pie del modelo, no la telemetría de
posición real. Esto supone inicialización de pie sobre el suelo z=0; para otro
robot o altura inicial deben ajustarse los anclajes y extrínsecos.

En `lidar3d`, **FAST-LIO2 conserva la odometría LiDAR–inercial** y RTAB-Map
agrega cierres por registro ICP de escaneos 3D y optimización del grafo, siguiendo
el [ejemplo oficial de SLAM LiDAR de RTAB-Map](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/lidar3d.launch.py).
No se usan imágenes ni ground truth para cerrar bucles. La configuración está
en `configs/navigation/rtabmap_lidar.yaml`: búsqueda espacial a 2 m, memoria
reciente de 10 nodos, ICP con vóxeles de 10 cm y al menos 40% de correspondencias.
Se rechazan correcciones ICP mayores a 75 cm o 0.35 rad y cierres que producen
un error inconsistente en el grafo. Esto permite corregir deriva moderada al
revisitar lugares; no es reconocimiento global independiente de la posición.
Si la deriva deja el lugar fuera del radio de búsqueda o el entorno carece de
geometría distintiva, el cierre puede no detectarse o ser rechazado.

RTAB-Map publica el único `map -> odom`, mientras que `/odom` y
`odom -> base_link` siguen continuos desde FAST-LIO2. Así las correcciones
alcanzan Nav2 sin saltos en las velocidades locales. `/slam/registered_points`
sigue disponible para inspeccionar FAST-LIO2 en `camera_init`; la nube pública
`/slam/map_points` se reconstruye desde los escaneos con las poses optimizadas,
eliminando puntos anteriores a cada corrección. Conserva toda la altura del
escaneo, separada de los filtros de altura del mapa de navegación.

La nube XYZ se voxeliza a 8 cm, limita su salida a 400000 puntos y conserva
hasta 1000 escaneos para acotar memoria; al superar ese límite retira los más
antiguos de la visualización. `rtabmap.db` conserva la historia completa.
`loop_closures.json` registra cierres aceptados, nodos, enlaces y `map_to_odom`;
`/slam/info` y `/slam/map_graph` permiten inspeccionar el backend en ROS.
`/slam/save_map` devuelve la última nube optimizada exportada. El mapa 2D se
corrige directamente desde el grafo de RTAB-Map. Se mantiene la hipótesis de
entorno mayormente estático: el cierre no resuelve obstáculos móviles.

Para verificar el backend con un recorrido sintético y deriva conocida, en un
dominio ROS separado de la navegación:

```bash
source /opt/ros/jazzy/setup.bash
source scripts/slam_env.sh
ROS_DOMAIN_ID=46 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  /usr/bin/python3 scripts/check_navigation_lidar_loop.py \
  --output-dir results/navigation/lidar_loop_validation
```

La carpeta de validación debe ser nueva. Esta prueba arranca el mismo backend
LiDAR de producción, inyecta 22 cm de deriva en un recorrido cerrado, exige
cierres aceptados y enlaces de grafo, y verifica reducción del error al regresar.
Los datos sintéticos tienen geometría ideal; no representan precisión real del
robot. Los mapas exportados y `validation.json` quedan junto a los logs.

Validación de esta integración: 29 pruebas pasaron. El backend final aceptó
32 cierres en el recorrido sintético y redujo los 22 cm de deriva inyectada.
MuJoCo registró 12 cierres y el objetivo Nav2 `(1, 0)` terminó en `SUCCEEDED`.
El objetivo de regreso se interrumpió al alcanzar la duración de la sesión,
por lo que esa prueba no valida el regreso completo. Al cerrar también se
observó un fallo del proceso nativo `collision_monitor`; FAST-LIO2, el grafo y
la exportación terminaron. Informes en `results/navigation/lidar_loop_final/`,
`lidar_loop_final_goal.json`, `lidar_loop_return_goal.json` y
`lidar_loop_backend_verified/validation.json`.

El script configura [publicación DDS asíncrona](https://github.com/ros2/rmw_fastrtps#change-publication-mode)
para reducir bloqueos de envío durante el procesamiento. En las pruebas con
publicación síncrona se observaron respuestas de servicios Nav2 que agotaban su
tiempo y detenían su activación; los siguientes arranques asíncronos completaron
esa activación.

Los estimadores no consumen `/ground_truth/odom`; ese topic permanece aislado
para evaluación. El bridge inhibe comandos cuando falta odometría reciente.
La prueba comprueba recepción de mapas y odometría, desplazamiento y error XY
contra ground truth únicamente en el proceso de evaluación; no sustituye una
prueba de recorridos largos ni de cierre de bucles.

Validación local del 2026-10-02, con avance y giro mediante `/cmd_vel`, Nav2
activo y cierre normal de sesiones de 30 segundos de navegación:

| Motor | Desplazamiento máximo | Error XY máximo | Mapa guardado |
| --- | --- | --- | --- |
| FAST-LIO2 | 1.87 m | 0.039 m | PCD XYZ, 63371 puntos |
| RTAB-Map RGB-D | 2.10 m | 0.079 m | PCD XYZRGB, 4130 puntos y base RTAB-Map |

Informes: `results/navigation/lidar_nav_slam_check.json` y
`results/navigation/rgbd_nav_slam_check.json`. Estas pruebas verifican sensores,
estimación, mapeo y activación de Nav2; no evalúan un objetivo autónomo de Nav2.

Para comprobar también el recorrido autónomo, con una sesión de navegación
activa y un destino libre en el mapa:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
/usr/bin/python3 scripts/check_navigation_goal.py --x 1 --y 0 --yaw 0 \
  --output results/navigation/goal_check.json
```

La prueba espera que Nav2 esté activo, envía `NavigateToPose` y comprueba scans,
comandos de velocidad, desplazamiento y resultado `SUCCEEDED`. Ambos motores
completaron el destino `(1, 0)` en las pruebas locales. Informes:
`results/navigation/lidar_goal_check.json` y `results/navigation/rgbd_goal_check.json`.

Las nubes LiDAR se retienen brevemente hasta disponer del TF estimado de su
captura, porque la odometría llega después del sensor. Esto evita descartar
todos los scans y dejar al monitor de colisiones de Nav2 bloqueando el movimiento
por falta de datos. La cola está limitada a 10 nubes y descarta las de más de
500 ms; no se desactiva el monitor de colisiones.

En RGB-D, el monitor usa el scan en su instante de captura, sin la compensación
`base_shift_correction` que exige un TF de odometría al instante actual. La
odometría visual llega con retraso y esa exigencia provocaba paradas por
extrapolación futura, especialmente con RViz y MuJoCo abiertos.

La grilla visual tiene un tamaño mínimo de 20 m; las zonas no observadas siguen
siendo desconocidas y Nav2 conserva `allow_unknown`. Esto permite planificar
destinos fuera del campo inicial de la cámara, manteniendo la detección de
obstáculos del LiDAR. RTAB-Map filtra puntos a menos de 0.5 m y dentro de la
huella del robot (0.66 × 0.70 m, altura 1.5 m), y limpia una huella de radio 0.30 m
en las poses recorridas para evitar obstáculos del propio cuerpo.

También se probaron destinos RGB-D con ambas interfaces abiertas: `(0.8, -0.8)`
con yaw `-0.785`, y `(-0.6, -0.7)` con yaw `-2.28`. Ambos terminaron en
`SUCCEEDED`. Informes: `results/navigation/rgbd_gui_goal_check.json` y
`results/navigation/rgbd_reverse_goal_check.json`.


## Navegación cerca de obstáculos

`configs/navigation/nav2_tuning.yaml` configura ambos SLAM. La huella mide
0.56 × 0.60 m más 0.05 m de margen por lado. La inflación mantiene radio
0.7 m y factor 5.0, coincidente con el factor usado por Pure Pursuit para
regular velocidad. La ventana local mide 4 × 4 m. El LiDAR marca hasta 5 m
y limpia hasta 8 m, incluidos retornos infinitos válidos. Se conserva el
monitor de colisiones sobre la huella, ahora después de la ayuda de giro.

Validación en MuJoCo con inicio junto a una pared dentro de inflación:

- `results/navigation/clearance_lidar_isolated_goal.json`: destino alcanzado;
  coste inicial 32, desplazamiento máximo 1.33 m.
- `results/navigation/clearance_rgbd_goal.json`: destino alcanzado con TSDF
  activo; coste inicial 41, desplazamiento máximo 1.11 m.
- `results/navigation/collision_pipeline_verified/validation.json`: seis
  comprobaciones satisfactorias con el monitor nativo: avance libre, parada
  ante obstáculo, recuperación, traslación de giro comprobada, cancelación
  mediante orden cero y parada por escaneo vencido.

Los parámetros de lanzamiento `initial_x`, `initial_y`, `initial_yaw` colocan
el robot en la simulación sin proporcionar posición perfecta al estimador.
`scene:=/ruta/escena.xml` permite probar otra escena. Las pruebas junto a pared
usaron `initial_x:=2.5 initial_y:=-0.65`, con objetivo del mapa SLAM `(1, -0.7)`.
El comprobador `scripts/check_navigation_goal.py` acepta `--require-inflated-start`.
`scripts/check_navigation_collision_pipeline.py` comprueba el monitor aislado;
usar un ROS_DOMAIN_ID distinto al de la simulación.

Limitación: el pasillo de 0.8 m terminó abortado; el de 1.0 m avanzó 0.60 m
pero no completó el objetivo antes del límite de tiempo. Reportes:
`results/navigation/clearance_corridor_verified_goal.json` y
`results/navigation/clearance_corridor_1m_goal.json`. El ancho de la huella
no garantiza espacio para girar o absorber oscilaciones y error de odometría.
Se conservaron los controles de colisión y márgenes: todavía no se acredita
navegación fiable en esos pasillos.

## Recorridos y localización 3D en otra sesión

Los dos backends admiten `mode:=localization map:=/ruta/rtabmap.db`.
Se mantienen sus odometrías de sensores (FAST-LIO2 o RGB-D); RTAB-Map
compara las observaciones contra el mapa guardado con `Mem/IncrementalMemory=false`
y `Mem/InitWMWithAllNodes=true`. No se inicia AMCL en estos modos.
La configuración sigue el [ejemplo oficial de RTAB-Map](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/robot_mapping_demo.launch.py).

Primera sesión, elegir `lidar3d` o `rgb-d`:

```bash
bash scripts/navigation.sh --slam lidar3d slam_output:="$PWD/maps/lidar_training"
```

Recorrer el entorno con Nav2 Goal. Para validar una secuencia desde otra terminal
con ROS y el entorno SLAM cargados:

```bash
source /opt/ros/jazzy/setup.bash
source scripts/slam_env.sh
export ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
python3 scripts/check_navigation_route.py \
  --goal 1 0 0 --goal 0.2 0 0 --timeout 90 \
  --output results/navigation/training_route
```

Cada objetivo tiene un informe separado; `route.json` exige que todos lleguen
con lecturas de sensores y movimiento comprobado para destinos fuera de la tolerancia inicial. Se registran posiciones y distancia al objetivo en el marco del mapa; un destino ya cercano puede completarse sin exigir 25 cm de marcha.
La secuencia se detiene ante el primer fallo. Los destinos deben elegirse
según las zonas libres del mapa, en metros y yaw en radianes.

Cerrar la primera sesión con Ctrl+C y esperar el mensaje de RTAB-Map que
confirma el guardado de la base de datos. `/slam/save_map` exporta la nube PCD;
**el archivo necesario para localizarse es `rtabmap.db`**, no la nube ni la malla.
Conservar también `slam_session.json`, que identifica el backend.

Segunda sesión:

```bash
bash scripts/navigation.sh --slam lidar3d mode:=localization \
  map:="$PWD/maps/lidar_training/rtabmap.db" \
  slam_output:="$PWD/results/navigation/lidar_localization"
```

Para RGB-D sustituir `lidar3d` por `rgb-d` y usar su propia base de datos.
La carpeta de salida debe ser nueva: se copia la base mediante SQLite backup
para conservar intacto el mapa original. No se agregan nuevos nodos al mapa
cuando se localiza. Nav2 espera una coincidencia aceptada con el mapa, además
de mapa, odometría y escaneos recientes; si no la consigue, no se activa.
`readiness.json` registra los identificadores de las coincidencias y el resultado.

En LiDAR, el registro ICP por proximidad necesita arrancar cerca de una zona
conocida y con orientación compatible. No garantiza localización global desde
cualquier lugar: `RGBD/StartAtOrigin=true` sólo proporciona una hipótesis inicial,
y no cuenta como coincidencia verificada. RGB-D requiere suficientes rasgos
visuales compartidos. La posición perfecta de MuJoCo no alimenta ninguno de
los estimadores.

Validación del 2026-10-03:

- Mapa LiDAR nuevo guardado en
  `results/navigation/route_training_lidar_20261003/rtabmap.db` (10 nodos).
  La sesión nueva `route_localization_lidar_20261003` reconoció el nodo 1
  y completó dos destinos consecutivos `(0.7, 0)` y `(1.2, 0)`.
  Informes: `readiness.json` y `validation/route.json` dentro de esa carpeta.
  La distancia final medida del segundo tramo fue 0.156 m; la del primero,
  0.395 m. El goal checker stateful puede mantener el criterio XY mientras
  alinea yaw y la asistencia añade traslación: éxito de Nav2 no acredita
  precisión fina ni ausencia de deriva durante el giro final.
- RGB-D recuperó el mapa de `clearance_rgbd/rtabmap.db` (199 nodos), con
  coincidencias visuales registradas en
  `results/navigation/relocalization_rgbd_20261003/readiness.json`.
  Hubo llegadas aisladas y también recorridos que no completaron todos sus
  tramos; no se consideran aprobados por reconocer el mapa solamente.
- `results/navigation/session_restore_integrity_20261003.json` registra
  integridad SQLite y recuentos idénticos de nodos antes/después de localizar.
  Las pruebas unitarias también verifican que escribir en la copia no cambia
  el hash del mapa original y rechazan mapas vacíos o de otro backend.

La primera prueba de construcción del mapa LiDAR activó parcialmente Nav2,
pero el planner no completó su activación y no se envió ningún objetivo.
El informe `route_training_lidar_20261003/validation/route.json` conserva ese
fallo; el recorrido satisfactorio corresponde a la sesión de localización.
Los ensayos junto a pared con el mapa LiDAR anterior tampoco completaron el
recorrido dentro del plazo. Estas diferencias quedan registradas para evitar
confundir una prueba puntual con fiabilidad general.

El recorrido RGB-D final también pasó sus dos destinos `(0.6, -0.5)` y
`(1.1, -0.7)` en una sesión recuperada. Informe:
`results/navigation/route_localization_rgbd_verified/validation_after_tf_ready/route.json`.
Las distancias finales medidas fueron 0.107 m y 0.290 m, respectivamente.
El intento inmediato tras activación falló por no estar disponible aún TF en
el navegador. El comprobador ahora exige TF local y espera 1 s adicional
para su propagación antes de enviar el objetivo. El mapa cargado se conserva
sin añadir nodos; las estimaciones siguen dependiendo de los sensores.

Verificación de código: 34 tests satisfactorios (incluidos aislamiento de base
SQLite y regresiones del puente, nubes, mallas y TSDF), Ruff y `git diff --check`.

Mapa RGB-D con mallado preparado el 2026-10-03:
`maps/rgbd_mallado_demo_20261003/`. Se recorrieron dos destinos y se guardaron
110 nodos en RTAB-Map, con una malla TSDF coloreada de 96 648 triángulos.
Es el sector observado durante ese recorrido; no es un relevamiento completo
de toda la instalación. Para probarlo con RViz y el simulador:

```bash
bash scripts/navigation.sh --slam rgb-d mode:=localization \
  map:="$PWD/maps/rgbd_mallado_demo_20261003/rtabmap.db"
```

Si existe `map_mesh.ply` junto a la base, la sesión de localización copia
la malla y sus metadatos, publica directamente esa geometría en
`/slam/map_mesh` y habilita **SLAM 3D mesh** en RViz. No sustituye la malla
completa por una reconstrucción parcial de las imágenes de la sesión nueva.
Los cambios del mapa durante mapping siguen reconstruyendo TSDF normalmente.

## Sensores con condiciones aproximadas de hardware: `--real`

Para construir un mapa nuevo con imperfecciones de sensores:

```bash
bash scripts/navigation.sh --slam rgb-d --real
bash scripts/navigation.sh --slam lidar3d --real
```

`--real` activa `real_sensors:=true` y odometría de sensores. Es independiente
de `--real-odom`, que selecciona el estimador sin añadir imperfecciones.
No se combina con `--ideal`. El perfil está en
`configs/navigation/sensors_real.yaml` y queda incluido en `config.yaml` de
los resultados del simulador. Sin `--real` se conserva la generación anterior.

El perfil usa hipótesis configurables, no una calibración de un dispositivo:
LiDAR con ruido radial de 1 cm más 0.05 % de distancia, 2 % de pérdida de
retornos y rango mínimo de 0.2 m; profundidad de 0.2–6 m con ruido
`0.001 + 0.001*z²` m, cuantización de 1 mm, 1 % de píxeles inválidos y pérdidas
adicionales en discontinuidades; RGB con ruido de 1.5 niveles y fluctuaciones
de exposición. IMU de navegación con ruido por muestra y sesgo fijo por
sesión en acelerómetro y giróscopo. Los ruidos usan semillas reproducibles.

La cámara tiene entrega diferida de 35 ± 10 ms y el LiDAR de 20 ± 5 ms,
en tiempo de simulación, además del coste real de generación y transporte.
Se conservan las marcas de captura; no se falsifican timestamps ni se bloquea
el paso de física para esperar sensores. La cola está acotada a ocho frames.
La proyección Nav2 del LiDAR se calcula de la misma nube degradada que recibe
SLAM. La IMU de la política de balance y la verdad de simulación no se alteran.

El modelo aún conserva el patrón de rayos existente: no reproduce el patrón
no repetitivo exacto del MID-360, reflectividad de materiales, propagación
óptica, estéreo físico ni rolling shutter. Las cifras son parámetros de prueba;
la precisión depende del entorno y del equipo. Referencias del fabricante:
[Livox MID-360](https://www.livoxtech.com/mid-360) y
[metodología RealSense de calidad de profundidad](https://www.realsenseai.com/wp-content/uploads/2019/11/RealSense_DepthQualityTesting.pdf).

Pruebas cortas de `--real` (2026-10-03, robot quieto): FAST-LIO inicializó
la IMU y RTAB-Map guardó cuatro nodos y una nube de 5 130 puntos;
RGB-D mantuvo odometría y generó una malla TSDF. Ambos simuladores terminaron
sin caídas ni watchdogs. Artefactos en `results/navigation/real_sensors_lidar_check`
y `results/navigation/real_sensors_rgbd_check`. Estas pruebas verifican la
integración; todavía no validan precisión ni recorridos prolongados con ruido.

## Bloqueo LiDAR junto a paredes con `--real`

El perfil específico `configs/navigation/nav2_lidar.yaml` se aplica cuando
`--slam lidar3d` está seleccionado. La huella global de planificación conserva
0.66 × 0.70 m, mientras que controlador local y monitor utilizan una envolvente
coherente de 0.66 × 0.64 m. La anchura nominal de colisión medida en postura
inicial es aproximadamente 0.55 m: la envolvente local mantiene 4.5 cm de margen
por lado. Este ajuste distingue el espacio preferido para planificar del espacio
que exige una parada inmediata; no elimina la comprobación de obstáculos.

Antes, el monitor recibía la huella globalmente ampliada mediante el tópico
`published_footprint`. Una pared dentro de ese margen producía tiempo de colisión
cero y anulaba incluso órdenes para salir de la zona. El monitor LiDAR ahora usa
puntos explícitos y conserva la predicción de colisión de 1 s, tres retornos
mínimos, escaneos recientes y comprobación de la asistencia de giro. El umbral
para alineación inicial de Pure Pursuit es 1 rad: los errores moderados pueden
corregirse con un arco comprobado, sin exigir primero un giro junto a la pared.
RGB-D mantiene su perfil anterior.

Verificación del 2026-10-03:

- `results/navigation/lidar_wall_collision_verified_retry/validation.json`:
  nueve comprobaciones del monitor nativo aprobadas, incluyendo movimiento
  paralelo a pared a 0.335 m, salida alejándose de ella y limitación del avance
  hacia la pared, además de parada ante obstáculos, falta de escaneos y orden cero.
- `results/navigation/lidar_wall_real_fixed_goal.json`: inicio dentro de inflación
  (coste 51), 2 541 órdenes de movimiento y desplazamiento máximo de 1.164 m.
  No confirmó éxito de NavigateToPose antes del límite; error final de posición
  0.300 m. Esto acredita movimiento desde la proximidad, no llegada precisa.
- El inicio más ajustado a 0.35 m dejó el monitor anterior anulando todas las
  órdenes. Un primer ajuste parcial siguió fallando; la prueba siguiente de la
  envolvente coherente se invalidó porque Nav2 no terminó su activación.
  No se presenta ese caso como navegación completa aprobada.

Si los retornos invaden la envolvente local, el monitor todavía debe detenerse.
No está validado iniciar en contacto con la pared ni en un pasillo cuyo espacio
impide girar. Para repetir sólo el monitor, usar un ROS_DOMAIN_ID distinto al de
la simulación y `scripts/check_navigation_collision_pipeline.py --backend lidar3d
--output-dir /ruta/nueva`; `--backend rgb-d` comprueba el perfil común anterior.

## Renovación de mallas RGB-D con ruido

Durante mapping, las vistas repetidas conservan ahora las tres observaciones
más recientes, reemplazando las antiguas. Antes se aceptaban las primeras tres
y se descartaban todas las siguientes, fijando los errores iniciales de profundidad.
Retirar una observación obliga a reconstruir TSDF desde las vistas retenidas:
las superficies antiguas dejan de aportar peso y pueden desaparecer al llegar
mediciones mejores. Se conservan los otros puntos de vista y el límite global
de frames. Las correcciones del grafo también siguen provocando reconstrucción.

La actualización permanece limitada por `mesh_update_period` (5 s reales por
defecto); no es instantánea ni supone reemplazar toda la geometría tras una sola
medición. No se fuerza un piso plano usando la posición perfecta de simulación.
Ruido persistente y errores de pose todavía pueden deformar las superficies.
`map_mesh.json` identifica la política y los IDs de los frames usados.

Regresión geométrica: tres mediciones equivocadas de una superficie a 2.25 m
se sustituyeron por tres nuevas a 2.00 m, verificando que la malla final abandona
la superficie anterior y queda a menos de 1 cm de la correcta. También se
comprueba que renovar una vista conserva las demás zonas del mapa.
# Estimación corporal por piernas

Activación opcional para ambos SLAM 3D:

```bash
bash scripts/navigation.sh --slam lidar3d --real --feet-estimation
bash scripts/navigation.sh --slam rgb-d --real --feet-estimation
```

El estimador procesa a 50 Hz las posiciones/velocidades articulares, torque
del actuador y la IMU de pelvis. Calcula cinemática y Jacobianos de los pies
en un estado privado del modelo, construido exclusivamente con esos sensores.
No copia posición/velocidad global, contactos ni fuerzas perfectas de MuJoCo.
Con `--real`, el acelerómetro y giroscopio de esta estimación reciben ruido y
sesgo independientes de la IMU de FAST-LIO; encoders, torque y orientación del
SDK conservan las características actuales del simulador.

La carga vertical se infiere mediante un modelo cuasiestático de torque y
gravedad. La confianza depende de carga, error del modelo, altura relativa
de los pies y acuerdo entre sus velocidades. Usa histéresis para el apoyo y
permite doble apoyo. La aceleración propaga velocidad, y los apoyos confiables
la corrigen; estima lentamente sesgo de aceleración durante apoyo quieto.

El adaptador de SLAM integra los incrementos de velocidad de los pies entre
mediciones nativas y los combina conservadoramente con la posición XY nativa.
Publica el resultado en `/odom` y `/odometry/slam_3d`, que alimentan Nav2 y
RTAB-Map. Conserva orientación y altura nativas; FAST-LIO sigue siendo el
estimador LiDAR-inercial original. No fusiona por segunda vez un quaternion
IMU ni usa posición integrada de piernas como una medición absoluta independiente.
La política de locomoción conserva sus entradas originales.

Diagnóstico: `/feet_estimation/state` contiene velocidad, varianza, confianza,
apoyos, carga inferida y posición relativa de pies; `/feet_estimation/fusion_status`
contiene contadores `used` y `fallbacks`. Sin apoyo confiable, con intervalos
mayores de 0.3 s, muestras de piernas separadas más de 0.06 s o discrepancias
grandes, se utiliza nuevamente la odometría nativa. Este modo requiere
`--slam lidar3d` o `--slam rgb-d` y es compatible con localización.

Es una estimación inicial para marcha erguida: el torque dinámico, un pie
balanceándose cerca del suelo o el deslizamiento simultáneo de ambos pies
pueden confundirse con apoyo. No equivale a sensores plantares ni garantiza
mayor precisión; conviene comparar recorridos con/sin la opción antes de
usarla como referencia de desempeño.

Para comprobar publicación, contribución a odometría y un avance breve en una
sesión de simulación con `nav2:=false`:

```bash
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=43 /usr/bin/python3 scripts/check_navigation_feet.py --drive \
  --duration 30 --output results/navigation/feet_validation.json
```

La referencia perfecta se utiliza únicamente en este comprobador para medir
error. El resultado comprueba que hay apoyos válidos, actualizaciones fusionadas
y desplazamiento, no que la opción supere al SLAM nativo.

## Inclinar la cámara RGB-D en RViz (simulación)

El lanzamiento normal agrega el panel **Cámara RGB-D**, fuera de la vista 3D.
Mové el deslizador hacia **Arriba** o **Abajo**, o ingresá el ángulo en grados.
El botón **Volver a posición original** restaura exactamente la orientación
configurada al arrancar (0° de desplazamiento adicional). El rango es de -30°
a +45°; positivo significa mirar hacia abajo. No requiere la herramienta Interact.
El panel muestra el ángulo observado en la última imagen y avisa si dejan de
llegar imágenes recientes. No guarda ni aplica un ángulo al abrir RViz.

Si el panel está oculto, activalo desde **Panels → Cámara RGB-D**. También se
puede agregar con **Panels → Add New Panel → g1_rviz_plugins/CameraPanel**.
Para compilarlo después de actualizar el repositorio:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select g1_rviz_plugins --symlink-install
```

`setup.sh` lo compila junto a los demás paquetes. Reiniciar el lanzamiento
carga el plugin desde `install/setup.bash`. Las esferas se reemplazaron por este
panel. El panel publica `/camera/pitch_command` (`std_msgs/Float64`); el puente
reenvía la consigna al sensor y publica `/camera/pitch_state` desde las imágenes.

RGB y profundidad usan el mismo ángulo. Cada imagen transporta el ángulo de su
captura y el puente publica TF dinámico de `d435_link` al frame óptico con ese
mismo timestamp. No se modifica el cuello, el torso ni el LiDAR. Es un soporte
móvil virtual: no representa una articulación del G1 físico. Cambiar de vista
puede hacer que la percepción pierda temporalmente un objeto o su estabilidad.
La odometría RGB-D también puede perder seguimiento durante cambios bruscos;
para explorar ángulos conviene mantener el robot detenido y usar odometría
independiente de la cámara.

### Vista de profundidad

**D435i Depth** muestra ahora `/camera/depth/preview`: una imagen RGB de
profundidad con escala fija de **0.2 a 6 m**, de azul (cerca) a rojo (lejos).
El violeta indica profundidad inválida o fuera de ese rango. La leyenda incluye
el porcentaje sin lectura; un fondo lejano no cambia el contraste de los
objetos cercanos. El tópico métrico `/camera/depth/image_rect_raw` permanece en
`32FC1` y sigue siendo la entrada de percepción/odometría. La vista previa no
inventa mediciones cuando la cámara apunta fuera del alcance o hacia el robot.
