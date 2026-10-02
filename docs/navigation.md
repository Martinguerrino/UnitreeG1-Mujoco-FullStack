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
El LiDAR sólo permite cada semionda si hay al menos 0.8 m
libres en el sentido correspondiente. Holosoma continúa disponible mediante
`configs/locomotion_holosoma.yaml`, pero ya no es la opción predeterminada.
Durante el vaivén, `angular_gain: 36.0` amplifica la orden angular de Nav2,
limitada a 1.0 rad/s y con una rampa de 4.0 rad/s². Ambas semiondas mantienen
una consigna de 1.0 rad/s tras la rampa mientras Nav2 siga pidiendo alineación,
independientemente de la fase longitudinal, de modo que tanto
el retroceso como el avance contribuyen al giro. La traslación usa pulsos de
coseno elevado idénticos en ambos sentidos, con pendiente cero al arrancar,
detenerse y cambiar de dirección. Una orden cero detiene la ayuda.
El asistente no recibe el objetivo ni mide directamente el error de alineación:
Nav2 continúa decidiendo cuándo termina el giro.
La rotación previa al seguimiento termina al quedar dentro de 90 grados de la
dirección local del camino (`rotate_to_heading_min_angle: 1.571`, equivalente
al 50% de una media vuelta). Desde ese
momento Pure Pursuit avanza y corrige el error restante mientras camina; el
vaivén se desactiva cuando la orden lineal de Nav2 supera 0.05 m/s.
La ayuda angular continúa durante el seguimiento: `tracking_angular_gain: 3.0`
multiplica la orden de giro, con el mismo límite de 1.0 rad/s y rampa de
4.0 rad/s². No aplica un mínimo angular durante la marcha: la corrección
disminuye proporcionalmente y una orden angular cero se respeta inmediatamente.
Igualar consignas no garantiza idéntico giro físico en ambas fases. Esta ayuda
modifica la curvatura calculada por Nav2; falta validar recorridos con obstáculos
y la llegada al objetivo con esta ganancia.
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
  completo del humanoide. La huella circular de 0.4 m es una aproximación.
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

# Nube geométrica XYZ: FAST-LIO2 + IMU de la parte superior del cuerpo.
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
`slam_output`. Ambos escriben **`map.pcd`** cada 30 segundos de simulación y
al cerrar normalmente. `rgb-d` conserva también **`rtabmap.db`**, con imágenes,
profundidad, trayectoria y grafo para reutilización/exportación mediante las
herramientas de RTAB-Map. El PCD visual conserva el campo RGB; no es una malla
texturizada. Al reutilizar explícitamente una carpeta existente RTAB-Map puede
abrir su base de datos anterior; usar una carpeta nueva para una sesión nueva.

En RViz, `SLAM 3D map` muestra `/slam/map_points`. Ambos motores proporcionan
`/odom` y las transformaciones estimadas para Nav2. `/map` es una proyección 2D
para planificación, adicional al mapa 3D; FAST-LIO la obtiene mediante rayos de
obstáculos medidos y RTAB-Map mediante su generador de ocupación.

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

FAST-LIO2 proporciona odometría LiDAR–inercial y mapeo incremental; **no añade
cierre de bucles global** en esta integración. La nube se voxeliza a 8 cm y se
limita a 400000 vóxeles para acotar memoria; al llegar al límite deja de añadir
celdas nuevas. La proyección 2D está acotada a ±50 m y está pensada para un
entorno estático. RTAB-Map sí realiza optimización y cierre de bucles, y su nube
exportada reemplaza el mapa anterior para respetar esas correcciones.

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
