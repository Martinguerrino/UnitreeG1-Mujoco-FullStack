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
