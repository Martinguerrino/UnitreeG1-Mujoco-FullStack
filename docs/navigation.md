# Nav2 y SLAM en el escenario Maxwell

Integración inicial, pendiente de validar extremo a extremo con Nav2 instalado.
La locomoción larga tampoco tiene aún una validación de estabilidad completa.

Validación local del 2026-09-19: 25 tests existentes y 3 tests nuevos pasan.
`scripts/check_navigation_bridge.py` verificó MuJoCo + DDS + ROS con un comando
real de `/cmd_vel`: desplazamiento de 0.2796 m, cero caídas/watchdogs y recepción
de 69 escaneos y 328 mensajes de odometría/reloj. Reporte físico:
`results/navigation/20260919T203402_820d64a7/metrics.json`.
No incluye planificación Nav2 ni mapeo SLAM: falta instalar esos paquetes
(la instalación automática se detuvo porque sudo requiere contraseña).

El flujo es RViz → NavigateToPose → planificador Nav2 → controlador Nav2 →
cmd_vel → política RL Lab original → LowCmd DDS → MuJoCo. Nav2 calcula los caminos;
la política produce las consignas articulares. No se usan los waypoints del
test mesa-a-mesa para conducir esta modalidad.

SLAM Toolbox construye `/map` y publica `map → odom`. El puente publica
`odom → base_link → laser`, `/odom`, `/clock` y `/scan`. Todos los nodos Nav2
y SLAM usan tiempo simulado. ROS usa Python del sistema y FastDDS en dominio
43; SDK2 permanece en el proceso Conda y dominio 42. Los dos procesos se
comunican por UDP exclusivamente en 127.0.0.1 (19420/19421). Ejecutar una sola
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
- El láser virtual estabilizado usa rayos físicos a 0.78 m de altura, 360 rayos,
  10 Hz simulados y 20 m de alcance. Sólo ve geometría del grupo 2 del escenario
  Maxwell. Esta altura detecta las superficies de ambas mesas; un escáner 2D
  puede omitir obstáculos bajos y desniveles. No usar para validar rampas.
- Se utiliza una base planar para navegación; la política controla el balance
  completo del humanoide. La huella circular de 0.4 m es una aproximación.
- Los parámetros efectivos de Nav2 se generan desde los defaults de la versión
  Jazzy instalada y se guardan en `results/navigation/nav2_*.yaml`.
- Falta validar formación del mapa, planificación, cancelación, llegada y
  estabilidad con el stack completo. Los tests del sensor/puente por sí solos
  no demuestran navegación autónoma exitosa.

Referencia: https://docs.nav2.org/jazzy/tutorials/general_tutorials/navigation2_with_slam/navigation2_with_slam/
