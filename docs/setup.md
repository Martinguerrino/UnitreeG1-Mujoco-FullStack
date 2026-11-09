# Instalación desde un clon limpio

El punto de entrada es [`../setup.sh`](../setup.sh). Ejecutarlo como usuario
normal en Ubuntu 24.04 amd64; APT pide sudo cuando lo necesita.
Internet, espacio para dependencias/modelos y una pantalla/controlador gráfico
son necesarios para las ventanas de MuJoCo y RViz. El setup no instala drivers
de GPU ni abre automáticamente una simulación.

```bash
bash setup.sh
```

Prepara ROS 2 Jazzy, MoveIt/ros2_control, Nav2/SLAM Toolbox, el entorno Python,
bridge/políticas DDS, assets Dex1, percepción YOLO, FAST-LIO2/RTAB-Map/TSDF,
exportación de video y ACT/Okura. Finalmente compila y comprueba importaciones,
paquetes ROS y carga del modelo ONNX. La instalación puede tardar y descargar
varios GB; los checkpoints experimentales son una parte importante del tamaño.

Si ROS no está instalado, configura su repositorio con `ros2-apt-source` siguiendo
la [instalación oficial de Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
No hace una actualización general de Ubuntu.

## Elegir Python

Python debe ser 3.12 y pertenecer a un entorno Conda o virtualenv. Se puede usar
cualquier nombre de entorno y cualquier instalación de Conda:

```bash
conda create -n robot-g1 python=3.12 pip -y
conda activate robot-g1
bash setup.sh --python "$CONDA_PREFIX/bin/python"
```

Sin `--python`, usa `G1_PYTHON`, el entorno activo, la configuración local previa
o `.venv`; si no hay ninguno, crea `.venv` con el Python del sistema.
No modifica `.bashrc`, ni instala paquetes pip en el Python global de Ubuntu.
Los paquetes del entorno elegido sí se instalan/actualizan según los requisitos.
El intérprete queda registrado en `.g1-runtime.json`, que no se publica en Git.

Después, `bash scripts/navigation.sh` usa ese intérprete para MuJoCo. ROS usa
el Python del sistema para conservar sus bibliotecas compatibles con Jazzy.
`source scripts/activate.sh` pone el Python elegido en PATH para los comandos
de desarrollo; no requiere un nombre fijo de entorno.

Para cambiar la selección sin instalar nuevamente:

```bash
/usr/bin/python3 g1_backend/python_runtime.py --save /ruta/a/mi/entorno/bin/python
```

Los lanzamientos también aceptan `G1_PYTHON=/ruta/python`; el launch de navegación
conserva la opción explícita `sim_python:=/ruta/python`.

## Opciones

```bash
bash setup.sh --help
bash setup.sh --dry-run
bash setup.sh --check
bash setup.sh --skip-experiments  # no descarga ACT/Okura
bash setup.sh --skip-slam3d       # conserva navegación 2D y percepción
```

`--skip-system` omite APT; requiere que los paquetes del sistema ya estén
instalados. En ese modo rosdep comprueba las dependencias faltantes sin
instalarlas. `--skip-build` permite posponer colcon.
El instalador se detiene en el primer error y puede volver a ejecutarse después
de corregirlo. Las descargas DDS/Dex1 se verifican y reutilizan cuando coinciden.

YOLO se instala desde el ONNX de `assets/perception/`, con licencia AGPL-3.0 y
SHA-256 en su manifiesto. Para entrenar otra vez:

```bash
bash scripts/setup_perception_yolo.sh --train
```

## Ejecutar

```bash
bash scripts/navigation.sh perception:=true
bash scripts/navigation.sh --slam rgb-d perception:=true
bash scripts/navigation.sh --slam lidar3d perception:=true
bash scripts/sim.sh moveit:=true rviz:=true
```

Ejecutar una simulación por vez. Instalar todo no significa que todas las
funciones experimentales estén validadas; consultar el estado en el README.

Si MuJoCo se cierra al abrir la ventana desde VS Code Snap con un error de
`GLib-GIO` sobre la clave `antialiasing`, revisá el entorno heredado del IDE.
`scripts/navigation.sh` y `scripts/activate.sh` cargan `host_gui_env.sh`, que
elimina esquemas/módulos de Snap y recupera los directorios de datos del host.
La limpieza conserva las variables de selección de OpenGL, NVIDIA y CUDA.
Esto evita que un fallo de GTK cierre el simulador antes de iniciar el SLAM.

## Publicación del repositorio

`.gitignore` excluye `.venv`, configuración local de Python, dependencias y
pesos descargados en `artifacts`, compilaciones, registros, resultados, videos,
cachés y carpetas locales del editor/agente. No hay que subir esas carpetas.
Se conservan el modelo ONNX de distribución, sus licencias, código, pruebas,
documentación y el mapa de ejemplo que usa la guía de localización.
