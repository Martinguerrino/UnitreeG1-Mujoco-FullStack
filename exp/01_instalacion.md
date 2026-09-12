# Instalación y entorno

Hay dos administradores de dependencias porque resuelven problemas distintos:

- Conda (`mujoco-unitree`) contiene Python científico, el binding oficial de
  MuJoCo y herramientas de tests. Se evita instalar toolkits GUI duplicados;
  MuJoCo usa su viewer y RViz proviene de ROS/APT.
- APT contiene ROS 2 Jazzy, `rclpy`, `ros2_control` y binarios compilados para
  Ubuntu 24.04. No se mezclan estos binarios dentro de Conda.

## Activar Conda

```bash
source /home/martin/miniforge3/etc/profile.d/conda.sh
conda activate mujoco-unitree
```

El archivo `environment.yml` es la receta. Si el entorno se pierde:

```bash
conda env create -f environment.yml
```

Para actualizarlo tras cambiar la receta:

```bash
conda env update -n mujoco-unitree -f environment.yml --prune
```

## Dependencias del sistema

Instala ROS 2 Jazzy siguiendo la documentación oficial para Ubuntu Noble y luego:

```bash
./scripts/install_ros_dependencies.sh
./scripts/bootstrap.sh
```

El script muestra la operación y usa `sudo` solo para APT y la inicialización
global de rosdep. En esta máquina ROS base ya existe; faltan los paquetes de
control indicados y la contraseña debe introducirla el dueño del equipo.

`bootstrap.sh` prefiere el binario oficial. Si no existe, importa
`mujoco_ros2_control` en `src/vendor` usando el commit exacto de
`dependencies.repos`; después ejecuta `rosdep` y no modifica código propio.

## Flujo diario

```bash
source scripts/activate.sh
./scripts/build.sh
./scripts/sim.sh
```

`activate.sh` debe ejecutarse con `source`, porque un proceso hijo no puede
cambiar el entorno de la terminal padre.
