# Estado de validación

Al cerrar esta implementación se comprobó en la máquina del proyecto:

- El entorno Conda `mujoco-unitree` existe con Python 3.12.14 y MuJoCo 3.3.6.
- MuJoCo compila el MJCF y sus meshes: `nu=29`, `nq=36`, `nv=35`.
- URDF, MJCF y YAML contienen exactamente los mismos 29 joints controlados.
- Una simulación física de cinco segundos con pelvis fijada permanece finita y
  cerca de la pose inicial; sin fijación cae, como corresponde al no existir
  todavía un controlador de balance.
- Los cuatro paquetes de las fases 0–4 compilan con `colcon`.
- Pasan 5 tests de contrato/física y 34 tests ROS de lint/XML/licencia.
- `source scripts/activate.sh` activa Conda y ROS Jazzy correctamente.

Después de esa primera validación también se ejecutó el launch completo de las
fases 0–4: MuJoCo, los controladores, TF y RViz arrancan en este host. La fase 5
añade dos paquetes nuevos. Sus contratos estáticos están probados, pero para
compilarlos y validarlos en runtime falta instalar `ros-jazzy-moveit`; APT
requiere la contraseña personal de `sudo`.

El único paso manual pendiente es:

```bash
./scripts/install_ros_dependencies.sh
./scripts/build.sh
./scripts/sim.sh moveit:=true rviz:=true
```

Después conviene ejecutar primero `move_to_pose` con `--plan-only` y luego sin
esa opción. La fase 5 queda implementada en código, pero solo debe declararse
validada en runtime después de esa prueba.
