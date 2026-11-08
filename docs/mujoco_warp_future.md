# Migración futura de física a MuJoCo Warp

Estado: propuesta, fuera del backend CUDA actual. La física sigue usando MuJoCo
CPU (`mj_step`). El raycasting CUDA nuevo solo calcula mediciones; no integra
movimientos ni resuelve contactos.

## Qué cambiaría

MuJoCo Warp es otra implementación del simulador, con cálculos en GPU. Su mayor
interés para este repositorio son muchos entornos paralelos: evaluaciones,
entrenamiento de políticas y barridos de parámetros. Una simulación individual
con ROS y lecturas CPU no tiene una mejora garantizada.

La intención es conservar las ecuaciones y el modelo físico. No se promete
identidad paso a paso: precisión numérica, orden de operaciones, capacidades del
solver y funciones soportadas pueden producir diferencias. Contactos de pies,
equilibrio y agarres requieren validación específica.

## Dependencias y aislamiento

- Crear un entorno opcional propio y fijar versiones compatibles de MuJoCo,
  MuJoCo Warp y NVIDIA Warp. No reemplazar el entorno CPU.
- Leer la matriz de compatibilidad vigente antes de elegir versiones.
- Conservar los artefactos MJCF, URDF, políticas y checksums existentes.
- Incorporar un adaptador de simulador separado, seleccionado explícitamente
  durante la etapa experimental. No activarlo automáticamente hasta validarlo.

## Plan de implementación

1. Inventariar en los MJCF contactos, constraints, equalities, actuadores,
   sensores, mallas, cuerpos articulados y geometrías de las escenas importadas.
2. Cargar el mismo modelo CPU y transferir modelo/estado al dispositivo; definir
   presupuestos de contactos y memoria para el G1 y las escenas Maxwell.
3. Exponer el mismo contrato de estado, comandos, timestamps, watchdog y resets.
4. Mantener comandos DDS y ROS fuera del dispositivo; medir el coste de copias.
5. Adaptar IMU, sensores articulares y estimación de pies. Las lecturas deben
   conservar su fase respecto de la integración (`mj_step` actualmente entrega
   sensores correspondientes al comienzo del paso).
6. Revisar Dex1, grippers, transferencias y equalities. Comparar fuerzas y
   contactos, no solo posiciones finales.
7. Integrar cámaras y LiDAR con poses del nuevo simulador, sin introducir verdad
   simulada en los algoritmos de odometría/SLAM.
8. Implementar reset y recuperación CPU usando un snapshot válido completo;
   no continuar desde un estado parcialmente integrado tras un error GPU.
9. Añadir rollouts por lotes independientes del lazo ROS de un único robot.

## Validación necesaria

Comparar con igual timestep, modelo, política, seed y comandos:

- Robot inmóvil: posición, velocidad, torque y fuerzas normales.
- Caminata y giro: deriva, caídas, estabilidad y tracking de velocidad.
- Contactos: penetración, fricción, impacto y secuencia de apoyos.
- Manipulación: trayectoria de brazos, cierre, agarre, transporte y liberación.
- Sensores: fases, timestamps, frames y unidades.
- Reinicio, pérdida de comandos, fallo de GPU y recuperación CPU.
- Rendimiento para 1, 8, 32 y más entornos, midiendo warmup, p95/p99, memoria y
  tiempo total incluyendo sincronización y transferencias.

La aceptación debe usar tolerancias físicas justificadas, además de resultados
funcionales. Un test de carga del modelo no demuestra equivalencia física.

Documentación oficial:
https://mujoco.readthedocs.io/en/stable/mjwarp/index.html
