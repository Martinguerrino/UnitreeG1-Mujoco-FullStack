# Referencias y decisión de integración

Auditoría de código y documentación del 2026-09-12. ACT queda aplazado por
instrucción posterior del usuario. No se entrena ni se incorpora un planner.

## Backend y comunicación

- [Unitree MuJoCo](https://github.com/unitreerobotics/unitree_mujoco), revisión
  `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d`: tiene ejecutables C++ y Python.
  Se usa su `simulate_python/unitree_sdk2py_bridge.py` como dependencia externa
  verificada. No se copia su repositorio al código fuente del proyecto.
  Publica `rt/lowstate`, `rt/sportmodestate`; recibe `rt/lowcmd`.
  G1 utiliza `unitree_hg`, no `unitree_go`, para mensajes low-level.
- [SDK2](https://github.com/unitreerobotics/unitree_sdk2), revisión
  `9754cd153af3da471b0fe5f3aa535e426fb11db3`: referencia de control oficial.
- [SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python), revisión
  `65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5`: se instalan sus mensajes,
  canales y CRC, sin reimplementar el SDK. El ejemplo G1 confirma PR=0,
  seguimiento de `mode_machine`, 29 motores y CRC de LowCmd.
- [Menagerie G1](https://github.com/google-deepmind/mujoco_menagerie/tree/main/unitree_g1)
  sigue siendo referencia alternativa de modelo. No se cambia el modelo
  oficial existente: ya tiene actuadores y sensores en orden SDK.

El bridge Python oficial aplica el PD al recibir un comando. El wrapper local
reaplica ese mismo método en cada paso físico, serializa acceso a MjData entre
callbacks y física y añade watchdog. Se corrige mediante inicialización en la
subclase la inconsistencia upstream entre `have_frame_sensor` y
`have_frame_sensor_`. El archivo oficial se conserva intacto y con SHA-256.
No se añade control de qpos/qvel durante episodios.

## Locomoción seleccionada

Se elige el ONNX **G1 29 DoF velocity/v0 de Unitree RL Lab**, revisión
`4960b84732b0c2ec593dccbfe963fda1bcd7b1e3`:

[checkpoint y parámetros de deployment](https://github.com/unitreerobotics/unitree_rl_lab/tree/4960b84732b0c2ec593dccbfe963fda1bcd7b1e3/deploy/robots/g1_29dof/config/policy/velocity/v0).

El árbol de esa revisión contiene realmente el checkpoint, no solo instrucciones
para entrenarlo. Se descargaron 1 658 204 bytes y ONNX Runtime confirmó entrada
`obs [1,480]` y salida `actions [1,29]`. La elección prioriza el contrato de 29
motores, deployment oficial SDK2 y sim2sim documentado. Isaac Lab no se instala.
Los archivos, revisiones y SHA-256 están en `configs/sources.lock.json`.

[Unitree RL MJLab](https://github.com/unitreerobotics/unitree_rl_mjlab), revisión
`1425b15f73bd4095f0df53709d7c389c3eb9e790`, también contiene G1 velocity/v0
ONNX y deploy.yaml. Es una alternativa válida; su mapa de joints es identidad
y sus ganancias/escalas difieren de RL Lab. No son checkpoints intercambiables
usando el mismo preprocesamiento. No se importa su framework de entrenamiento.

[G1 Locomotion RL](https://github.com/adamson-gh/g1-locomotion-rl), revisión
`9d98932afc9c3e19aaf3af868e072c8ca0f1bf33`, usa Gymnasium/SB3 PPO y Menagerie.
En el árbol consultado no aparecieron pesos `.zip`, `.pt` u `.onnx`; no se asume
que su README de evaluación provea un checkpoint descargable. Requeriría
reproducir sus observaciones y acciones y sustituir el acceso directo al entorno
por un adaptador DDS.

[Unitree-G1-RL](https://github.com/idan0405/Unitree-G1-RL), revisión
`a5f8f5f57f0338c8bf8ec297dd84dc3059043dce`, se conserva como referencia
secundaria PPO/locomoción. No se cargó ni validó su checkpoint. Ninguna de estas
alternativas desplaza al export oficial seleccionado por facilidad de carga.

## Manipulación: proveedor pendiente

[humanoid_vla](https://github.com/ozkannceylan/humanoid_vla), revisión
`3d4bf2f040d6cb9f867becf1dc1b97b9dc3bef12`, describe ACT, cámara egocéntrica
480×640 RGB, control a 30 Hz y física a 500 Hz; hay variantes de estado/acción
para tareas simples y bimanuales. Su flujo apunta a checkpoints `best.pt`
generados por sus scripts; no se localizó un checkpoint en el árbol consultado.
Su demostración y sus rutas cinemáticas no prueban compatibilidad SDK2.
No se incorporan NL, ROS task manager ni código de entrenamiento.

[Unitree LeRobot](https://github.com/unitreerobotics/unitree_lerobot), revisión
`41c2805742de879ddab2d8d6beaeaf215f876395`, se mantiene como referencia de
formatos, procesadores e inferencia. El proyecto ya tiene LeRobot opcional.

El [checkpoint Okura](https://huggingface.co/sotata/act-okura-pick-06102026)
está auditado en `exp/10_okura_act.md`: 16 estados/acciones, pinzas Dex1,
RGB 480×640 y chunks de 100. Eso no coincide con las manos rígidas actuales.
La instrucción posterior del usuario aplaza ACT: no se conecta al controlador,
no se cambia la escena ni se inventa otra política de manipulación.

`ManipulationPolicy` y el proveedor `module:factory` permiten integrar otra
política después. Configurar el proveedor no equivale a validar embodiment,
manos, cámaras ni éxito físico de agarre/colocación.
