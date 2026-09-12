# Arquitectura del workspace

`g1_description` es la fuente geométrica para ROS. Contiene links, joints,
límites, masas, meshes e interfaces de hardware. No inicia procesos.

`g1_mujoco` es la fuente física: contactos, actuadores, sensores, gravedad,
solver y escena. Mantenerla separada permite reemplazar MuJoCo sin cambiar las
interfaces superiores.

`g1_control` define cómo ROS utiliza las interfaces. El controlador de cuerpo
completo recibe trayectorias y escribe esfuerzo. `arm_pose.py` es un cliente, no
un controlador: pide una trayectoria y espera el resultado de la acción.

`g1_bringup` compone la aplicación. Es el único lugar que conoce todos los
paquetes y ofrece el comando central pedido por el plan.

La carpeta `src/vendor` se crea desde `dependencies.repos`. Está ignorada como
dependencia descargada y nunca debe editarse para implementar lógica propia.

