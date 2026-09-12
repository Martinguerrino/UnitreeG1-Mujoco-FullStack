# Roadmap con puertas de calidad

La siguiente fase no es “todo MoveIt”. Primero hay que ejecutar y registrar:

1. Arranque headless fijo durante al menos 60 segundos sin NaN ni inestabilidad.
2. Ambos controladores activos y `/joint_states` a aproximadamente 100 Hz.
3. Error de tracking aceptable en `home` y `reach_forward`.
4. TF válido desde `world` hasta ambas manos y ambos pies.

Después se agrega MoveIt para los brazos, con SRDF, límites cinemáticos y una
mesa como collision object. Solo entonces tiene sentido diseñar Pick/Place.

Para Pick hace falta seleccionar hardware de mano. El G1 base incluido termina
en manos rígidas; una acción que diga “success” sin contacto y cierre reales
sería una demo falsa.

La locomoción llegará después mediante una interfaz `vx`, `vy`, `yaw_rate`. La
policy queda detrás de esa interfaz para poder cambiar entre una policy oficial,
una propia o un controlador de prueba sin reescribir GoTo.

BehaviorTree, percepción, VLM y RL permanecen fuera del loop articular. Esa
separación limita fallos y hace cada experimento reemplazable.
