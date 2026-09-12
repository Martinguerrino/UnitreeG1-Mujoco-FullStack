# URDF y MJCF

URDF y MJCF describen el mismo robot con objetivos distintos.

URDF responde a “¿qué link está conectado con cuál?” y alimenta TF, RViz,
MoveIt y `ros2_control`. MJCF responde a “¿cómo se mueve y contacta físicamente?”
y alimenta MuJoCo.

Los nombres articulares son el contrato común. Si `left_elbow_joint` cambia en
un modelo pero no en el otro, un comando podría llegar al motor equivocado o no
llegar. `tests/test_model_contract.py` compara automáticamente los 29 nombres.

El G1 tiene base flotante: MuJoCo integra seis velocidades de la pelvis, pero
esas coordenadas no son motores. Por eso el modelo tiene `nq=36`, `nv=35` y
solo `nu=29` actuadores.

La pelvis sí se mueve. MuJoCo publica su pose como odometría y
`floating_base_tf.py` la convierte en el transform dinámico `odom → pelvis`.
El transform fijo `world → odom` completa un árbol TF conectado sin fingir que
la base permanece quieta.

Los meshes se guardan una sola vez. Al arrancar, `simulation.launch.py` crea en
`/tmp` un MJCF temporal y sustituye `meshdir` por la ruta instalada de
`g1_description`. El original versionado permanece inmutable y los paquetes no
dependen de dónde clonaste el repo.
