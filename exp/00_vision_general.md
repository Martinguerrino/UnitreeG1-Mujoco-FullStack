# Visión general

Este repo busca enseñar una arquitectura robótica, no ocultarla detrás de una
única policy. Cada capa tiene un contrato comprobable:

```text
                 g1_bringup
                      │
        ┌─────────────┼─────────────┐
        ▼             ▼             ▼
 g1_description   g1_control    g1_mujoco
 URDF + TF        trayectorias   MJCF + física
        └─────────────┬─────────────┘
                      ▼
              mujoco_ros2_control
```

Hoy se puede pedir una trayectoria articular y observar el estado resultante.
Ese ciclo cerrado es el cimiento de MoveIt, Pick/Place y locomoción. Agregar esas
capas antes de comprobarlo produciría errores difíciles de localizar.

Una decisión importante es usar el G1 oficial de 29 DOF sin manos articuladas.
Los extremos son manos de goma rígidas. Permite aprender control de brazos y
planificación; una fase Pick real requerirá elegir y modelar Dex1, Dex3 u otra
mano, decisión que no conviene fingir con un gripper inexistente.

