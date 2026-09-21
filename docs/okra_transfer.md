# Traslado experimental con Okura ACT

El modo `transfer` reproduce un objeto alargado tipo okra, de aproximadamente
10 cm y 13 g, una bandeja negra elevada y una cesta blanca. Las dimensiones,
materiales y cámara son estimaciones de las demostraciones; no una réplica
calibrada del montaje de entrenamiento. El objeto tiene articulación libre y
se mueve exclusivamente mediante contactos y gravedad.

```bash
conda activate mujoco-unitree
MUJOCO_GL=egl python -m experiments.okura.control --mode transfer --output results/mi_traslado
```

Usar un directorio de salida nuevo. Requiere los assets Dex1, checkpoint ACT y
runtime DDS preparados según [Dex1 + ACT](dex1_act.md).
La configuración está en `configs/okura_transfer.yaml`.

## Secuencia implementada

1. ACT original controla los brazos y las pinzas con imagen de 640 × 480.
2. Contacto de ambos dedos derechos y elevación de 3 cm durante 0,3 s habilitan
   el cambio de controlador. Se conservan las posiciones de los brazos y se
   mantiene cerrada la pinza derecha; ACT deja de enviar acciones.
3. Después de 1 s de retención, se libera el soporte de pelvis y se activa la
   política de locomoción. Tras 2 s, comienza el recorrido hacia la segunda mesa,
   retrocediendo primero para alejarse de la mesa inicial.
4. Sobre la segunda mesa se baja el brazo mediante su Jacobiano y se abre la
   pinza al llegar a la superficie. El éxito requiere contacto con esa mesa,
   pinza abierta, ausencia de contacto con la mano y objeto estable durante 0,5 s.

La pérdida del agarre, caída del robot o vencimiento de un plazo abortan el
episodio. El observador usa posiciones y contactos internos de MuJoCo para
verificar y navegar: todavía no es un sistema autónomo basado solo en cámara.
La combinación de locomoción con brazos retenidos es experimental y no equivale
a una política entrenada para transportar objetos.

## Procedencia y límites de reproducción

El [dataset público](https://huggingface.co/datasets/sotata/okura-pick-06102026)
explica que el checkpoint original se entrenó con 16 dimensiones. Esa colección
fue reemplazada por otra de 8 dimensiones del brazo derecho; los datos originales
quedaron archivados fuera de Hugging Face. Se conserva el checkpoint de 16
dimensiones y su normalización. Solo se toma el estado inicial derecho de la
colección pública, revisión `b47cc2ec337360154a4b8e44ded18ba40da520e3`;
el izquierdo parte de la media del checkpoint.

Los tests de máquina de estados comprueban las condiciones de transición y aborto.
La prueba mecánica comprueba retención y liberación de un elipsoide de 13 g con
contactos físicos. Ninguna de estas pruebas demuestra un agarre autónomo de ACT
ni un traslado completo. Los resultados físicos se registran por separado en
`docs/okra_transfer_validation.json`.

Cada ejecución guarda vídeo, imágenes, entradas de ACT, trayectoria, configuración,
procedencia y eventos. `report.completed` indica que terminó el ejecutable;
el éxito de la tarea se consulta en `report.transfer.success`.
