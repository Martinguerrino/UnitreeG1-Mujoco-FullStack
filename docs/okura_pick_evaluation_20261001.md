**Resultado del 1 de octubre de 2026: 0 agarres en 5 episodios completos de ACT, de 20 segundos simulados cada uno.** La prueba mecánica de Dex1 pasó. Esto evalúa nuestro montaje aproximado; no demuestra que el checkpoint falle en su montaje de entrenamiento.

Se ejecutaron los pesos originales de `sotata/act-okura-pick-06102026`, SHA-256 `e15b0dfdbadfc93fa12d20caee0f69d2bc39b32be1e52ed5749326f1648708fc`, conservando los normalizadores. El [autor](https://huggingface.co/sotata/act-okura-pick-06102026) especifica una cámara RGB 640×480, estado de 16 dimensiones y bloques de 100 acciones. La pelvis permaneció fija. El objeto tipo okra, de 13 g, tuvo articulación libre y se movió por contactos y gravedad.

El éxito exigió contacto de ambos dedos de una misma mano y elevación de más de 5 cm durante al menos 0,5 s. El fin del ejecutable no se contó como éxito.

| Ensayo | Actualización nominal | Máximo de dedos en contacto | Elevación máxima | Resultado |
|---|---:|---:|---:|---|
| Bandeja, soportes y cesta existentes | 100 acciones | 1 | 0,349 mm | Sin agarre |
| Mesa lisa, objeto x=0,43 / y=-0,13 m | 100 acciones | 1 | 0,754 mm | Empuja el objeto y cae de la mesa |
| Misma mesa y objeto | 30 acciones | 0 | 0 mm | Sin agarre |
| Misma escena, cámara inclinada 60° | 30 acciones | 0 | 0 mm | Sin agarre |
| Cámara 60°, objeto visible x=0,48 / y=-0,23 m | 30 acciones | 0 | 0 mm | Sin agarre |

Los cuatro ensayos finales se ejecutaron en serie, tras corregir la publicación del estado de pinzas. No hubo comandos inválidos, recortes de límites articulares, watchdog corporal inesperado ni pasos con comandos de pinzas vencidos. Cada episodio guardó 201 cuadros de video y una trayectoria numérica finita. Son pruebas de diagnóstico, una por configuración; no estiman una tasa de éxito poblacional.

Las pinzas abrieron hasta 5,4 rad de motor, cerraron hasta 0 y reabrieron. La pérdida deliberada de comandos activó el watchdog y la amortiguación. Las pruebas existentes de contacto y retención de Dex1 también pasaron; esta prueba de apertura/cierre no es un pick autónomo.

En el ejecutable experimental, el estado de Dex1 se publicaba desde el bucle físico, que también renderizaba. Dos intentos simultáneos se interrumpieron por estado DDS atrasado a los 8,002 y 8,802 s. Se conservan como ensayos de infraestructura y se excluyen de los cinco episodios completos. Se agregó un publicador periódico independiente, que lee el estado físico bajo el mismo mutex. El timeout de comandos sigue activo y se volvió a probar. Otros dos intentos de preparación fallaron antes de ejecutar ACT por depender de una mesa de destino ausente; también se excluyen.

La ejecución corporal funciona, pero no reproduce perfectamente las acciones. En los cuatro episodios finales, el percentil 95 del error articular de brazos fue 0,056–0,063 rad (aproximadamente 3,2–3,6°). Para la pinza derecha fue 0,101–0,123 rad de motor, equivalente a unos 0,82–1,00 mm de desplazamiento prismático. Estas cifras describen seguimiento de objetivos aplicados, no precisión de la punta de la pinza respecto del objeto. El limitador de velocidad también puede separar los objetivos aplicados de las acciones originales de ACT.

La medición geométrica refuerza el diagnóstico de alineación. En la mesa lisa con actualización de 30 acciones, a los 18,1 s, las superficies de los dos dedos derechos quedaron a 9,36 y 3,60 mm del objeto. Con cámara a 60°, la mejor aproximación simultánea dejó 8,84 y 14,90 mm. Con el objeto desplazado y visible, los dedos no llegaron: la mejor pareja simultánea dejó 51,06 y 118,14 mm. Son distancias firmadas entre superficies de colisión, reconstruidas desde `physics_qpos` a 10 Hz, incluyendo el tallo; no reemplazan los contactos medidos en cada paso físico.

La normalización no mostró la explosión anterior de la pinza izquierda: su entrada normalizada permaneció en cero. El máximo absoluto de las características normalizadas de estado fue menor a 2,5 en todos los episodios completos. Esto descarta aquel error concreto; no certifica que la distribución conjunta de imágenes y estados coincida con el entrenamiento.

Una prueba offline mantuvo fijo el estado y cambió únicamente la imagen. La salida cambió: la imagen de cámara a 60° produjo una diferencia RMS de 0,092 rad en brazos y 1,175 rad de motor en pinza derecha frente a la referencia. Esto demuestra sensibilidad a la imagen, no localización correcta del objeto. La imagen negra se utilizó solamente como diagnóstico, nunca para controlar el robot.

**Qué cambiar a continuación, en orden:**

1. Calibrar la cámara contra las demostraciones: posición respecto del torso, inclinación, campo de visión, orientación RGB y escala del objeto en píxeles. El ensayo de 60° modifica un solo parámetro; no es una calibración. Comparar cuadros reales y simulados con referencias geométricas identificables.
2. Reconciliar el estado inicial completo de 16 valores. Hoy el brazo derecho parte de un episodio público y el izquierdo de la media del checkpoint. Recuperar el reset original y verificar el montaje del robot, la altura de mesa y el marco de la pinza. La documentación del repositorio registra que el dataset público de 8 dimensiones no equivale al original de 16.
3. Medir y reducir el error de seguimiento mediante una trayectoria de referencia: comparar cinemática ideal y servo físico, ajustar ganancias y amortiguación respetando límites de esfuerzo, y medir el efecto del límite de 1,5 rad/s. Unos milímetros importan para este objeto. No atribuir automáticamente toda la desviación a ganancias: la cámara y la trayectoria predicha también cambian el resultado.
4. Revisar la aplicación temporal de ACT. La inferencia tarda alrededor de 0,6 s de host en estos ensayos. Durante ese tiempo continúa el bloque anterior y el nuevo bloque empieza después de la observación. Registrar edad de imagen y desfase observación→acción; comparar compensación del desfase y agregación temporal con el procedimiento original. Actualizar después de 30 acciones no resolvió el agarre en nuestra escena.
5. Si el montaje ya está alineado y sigue fallando, recoger demostraciones en esta simulación y ajustar ACT. Incluir las escenas, estados iniciales y objetos que realmente queremos manipular. Una tarea de cooperación entre dos manos requiere demostraciones de esa cooperación.

No hay evidencia para priorizar un aumento de fricción o fuerza: falta contacto bilateral sostenido, antes de llegar al problema de retención. Tampoco conviene integrar transporte o Nav2 como siguiente prueba de manipulación hasta resolver el pick aislado.

Para reproducir un episodio con mesa lisa, en un directorio nuevo:

```bash
conda activate mujoco-unitree
MUJOCO_GL=egl python -m experiments.okura.control \
  --mode pick --transfer-config configs/okura_pick_eval.yaml \
  --duration 20 --execute-steps 30 --domain 54 \
  --output results/okura_pick_nuevo
```

Agregar `--camera-pitch 60` compara inclinación; usar `configs/okura_pick_eval_visible.yaml` desplaza el objeto fuera de la pinza inicial. `--mode pick` mantiene la pelvis fija y no activa traslado. `hand`/`claw` en los lanzadores generales no inicia ACT; este ejecutable usa Dex1 explícitamente.

El análisis se repite con:

```bash
python -m experiments.okura.analyze results/okura_pick_nuevo \
  --output results/okura_pick_nuevo/diagnostics.json
# Matplotlib está instalado en el Python ROS de esta máquina:
MPLCONFIGDIR=/tmp/okura_matplotlib /usr/bin/python3 -m experiments.okura.plot \
  results/okura_pick_nuevo --output results/okura_pick_nuevo/comparison.png
```

Los resultados persistentes están en [okura_pick_evaluation_20261001.json](okura_pick_evaluation_20261001.json). Videos, entradas exactas, acciones, trayectorias, hashes y scripts de diagnóstico quedan en `results/okura_eval_20261001_*`. El gráfico es `results/okura_eval_20261001_comparison.png`. `results/` no se versiona. Validación de código: 20 tests de Okura y Dex1 pasan, Ruff y `git diff --check` pasan.
