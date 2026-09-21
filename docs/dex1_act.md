# Dex1 + ACT Okura: ejecución física experimental

Se incorporaron **dos pinzas Dex1** al G1 de la escena experimental existente.
La apertura/cierre y la ejecución de ACT por DDS funcionan. **El agarre todavía
no funciona en los ensayos realizados**: ejecutar acciones válidas no acredita
que el checkpoint resuelva la tarea.

## Modelo y control

- Geometría, montaje a 0.0415 m de la muñeca, masas, inercias, ejes y límites
  proceden del [URDF oficial G1 con Dex1](https://github.com/unitreerobotics/unitree_ros/blob/7d6075f7f58588b189b940130e3edab3c839b2df/robots/g1_description/g1_29dof_mode_15_with_dex1_1.urdf).
  Se descargan cinco mallas y ese URDF, no el repositorio completo.
- `g1_backend/dex1.py` convierte esos elementos a MJCF. **Es una conversión local,
  no un MJCF publicado por Unitree**. Se quitan las manos de goma de esta escena.
  El modelo tiene 29 actuadores corporales y cuatro actuadores prismáticos,
  dos dedos por mano; ambos reciben el mismo objetivo de apertura.
- Motor 0 cierra; 5.4 abre, según el controlador oficial Dex1. El código de
  [Unitree Isaac Lab](https://github.com/unitreerobotics/unitree_sim_isaaclab/blob/e30c25b1dffdf92ada1d6c8c1fe9a47bdde0fecc/tools/data_convert.py)
  convierte eso a `q_isaac = 0.024 - 0.044 * motor_rad / 5.4`.
  **Los ejes del URDF tienen el sentido de apertura contrario**: sus mallas de
  colisión se separan al aumentar q. Por eso el adaptador aplica
  `q_urdf = -0.020 + 0.044 * motor_rad / 5.4 = 0.004 - q_isaac`.
  La prueba verifica la apertura contra los vértices reales de las mallas y la
  retención/liberación de un objeto libre. Copiar directamente la ecuación Isaac
  al URDF invierte abrir/cerrar, aunque el test de ida/vuelta de unidades pase.
  El transporte reproduce la cuantización del estado de Unitree: redondeo de
  la posición media de los dedos a 1 mm, conversión y redondeo a 0.001 rad.
  La posición física y la inversa continua siguen disponibles en `GripperPlant`;
  la cuantización solo afecta al mensaje DDS, no modifica articulaciones.
- Las manos usan `rt/dex1/{left,right}/{cmd,state}`, mensajes SDK2
  `MotorCmds_`/`MotorStates_`, independientes del LowCmd corporal.
  El servo local limita fuerza a 20 N por dedo y velocidad objetivo a 0.2 m/s.
  Es una aproximación de simulación, no identificación de la transmisión real.
- El bridge corporal oficial permanece intacto y verificado por hash. Recibe
  metadatos del modelo corporal original y datos del modelo físico extendido:
  los primeros 29 actuadores y todos los sensores corporales conservan el orden.
  Las pruebas verifican esa compatibilidad; las pinzas no desplazan el IMU.
- Tras 0.25 s sin comandos se aplica amortiguación. Solo el reset asigna `qpos`;
  durante la ejecución las fuerzas actúan mediante `ctrl`. La caja tiene una
  articulación libre y nunca se suelda a la mano.

## ACT y alcance

`experiments/okura/control.py` utiliza `OkuraPolicy.chunk(state, image)` en un
worker para que la inferencia CPU no interrumpa la publicación DDS. Las entradas
son las 14 posiciones medidas de brazos, dos posiciones medidas de pinzas y RGB
640×480 de `cam_left_high`. Conserva pesos y normalizadores originales.

Por defecto consume bloques de 100 acciones a 30 Hz de simulación, según el
checkpoint. `--execute-steps 30` permite probar horizonte recedente. Durante la
inferencia mantiene el último destino; no inventa acciones nuevas. Limita cambios
de destino de brazos a 1.5 rad/s y registra recortes de límites articulares.
La física apunta a 500 Hz, pero render e inferencia hacen que el reloj simulado
avance más lento que el real. El video usa tiempo simulado.

La política se selecciona con `--policy-factory modulo:constructor` y
`--checkpoint ruta`. El constructor recibe `(checkpoint, threads=2)` y expone
`chunk(state16, rgb) -> acciones100x16`. Se valida forma y finitud antes de actuar.
El modelo de manos y su transporte permanecen independientes de la política.

Este ejecutable valida manipulación aislada con pelvis fijada, reutilizando mesa,
caja y cámara del experimento Okura. **No habilita todavía pick/place en el Task
Executor general ni combina locomoción con manipulación**. La configuración
general `manipulation.yaml` sigue sin proveedor validado para esa tarea.

La cámara no está calibrada contra el entrenamiento, el objeto es una caja y
la postura inicial procede de la media estadística del checkpoint, no de un
reset documentado por su autor. Además persiste la discrepancia entre metadatos
públicos del dataset (8 dimensiones) y checkpoint/model card (16). Estas diferencias
impiden atribuir el fallo a una causa única; las pruebas no demuestran que el
checkpoint sea defectuoso. No se entrena ni se modifica la política.

## Reproducción

```bash
conda activate mujoco-unitree
python scripts/setup_dds.py
python scripts/setup_dex1.py
python -m experiments.okura.checkpoint --download
# Dependencias ACT: ver exp/10_okura_act.md; video: requirements-video.txt.
MUJOCO_GL=egl python -m experiments.okura.control --mode mechanical \
  --duration 7.2 --test-watchdog --output results/dex1_nuevo
MUJOCO_GL=egl python -m experiments.okura.control --mode act \
  --duration 15 --output results/act_nuevo
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests
```

Cada episodio requiere un directorio nuevo y proceso nuevo. DDS se limita a
loopback; `--domain` permite aislar procesos simultáneos. `configs/dex1.sources.json`
fija revisiones y SHA-256; el cargador comprueba los assets antes de compilar.

La salida contiene `report.json`, `trajectory.npz` (estado, objetivos, pinzas,
objeto/contactos), `policy.npz` (chunks originales), `input_*.npz` (entradas exactas
para repetir inferencia), imágenes inicial/final, `camera.mp4` y `provenance.json`.
Los ensayos anteriores a la ampliación del registro no incluyen entradas exactas
ni provenance. Los resúmenes persistentes están en `dex1_act_validation.json`.

El éxito exige **contacto con ambos dedos de una misma mano y elevación de la
caja de más de 5 cm durante al menos 0.5 s**. No se utiliza el fin de un chunk como
criterio de agarre. Los ensayos no validan traslado ni colocación.

## Pruebas de contacto y resultados

El test de geometría comprueba que el comando 5.4 realmente separa las superficies
de colisión más de 85 mm. Una prueba mecánica aislada cierra sobre la misma caja
de 6 cm y 100 g, inicialmente entre los dedos; activa gravedad tras el cierre,
exige sostenerla un segundo con menos de 5 mm de descenso y verifica que caiga
al abrir. Esta preparación corresponde al fixture de contacto, **no a ACT ni a
un pick autónomo**. No hay restricciones de igualdad sobre el objeto.

La escena Dex1 utiliza conos de fricción elípticos e `impratio=10`, siguiendo las
[indicaciones de MuJoCo para reducir deslizamiento](https://mujoco.readthedocs.io/en/3.3.3/modeling.html#preventing-slip).
Con los valores anteriores, la caja se deslizaba aun estando entre ambos dedos.
El ajuste mejora la resolución del contacto; no altera la masa, no aumenta la
fricción para forzar éxito y no añade una unión a la mano.

Los primeros ensayos detectaron y corrigieron dos fallos de integración: inversión
del sentido de apertura al reutilizar coordenadas Isaac, y deslizamiento del
solver. Sus reportes se conservan como diagnóstico y **no se cuentan como
validación del modelo final** (`results/dex1_SUPERSEDED.md`).

## Normalización y estado de la pinza izquierda

El checkpoint tiene media y desviación **cero** para la pinza izquierda. En el
ensayo de diagnóstico, un error de servo de aproximadamente 0.00064 rad se
transformaba en una entrada normalizada de más de 63.000 (división por epsilon).
Esto se detectó inspeccionando las observaciones efectivamente enviadas a ACT.

Se restableció el redondeo usado por el publicador oficial de estado Unitree:
posición prismática a 1 mm y valor de motor a 0.001 rad. Con ello el extremo cerrado
publica cero y la característica normalizada también es cero. No se altera el
normalizador ni se sobrescribe el estado físico. La inversa sin cuantizar continúa
disponible para pruebas mecánicas. El test verifica ambas representaciones.

La corrección amplía el movimiento de ACT, pero no garantiza un pick: quedan
sin calibrar imagen, postura inicial y objeto. Los ensayos anteriores al cambio
de encoder se conservan como diagnóstico, no como validación de esta versión.

## Validación final

El 13 de septiembre de 2026: **44 tests pasan (7.17 s)**; Ruff, formato y
`git diff --check` pasan. Se decodificaron los tres videos y se comprobaron las
entradas, los chunks finitos y los hashes contra el código final.

| Ensayo | Resultado |
|---|---|
| `dex1_encoder_mechanical` | Abre/cierra/reabre; pérdida deliberada activa watchdog y amortiguación |
| `okura_dex1_encoder_a`, caja x=0.40 m | 381 acciones, 4 chunks, 15 s simulados; sin agarre |
| `okura_dex1_encoder_b`, caja x=0.55 m | 799 acciones, 8 chunks, 30 s simulados; sin agarre |

**0/2 agarres autónomos**, 1.180 acciones ACT. No hubo comandos inválidos, recortes
articulares de brazos ni watchdog inesperado en los ensayos ACT. El brazo se
movió hasta 0.61 rad, pero no hubo contacto de los dedos con la caja. En el ensayo
A la caja subió transitoriamente 6.8 mm, sin sujeción: no cuenta como lift exitoso.
La característica normalizada de pinza izquierda quedó en cero en ambos ensayos.

La siguiente dependencia para avanzar con Okura es reproducir cámara, objeto y
postura de inicio de sus demostraciones y reconciliar el contrato del dataset.
La ejecución física está probada; pick/place autónomo permanece pendiente.
