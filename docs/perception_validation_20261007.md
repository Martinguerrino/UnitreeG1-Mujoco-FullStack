# Validación de percepción — 2026-10-07

Se ejecutó `scripts/evaluate_perception.py` sobre
`src/g1_mujoco/scenes/maxwellrobotics/g1_navigation_scene.xml`, con el G1,
la D435i montada y postura nominal de RL Lab. Resolución 424 × 240, semilla 42;
0.6/1.0/1.5 m desde el cubo rojo, ángulos -20/0/20 grados, ideal y ruido del
perfil existente: 18 configuraciones × 5 capturas = 90 observaciones.

No se ejecutó marcha. La geometría real y la segmentación se usaron únicamente
para evaluar; no alimentaron el detector ni el tracker. Cada configuración
comienza un tracker independiente. Los errores dimensionales comparan
geometría visible estimada con dimensiones completas del objeto.
La transformación de cámara es exacta en este evaluador: no se agregan errores
de calibración/odometría ni latencia de entrega.

## Resultados

| Distancia / perfil | Cajas visibles asociadas / referencias por captura | Candidatos extra | RMSE del centro de las cajas asociadas, filtrado |
|---|---:|---:|---:|
| 0.6 m / ideal | 15 / 30 | 5 | 0.39 mm |
| 0.6 m / ruido | 16 / 30 | 9 | 6.45 mm |
| 1.0 m / ideal | 0 / 15 | 5 | Sin asociaciones |
| 1.0 m / ruido | 5 / 15 | 4 | 15.89 mm |
| 1.5 m / ideal | 0 / 0 | 0 | Cajas fuera del campo de visión |
| 1.5 m / ruido | 0 / 0 | 0 | Cajas fuera del campo de visión |

Cada fila agrega tres ángulos y cinco capturas. Las referencias visibles se
cuentan si tienen al menos 20 píxeles en la segmentación, aunque la tapa puede
estar oculta. Las asociaciones usan un umbral de 20 cm; los errores sólo
resumen asociaciones aceptadas. Un error pequeño no compensa un fallo de
detección ni un candidato extra.

El cubo rojo se asoció en las 30 capturas a 0.6 m. La caja amarilla sólo se
asoció en una de esas 30 capturas; todavía no se acredita detección fiable de
ella. Los candidatos extra confirman que el detector geométrico no ofrece
clasificación semántica general. El estado `confirmed` indica repetición de
una asociación, no una garantía de que el objeto sea una caja.

Para el cubo rojo, a 0.6 m y 0 grados con ruido, la variación espacial de centro
(norma de desviaciones por eje) bajó de **0.630 mm a 0.190 mm** con el filtro.
En los tres ángulos cercanos con ruido no hubo cambios de ID del cubo rojo.
Son cinco muestras por configuración, no una estimación estadística general.
El filtro puede conservar sesgo y retrasar el seguimiento de un objeto móvil.

La mesa se asoció en 60 de 90 capturas. El RMSE de su centro fue 25.7 cm sin
filtrar y 26.3 cm filtrado, comparando la superficie visible con el tablero
completo. El recorte visual y otras superficies detectadas limitan la
interpretación. No se acredita reconstrucción completa del tablero ni mejora
de precisión absoluta por filtrar.

## Evidencia reproducible

- `results/perception/maxwell_evaluation/evaluation.json`: configuración,
  hash del escenario, métricas, visibilidad, fallos e IDs por captura.
- NPZ por configuración en la misma carpeta: último RGB, profundidad,
  calibración y transformación óptica.
- 20 pruebas de percepción satisfactorias, incluyendo ROS real en dominio 93:
  asociación uno a uno, ruido, filtro, simetrías, pérdida/reaparición/expiración,
  reinicio de reloj, movimiento del frame del robot, puntuación de errores,
  IDs estables y publicación de colores/etiquetas/ejes.

```bash
"$G1_PYTHON" scripts/evaluate_perception.py
```

Esto deja una herramienta para medir y localizar problemas del detector,
además del seguimiento y la visualización. Los límites detectados requieren
mejoras posteriores de segmentación, cobertura de cámara y reconocimiento;
no se ocultan ni se declaran resueltos por estas mejoras.
