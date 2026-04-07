# Reidentified Circle Determinism Probe

Same `reidentified_sitl + circle` run repeated 10 times in headless mode.
Landing was skipped after tracking-log collection to measure trajectory-window repeatability only.

| repeat | RMSE3D [m] | mean XY [m] | max XY [m] | max XY progress | first timestamp [us] |
|---:|---:|---:|---:|---:|---:|
| 1 | 0.475 | 0.442 | 0.755 | 0.21 | 64300000 |
| 2 | 0.477 | 0.441 | 0.796 | 0.21 | 63096000 |
| 3 | 0.459 | 0.425 | 0.751 | 0.21 | 62380000 |
| 4 | 0.460 | 0.425 | 0.784 | 0.21 | 62724000 |
| 5 | 0.458 | 0.425 | 0.687 | 0.21 | 62532000 |
| 6 | 0.468 | 0.435 | 0.753 | 0.21 | 67528000 |
| 7 | 0.473 | 0.439 | 0.755 | 0.21 | 66364000 |
| 8 | 0.458 | 0.424 | 0.759 | 0.21 | 63888000 |
| 9 | 0.469 | 0.434 | 0.782 | 0.21 | 62784000 |
| 10 | 0.482 | 0.447 | 0.784 | 0.21 | 63592000 |

RMSE mean: `0.468 m`
RMSE median: `0.469 m`
RMSE std: `0.008 m`
RMSE range: `0.458..0.482 m`
Bitwise-identical CSVs: `False`
