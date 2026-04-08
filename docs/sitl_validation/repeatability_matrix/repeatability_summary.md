# SITL 10-repeat trajectory matrix

Repetitions per model/trajectory: `10`

| Trajectory | Model | RMSE mean [m] | RMSE median [m] | RMSE std [m] | RMSE min..max [m] | Bitwise identical |
|---|---|---:|---:|---:|---:|---:|
| hairpin | Stock x500 SITL | 0.6153 | 0.6146 | 0.0053 | 0.6087..0.6222 | False |
| hairpin | Stock SDF exact | 0.6127 | 0.6129 | 0.0047 | 0.6050..0.6197 | False |
| hairpin | Stock SDF -100g | 0.6054 | 0.6061 | 0.0037 | 0.5982..0.6100 | False |
| hairpin | jMAVSim prior SDF | 0.3585 | 0.3590 | 0.0039 | 0.3513..0.3651 | False |
| hairpin | jMAVSim prior +100g SDF | 0.3652 | 0.3657 | 0.0025 | 0.3600..0.3683 | False |
| hairpin | Re-identified from SITL ident | 0.3591 | 0.3591 | 0.0031 | 0.3550..0.3666 | False |
| hairpin | Re-identified from +100g SITL ident | 0.3664 | 0.3651 | 0.0054 | 0.3597..0.3777 | False |
