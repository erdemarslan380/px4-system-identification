Place the latest full QGroundControl parameter dump for your real vehicle here.

Recommended filename:
- current_vehicle.params

The optimization and experimental-validation tools can reference this file as a baseline parameter snapshot after firmware updates.

To generate reusable restore files after an upgrade:

```bash
cd ~/px4-custom
python3 experimental_validation/calibration_restore.py \
  --input experimental_validation/qgc/current_vehicle.params \
  --out-dir experimental_validation/qgc/restore
```
