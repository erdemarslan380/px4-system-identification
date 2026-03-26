Place the latest full QGroundControl parameter dump for your real vehicle here.

Recommended filename
- `current_vehicle.params`

This repository uses that file for two jobs:
- restoring calibration values after a firmware update
- carrying baseline vehicle parameters into the identification workflow when needed

Create reusable restore files
```bash
cd ~/px4-system-identification
python3 experimental_validation/calibration_restore.py \
  --input experimental_validation/qgc/current_vehicle.params \
  --out-dir experimental_validation/qgc/restore
```
