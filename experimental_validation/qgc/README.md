Place the latest full vehicle parameter snapshot here.

Recommended filename:
- `current_vehicle.params`

This folder is for one practical job:
- keep one authoritative parameter snapshot for the real vehicle,
- regenerate the calibration and RC restore subset from that snapshot,
- rebuild firmware with that subset embedded as `rc.board_defaults`.

Recommended path: QGroundControl UI
1. open `Parameters > Tools > Save to file`,
2. save the file here as `current_vehicle.params`,
3. regenerate the restore files:
```bash
cd ~/px4-system-identification
python3 experimental_validation/calibration_restore.py \
  --input experimental_validation/qgc/current_vehicle.params \
  --out-dir experimental_validation/qgc/restore \
  --board-defaults overlay/ROMFS/px4fmu_common/init.d/rc.board_defaults
```

Fallback path if the board is already reachable over MAVLink:
```bash
cd ~/px4-system-identification
./examples/update_vehicle_calibration_snapshot.sh udpin:127.0.0.1:14550 57600
```

Outputs:
- `restore/restore_calibration.params`
- `restore/restore_calibration.nsh`
- `restore/rc.board_defaults`
