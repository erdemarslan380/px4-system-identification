# Baseline HIL Quickstart

Bu akışta yalnız baseline PX4 kullanılır.

- `custom_pos_control` kapalı
- `trajectory_reader` kapalı
- `jMAVSim` görünür çalışır
- `CDC` yalnız `jMAVSim` içindir

## 1. Temiz baseline GUI oturumu

```bash
/home/earsub/px4-system-identification/examples/start_hitl_px4_baseline_gui.sh
```

Gerekirse portu açık ver:

```bash
/home/earsub/px4-system-identification/examples/start_hitl_px4_baseline_gui.sh ~/PX4-Autopilot-Identification /dev/ttyACM1 921600
```

## 2. Otomatik baseline hover testi

`jMAVSim` açıkken:

```bash
python3 /home/earsub/px4-system-identification/examples/run_hitl_px4_baseline_hover.py --endpoint udpin:127.0.0.1:14550 --hover-z -2.0
```

## 3. Manual takeoff -> offboard hold

`jMAVSim` açıkken:

```bash
python3 /home/earsub/px4-system-identification/examples/run_hitl_px4_manual_offboard_hold.py --endpoint udpin:127.0.0.1:14550 --trigger-z -0.8
```

Beklenen akış:

1. RC ile `Position` modda arm et
2. elle yüksel
3. `z <= -0.8` olunca helper `OFFBOARD`a geçer
4. o anki konumu hold eder

## 4. Log

PX4 logger path’i uçuş sırasında terminale yazdırılır:

- örnek: `/fs/microsd/log/2026-04-02/07_11_39.ulg`
