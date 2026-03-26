"""Helpers to build repeated Gazebo system-identification study plans."""

from __future__ import annotations

from collections import OrderedDict
from pathlib import Path
from typing import Mapping


IDENTIFICATION_PROFILE_ORDER: tuple[str, ...] = (
    "mass_vertical",
    "hover_thrust",
    "roll_sweep",
    "pitch_sweep",
    "yaw_sweep",
    "drag_x",
    "drag_y",
    "drag_z",
    "motor_step",
)

DEFAULT_REPEAT_COUNTS: Mapping[str, int] = OrderedDict(
    (
        ("mass_vertical", 3),
        ("hover_thrust", 2),
        ("roll_sweep", 2),
        ("pitch_sweep", 2),
        ("yaw_sweep", 2),
        ("drag_x", 2),
        ("drag_y", 2),
        ("drag_z", 2),
        ("motor_step", 3),
    )
)


def build_identification_plan(
    *,
    plan_name: str = "x500_identification_comprehensive",
    results_root: str = "runs/x500_identification_comprehensive",
    rootfs: str = "build/px4_sitl_default/rootfs",
    repeats: Mapping[str, int] | None = None,
) -> dict:
    if repeats is None:
        repeat_map = OrderedDict(DEFAULT_REPEAT_COUNTS)
    else:
        repeat_map = OrderedDict((str(key), max(0, int(value))) for key, value in repeats.items())

    tasks: list[dict] = []
    for profile in IDENTIFICATION_PROFILE_ORDER:
        repeat_count = int(repeat_map.get(profile, 0))
        for repeat_index in range(1, repeat_count + 1):
            tasks.append(
                {
                    "id": f"x500_{profile}_r{repeat_index:02d}",
                    "controller": "sysid",
                    "traj_id": 0,
                    "mission_mode": "identification",
                    "ident_profile": profile,
                }
            )

    return {
        "name": str(plan_name),
        "rootfs": str(rootfs),
        "output_dir": str(results_root),
        "defaults": {
            "takeoff_alt": 2.0,
            "takeoff_timeout": 45.0,
            "trajectory_timeout": "auto",
            "sim_speed_factor": 1.0,
            "simulator": "gz",
            "simulator_vehicle": "x500",
            "simulator_world": "default",
            "landing_mode": "land",
            "trace_window": "offboard",
            "engagement_dwell_s": 2.0,
            "mission_mode": "identification",
        },
        "sorties": tasks,
    }


def write_identification_plan(path: str | Path, **kwargs) -> Path:
    import yaml

    plan = build_identification_plan(**kwargs)
    target = Path(path).resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(yaml.safe_dump(plan, sort_keys=False), encoding="utf-8")
    return target
