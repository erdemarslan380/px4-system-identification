from __future__ import annotations

import json
import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.paper_artifacts import COMMON_LOGGED_START, DEFAULT_TRAJECTORIES, TrajectoryCase, _trajectory_reference


@dataclass(frozen=True)
class ValidationTrajectory:
    traj_id: int
    name: str
    duration_s: float


DEFAULT_VALIDATION_TRAJECTORIES: tuple[ValidationTrajectory, ...] = (
    ValidationTrajectory(100, "hairpin", 28.0),
    ValidationTrajectory(101, "lemniscate", 30.0),
    ValidationTrajectory(102, "circle", 30.0),
    ValidationTrajectory(103, "time_optimal_30s", 30.0),
    ValidationTrajectory(104, "minimum_snap_50s", 50.0),
)

_DT = 0.02
_PRE_HOLD_S = 2.0
_POST_HOLD_S = 2.0


def _case_map() -> dict[str, TrajectoryCase]:
    return {case.name: case for case in DEFAULT_TRAJECTORIES}


def _finite_diff(values: np.ndarray, dt: float) -> np.ndarray:
    grad = np.gradient(values, dt, edge_order=2)
    grad[0] = grad[1]
    grad[-1] = grad[-2]
    return grad


def build_validation_samples(case_name: str, *, dt: float = _DT, pre_hold_s: float = _PRE_HOLD_S, post_hold_s: float = _POST_HOLD_S) -> np.ndarray:
    case = _case_map()[case_name]
    moving_samples = max(2, int(round(case.duration_s / dt)) + 1)
    _, x, y, z = _trajectory_reference(case, moving_samples)

    hold_pre = max(0, int(round(pre_hold_s / dt)))
    hold_post = max(0, int(round(post_hold_s / dt)))

    start = np.array(COMMON_LOGGED_START, dtype=float)
    x_full = np.concatenate([np.full(hold_pre, start[0]), x, np.full(hold_post, start[0])])
    y_full = np.concatenate([np.full(hold_pre, start[1]), y, np.full(hold_post, start[1])])
    z_full = np.concatenate([np.full(hold_pre, start[2]), z, np.full(hold_post, start[2])])

    vx = _finite_diff(x_full, dt)
    vy = _finite_diff(y_full, dt)
    vz = _finite_diff(z_full, dt)
    ax = _finite_diff(vx, dt)
    ay = _finite_diff(vy, dt)
    az = _finite_diff(vz, dt)

    yaw = np.unwrap(np.arctan2(vy, vx))
    speed_xy = np.hypot(vx, vy)
    yaw[speed_xy < 0.05] = 0.0
    for i in range(1, len(yaw)):
        if speed_xy[i] < 0.05:
            yaw[i] = yaw[i - 1]
    yawspeed = _finite_diff(yaw, dt)

    return np.stack([x_full, y_full, z_full, vx, vy, vz, ax, ay, az, yaw, yawspeed], axis=1).astype(np.float32)


def export_validation_trajectories(
    trajectories_dir: str | Path,
    *,
    entries: Iterable[ValidationTrajectory] = DEFAULT_VALIDATION_TRAJECTORIES,
    dt: float = _DT,
    pre_hold_s: float = _PRE_HOLD_S,
    post_hold_s: float = _POST_HOLD_S,
) -> dict:
    trajectories_dir = Path(trajectories_dir).resolve()
    trajectories_dir.mkdir(parents=True, exist_ok=True)

    manifest_entries: list[dict] = []
    for entry in entries:
        samples = build_validation_samples(entry.name, dt=dt, pre_hold_s=pre_hold_s, post_hold_s=post_hold_s)
        target = trajectories_dir / f"id_{entry.traj_id}.traj"
        with target.open("wb") as f:
            for row in samples:
                f.write(struct.pack("11f", *row.tolist()))

        manifest_entries.append(
            {
                "traj_id": entry.traj_id,
                "name": entry.name,
                "duration_s": entry.duration_s,
                "pre_hold_s": pre_hold_s,
                "post_hold_s": post_hold_s,
                "sample_dt_s": dt,
                "num_samples": int(samples.shape[0]),
                "start_pose": {
                    "x_m": float(samples[0, 0]),
                    "y_m": float(samples[0, 1]),
                    "z_m": float(samples[0, 2]),
                },
                "path": str(target),
            }
        )

    manifest = {
        "common_logged_start": {"x_m": COMMON_LOGGED_START[0], "y_m": COMMON_LOGGED_START[1], "z_m": COMMON_LOGGED_START[2]},
        "entries": manifest_entries,
    }
    (trajectories_dir / "validation_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Export the five validation trajectories into PX4 .traj files.")
    ap.add_argument("--trajectories-dir", default="~/PX4-Autopilot/build/px4_sitl_default/rootfs/trajectories")
    ap.add_argument("--pre-hold-s", type=float, default=_PRE_HOLD_S)
    ap.add_argument("--post-hold-s", type=float, default=_POST_HOLD_S)
    args = ap.parse_args()

    manifest = export_validation_trajectories(Path(args.trajectories_dir).expanduser(), pre_hold_s=args.pre_hold_s, post_hold_s=args.post_hold_s)
    print(json.dumps(manifest, indent=2))
