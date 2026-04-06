from __future__ import annotations

import hashlib
import json
import struct
import shutil
from pathlib import Path
from typing import Iterable

import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.trajectory_catalog import (
    COMMON_LOGGED_START,
    DEFAULT_VALIDATION_TRAJECTORIES,
    ValidationTrajectory,
    load_validation_trajectory_samples,
    validate_shipped_trajectories,
    validation_trajectory_asset_path,
)


def export_validation_trajectories(
    trajectories_dir: str | Path,
    *,
    entries: Iterable = DEFAULT_VALIDATION_TRAJECTORIES,
    freeze_yaw: bool = False,
    yaw_value: float = 0.0,
) -> dict:
    validate_shipped_trajectories()
    trajectories_dir = Path(trajectories_dir).resolve()
    trajectories_dir.mkdir(parents=True, exist_ok=True)

    manifest_entries: list[dict] = []
    for entry in entries:
        source = validation_trajectory_asset_path(entry)
        target = trajectories_dir / f"id_{entry.traj_id}.traj"
        if freeze_yaw:
            data = bytearray(source.read_bytes())
            if len(data) % 44 != 0:
                raise RuntimeError(f"invalid .traj size for {source}")
            for offset in range(0, len(data), 44):
                struct.pack_into("f", data, offset + 9 * 4, float(yaw_value))
                struct.pack_into("f", data, offset + 10 * 4, 0.0)
            target.write_bytes(data)
        else:
            shutil.copyfile(source, target)
        samples = load_validation_trajectory_samples(entry)
        output_sha256 = hashlib.sha256(target.read_bytes()).hexdigest()
        manifest_entries.append(
            {
                "traj_id": entry.traj_id,
                "name": entry.name,
                "duration_s": entry.nominal_duration_s,
                "sample_dt_s": round(entry.nominal_duration_s / max(samples.shape[0] - 1, 1), 6),
                "num_samples": int(samples.shape[0]),
                "start_pose": {
                    "x_m": float(samples[0, 0] + COMMON_LOGGED_START[0]),
                    "y_m": float(samples[0, 1] + COMMON_LOGGED_START[1]),
                    "z_m": float(samples[0, 2] + COMMON_LOGGED_START[2]),
                },
                "end_pose": {
                    "x_m": float(samples[-1, 0] + COMMON_LOGGED_START[0]),
                    "y_m": float(samples[-1, 1] + COMMON_LOGGED_START[1]),
                    "z_m": float(samples[-1, 2] + COMMON_LOGGED_START[2]),
                },
                "source_path": str(source),
                "path": str(target),
                "sha256": output_sha256,
                "source_sha256": entry.sha256,
            }
        )

    manifest = {
        "common_logged_start": {
            "x_m": COMMON_LOGGED_START[0],
            "y_m": COMMON_LOGGED_START[1],
            "z_m": COMMON_LOGGED_START[2],
        },
        "source_mode": "yaw_frozen_binary_trajectories" if freeze_yaw else "shipped_binary_trajectories",
        "freeze_yaw": freeze_yaw,
        "yaw_value_rad": float(yaw_value),
        "entries": manifest_entries,
    }
    (trajectories_dir / "validation_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Install the five shipped validation trajectories into a PX4 trajectories directory.")
    ap.add_argument("--trajectories-dir", default="~/PX4-Autopilot-Identification/build/px4_sitl_default/rootfs/trajectories")
    args = ap.parse_args()

    manifest = export_validation_trajectories(Path(args.trajectories_dir).expanduser())
    print(json.dumps(manifest, indent=2))
