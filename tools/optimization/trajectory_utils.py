#!/usr/bin/env python3
"""Helpers for trajectory metadata used by tuning plans and dashboards."""

from __future__ import annotations

import math
import struct
from pathlib import Path


# Must match TrajSample in src/modules/trajectory_reader/trajectory_reader.hpp:
# px, py, pz, vx, vy, vz, ax, ay, az, yaw, yawspeed
TRAJ_SAMPLE_STRUCT = struct.Struct("<11f")
TRAJECTORY_DT_S = 0.05


def _trajectory_candidates(rootfs: Path, traj_id: int) -> list[Path]:
    rootfs = Path(rootfs).resolve()
    name = f"id_{int(traj_id)}.traj"
    candidates = [rootfs / "trajectories" / name]

    for parent in rootfs.parents:
        candidate = parent / "trajectories" / name
        if candidate not in candidates:
            candidates.append(candidate)

    return candidates


def trajectory_file(rootfs: Path, traj_id: int) -> Path:
    candidates = _trajectory_candidates(rootfs, traj_id)
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def trajectory_sample_count(rootfs: Path, traj_id: int) -> int:
    path = trajectory_file(rootfs, traj_id)
    if not path.exists():
        raise RuntimeError(f"Trajectory file not found: {path}")
    size = path.stat().st_size
    if size <= 0 or size % TRAJ_SAMPLE_STRUCT.size != 0:
        raise RuntimeError(f"Invalid trajectory file size: {path} ({size} bytes)")
    return size // TRAJ_SAMPLE_STRUCT.size


def trajectory_duration_s(rootfs: Path, traj_id: int) -> float:
    return float(trajectory_sample_count(rootfs, traj_id) * TRAJECTORY_DT_S)


def recommended_trajectory_timeout_s(duration_s: float, scale: float = 1.6, margin_s: float = 12.0) -> float:
    duration_s = max(0.0, float(duration_s))
    return max(25.0, duration_s * float(scale) + float(margin_s))


def trajectory_points(rootfs: Path, traj_id: int, *, max_points: int | None = None) -> list[dict[str, float]]:
    path = trajectory_file(rootfs, traj_id)
    if not path.exists():
        raise RuntimeError(f"Trajectory file not found: {path}")

    data = path.read_bytes()
    if len(data) <= 0 or len(data) % TRAJ_SAMPLE_STRUCT.size != 0:
        raise RuntimeError(f"Invalid trajectory file size: {path} ({len(data)} bytes)")

    total = len(data) // TRAJ_SAMPLE_STRUCT.size
    step = 1
    if max_points and max_points > 0 and total > max_points:
        step = max(1, math.ceil(total / float(max_points)))

    out: list[dict[str, float]] = []
    for idx in range(0, total, step):
        base = idx * TRAJ_SAMPLE_STRUCT.size
        fields = TRAJ_SAMPLE_STRUCT.unpack_from(data, base)
        out.append({
            "index": float(idx),
            "t": float(idx * TRAJECTORY_DT_S),
            "x": float(fields[0]),
            "y": float(fields[1]),
            "z": float(fields[2]),
            "vx": float(fields[3]),
            "vy": float(fields[4]),
            "vz": float(fields[5]),
            "ax": float(fields[6]),
            "ay": float(fields[7]),
            "az": float(fields[8]),
            "yaw": float(fields[9]),
            "yawspeed": float(fields[10]),
        })

    if out and int(out[-1]["index"]) != total - 1:
        fields = TRAJ_SAMPLE_STRUCT.unpack_from(data, (total - 1) * TRAJ_SAMPLE_STRUCT.size)
        out.append({
            "index": float(total - 1),
            "t": float((total - 1) * TRAJECTORY_DT_S),
            "x": float(fields[0]),
            "y": float(fields[1]),
            "z": float(fields[2]),
            "vx": float(fields[3]),
            "vy": float(fields[4]),
            "vz": float(fields[5]),
            "ax": float(fields[6]),
            "ay": float(fields[7]),
            "az": float(fields[8]),
            "yaw": float(fields[9]),
            "yawspeed": float(fields[10]),
        })

    return out


def parse_timeout_value(raw: object, *, default_s: float, duration_s: float) -> float:
    if raw is None:
        return float(default_s)
    if isinstance(raw, str) and raw.strip().lower() == "auto":
        return recommended_trajectory_timeout_s(duration_s)
    value = float(raw)
    if not math.isfinite(value) or value <= 0.0:
        raise RuntimeError(f"Invalid timeout value: {raw}")
    return value
