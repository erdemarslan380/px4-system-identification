#!/usr/bin/env python3

from __future__ import annotations

import argparse
import hashlib
import json
import struct
from pathlib import Path

import numpy as np


FLOATS_PER_SAMPLE = 11
SAMPLE_STRUCT = struct.Struct("<" + "f" * FLOATS_PER_SAMPLE)
SAMPLE_DT_S = 0.02


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Time-scale validation .traj assets while preserving shape."
    )
    parser.add_argument(
        "--assets-dir",
        default="assets/validation_trajectories",
        help="Directory containing id_XXX.traj files",
    )
    parser.add_argument(
        "--scale",
        action="append",
        required=True,
        metavar="TRAJ_ID=FACTOR",
        help="Time-scale factor for a trajectory, e.g. 102=1.30",
    )
    parser.add_argument(
        "--write-metadata",
        help="Optional JSON file for durations/checksums summary",
    )
    return parser.parse_args()


def parse_scale_overrides(items: list[str]) -> dict[int, float]:
    overrides: dict[int, float] = {}
    for item in items:
        key, sep, value = item.partition("=")
        if not sep:
            raise ValueError(f"invalid scale override: {item!r}")
        traj_id = int(key)
        factor = float(value)
        if factor <= 0.0:
            raise ValueError(f"scale factor must be positive for trajectory {traj_id}")
        overrides[traj_id] = factor
    return overrides


def read_samples(path: Path) -> np.ndarray:
    data = path.read_bytes()
    if len(data) % SAMPLE_STRUCT.size != 0:
        raise RuntimeError(f"{path} has invalid size {len(data)}")
    rows = [
        SAMPLE_STRUCT.unpack_from(data, offset)
        for offset in range(0, len(data), SAMPLE_STRUCT.size)
    ]
    return np.asarray(rows, dtype=np.float64)


def write_samples(path: Path, samples: np.ndarray) -> None:
    payload = bytearray()
    for row in samples:
        payload.extend(SAMPLE_STRUCT.pack(*[float(value) for value in row]))
    path.write_bytes(payload)


def resample_trajectory(samples: np.ndarray, time_scale: float) -> np.ndarray:
    old_count = samples.shape[0]
    if old_count < 2:
        raise RuntimeError("trajectory must contain at least two samples")

    old_steps = old_count - 1
    new_steps = max(1, int(round(old_steps * time_scale)))
    new_count = new_steps + 1
    scale_eff = new_steps / old_steps

    old_t = np.arange(old_count, dtype=np.float64) * SAMPLE_DT_S
    new_t = np.arange(new_count, dtype=np.float64) * SAMPLE_DT_S
    query_t = np.clip(new_t / scale_eff, 0.0, old_t[-1])

    out = np.zeros((new_count, FLOATS_PER_SAMPLE), dtype=np.float64)

    for col in range(3):
        out[:, col] = np.interp(query_t, old_t, samples[:, col])

    for col in range(3, 6):
        out[:, col] = np.interp(query_t, old_t, samples[:, col]) / scale_eff

    for col in range(6, 9):
        out[:, col] = np.interp(query_t, old_t, samples[:, col]) / (scale_eff * scale_eff)

    yaw_unwrapped = np.unwrap(samples[:, 9])
    out[:, 9] = np.interp(query_t, old_t, yaw_unwrapped)
    out[:, 10] = np.interp(query_t, old_t, samples[:, 10]) / scale_eff

    return out


def main() -> None:
    args = parse_args()
    assets_dir = Path(args.assets_dir).expanduser().resolve()
    overrides = parse_scale_overrides(args.scale)

    metadata: dict[str, dict[str, float | str | int]] = {}

    for traj_id, factor in sorted(overrides.items()):
        path = assets_dir / f"id_{traj_id}.traj"
        samples = read_samples(path)
        scaled = resample_trajectory(samples, factor)
        write_samples(path, scaled)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        metadata[str(traj_id)] = {
            "scale_factor": factor,
            "num_samples": int(scaled.shape[0]),
            "duration_s": round((scaled.shape[0] - 1) * SAMPLE_DT_S, 6),
            "sha256": digest,
        }

    if args.write_metadata:
        Path(args.write_metadata).write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    else:
        print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
