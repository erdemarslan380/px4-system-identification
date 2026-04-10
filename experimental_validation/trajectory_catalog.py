from __future__ import annotations

import hashlib
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_ASSET_DIR = REPO_ROOT / "assets" / "validation_trajectories"
COMMON_LOGGED_START = (0.0, 0.0, -3.0)


@dataclass(frozen=True)
class ValidationTrajectory:
    traj_id: int
    name: str
    nominal_duration_s: float
    filename: str
    sha256: str


DEFAULT_VALIDATION_TRAJECTORIES: tuple[ValidationTrajectory, ...] = (
    ValidationTrajectory(100, "hairpin", 27.7, "id_100.traj", "d7ebc694abc3bbc3b852f8b962f888c1d0d1f87d015aee0aefda541668ab6367"),
    ValidationTrajectory(101, "lemniscate", 23.18, "id_101.traj", "58d656be8bca8f2b2de3a3955bd6299175842d3bb9ac35e4d475c0e525a2c136"),
    ValidationTrajectory(102, "circle", 18.9, "id_102.traj", "578dba4caa26c415903c2e21afba67143910bffa21aae096a5b7f16c8089fdbc"),
    ValidationTrajectory(103, "time_optimal_30s", 11.0, "id_103.traj", "76f3a938d86ed299923c218b87456b49632c6bb43b111d579f5e95493067cd64"),
    ValidationTrajectory(104, "minimum_snap_50s", 23.44, "id_104.traj", "5248b146160334fcbbf091c3c5dbe99c25bda3856b6d0eaf84b5a51eeaa49e89"),
)


def validation_trajectory_asset_path(entry: ValidationTrajectory) -> Path:
    return TRAJECTORY_ASSET_DIR / entry.filename


def validation_trajectory_case_map() -> dict[str, ValidationTrajectory]:
    return {entry.name: entry for entry in DEFAULT_VALIDATION_TRAJECTORIES}


def validation_trajectory_id_map() -> dict[int, ValidationTrajectory]:
    return {entry.traj_id: entry for entry in DEFAULT_VALIDATION_TRAJECTORIES}


def validate_shipped_trajectories() -> None:
    for entry in DEFAULT_VALIDATION_TRAJECTORIES:
        path = validation_trajectory_asset_path(entry)
        if not path.exists():
            raise FileNotFoundError(path)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != entry.sha256:
            raise RuntimeError(f"trajectory checksum mismatch for {path}: {digest} != {entry.sha256}")


def load_validation_trajectory_samples(entry: ValidationTrajectory) -> np.ndarray:
    path = validation_trajectory_asset_path(entry)
    data = path.read_bytes()
    if len(data) % 44 != 0:
        raise RuntimeError(f"invalid .traj size for {path}")
    rows = []
    for offset in range(0, len(data), 44):
        rows.append(struct.unpack("11f", data[offset : offset + 44]))
    return np.asarray(rows, dtype=np.float32)


def load_validation_reference(case_name: str, *, resample_to: int | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    entry = validation_trajectory_case_map()[case_name]
    samples = load_validation_trajectory_samples(entry)
    if samples.shape[0] < 2:
        raise RuntimeError(f"trajectory {entry.filename} has too few samples")

    t_native = np.linspace(0.0, entry.nominal_duration_s, samples.shape[0], dtype=float)
    x_native = samples[:, 0].astype(float) + COMMON_LOGGED_START[0]
    y_native = samples[:, 1].astype(float) + COMMON_LOGGED_START[1]
    z_native = samples[:, 2].astype(float) + COMMON_LOGGED_START[2]

    if resample_to is None or resample_to == samples.shape[0]:
        return t_native, x_native, y_native, z_native

    t_resampled = np.linspace(0.0, entry.nominal_duration_s, resample_to, dtype=float)
    x_resampled = np.interp(t_resampled, t_native, x_native)
    y_resampled = np.interp(t_resampled, t_native, y_native)
    z_resampled = np.interp(t_resampled, t_native, z_native)
    return t_resampled, x_resampled, y_resampled, z_resampled
