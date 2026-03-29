from __future__ import annotations

IDENTIFICATION_PROFILES = {
    "hover_thrust": 0,
    "roll_sweep": 1,
    "pitch_sweep": 2,
    "yaw_sweep": 3,
    "drag_x": 4,
    "drag_y": 5,
    "drag_z": 6,
    "mass_vertical": 7,
    "motor_step": 8,
}

IDENTIFICATION_DURATIONS_S = {
    "hover_thrust": 26.0,
    "roll_sweep": 28.0,
    "pitch_sweep": 28.0,
    "yaw_sweep": 24.0,
    "drag_x": 30.0,
    "drag_y": 30.0,
    "drag_z": 30.0,
    "mass_vertical": 36.0,
    "motor_step": 24.0,
}

TRAJECTORY_DURATIONS_S = {
    100: 23.0,
    101: 19.0,
    102: 15.0,
    103: 11.0,
    104: 14.0,
}


def identification_profile_index(name: str) -> int:
    return IDENTIFICATION_PROFILES[name]


def identification_duration_s(name: str) -> float:
    return IDENTIFICATION_DURATIONS_S[name]


def trajectory_duration_s(traj_id: int) -> float:
    return TRAJECTORY_DURATIONS_S[traj_id]
