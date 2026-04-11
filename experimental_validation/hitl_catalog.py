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
    "actuator_lag_collective": 9,
    "bridge_probe_xy": 10,
}

IDENTIFICATION_DURATIONS_S = {
    "hover_thrust": 18.0,
    "roll_sweep": 18.0,
    "pitch_sweep": 18.0,
    "yaw_sweep": 16.0,
    "drag_x": 18.0,
    "drag_y": 18.0,
    "drag_z": 14.0,
    "mass_vertical": 22.0,
    "motor_step": 14.0,
    "actuator_lag_collective": 12.0,
    "bridge_probe_xy": 12.0,
}

TRAJECTORY_DURATIONS_S = {
    100: 27.7,
    101: 23.18,
    102: 18.9,
    103: 11.0,
    104: 23.44,
}

CAMPAIGNS = {
    "identification_only": {
        "ident_profiles": [
            "hover_thrust",
            "mass_vertical",
            "roll_sweep",
            "pitch_sweep",
            "yaw_sweep",
            "drag_x",
            "drag_y",
            "drag_z",
            "motor_step",
            "actuator_lag_collective",
            "bridge_probe_xy",
        ],
        "trajectory_ids": [],
    },
    "identification_plus_lemniscate_calibration": {
        "ident_profiles": [
            "hover_thrust",
            "mass_vertical",
            "roll_sweep",
            "pitch_sweep",
            "yaw_sweep",
            "drag_x",
            "drag_y",
            "drag_z",
            "motor_step",
            "actuator_lag_collective",
            "bridge_probe_xy",
        ],
        "trajectory_ids": [101],
    },
    "trajectory_only": {
        "ident_profiles": [],
        "trajectory_ids": [100, 101, 102, 103, 104],
    },
    "full_stack": {
        "ident_profiles": [
            "hover_thrust",
            "mass_vertical",
            "roll_sweep",
            "pitch_sweep",
            "yaw_sweep",
            "drag_x",
            "drag_y",
            "drag_z",
            "motor_step",
            "actuator_lag_collective",
            "bridge_probe_xy",
        ],
        "trajectory_ids": [100, 101, 102, 103, 104],
    },
}


def identification_profile_index(name: str) -> int:
    return IDENTIFICATION_PROFILES[name]


def identification_duration_s(name: str) -> float:
    return IDENTIFICATION_DURATIONS_S[name]


def trajectory_duration_s(traj_id: int) -> float:
    return TRAJECTORY_DURATIONS_S[traj_id]


def campaign_ident_profiles(name: str) -> list[str]:
    return list(CAMPAIGNS[name]["ident_profiles"])


def campaign_trajectory_ids(name: str) -> list[int]:
    return list(CAMPAIGNS[name]["trajectory_ids"])


def campaign_expected_duration_s(name: str, return_buffer_s: float = 2.0) -> float:
    ident_total = sum(identification_duration_s(profile) for profile in campaign_ident_profiles(name))
    trajectory_total = sum(trajectory_duration_s(traj_id) for traj_id in campaign_trajectory_ids(name))
    segment_count = len(campaign_ident_profiles(name)) + len(campaign_trajectory_ids(name))
    return ident_total + trajectory_total + max(0, segment_count - 1) * return_buffer_s
