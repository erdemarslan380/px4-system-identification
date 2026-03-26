"""Synthetic noiseless benchmark for upper-bound system-identification recovery."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.compare_with_sdf import compare_identified_to_sdf
from experimental_validation.identification import estimate_parameters_from_identification_log
from experimental_validation.reference_models import default_x500_reference


def build_perfect_recovery_rows() -> list[dict[str, float | str]]:
    ref = default_x500_reference()
    gravity = 9.80665
    mass = float(ref["mass"]["mass_kg"])
    ixx = float(ref["inertia"]["x"]["inertia_kgm2"])
    iyy = float(ref["inertia"]["y"]["inertia_kgm2"])
    izz = float(ref["inertia"]["z"]["inertia_kgm2"])
    tau_up = float(ref["motor_model"]["time_constant_up_s"])
    tau_down = float(ref["motor_model"]["time_constant_down_s"])
    max_rot = float(ref["motor_model"]["max_rot_velocity_radps"])
    motor_constant = float(ref["motor_model"]["motor_constant"])
    moment_constant = float(ref["motor_model"]["moment_constant"])
    rotor_drag = float(ref["motor_model"]["rotor_drag_coefficient"])
    rolling_moment = float(ref["motor_model"]["rolling_moment_coefficient"])
    slowdown = float(ref["motor_model"]["rotor_velocity_slowdown_sim"])

    thrust_scale = 33.0
    drag_x = 0.18
    drag_y = 0.21
    drag_z = 0.27

    rows: list[dict[str, float | str]] = []

    for i in range(1, 61):
        t_s = 0.02 * i
        roll = 0.10 * math.sin(0.17 * i)
        pitch = 0.08 * math.cos(0.13 * i)
        az = 0.8 * math.sin(0.07 * i)
        thrust_vertical = mass * (gravity + az)
        thrust_total = thrust_vertical / max(math.cos(roll) * math.cos(pitch), 1e-6)
        rows.append({
            "timestamp_us": t_s * 1e6,
            "profile": "hover_thrust",
            "thrust_cmd": thrust_total / thrust_scale,
            "thrust_n": thrust_total,
            "az_world_mps2": az,
            "roll": roll,
            "pitch": pitch,
        })

    for i in range(1, 91):
        t_s = 2.0 + 0.02 * i
        p_dot = 0.35 + 0.01 * i
        q = 0.45 + 0.004 * i
        r = -0.30 + 0.003 * i
        tau_x = ixx * p_dot + (izz - iyy) * q * r
        rows.append({
            "timestamp_us": t_s * 1e6,
            "profile": "roll_sweep",
            "p": 0.4,
            "q": q,
            "r": r,
            "p_dot_radps2": p_dot,
            "q_dot_radps2": 0.0,
            "r_dot_radps2": 0.0,
            "tau_x_nm": tau_x,
        })

    for i in range(1, 91):
        t_s = 4.0 + 0.02 * i
        q_dot = 0.40 + 0.012 * i
        p = 0.42 + 0.003 * i
        r = 0.31 + 0.002 * i
        tau_y = iyy * q_dot + (ixx - izz) * p * r
        rows.append({
            "timestamp_us": t_s * 1e6,
            "profile": "pitch_sweep",
            "p": p,
            "q": 0.35,
            "r": r,
            "p_dot_radps2": 0.0,
            "q_dot_radps2": q_dot,
            "r_dot_radps2": 0.0,
            "tau_y_nm": tau_y,
        })

    for i in range(1, 91):
        t_s = 6.0 + 0.02 * i
        r_dot = 0.33 + 0.011 * i
        p = 0.37 + 0.002 * i
        q = 0.41 + 0.0025 * i
        tau_z = izz * r_dot + (iyy - ixx) * p * q
        rows.append({
            "timestamp_us": t_s * 1e6,
            "profile": "yaw_sweep",
            "p": p,
            "q": q,
            "r": 0.28,
            "p_dot_radps2": 0.0,
            "q_dot_radps2": 0.0,
            "r_dot_radps2": r_dot,
            "tau_z_nm": tau_z,
            "yaw_moment_basis_n": tau_z / moment_constant,
        })

    for i in range(1, 81):
        vx = 0.8 + 0.04 * i
        vy = 0.7 + 0.03 * i
        vz = 0.6 + 0.02 * i
        rows.append({
            "timestamp_us": (8.0 + 0.02 * i) * 1e6,
            "profile": "drag_x",
            "vx_mps": vx,
            "ax_drag_mps2": -(drag_x / mass) * abs(vx) * vx,
        })
        rows.append({
            "timestamp_us": (10.0 + 0.02 * i) * 1e6,
            "profile": "drag_y",
            "vy_mps": vy,
            "ay_drag_mps2": -(drag_y / mass) * abs(vy) * vy,
        })
        rows.append({
            "timestamp_us": (12.0 + 0.02 * i) * 1e6,
            "profile": "drag_z",
            "vz_mps": vz,
            "az_drag_mps2": -(drag_z / mass) * abs(vz) * vz,
        })

    actual = 0.0
    command = 0.0
    dt = 0.005
    t_s = 14.0
    for k in range(150):
        rows.append(_motor_row(t_s, command, actual, slowdown, motor_constant))
        next_command = max_rot if k >= 5 else 0.0
        actual = next_command - (next_command - actual) * math.exp(-dt / tau_up)
        command = next_command
        t_s += dt

    for _ in range(180):
        rows.append(_motor_row(t_s, command, actual, slowdown, motor_constant))
        next_command = 0.0
        actual = next_command - (next_command - actual) * math.exp(-dt / tau_down)
        command = next_command
        t_s += dt

    for row in rows:
        if row.get("profile") == "motor_step":
            omega = float(row["rotor_0_actual_radps"])
            thrust_per_rotor = motor_constant * omega * omega
            row["rotor_0_thrust_n"] = thrust_per_rotor
            row["rotor_1_thrust_n"] = thrust_per_rotor
            row["rotor_2_thrust_n"] = thrust_per_rotor
            row["rotor_3_thrust_n"] = thrust_per_rotor
            row["observed_max_rot_velocity_radps"] = max_rot
            row["rotor_velocity_slowdown_sim"] = slowdown
            row["yaw_moment_basis_n"] = 4.0 * thrust_per_rotor
            row["tau_z_nm"] = row["yaw_moment_basis_n"] * moment_constant
            omega_sq_sum = 4.0 * omega * omega
            row["drag_force_body_x_n"] = rotor_drag * omega_sq_sum
            row["drag_basis_body_x"] = omega_sq_sum
            row["rolling_moment_body_x_nm"] = rolling_moment * omega_sq_sum
            row["rolling_basis_body_x"] = omega_sq_sum

    return rows


def _motor_row(t_s: float, command: float, actual: float, slowdown: float, motor_constant: float) -> dict[str, float | str]:
    thrust_total = 4.0 * motor_constant * actual * actual
    return {
        "timestamp_us": t_s * 1e6,
        "profile": "motor_step",
        "thrust_cmd": 0.55,
        "thrust_n": thrust_total,
        "az_world_mps2": 0.0,
        "rotor_0_cmd_radps": command,
        "rotor_1_cmd_radps": command,
        "rotor_2_cmd_radps": command,
        "rotor_3_cmd_radps": command,
        "rotor_0_actual_radps": actual,
        "rotor_1_actual_radps": actual,
        "rotor_2_actual_radps": actual,
        "rotor_3_actual_radps": actual,
        "rotor_0_joint_vel_radps": actual / slowdown,
        "rotor_1_joint_vel_radps": actual / slowdown,
        "rotor_2_joint_vel_radps": actual / slowdown,
        "rotor_3_joint_vel_radps": actual / slowdown,
    }


def run_perfect_recovery_benchmark() -> dict[str, Any]:
    reference = default_x500_reference()
    rows = build_perfect_recovery_rows()
    identified = estimate_parameters_from_identification_log(rows)
    comparison = compare_identified_to_sdf(identified, reference)
    return {
        "identified": identified,
        "reference": reference,
        "comparison": comparison,
        "row_count": len(rows),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("examples/paper_assets/perfect_recovery_benchmark.json"),
        help="Where to write the benchmark JSON summary.",
    )
    args = parser.parse_args(argv)

    payload = run_perfect_recovery_benchmark()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(args.out.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
