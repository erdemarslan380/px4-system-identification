"""Parsing and estimation helpers for PX4/Gazebo identification flights."""

from __future__ import annotations

import math
import re
from pathlib import Path
from typing import Dict, Iterable, List, Sequence

from experimental_validation.estimators import (
    GRAVITY_MPS2,
    AxisInertiaEstimate,
    DiagonalInertiaTensorEstimate,
    DragEstimate,
    HoverMassEstimate,
    ScalarEstimate,
    StepResponseDynamicsEstimate,
    ThrustScaleEstimate,
    estimate_axis_inertia,
    estimate_axis_inertia_with_coupling,
    estimate_diagonal_inertia_tensor,
    estimate_hover_mass,
    estimate_max_value,
    estimate_proportional_scale,
    estimate_quadratic_drag,
    estimate_ratio,
    estimate_step_response_dynamics,
    estimate_thrust_scale,
    estimate_time_constant,
    load_numeric_csv,
)


def _scalar_rmse(values: Sequence[float], target: float) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum((float(value) - float(target)) ** 2 for value in values) / len(values))


def load_gazebo_truth_csv(path: str | Path) -> list[dict[str, float]]:
    rows = [dict(row) for row in load_numeric_csv(path)]
    for row in rows:
        if "sim_time_us" in row and "t_s" not in row:
            row["t_s"] = float(row["sim_time_us"]) * 1e-6
    return rows


def _load_profile_strings(path: str | Path) -> list[str]:
    text_rows = Path(path).read_text(encoding="utf-8").splitlines()
    if not text_rows:
        return []
    headers = [part.strip() for part in text_rows[0].split(",")]
    if "profile" not in headers:
        return []
    profile_index = headers.index("profile")
    values: list[str] = []
    for line in text_rows[1:]:
        if not line.strip():
            continue
        if line.startswith(("INFO ", "WARN ", "ERROR ")):
            continue
        parts = [part.strip() for part in line.split(",")]
        if not parts:
            continue
        try:
            float(parts[0])
        except (TypeError, ValueError):
            continue
        values.append(parts[profile_index] if profile_index < len(parts) else "")
    return values


def _merge_truth_rows(
    identification_rows: list[dict[str, float | str]],
    truth_rows: list[dict[str, float]],
    *,
    truth_field_allowlist: set[str] | None = None,
    tolerance_s: float = 0.05,
    allow_relative_alignment: bool = True,
) -> None:
    if not identification_rows or not truth_rows:
        return

    ident_times = [float(row.get("t_s", 0.0)) for row in identification_rows]
    truth_times = [float(row.get("t_s", 0.0)) for row in truth_rows]
    ident_t0 = ident_times[0]
    truth_t0 = truth_times[0]
    prefer_absolute = abs(ident_t0 - truth_t0) <= max(tolerance_s * 4.0, 0.25) or not allow_relative_alignment
    truth_idx = 0

    for row, ident_time in zip(identification_rows, ident_times):
        while truth_idx + 1 < len(truth_rows):
            curr_time = truth_times[truth_idx] if prefer_absolute else (truth_times[truth_idx] - truth_t0)
            next_time = truth_times[truth_idx + 1] if prefer_absolute else (truth_times[truth_idx + 1] - truth_t0)
            ident_target = ident_time if prefer_absolute else (ident_time - ident_t0)
            curr_diff = abs(curr_time - ident_target)
            next_diff = abs(next_time - ident_target)
            if next_diff <= curr_diff:
                truth_idx += 1
            else:
                break
        truth_target = truth_times[truth_idx] if prefer_absolute else (truth_times[truth_idx] - truth_t0)
        ident_target = ident_time if prefer_absolute else (ident_time - ident_t0)
        if abs(truth_target - ident_target) <= tolerance_s:
            for key, value in truth_rows[truth_idx].items():
                if key == "t_s":
                    continue
                if truth_field_allowlist is not None and key not in truth_field_allowlist:
                    continue
                row[key] = value


def _expand_truth_rows(
    identification_rows: list[dict[str, float | str]],
    truth_rows: list[dict[str, float]],
    *,
    truth_field_allowlist: set[str] | None = None,
    tolerance_s: float = 0.05,
    allow_relative_alignment: bool = True,
) -> list[dict[str, float | str]]:
    if not identification_rows or not truth_rows:
        return []

    ident_times = [float(row.get("t_s", 0.0)) for row in identification_rows]
    truth_times = [float(row.get("t_s", 0.0)) for row in truth_rows]
    ident_t0 = ident_times[0]
    truth_t0 = truth_times[0]
    prefer_absolute = abs(ident_t0 - truth_t0) <= max(tolerance_s * 4.0, 0.25) or not allow_relative_alignment
    ident_idx = 0
    out: list[dict[str, float | str]] = []

    rotor_indices = _detect_rotor_indices(identification_rows)
    passthrough_keys = [
        "profile",
        "thrust_cmd",
        "controller",
        *[f"motor_{i}" for i in rotor_indices],
    ]

    for truth_row, truth_time in zip(truth_rows, truth_times):
        while ident_idx + 1 < len(identification_rows):
            curr_time = ident_times[ident_idx] if prefer_absolute else (ident_times[ident_idx] - ident_t0)
            next_time = ident_times[ident_idx + 1] if prefer_absolute else (ident_times[ident_idx + 1] - ident_t0)
            truth_target = truth_time if prefer_absolute else (truth_time - truth_t0)
            curr_diff = abs(curr_time - truth_target)
            next_diff = abs(next_time - truth_target)
            if next_diff <= curr_diff:
                ident_idx += 1
            else:
                break
        ident_target = ident_times[ident_idx] if prefer_absolute else (ident_times[ident_idx] - ident_t0)
        truth_target = truth_time if prefer_absolute else (truth_time - truth_t0)
        if abs(ident_target - truth_target) > tolerance_s:
            continue
        if truth_field_allowlist is None:
            merged = dict(truth_row)
        else:
            merged = {key: value for key, value in truth_row.items() if key in truth_field_allowlist}
        ident_row = identification_rows[ident_idx]
        for key in passthrough_keys:
            if key in ident_row:
                merged[key] = ident_row[key]
        out.append(merged)
    return out


def load_identification_csv(
    path: str | Path,
    truth_csv: str | Path | None = None,
    truth_field_allowlist: set[str] | None = None,
    allow_relative_truth_alignment: bool = True,
) -> list[dict[str, float | str]]:
    rows: list[dict[str, float | str]] = [dict(row) for row in load_numeric_csv(path)]

    profile_values = _load_profile_strings(path)
    if profile_values:
        merged: list[dict[str, float | str]] = []
        row_index = 0
        for profile in profile_values:
            if row_index >= len(rows):
                break
            enriched = dict(rows[row_index])
            enriched["profile"] = profile
            merged.append(enriched)
            row_index += 1
        rows = merged or rows

    _apply_identification_aliases(rows)

    if truth_csv:
        truth_rows = load_gazebo_truth_csv(truth_csv)
        _merge_truth_rows(
            rows,
            truth_rows,
            truth_field_allowlist=truth_field_allowlist,
            allow_relative_alignment=allow_relative_truth_alignment,
        )
        if len(truth_rows) >= len(rows) * 2:
            rows.extend(
                _expand_truth_rows(
                    rows,
                    truth_rows,
                    truth_field_allowlist=truth_field_allowlist,
                    allow_relative_alignment=allow_relative_truth_alignment,
                )
            )

    _apply_identification_aliases(rows)
    return rows


def _apply_identification_aliases(rows: list[dict[str, float | str]]) -> None:
    rotor_indices = _detect_rotor_indices(rows)

    def _finite_or_none(value: object) -> float | None:
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            return None
        return numeric if math.isfinite(numeric) else None

    global_max_rot_velocity = 0.0
    global_max_rotor_command = 0.0
    for row in rows:
        for key in ["observed_max_rot_velocity_radps", *[f"rotor_{i}_actual_radps" for i in rotor_indices]]:
            value = row.get(key)
            if value is None:
                continue
            value = float(value)
            if math.isfinite(value):
                global_max_rot_velocity = max(global_max_rot_velocity, abs(value))
        for key in [f"rotor_{i}_cmd_radps" for i in rotor_indices]:
            value = _finite_or_none(row.get(key))
            if value is not None:
                global_max_rotor_command = max(global_max_rotor_command, abs(value))

    for row in rows:
        if "timestamp_us" in row and "t_s" not in row:
            row["t_s"] = float(row["timestamp_us"]) * 1e-6
        if "sim_time_us" in row and "truth_t_s" not in row:
            row["truth_t_s"] = float(row["sim_time_us"]) * 1e-6
        if "sim_time_us" in row and "t_s" not in row:
            row["t_s"] = float(row["sim_time_us"]) * 1e-6

        if "ax" in row and "ax_world_mps2" not in row:
            row["ax_world_mps2"] = float(row["ax"])
            row["ax_drag_mps2"] = float(row["ax"])
        if "ay" in row and "ay_world_mps2" not in row:
            row["ay_world_mps2"] = float(row["ay"])
            row["ay_drag_mps2"] = float(row["ay"])
        if "az" in row and "az_world_mps2" not in row:
            row["az_world_mps2"] = float(row["az"])
            row["az_drag_mps2"] = float(row["az"])

        if "vel_x" in row and "vx_mps" not in row:
            row["vx_mps"] = float(row["vel_x"])
        if "vel_y" in row and "vy_mps" not in row:
            row["vy_mps"] = float(row["vel_y"])
        if "vel_z" in row and "vz_mps" not in row:
            row["vz_mps"] = float(row["vel_z"])
        if "vx" in row and "vx_mps" not in row:
            row["vx_mps"] = float(row["vx"])
        if "vy" in row and "vy_mps" not in row:
            row["vy_mps"] = float(row["vy"])
        if "vz" in row and "vz_mps" not in row:
            row["vz_mps"] = float(row["vz"])

        if "total_prop_thrust_n" in row and "thrust_n" not in row:
            row["thrust_n"] = float(row["total_prop_thrust_n"])
        if "total_torque_body_x_nm" in row:
            row.setdefault("tau_x_nm", float(row["total_torque_body_x_nm"]))
        if "total_torque_body_y_nm" in row:
            row.setdefault("tau_y_nm", float(row["total_torque_body_y_nm"]))
        if "total_torque_body_z_nm" in row:
            row.setdefault("tau_z_nm", float(row["total_torque_body_z_nm"]))
        if "torque_sp_roll" in row and "tau_x_nm" not in row:
            row["tau_x_nm"] = float(row["torque_sp_roll"])
        if "torque_sp_pitch" in row and "tau_y_nm" not in row:
            row["tau_y_nm"] = float(row["torque_sp_pitch"])
        if "torque_sp_yaw" in row and "tau_z_nm" not in row:
            row["tau_z_nm"] = float(row["torque_sp_yaw"])
        if "p_dot_body" in row and "p_dot_radps2" not in row:
            row["p_dot_radps2"] = float(row["p_dot_body"])
        if "q_dot_body" in row and "q_dot_radps2" not in row:
            row["q_dot_radps2"] = float(row["q_dot_body"])
        if "r_dot_body" in row and "r_dot_radps2" not in row:
            row["r_dot_radps2"] = float(row["r_dot_body"])

        if "rate_sp_thrust_z" in row and "thrust_cmd" not in row:
            row["thrust_cmd"] = abs(float(row["rate_sp_thrust_z"]))
        elif "att_sp_thrust_z" in row and "thrust_cmd" not in row:
            row["thrust_cmd"] = abs(float(row["att_sp_thrust_z"]))
        elif "thrust_cmd" not in row:
            motor_values = [
                abs(float(row[f"motor_{i}"]))
                for i in rotor_indices
                if f"motor_{i}" in row
            ]
            if motor_values:
                row["thrust_cmd"] = sum(motor_values) / len(motor_values)

        effective_max_rot = max(
            global_max_rot_velocity,
            abs(float(row.get("observed_max_rot_velocity_radps", 0.0) or 0.0)),
            1.0,
        )
        normalized_rotor_commands = 0.0 < global_max_rotor_command <= 1.5 and effective_max_rot > 10.0
        for rotor_index in rotor_indices:
            motor_key = f"motor_{rotor_index}"
            cmd_key = f"rotor_{rotor_index}_cmd_radps"
            inferred_key = f"rotor_{rotor_index}_cmd_inferred"
            esc_rpm_key = f"esc_{rotor_index}_rpm"
            if esc_rpm_key in row and f"rotor_{rotor_index}_actual_radps" not in row:
                esc_rpm = _finite_or_none(row.get(esc_rpm_key))
                if esc_rpm is not None:
                    row[f"rotor_{rotor_index}_actual_radps"] = esc_rpm * (2.0 * math.pi / 60.0)
            current_cmd = _finite_or_none(row.get(cmd_key))
            existing_inferred = _finite_or_none(row.get(inferred_key))
            if existing_inferred is None:
                row[inferred_key] = 0.0 if current_cmd is not None else 1.0
            else:
                row[inferred_key] = 1.0 if existing_inferred >= 0.5 else 0.0
            should_backfill = current_cmd is None or row[inferred_key] >= 0.5
            if not should_backfill and row[inferred_key] < 0.5 and normalized_rotor_commands:
                should_backfill = True
            if should_backfill and motor_key in row:
                motor_value = max(abs(float(row[motor_key])), 0.0)
                row[cmd_key] = motor_value * effective_max_rot
                row[inferred_key] = 1.0
        if "collective_cmd_radps" not in row:
            command_values = [
                float(row[key])
                for key in (f"rotor_{i}_cmd_radps" for i in rotor_indices)
                if key in row and math.isfinite(float(row[key]))
            ]
            if command_values:
                row["collective_cmd_radps"] = sum(command_values) / len(command_values)
        if "collective_actual_radps" not in row:
            actual_values = [
                float(row[key])
                for key in (f"rotor_{i}_actual_radps" for i in rotor_indices)
                if key in row and math.isfinite(float(row[key]))
            ]
            if actual_values:
                row["collective_actual_radps"] = sum(actual_values) / len(actual_values)

    grouped: Dict[str, list[dict[str, float | str]]] = {}
    for row in rows:
        profile = str(row.get("profile") or "unknown").strip() or "unknown"
        run_index = _finite_or_none(row.get("run_index"))
        if run_index is None:
            group_key = profile
        else:
            group_key = f"{profile}|run:{int(run_index)}"
        grouped.setdefault(group_key, []).append(row)

    for profile_rows in grouped.values():
        profile_rows.sort(key=lambda row: float(row.get("t_s", 0.0)))

        def _derive_rate_accel(rate_column: str, accel_column: str) -> None:
            for index, row in enumerate(profile_rows):
                if rate_column not in row:
                    continue
                prev_index = index - 1
                next_index = index + 1
                derived: float | None = None

                if prev_index >= 0 and next_index < len(profile_rows):
                    prev_row = profile_rows[prev_index]
                    next_row = profile_rows[next_index]
                    if rate_column in prev_row and rate_column in next_row:
                        dt = float(next_row.get("t_s", 0.0)) - float(prev_row.get("t_s", 0.0))
                        if dt > 1e-6:
                            derived = (float(next_row[rate_column]) - float(prev_row[rate_column])) / dt

                if derived is None and prev_index >= 0:
                    prev_row = profile_rows[prev_index]
                    if rate_column in prev_row:
                        dt = float(row.get("t_s", 0.0)) - float(prev_row.get("t_s", 0.0))
                        if dt > 1e-6:
                            derived = (float(row[rate_column]) - float(prev_row[rate_column])) / dt

                if derived is None:
                    continue

                current = _finite_or_none(row.get(accel_column))
                if current is None or abs(current) <= 1e-6 or rate_column.endswith("_body"):
                    row[accel_column] = derived

        _derive_rate_accel("p_body", "p_dot_radps2")
        _derive_rate_accel("q_body", "q_dot_radps2")
        _derive_rate_accel("r_body", "r_dot_radps2")
        _derive_rate_accel("p", "p_dot_radps2")
        _derive_rate_accel("q", "q_dot_radps2")
        _derive_rate_accel("r", "r_dot_radps2")

        previous = None
        for row in profile_rows:
            if previous is not None:
                dt = float(row.get("t_s", 0.0)) - float(previous.get("t_s", 0.0))
                if dt > 1e-6:
                    if "p" in row and "p" in previous:
                        derived = (float(row["p"]) - float(previous["p"])) / dt
                        current = _finite_or_none(row.get("p_dot_radps2"))
                        if current is None or (abs(current) <= 1e-6 and abs(derived) > 1e-4):
                            row["p_dot_radps2"] = derived
                    if "q" in row and "q" in previous:
                        derived = (float(row["q"]) - float(previous["q"])) / dt
                        current = _finite_or_none(row.get("q_dot_radps2"))
                        if current is None or (abs(current) <= 1e-6 and abs(derived) > 1e-4):
                            row["q_dot_radps2"] = derived
                    if "r" in row and "r" in previous:
                        derived = (float(row["r"]) - float(previous["r"])) / dt
                        current = _finite_or_none(row.get("r_dot_radps2"))
                        if current is None or (abs(current) <= 1e-6 and abs(derived) > 1e-4):
                            row["r_dot_radps2"] = derived
            previous = row

        for row in profile_rows:
            if all(key in row for key in ("motor_0", "motor_1", "motor_2", "motor_3")):
                m0 = float(row["motor_0"])
                m1 = float(row["motor_1"])
                m2 = float(row["motor_2"])
                m3 = float(row["motor_3"])
                row.setdefault("roll_torque_proxy", (m1 + m2) - (m0 + m3))
                row.setdefault("pitch_torque_proxy", (m0 + m1) - (m2 + m3))
                row.setdefault("yaw_torque_proxy", (-m0 + m1 - m2 + m3))

        def _derive_linear_accel(velocity_column: str, accel_column: str, alias_column: str) -> None:
            for index, row in enumerate(profile_rows):
                if velocity_column not in row:
                    continue
                prev_index = index - 1
                next_index = index + 1
                derived: float | None = None

                if prev_index >= 0 and next_index < len(profile_rows):
                    prev_row = profile_rows[prev_index]
                    next_row = profile_rows[next_index]
                    if velocity_column in prev_row and velocity_column in next_row:
                        dt = float(next_row.get("t_s", 0.0)) - float(prev_row.get("t_s", 0.0))
                        if dt > 1e-6:
                            derived = (float(next_row[velocity_column]) - float(prev_row[velocity_column])) / dt

                if derived is None and prev_index >= 0:
                    prev_row = profile_rows[prev_index]
                    if velocity_column in prev_row:
                        dt = float(row.get("t_s", 0.0)) - float(prev_row.get("t_s", 0.0))
                        if dt > 1e-6:
                            derived = (float(row[velocity_column]) - float(prev_row[velocity_column])) / dt

                if derived is None:
                    continue

                current = _finite_or_none(row.get(accel_column))
                if current is None or abs(current) <= 1e-6:
                    row[accel_column] = derived
                    row[alias_column] = derived

        _derive_linear_accel("vx_mps", "ax_world_mps2", "ax_drag_mps2")
        _derive_linear_accel("vy_mps", "ay_world_mps2", "ay_drag_mps2")
        _derive_linear_accel("vz_mps", "az_world_mps2", "az_drag_mps2")

        for row in profile_rows:
            if "ax_world_mps2" in row and ("ax" not in row or abs(_finite_or_none(row.get("ax")) or 0.0) <= 1e-6):
                row["ax"] = float(row["ax_world_mps2"])
            if "ay_world_mps2" in row and ("ay" not in row or abs(_finite_or_none(row.get("ay")) or 0.0) <= 1e-6):
                row["ay"] = float(row["ay_world_mps2"])
            if "az_world_mps2" in row and ("az" not in row or abs(_finite_or_none(row.get("az")) or 0.0) <= 1e-6):
                row["az"] = float(row["az_world_mps2"])
            if "az_world_mps2" in row and "collective_response_up_mps2" not in row:
                row["collective_response_up_mps2"] = -float(row["az_world_mps2"])


def _detect_rotor_indices(rows: Iterable[dict[str, float | str]]) -> list[int]:
    indices: set[int] = set()
    for row in rows:
        for key in row.keys():
            match = re.match(r"^(?:motor|rotor|esc)_(\d+)(?:_|$)", str(key))
            if match:
                indices.add(int(match.group(1)))
    if not indices:
        return [0, 1, 2, 3]
    return sorted(indices)


def _zero_inertia(axis: str) -> AxisInertiaEstimate:
    return AxisInertiaEstimate(axis=axis, inertia_kgm2=0.0, sample_count=0, rmse_nm=0.0)


def _zero_drag(axis: str) -> DragEstimate:
    return DragEstimate(axis=axis, coefficient=0.0, sample_count=0, rmse_n=0.0)


def _zero_scalar() -> ScalarEstimate:
    return ScalarEstimate(value=0.0, sample_count=0, rmse=0.0)


def _zero_step_response_dynamics() -> StepResponseDynamicsEstimate:
    return StepResponseDynamicsEstimate(delay_s=_zero_scalar(), time_constant_s=_zero_scalar())


def _estimate_mass_from_collective_gain_anchor(
    specific_force_gain: ScalarEstimate,
    *,
    thrust_scale_n_per_cmd: float,
) -> HoverMassEstimate | None:
    if specific_force_gain.sample_count <= 0:
        return None
    gain = float(specific_force_gain.value)
    thrust_scale = float(thrust_scale_n_per_cmd)
    if not math.isfinite(gain) or not math.isfinite(thrust_scale):
        return None
    if abs(gain) <= 1e-6 or thrust_scale <= 0.0:
        return None

    mass_kg = thrust_scale / gain
    propagated_rmse = abs(thrust_scale) * float(specific_force_gain.rmse) / max(abs(gain) ** 2, 1e-9)
    return HoverMassEstimate(
        mass_kg=mass_kg,
        sample_count=specific_force_gain.sample_count,
        std_kg=propagated_rmse,
        gravity_mps2=GRAVITY_MPS2,
    )


def _truth_values(rows: list[dict[str, float]], column: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = row.get(column)
        if value is None:
            continue
        value = float(value)
        if math.isfinite(value):
            values.append(value)
    return values


def _truth_scalar_estimate(rows: list[dict[str, float]], column: str) -> ScalarEstimate | None:
    values = _truth_values(rows, column)
    if not values:
        return None
    values.sort()
    median = values[len(values) // 2]
    return ScalarEstimate(
        value=float(median),
        sample_count=len(values),
        rmse=_scalar_rmse(values, float(median)) if len(values) > 1 else 0.0,
    )


def _truth_mass_estimate(rows: list[dict[str, float]]) -> HoverMassEstimate | None:
    values = _truth_values(rows, "truth_mass_kg")
    if not values:
        return None
    values.sort()
    median = values[len(values) // 2]
    return HoverMassEstimate(
        mass_kg=float(median),
        sample_count=len(values),
        std_kg=0.0 if len(values) < 2 else _scalar_rmse(values, float(median)),
        gravity_mps2=GRAVITY_MPS2,
    )


def _truth_inertia_estimate(rows: list[dict[str, float]], axis: str) -> AxisInertiaEstimate | None:
    column = {
        "x": "truth_ixx_kgm2",
        "y": "truth_iyy_kgm2",
        "z": "truth_izz_kgm2",
    }[axis]
    values = _truth_values(rows, column)
    if not values:
        return None
    values.sort()
    median = values[len(values) // 2]
    return AxisInertiaEstimate(
        axis=axis,
        inertia_kgm2=float(median),
        sample_count=len(values),
        rmse_nm=0.0 if len(values) < 2 else _scalar_rmse(values, float(median)),
    )


def _safe_inertia_estimate(
    rows: list[dict[str, float]],
    axis: str,
    torque_column: str,
    angular_accel_column: str,
    warnings: list[str],
) -> AxisInertiaEstimate:
    if not rows:
        warnings.append(f"{axis}-axis inertia profile missing; inertia_{axis} set to 0.0 in this estimate.")
        return _zero_inertia(axis)

    simple_estimate: AxisInertiaEstimate | None = None
    coupled_estimate: AxisInertiaEstimate | None = None
    simple_error: ValueError | None = None
    coupled_error: ValueError | None = None

    try:
        simple_estimate = estimate_axis_inertia(
            rows,
            axis=axis,
            torque_column=torque_column,
            angular_accel_column=angular_accel_column,
        )
    except ValueError as exc:
        simple_error = exc

    try:
        coupled_estimate = estimate_axis_inertia_with_coupling(
            rows,
            axis=axis,
            torque_column=torque_column,
            angular_accel_column=angular_accel_column,
        )
    except ValueError as exc:
        coupled_error = exc

    if simple_estimate is None and coupled_estimate is None:
        detail = simple_error or coupled_error or ValueError("unknown error")
        warnings.append(f"{axis}-axis inertia estimate unusable ({detail}); inertia_{axis} set to 0.0 in this estimate.")
        return _zero_inertia(axis)

    if simple_estimate is None:
        warnings.append(f"{axis}-axis inertia estimate used coupling-aware fit because the simple fit was unavailable ({simple_error}).")
        return coupled_estimate  # type: ignore[return-value]

    if coupled_estimate is None:
        return simple_estimate

    simple_positive = simple_estimate.inertia_kgm2 > 0.0
    coupled_positive = coupled_estimate.inertia_kgm2 > 0.0
    if not simple_positive and coupled_positive:
        warnings.append(f"{axis}-axis inertia estimate used coupling-aware fit because the simple fit was non-physical.")
        return coupled_estimate

    if coupled_estimate.rmse_nm < simple_estimate.rmse_nm * 0.92:
        warnings.append(f"{axis}-axis inertia estimate used coupling-aware fit because it reduced torque-fit RMSE from {simple_estimate.rmse_nm:.4f} to {coupled_estimate.rmse_nm:.4f}.")
        return coupled_estimate

    return simple_estimate


def _safe_joint_inertia_estimate(
    roll_rows: list[dict[str, float]],
    pitch_rows: list[dict[str, float]],
    yaw_rows: list[dict[str, float]],
    warnings: list[str],
) -> DiagonalInertiaTensorEstimate | None:
    joint_rows = list(roll_rows) + list(pitch_rows) + list(yaw_rows)
    if not joint_rows:
        return None
    has_truth_torque = any(
        any(key in row for key in ("tau_x_nm", "tau_y_nm", "tau_z_nm"))
        for row in joint_rows
    )
    if not has_truth_torque:
        return None
    try:
        return estimate_diagonal_inertia_tensor(joint_rows)
    except ValueError as exc:
        warnings.append(f"joint inertia estimate unavailable ({exc}); falling back to axis-wise estimates.")
        return None


def _safe_drag_estimate(
    rows: list[dict[str, float]],
    axis: str,
    mass_kg: float,
    velocity_column: str,
    accel_column: str,
    warnings: list[str],
) -> DragEstimate:
    if not rows:
        warnings.append(f"drag_{axis} profile missing; drag_{axis} coefficient set to 0.0 in this estimate.")
        return _zero_drag(axis)
    try:
        return estimate_quadratic_drag(
            rows,
            axis=axis,
            mass_kg=mass_kg,
            velocity_column=velocity_column,
            accel_column=accel_column,
        )
    except ValueError as exc:
        warnings.append(f"drag_{axis} estimate unusable ({exc}); drag_{axis} coefficient set to 0.0 in this estimate.")
        return _zero_drag(axis)


def _safe_specific_drag_estimate(
    rows: list[dict[str, float]],
    axis: str,
    velocity_column: str,
    accel_column: str,
    warnings: list[str],
) -> ScalarEstimate:
    if not rows:
        warnings.append(f"specific_drag_{axis} profile missing; specific_drag_{axis} set to 0.0 in this estimate.")
        return _zero_scalar()
    try:
        return estimate_proportional_scale(
            [
                {
                    "feature": abs(float(row[velocity_column])) * float(row[velocity_column]),
                    "target": -float(row[accel_column]),
                }
                for row in rows
                if row.get(velocity_column) is not None
                and row.get(accel_column) is not None
                and math.isfinite(float(row[velocity_column]))
                and math.isfinite(float(row[accel_column]))
                and abs(float(row[velocity_column])) >= 0.1
            ],
            feature_column="feature",
            target_column="target",
            min_feature_abs=1e-6,
        )
    except ValueError as exc:
        warnings.append(
            f"specific_drag_{axis} estimate unusable ({exc}); specific_drag_{axis} set to 0.0 in this estimate."
        )
        return _zero_scalar()


def _safe_scalar_estimate(rows: list[dict[str, float]], label: str, warnings: list[str], func, *args, **kwargs) -> ScalarEstimate:
    if not rows:
        warnings.append(f"{label} profile missing; {label} set to 0.0 in this estimate.")
        return _zero_scalar()
    try:
        return func(rows, *args, **kwargs)
    except ValueError as exc:
        warnings.append(f"{label} estimate unusable ({exc}); {label} set to 0.0 in this estimate.")
        return _zero_scalar()


def _safe_step_response_dynamics_estimate(
    rows: list[dict[str, float]],
    *,
    direction: str,
    warnings: list[str],
) -> StepResponseDynamicsEstimate:
    if not rows:
        warnings.append(
            f"effective collective {direction} dynamics profile missing; effective collective {direction} dynamics set to 0.0 in this estimate."
        )
        return _zero_step_response_dynamics()
    try:
        return estimate_step_response_dynamics(
            rows,
            time_column="t_s",
            command_column="thrust_cmd",
            response_column="collective_response_up_mps2",
            direction=direction,
        )
    except ValueError as exc:
        warnings.append(
            f"effective collective {direction} dynamics estimate unusable ({exc}); effective collective {direction} dynamics set to 0.0 in this estimate."
        )
        return _zero_step_response_dynamics()


def _bridge_probe_axis_rows(
    rows: list[dict[str, float]],
    *,
    axis: str,
    sign: str = "all",
    min_ref_accel_abs: float = 0.20,
    dominance_ratio: float = 1.2,
) -> list[dict[str, float]]:
    axis = str(axis).lower()
    sign = str(sign).lower()
    if axis not in {"x", "y"}:
        return []

    feature_key = f"ref_a{axis}"
    other_axis = "y" if axis == "x" else "x"
    other_feature_key = f"ref_a{other_axis}"
    out: list[dict[str, float]] = []

    for row in rows:
        feature = row.get(feature_key)
        if feature is None:
            continue
        feature = float(feature)
        if not math.isfinite(feature) or abs(feature) < min_ref_accel_abs:
            continue

        other = float(row.get(other_feature_key, 0.0) or 0.0)
        vertical = float(row.get("ref_az", 0.0) or 0.0)
        if abs(feature) < dominance_ratio * max(abs(other), abs(vertical)):
            continue

        if sign == "positive" and feature <= 0.0:
            continue
        if sign == "negative" and feature >= 0.0:
            continue
        out.append(row)

    return out


def _safe_bridge_probe_axis_scale(
    rows: list[dict[str, float]],
    *,
    axis: str,
    feature_column: str,
    target_column: str,
    warnings: list[str],
    sign: str = "all",
    min_feature_abs: float = 0.05,
) -> ScalarEstimate:
    dominant_rows = _bridge_probe_axis_rows(rows, axis=axis, sign=sign)
    label = f"bridge_probe_{axis}_{target_column}_{sign}"
    if not dominant_rows:
        return _zero_scalar()
    try:
        return estimate_proportional_scale(
            dominant_rows,
            feature_column=feature_column,
            target_column=target_column,
            min_feature_abs=min_feature_abs,
        )
    except ValueError as exc:
        warnings.append(f"{label} unusable ({exc}); set to 0.0 in this estimate.")
        return _zero_scalar()


def _estimate_bridge_probe_observables(
    rows: list[dict[str, float]],
    warnings: list[str],
) -> dict[str, object]:
    if not rows:
        return {
            "available": False,
            "x": {
                "velocity_scale": _zero_scalar().as_dict(),
                "accel_scale": _zero_scalar().as_dict(),
                "accel_scale_positive": _zero_scalar().as_dict(),
                "accel_scale_negative": _zero_scalar().as_dict(),
            },
            "y": {
                "velocity_scale": _zero_scalar().as_dict(),
                "accel_scale": _zero_scalar().as_dict(),
                "accel_scale_positive": _zero_scalar().as_dict(),
                "accel_scale_negative": _zero_scalar().as_dict(),
            },
        }

    x_velocity_scale = _safe_bridge_probe_axis_scale(
        rows,
        axis="x",
        feature_column="ref_vx",
        target_column="vx_mps",
        warnings=warnings,
        min_feature_abs=0.05,
    )
    y_velocity_scale = _safe_bridge_probe_axis_scale(
        rows,
        axis="y",
        feature_column="ref_vy",
        target_column="vy_mps",
        warnings=warnings,
        min_feature_abs=0.05,
    )
    x_accel_scale = _safe_bridge_probe_axis_scale(
        rows,
        axis="x",
        feature_column="ref_ax",
        target_column="ax_world_mps2",
        warnings=warnings,
        min_feature_abs=0.10,
    )
    y_accel_scale = _safe_bridge_probe_axis_scale(
        rows,
        axis="y",
        feature_column="ref_ay",
        target_column="ay_world_mps2",
        warnings=warnings,
        min_feature_abs=0.10,
    )
    x_accel_scale_positive = _safe_bridge_probe_axis_scale(
        rows,
        axis="x",
        feature_column="ref_ax",
        target_column="ax_world_mps2",
        warnings=warnings,
        sign="positive",
        min_feature_abs=0.10,
    )
    x_accel_scale_negative = _safe_bridge_probe_axis_scale(
        rows,
        axis="x",
        feature_column="ref_ax",
        target_column="ax_world_mps2",
        warnings=warnings,
        sign="negative",
        min_feature_abs=0.10,
    )
    y_accel_scale_positive = _safe_bridge_probe_axis_scale(
        rows,
        axis="y",
        feature_column="ref_ay",
        target_column="ay_world_mps2",
        warnings=warnings,
        sign="positive",
        min_feature_abs=0.10,
    )
    y_accel_scale_negative = _safe_bridge_probe_axis_scale(
        rows,
        axis="y",
        feature_column="ref_ay",
        target_column="ay_world_mps2",
        warnings=warnings,
        sign="negative",
        min_feature_abs=0.10,
    )

    return {
        "available": True,
        "x": {
            "velocity_scale": x_velocity_scale.as_dict(),
            "accel_scale": x_accel_scale.as_dict(),
            "accel_scale_positive": x_accel_scale_positive.as_dict(),
            "accel_scale_negative": x_accel_scale_negative.as_dict(),
        },
        "y": {
            "velocity_scale": y_velocity_scale.as_dict(),
            "accel_scale": y_accel_scale.as_dict(),
            "accel_scale_positive": y_accel_scale_positive.as_dict(),
            "accel_scale_negative": y_accel_scale_negative.as_dict(),
        },
    }


def _safe_step_response_estimate(
    rows: list[dict[str, float]],
    *,
    label: str,
    command_column: str,
    response_column: str,
    direction: str,
    warnings: list[str],
    min_step_abs: float = 0.15,
    min_step_fraction_of_range: float = 0.2,
    min_response_abs: float = 0.1,
    max_event_window_s: float = 0.60,
) -> StepResponseDynamicsEstimate:
    if not rows:
        return _zero_step_response_dynamics()
    try:
        return estimate_step_response_dynamics(
            rows,
            time_column="t_s",
            command_column=command_column,
            response_column=response_column,
            direction=direction,
            min_step_abs=min_step_abs,
            min_step_fraction_of_range=min_step_fraction_of_range,
            min_response_abs=min_response_abs,
            max_event_window_s=max_event_window_s,
        )
    except ValueError as exc:
        warnings.append(f"{label} unusable ({exc}); set to 0.0 in this estimate.")
        return _zero_step_response_dynamics()


def _estimate_sample_period(rows: list[dict[str, float]]) -> dict[str, float]:
    if len(rows) < 2:
        return {"mean_dt_s": 0.0, "std_dt_s": 0.0, "sample_count": max(0, len(rows))}
    times = sorted(
        float(row["t_s"])
        for row in rows
        if row.get("t_s") is not None and math.isfinite(float(row["t_s"]))
    )
    if len(times) < 2:
        return {"mean_dt_s": 0.0, "std_dt_s": 0.0, "sample_count": len(times)}
    dts = [
        b - a
        for a, b in zip(times[:-1], times[1:])
        if math.isfinite(a) and math.isfinite(b) and (b - a) > 1e-6
    ]
    if not dts:
        return {"mean_dt_s": 0.0, "std_dt_s": 0.0, "sample_count": len(times)}
    mean_dt = sum(dts) / len(dts)
    variance = sum((dt - mean_dt) ** 2 for dt in dts) / max(1, len(dts) - 1)
    return {
        "mean_dt_s": float(mean_dt),
        "std_dt_s": float(math.sqrt(max(variance, 0.0))),
        "sample_count": int(len(dts)),
    }


def _mean_abs_estimate(rows: list[dict[str, float]], column: str) -> ScalarEstimate:
    values = [
        abs(float(row[column]))
        for row in rows
        if row.get(column) is not None and math.isfinite(float(row[column]))
    ]
    if not values:
        return _zero_scalar()
    mean_value = sum(values) / len(values)
    return ScalarEstimate(
        value=float(mean_value),
        sample_count=len(values),
        rmse=_scalar_rmse(values, mean_value) if len(values) > 1 else 0.0,
    )


def _safe_correlation_lag_estimate(
    rows: list[dict[str, float]],
    *,
    feature_column: str,
    target_column: str,
    label: str,
    warnings: list[str],
    min_feature_abs: float = 0.05,
    max_lag_s: float = 0.25,
) -> ScalarEstimate:
    seq = sorted(
        (
            (float(row["t_s"]), float(row[feature_column]), float(row[target_column]))
            for row in rows
            if row.get("t_s") is not None
            and row.get(feature_column) is not None
            and row.get(target_column) is not None
            and math.isfinite(float(row["t_s"]))
            and math.isfinite(float(row[feature_column]))
            and math.isfinite(float(row[target_column]))
        ),
        key=lambda item: item[0],
    )
    if len(seq) < 12:
        return _zero_scalar()

    times = [item[0] for item in seq]
    features = [item[1] for item in seq]
    targets = [item[2] for item in seq]
    dt_stats = _estimate_sample_period(rows)
    mean_dt = float(dt_stats["mean_dt_s"]) if dt_stats["mean_dt_s"] > 1e-6 else 0.01
    lag_step = max(0.005, min(0.02, mean_dt * 0.5))

    def _interp_feature(query_t: float) -> float | None:
        if query_t < times[0] or query_t > times[-1]:
            return None
        lo = 0
        hi = len(times) - 1
        while lo <= hi:
            mid = (lo + hi) // 2
            if times[mid] < query_t:
                lo = mid + 1
            elif times[mid] > query_t:
                hi = mid - 1
            else:
                return features[mid]
        upper = min(len(times) - 1, lo)
        lower = max(0, upper - 1)
        if lower == upper:
            return features[lower]
        t0 = times[lower]
        t1 = times[upper]
        if abs(t1 - t0) <= 1e-9:
            return features[lower]
        alpha = (query_t - t0) / (t1 - t0)
        return features[lower] + alpha * (features[upper] - features[lower])

    best_lag: float | None = None
    best_rmse = math.inf
    best_count = 0
    lag = 0.0
    while lag <= max_lag_s + 1e-9:
        aligned_features: list[float] = []
        aligned_targets: list[float] = []
        for t_s, _feature, target in zip(times, features, targets):
            shifted = _interp_feature(t_s - lag)
            if shifted is None or abs(shifted) < min_feature_abs:
                continue
            aligned_features.append(float(shifted))
            aligned_targets.append(float(target))
        if len(aligned_features) >= max(10, len(seq) // 6):
            denominator = sum(value * value for value in aligned_features)
            if denominator > 1e-9:
                scale = sum(
                    feature * target for feature, target in zip(aligned_features, aligned_targets)
                ) / denominator
                rmse = math.sqrt(
                    sum((scale * feature - target) ** 2 for feature, target in zip(aligned_features, aligned_targets))
                    / len(aligned_features)
                )
                if rmse < best_rmse:
                    best_rmse = rmse
                    best_lag = lag
                    best_count = len(aligned_features)
        lag += lag_step

    if best_lag is None:
        warnings.append(f"{label} unusable (no valid lag alignment); set to 0.0 in this estimate.")
        return _zero_scalar()

    return ScalarEstimate(
        value=float(best_lag),
        sample_count=int(best_count),
        rmse=float(best_rmse),
    )


def _estimate_state_chain_observables(
    bridge_probe_rows: list[dict[str, float]],
    warnings: list[str],
) -> dict[str, object]:
    if not bridge_probe_rows:
        return {
            "bridge_probe_xy": {
                "available": False,
                "sampling": _estimate_sample_period([]),
                "x": {
                    "velocity_correlation_lag_s": _zero_scalar().as_dict(),
                    "accel_correlation_lag_s": _zero_scalar().as_dict(),
                    "up": _zero_step_response_dynamics().as_dict(),
                    "down": _zero_step_response_dynamics().as_dict(),
                },
                "y": {
                    "velocity_correlation_lag_s": _zero_scalar().as_dict(),
                    "accel_correlation_lag_s": _zero_scalar().as_dict(),
                    "up": _zero_step_response_dynamics().as_dict(),
                    "down": _zero_step_response_dynamics().as_dict(),
                },
            }
        }

    x_rows = _bridge_probe_axis_rows(
        bridge_probe_rows,
        axis="x",
        min_ref_accel_abs=0.15,
        dominance_ratio=1.05,
    )
    y_rows = _bridge_probe_axis_rows(
        bridge_probe_rows,
        axis="y",
        min_ref_accel_abs=0.15,
        dominance_ratio=1.05,
    )

    x_up = _safe_step_response_estimate(
        x_rows,
        label="bridge_probe_state_chain_x_up",
        command_column="ref_ax",
        response_column="ax_world_mps2",
        direction="up",
        warnings=warnings,
    )
    x_down = _safe_step_response_estimate(
        x_rows,
        label="bridge_probe_state_chain_x_down",
        command_column="ref_ax",
        response_column="ax_world_mps2",
        direction="down",
        warnings=warnings,
    )
    y_up = _safe_step_response_estimate(
        y_rows,
        label="bridge_probe_state_chain_y_up",
        command_column="ref_ay",
        response_column="ay_world_mps2",
        direction="up",
        warnings=warnings,
    )
    y_down = _safe_step_response_estimate(
        y_rows,
        label="bridge_probe_state_chain_y_down",
        command_column="ref_ay",
        response_column="ay_world_mps2",
        direction="down",
        warnings=warnings,
    )
    x_velocity_lag = _safe_correlation_lag_estimate(
        x_rows,
        feature_column="ref_vx",
        target_column="vx_mps",
        label="bridge_probe_state_chain_x_velocity_lag",
        warnings=warnings,
        min_feature_abs=0.05,
    )
    y_velocity_lag = _safe_correlation_lag_estimate(
        y_rows,
        feature_column="ref_vy",
        target_column="vy_mps",
        label="bridge_probe_state_chain_y_velocity_lag",
        warnings=warnings,
        min_feature_abs=0.05,
    )
    x_accel_lag = _safe_correlation_lag_estimate(
        x_rows,
        feature_column="ref_ax",
        target_column="ax_world_mps2",
        label="bridge_probe_state_chain_x_accel_lag",
        warnings=warnings,
        min_feature_abs=0.10,
    )
    y_accel_lag = _safe_correlation_lag_estimate(
        y_rows,
        feature_column="ref_ay",
        target_column="ay_world_mps2",
        label="bridge_probe_state_chain_y_accel_lag",
        warnings=warnings,
        min_feature_abs=0.10,
    )

    return {
        "bridge_probe_xy": {
            "available": True,
            "sampling": _estimate_sample_period(bridge_probe_rows),
            "x": {
                "velocity_correlation_lag_s": x_velocity_lag.as_dict(),
                "accel_correlation_lag_s": x_accel_lag.as_dict(),
                "up": x_up.as_dict(),
                "down": x_down.as_dict(),
            },
            "y": {
                "velocity_correlation_lag_s": y_velocity_lag.as_dict(),
                "accel_correlation_lag_s": y_accel_lag.as_dict(),
                "up": y_up.as_dict(),
                "down": y_down.as_dict(),
            },
        }
    }


def _estimate_control_effort_observables(
    bridge_probe_rows: list[dict[str, float]],
    roll_rows: list[dict[str, float]],
    pitch_rows: list[dict[str, float]],
    yaw_rows: list[dict[str, float]],
) -> dict[str, object]:
    x_rows = _bridge_probe_axis_rows(
        bridge_probe_rows,
        axis="x",
        min_ref_accel_abs=0.15,
        dominance_ratio=1.05,
    )
    y_rows = _bridge_probe_axis_rows(
        bridge_probe_rows,
        axis="y",
        min_ref_accel_abs=0.15,
        dominance_ratio=1.05,
    )

    return {
        "bridge_probe_xy": {
            "x_dominant": {
                "mean_abs_rate_sp_pitch": _mean_abs_estimate(x_rows, "rate_sp_pitch").as_dict(),
                "mean_abs_torque_sp_pitch": _mean_abs_estimate(x_rows, "tau_y_nm").as_dict(),
                "mean_abs_alloc_unalloc_pitch": _mean_abs_estimate(x_rows, "alloc_unalloc_pitch").as_dict(),
                "mean_abs_pitch_rad": _mean_abs_estimate(x_rows, "pitch").as_dict(),
            },
            "y_dominant": {
                "mean_abs_rate_sp_roll": _mean_abs_estimate(y_rows, "rate_sp_roll").as_dict(),
                "mean_abs_torque_sp_roll": _mean_abs_estimate(y_rows, "tau_x_nm").as_dict(),
                "mean_abs_alloc_unalloc_roll": _mean_abs_estimate(y_rows, "alloc_unalloc_roll").as_dict(),
                "mean_abs_roll_rad": _mean_abs_estimate(y_rows, "roll").as_dict(),
            },
        },
        "attitude_sweeps": {
            "roll": {
                "mean_abs_rate_sp_roll": _mean_abs_estimate(roll_rows, "rate_sp_roll").as_dict(),
                "mean_abs_torque_sp_roll": _mean_abs_estimate(roll_rows, "tau_x_nm").as_dict(),
                "mean_abs_alloc_unalloc_roll": _mean_abs_estimate(roll_rows, "alloc_unalloc_roll").as_dict(),
            },
            "pitch": {
                "mean_abs_rate_sp_pitch": _mean_abs_estimate(pitch_rows, "rate_sp_pitch").as_dict(),
                "mean_abs_torque_sp_pitch": _mean_abs_estimate(pitch_rows, "tau_y_nm").as_dict(),
                "mean_abs_alloc_unalloc_pitch": _mean_abs_estimate(pitch_rows, "alloc_unalloc_pitch").as_dict(),
            },
            "yaw": {
                "mean_abs_rate_sp_yaw": _mean_abs_estimate(yaw_rows, "rate_sp_yaw").as_dict(),
                "mean_abs_torque_sp_yaw": _mean_abs_estimate(yaw_rows, "tau_z_nm").as_dict(),
                "mean_abs_alloc_unalloc_yaw": _mean_abs_estimate(yaw_rows, "alloc_unalloc_yaw").as_dict(),
            },
        },
        "allocation_global": {
            "mean_abs_alloc_torque_achieved": _mean_abs_estimate(bridge_probe_rows + roll_rows + pitch_rows + yaw_rows, "alloc_torque_achieved").as_dict(),
        },
    }


def _combine_scalar_estimates(estimates: Sequence[ScalarEstimate]) -> ScalarEstimate:
    usable = [estimate for estimate in estimates if estimate.sample_count > 0]
    if not usable:
        raise ValueError("no usable scalar estimates to combine")
    total_samples = sum(estimate.sample_count for estimate in usable)
    weighted_value = sum(estimate.value * estimate.sample_count for estimate in usable) / total_samples
    weighted_rmse = sum(estimate.rmse * estimate.sample_count for estimate in usable) / total_samples
    return ScalarEstimate(value=weighted_value, sample_count=total_samples, rmse=weighted_rmse)


def _extract_rotor_rows(
    rows: Iterable[dict[str, float]],
    rotor_indices: Sequence[int] | None = None,
) -> list[dict[str, float]]:
    rows = list(rows)
    out: list[dict[str, float]] = []
    rotor_indices = list(rotor_indices) if rotor_indices is not None else _detect_rotor_indices(rows)
    for row in rows:
        t_s = float(row.get("t_s", 0.0))
        run_index = float(row.get("run_index", 0.0))
        for rotor_index in rotor_indices:
            cmd_key = f"rotor_{rotor_index}_cmd_radps"
            joint_key = f"rotor_{rotor_index}_joint_vel_radps"
            actual_key = f"rotor_{rotor_index}_actual_radps"
            thrust_key = f"rotor_{rotor_index}_thrust_n"
            if not any(key in row for key in (cmd_key, joint_key, actual_key, thrust_key)):
                continue
            out.append({
                "rotor": float(rotor_index),
                "t_s": t_s,
                "run_index": run_index,
                "command_radps": float(row.get(cmd_key, math.nan)),
                "command_inferred": float(row.get(f"rotor_{rotor_index}_cmd_inferred", 0.0)),
                "joint_velocity_radps": abs(float(row.get(joint_key, math.nan))),
                "actual_velocity_radps": float(row.get(actual_key, math.nan)),
                "thrust_n": float(row.get(thrust_key, math.nan)),
            })
    return out


def _collapse_rotor_rows(per_rotor: list[dict[str, float]], *, time_epsilon_s: float = 1e-6) -> list[dict[str, float]]:
    if not per_rotor:
        return []

    per_rotor = sorted(
        per_rotor,
        key=lambda row: (
            float(row.get("run_index", 0.0)),
            float(row.get("t_s", 0.0)),
        ),
    )
    collapsed: list[dict[str, float]] = []

    for row in per_rotor:
        run_index = float(row.get("run_index", 0.0))
        t_s = float(row.get("t_s", 0.0))

        if (
            collapsed
            and abs(run_index - float(collapsed[-1].get("run_index", 0.0))) <= 1e-9
            and abs(t_s - float(collapsed[-1].get("t_s", 0.0))) <= time_epsilon_s
        ):
            current = collapsed[-1]
            current["command_radps"] = max(float(current.get("command_radps", math.nan)), float(row.get("command_radps", math.nan)))
            current["actual_velocity_radps"] = max(float(current.get("actual_velocity_radps", math.nan)), float(row.get("actual_velocity_radps", math.nan)))
            current["joint_velocity_radps"] = max(float(current.get("joint_velocity_radps", math.nan)), float(row.get("joint_velocity_radps", math.nan)))
            current["thrust_n"] = max(float(current.get("thrust_n", math.nan)), float(row.get("thrust_n", math.nan)))
            current["command_inferred"] = min(float(current.get("command_inferred", 1.0)), float(row.get("command_inferred", 1.0)))
            continue

        collapsed.append(dict(row))

    return collapsed


def _axis_projection_rows(
    rows: Iterable[dict[str, float]],
    *,
    target_prefix: str,
    feature_prefix: str,
    target_suffix: str = "",
    feature_suffix: str = "",
) -> list[dict[str, float]]:
    out: list[dict[str, float]] = []
    for row in rows:
        for axis in ("x", "y", "z"):
            target = row.get(f"{target_prefix}_{axis}{target_suffix}")
            feature = row.get(f"{feature_prefix}_{axis}{feature_suffix}")
            if target is None or feature is None:
                continue
            out.append({
                "feature": float(feature),
                "target": float(target),
            })
    return out


def _estimate_rotor_time_constant(rotor_rows: list[dict[str, float]], direction: str) -> ScalarEstimate:
    estimates: list[ScalarEstimate] = []

    def _fit_subset(per_rotor: list[dict[str, float]]) -> ScalarEstimate | None:
        if len(per_rotor) < 2:
            return None
        per_rotor = _collapse_rotor_rows(per_rotor)
        if len(per_rotor) < 2:
            return None

        max_command = max(abs(float(row.get("command_radps", 0.0))) for row in per_rotor)
        hold_tolerance = max(1.0, 0.002 * max_command)
        tau_samples: list[float] = []

        for prev, curr in zip(per_rotor[:-1], per_rotor[1:]):
            if abs(float(curr.get("run_index", 0.0)) - float(prev.get("run_index", 0.0))) > 1e-9:
                continue
            dt = float(curr.get("t_s", 0.0)) - float(prev.get("t_s", 0.0))
            if dt <= 1e-6:
                continue
            prev_command = float(prev.get("command_radps", math.nan))
            curr_command = float(curr.get("command_radps", math.nan))
            prev_actual = float(prev.get("actual_velocity_radps", math.nan))
            curr_actual = float(curr.get("actual_velocity_radps", math.nan))
            if not all(math.isfinite(value) for value in (prev_command, curr_command, prev_actual, curr_actual)):
                continue
            if abs(curr_command - prev_command) > hold_tolerance:
                continue

            error = prev_command - prev_actual
            if direction == "up" and error <= hold_tolerance:
                continue
            if direction == "down" and error >= -hold_tolerance:
                continue

            denom = prev_actual - prev_command
            numer = curr_actual - prev_command
            if abs(denom) <= 1e-6 or abs(numer) <= 1e-9:
                continue
            ratio = numer / denom
            if not (0.0 < ratio < 1.0):
                continue

            tau = -dt / math.log(ratio)
            if math.isfinite(tau) and 1e-4 < tau < 0.5:
                tau_samples.append(tau)

        if tau_samples:
            tau_samples.sort()
            lower_index = int(len(tau_samples) * 0.2)
            upper_index = max(lower_index + 1, int(math.ceil(len(tau_samples) * 0.8)))
            trimmed = tau_samples[lower_index:upper_index] or tau_samples
            tau_value = float(trimmed[len(trimmed) // 2])
            return ScalarEstimate(
                value=tau_value,
                sample_count=len(trimmed),
                rmse=_scalar_rmse(trimmed, tau_value) if len(trimmed) > 1 else 0.0,
            )

        try:
            return estimate_time_constant(
                per_rotor,
                time_column="t_s",
                command_column="command_radps",
                actual_column="actual_velocity_radps",
                direction=direction,
            )
        except ValueError:
            return None

    rotor_indices = sorted({int(row.get("rotor", -1)) for row in rotor_rows if int(row.get("rotor", -1)) >= 0})
    for rotor_index in rotor_indices:
        per_rotor_all = [row for row in rotor_rows if int(row.get("rotor", -1)) == rotor_index]
        if len(per_rotor_all) < 2:
            continue
        explicit_rows = [row for row in per_rotor_all if float(row.get("command_inferred", 0.0)) < 0.5]
        for candidate_rows in (explicit_rows, per_rotor_all):
            estimate = _fit_subset(candidate_rows)
            if estimate is not None:
                estimates.append(estimate)
                break
    return _combine_scalar_estimates(estimates)


def split_rows_by_profile(rows: Iterable[dict[str, float | str]]) -> Dict[str, list[dict[str, float]]]:
    grouped: Dict[str, list[dict[str, float]]] = {}
    for row in rows:
        profile = str(row.get("profile") or "unknown").strip() or "unknown"
        numeric = {key: float(value) for key, value in row.items() if key != "profile"}
        grouped.setdefault(profile, []).append(numeric)
    return grouped


def _combined_profile_rows(grouped: Dict[str, list[dict[str, float]]], *names: str) -> list[dict[str, float]]:
    out: list[dict[str, float]] = []
    for name in names:
        out.extend(grouped.get(name) or [])
    return out


def estimate_parameters_from_identification_log(
    rows: Iterable[dict[str, float | str]],
    *,
    prior_hints: dict | None = None,
    prefer_prior_mass_anchor: bool = False,
) -> dict:
    normalized_rows = [dict(row) for row in rows]
    _apply_identification_aliases(normalized_rows)
    grouped = split_rows_by_profile(normalized_rows)
    warnings: list[str] = []
    prior_hints = dict(prior_hints or {})

    numeric_rows = [dict(row) for profile_rows in grouped.values() for row in profile_rows]
    hover_rows = _combined_profile_rows(grouped, "hover_thrust", "hover", "mass_vertical")
    specific_force_gain = _zero_scalar()
    hover_command_mean = _zero_scalar()
    mass_anchor_candidates: dict[str, dict[str, float | str | dict[str, float]]] = {}
    if hover_rows:
        try:
            specific_force_gain = estimate_proportional_scale(
                [
                    {
                        "feature": abs(float(row["thrust_cmd"])),
                        "target": GRAVITY_MPS2 + float(row["az_world_mps2"]),
                    }
                    for row in hover_rows
                    if row.get("thrust_cmd") is not None
                    and row.get("az_world_mps2") is not None
                    and math.isfinite(float(row["thrust_cmd"]))
                    and math.isfinite(float(row["az_world_mps2"]))
                    and abs(float(row["thrust_cmd"])) > 1e-6
                ],
                feature_column="feature",
                target_column="target",
                min_feature_abs=1e-6,
            )
        except ValueError:
            specific_force_gain = _zero_scalar()
        try:
            hover_command_values = [
                abs(float(row["thrust_cmd"]))
                for row in hover_rows
                if row.get("thrust_cmd") is not None and math.isfinite(float(row["thrust_cmd"]))
            ]
            hover_command_mean = ScalarEstimate(
                value=sum(hover_command_values) / len(hover_command_values),
                sample_count=len(hover_command_values),
                rmse=0.0,
            )
        except (ValueError, ZeroDivisionError):
            hover_command_mean = _zero_scalar()

        anchor_thrust_scale = prior_hints.get("thrust_scale_n_per_cmd")
        anchor_label = str(prior_hints.get("label") or "prior_thrust_scale_anchor")
        if anchor_thrust_scale is not None:
            anchored_mass = _estimate_mass_from_collective_gain_anchor(
                specific_force_gain,
                thrust_scale_n_per_cmd=float(anchor_thrust_scale),
            )
            if anchored_mass is not None:
                mass_anchor_candidates[anchor_label] = {
                    "method": "collective_specific_force_gain + prior thrust_scale",
                    "thrust_scale_n_per_cmd": float(anchor_thrust_scale),
                    "estimated_mass": anchored_mass.as_dict(),
                }
        truth_mass = _truth_mass_estimate(hover_rows) or _truth_mass_estimate(numeric_rows)
        hover_has_thrust = any("thrust_n" in row for row in hover_rows)
        if truth_mass is not None:
            mass = truth_mass
            thrust = estimate_thrust_scale(
                hover_rows,
                mass_kg=mass.mass_kg,
                command_column="thrust_cmd",
                accel_z_column="az_world_mps2",
            ) if hover_has_thrust else ThrustScaleEstimate(
                thrust_scale_n_per_cmd=0.0,
                sample_count=0,
                rmse_n=0.0,
                gravity_mps2=GRAVITY_MPS2,
            )
            warnings.append("mass estimate used simulator truth inertia data in truth_assisted mode.")
        elif hover_has_thrust:
            mass = estimate_hover_mass(
                hover_rows,
                thrust_column="thrust_n",
                accel_z_column="az_world_mps2",
            )
            thrust = estimate_thrust_scale(
                hover_rows,
                mass_kg=mass.mass_kg,
                command_column="thrust_cmd",
                accel_z_column="az_world_mps2",
            )
        else:
            anchored_mass = None
            if prefer_prior_mass_anchor and anchor_thrust_scale is not None:
                anchored_mass = _estimate_mass_from_collective_gain_anchor(
                    specific_force_gain,
                    thrust_scale_n_per_cmd=float(anchor_thrust_scale),
                )
            if anchored_mass is not None:
                warnings.append(
                    "hover_thrust samples do not contain thrust_n; absolute mass is not directly observable from "
                    "normalized thrust alone. Using the research-mode prior thrust-scale anchor to convert the "
                    "identified collective specific-force gain into an estimated mass."
                )
                mass = anchored_mass
            else:
                warnings.append(
                    "hover_thrust samples do not contain thrust_n; absolute mass is not directly observable from "
                    "normalized thrust alone. Using a nominal 1.0 kg fallback while also reporting the indirect "
                    "collective specific-force gain."
                )
                mass = HoverMassEstimate(
                    mass_kg=1.0,
                    sample_count=len(hover_rows),
                    std_kg=0.0,
                    gravity_mps2=GRAVITY_MPS2,
                )
            thrust = estimate_thrust_scale(
                hover_rows,
                mass_kg=mass.mass_kg,
                command_column="thrust_cmd",
                accel_z_column="az_world_mps2",
            )
    else:
        warnings.append(
            "identification log does not contain hover_thrust/mass_vertical samples; "
            "using nominal fallbacks for mass and thrust-scale while estimating the remaining families."
        )
        mass = HoverMassEstimate(
            mass_kg=1.0,
            sample_count=0,
            std_kg=0.0,
            gravity_mps2=GRAVITY_MPS2,
        )
        thrust = ThrustScaleEstimate(
            thrust_scale_n_per_cmd=0.0,
            sample_count=0,
            rmse_n=0.0,
            gravity_mps2=GRAVITY_MPS2,
        )

    roll_rows = _combined_profile_rows(grouped, "roll_sweep", "roll_step")
    pitch_rows = _combined_profile_rows(grouped, "pitch_sweep", "pitch_step")
    yaw_rows = _combined_profile_rows(grouped, "yaw_sweep", "yaw_step")
    drag_x_rows = grouped.get("drag_x") or []
    drag_y_rows = grouped.get("drag_y") or []
    drag_z_rows = grouped.get("drag_z") or []
    bridge_probe_rows = grouped.get("bridge_probe_xy") or []

    truth_inertia_x = _truth_inertia_estimate(numeric_rows, "x")
    truth_inertia_y = _truth_inertia_estimate(numeric_rows, "y")
    truth_inertia_z = _truth_inertia_estimate(numeric_rows, "z")
    inertia_x = truth_inertia_x or _safe_inertia_estimate(
        roll_rows,
        "x",
        "tau_x_nm" if any("tau_x_nm" in row for row in roll_rows) else "roll_torque_proxy",
        "p_dot_radps2",
        warnings,
    )
    inertia_y = truth_inertia_y or _safe_inertia_estimate(
        pitch_rows,
        "y",
        "tau_y_nm" if any("tau_y_nm" in row for row in pitch_rows) else "pitch_torque_proxy",
        "q_dot_radps2",
        warnings,
    )
    inertia_z = truth_inertia_z or _safe_inertia_estimate(
        yaw_rows,
        "z",
        "tau_z_nm" if any("tau_z_nm" in row for row in yaw_rows) else "yaw_torque_proxy",
        "r_dot_radps2",
        warnings,
    )
    if truth_inertia_x or truth_inertia_y or truth_inertia_z:
        warnings.append("inertia estimate used simulator truth inertia data in truth_assisted mode.")

    joint_inertia = _safe_joint_inertia_estimate(roll_rows, pitch_rows, yaw_rows, warnings)
    if joint_inertia is not None:
        axis_pairs = (
            ("x", joint_inertia.x),
            ("y", joint_inertia.y),
            ("z", joint_inertia.z),
        )
        for axis_name, joint_axis in axis_pairs:
            current = {"x": inertia_x, "y": inertia_y, "z": inertia_z}[axis_name]
            choose_joint = False

            if current.sample_count <= 0 and joint_axis.sample_count > 0:
                choose_joint = True
            elif current.inertia_kgm2 <= 0.0 < joint_axis.inertia_kgm2:
                choose_joint = True
            elif joint_axis.sample_count > 0 and joint_axis.rmse_nm < current.rmse_nm * 0.95:
                choose_joint = True

            if choose_joint:
                warnings.append(
                    f"{axis_name}-axis inertia estimate used joint diagonal fit because it outperformed the axis-wise fit "
                    f"(rmse {current.rmse_nm:.4f} -> {joint_axis.rmse_nm:.4f})."
                )
                if axis_name == "x":
                    inertia_x = joint_axis
                elif axis_name == "y":
                    inertia_y = joint_axis
                else:
                    inertia_z = joint_axis

    drag_x = _safe_drag_estimate(drag_x_rows, "x", mass.mass_kg, "vx_mps", "ax_drag_mps2", warnings)
    drag_y = _safe_drag_estimate(drag_y_rows, "y", mass.mass_kg, "vy_mps", "ay_drag_mps2", warnings)
    drag_z = _safe_drag_estimate(drag_z_rows, "z", mass.mass_kg, "vz_mps", "az_drag_mps2", warnings)
    specific_drag_x = _safe_specific_drag_estimate(drag_x_rows, "x", "vx_mps", "ax_drag_mps2", warnings)
    specific_drag_y = _safe_specific_drag_estimate(drag_y_rows, "y", "vy_mps", "ay_drag_mps2", warnings)
    specific_drag_z = _safe_specific_drag_estimate(drag_z_rows, "z", "vz_mps", "az_drag_mps2", warnings)
    bridge_probe = _estimate_bridge_probe_observables(bridge_probe_rows, warnings)
    state_chain = _estimate_state_chain_observables(bridge_probe_rows, warnings)
    control_effort = _estimate_control_effort_observables(bridge_probe_rows, roll_rows, pitch_rows, yaw_rows)

    motor_rows = _combined_profile_rows(grouped, "motor_step", "actuator_lag_collective")
    rotor_rows = _extract_rotor_rows(numeric_rows)
    motor_rotor_rows = _extract_rotor_rows(motor_rows)
    rotor_rows_for_motor_dynamics = motor_rotor_rows if motor_rotor_rows else rotor_rows
    drag_coeff_rows = _axis_projection_rows(
        numeric_rows,
        target_prefix="drag_force_body",
        feature_prefix="drag_basis_body",
        target_suffix="_n",
    )
    rolling_coeff_rows = _axis_projection_rows(
        numeric_rows,
        target_prefix="rolling_moment_body",
        feature_prefix="rolling_basis_body",
        target_suffix="_nm",
    )

    truth_motor_constant = _truth_scalar_estimate(numeric_rows, "truth_motor_constant")
    motor_constant = truth_motor_constant or _safe_scalar_estimate(
        [
            {
                "omega_sq": row["actual_velocity_radps"] * row["actual_velocity_radps"],
                "thrust_n": row["thrust_n"],
            }
            for row in rotor_rows_for_motor_dynamics
            if math.isfinite(row.get("actual_velocity_radps", math.nan))
            and math.isfinite(row.get("thrust_n", math.nan))
        ],
        "motor_constant",
        warnings,
        estimate_proportional_scale,
        feature_column="omega_sq",
        target_column="thrust_n",
        min_feature_abs=1e-6,
    )
    truth_moment_constant = _truth_scalar_estimate(numeric_rows, "truth_moment_constant")
    moment_constant = truth_moment_constant or _safe_scalar_estimate(
        yaw_rows if yaw_rows else numeric_rows,
        "moment_constant",
        warnings,
        estimate_ratio,
        numerator_column="tau_z_nm",
        denominator_column="yaw_moment_basis_n",
        min_denominator_abs=1e-6,
    )
    truth_rotor_drag = _truth_scalar_estimate(numeric_rows, "truth_rotor_drag_coefficient")
    rotor_drag_coefficient = truth_rotor_drag or _safe_scalar_estimate(
        drag_coeff_rows,
        "rotor_drag_coefficient",
        warnings,
        estimate_proportional_scale,
        feature_column="feature",
        target_column="target",
        min_feature_abs=1e-6,
    )
    truth_rolling_moment = _truth_scalar_estimate(numeric_rows, "truth_rolling_moment_coefficient")
    rolling_moment_coefficient = truth_rolling_moment or _safe_scalar_estimate(
        rolling_coeff_rows,
        "rolling_moment_coefficient",
        warnings,
        estimate_proportional_scale,
        feature_column="feature",
        target_column="target",
        min_feature_abs=1e-6,
    )
    truth_rotor_slowdown = _truth_scalar_estimate(numeric_rows, "truth_rotor_velocity_slowdown_sim")
    rotor_velocity_slowdown = truth_rotor_slowdown or _safe_scalar_estimate(
        [
            {
                "numerator": row["actual_velocity_radps"],
                "denominator": row["joint_velocity_radps"],
            }
            for row in rotor_rows_for_motor_dynamics
            if math.isfinite(row.get("actual_velocity_radps", math.nan))
            and math.isfinite(row.get("joint_velocity_radps", math.nan))
            and abs(row.get("joint_velocity_radps", 0.0)) > 1e-6
        ],
        "rotor_velocity_slowdown_sim",
        warnings,
        estimate_ratio,
        numerator_column="numerator",
        denominator_column="denominator",
        min_denominator_abs=1e-6,
    )
    truth_max_rot_velocity = _truth_scalar_estimate(numeric_rows, "truth_max_rot_velocity_radps")
    max_rot_velocity = truth_max_rot_velocity or _safe_scalar_estimate(
        motor_rows if motor_rows else numeric_rows,
        "max_rot_velocity_radps",
        warnings,
        estimate_max_value,
        column="observed_max_rot_velocity_radps",
    )
    truth_time_constant_up = _truth_scalar_estimate(numeric_rows, "truth_time_constant_up_s")
    truth_time_constant_down = _truth_scalar_estimate(numeric_rows, "truth_time_constant_down_s")
    time_constant_up = truth_time_constant_up or _safe_scalar_estimate(rotor_rows_for_motor_dynamics, "time_constant_up_s", warnings, _estimate_rotor_time_constant, "up")
    time_constant_down = truth_time_constant_down or _safe_scalar_estimate(rotor_rows_for_motor_dynamics, "time_constant_down_s", warnings, _estimate_rotor_time_constant, "down")
    effective_collective_up = _safe_step_response_dynamics_estimate(motor_rows, direction="up", warnings=warnings)
    effective_collective_down = _safe_step_response_dynamics_estimate(motor_rows, direction="down", warnings=warnings)
    if truth_time_constant_up is None and time_constant_up.sample_count <= 0 and effective_collective_up.time_constant_s.sample_count > 0:
        time_constant_up = ScalarEstimate(
            value=float(effective_collective_up.time_constant_s.value),
            sample_count=int(effective_collective_up.time_constant_s.sample_count),
            rmse=float(effective_collective_up.time_constant_s.rmse),
        )
        warnings.append(
            "time_constant_up_s fell back to effective collective step-response dynamics because direct rotor telemetry was unavailable."
        )
    if truth_time_constant_down is None and time_constant_down.sample_count <= 0 and effective_collective_down.time_constant_s.sample_count > 0:
        time_constant_down = ScalarEstimate(
            value=float(effective_collective_down.time_constant_s.value),
            sample_count=int(effective_collective_down.time_constant_s.sample_count),
            rmse=float(effective_collective_down.time_constant_s.rmse),
        )
        warnings.append(
            "time_constant_down_s fell back to effective collective step-response dynamics because direct rotor telemetry was unavailable."
        )
    if any(value is not None for value in (
        truth_motor_constant,
        truth_moment_constant,
        truth_rotor_drag,
        truth_rolling_moment,
        truth_rotor_slowdown,
        truth_max_rot_velocity,
        truth_time_constant_up,
        truth_time_constant_down,
    )):
        warnings.append("motor-model estimate used simulator truth configuration fields in truth_assisted mode.")

    return {
        "mass": mass.as_dict(),
        "thrust_scale": thrust.as_dict(),
        "observability": {
            "absolute_mass_observable_from_current_logs": bool(hover_rows) and any("thrust_n" in row for row in hover_rows),
            "mass_anchor_candidates": mass_anchor_candidates,
            "prefer_prior_mass_anchor": bool(prefer_prior_mass_anchor),
            "prior_hints": prior_hints,
        },
        "indirect_observables": {
            "collective_specific_force_gain_mps2_per_cmd": specific_force_gain.as_dict(),
            "hover_command_abs_mean": hover_command_mean.as_dict(),
            "specific_drag": {
                "x": specific_drag_x.as_dict(),
                "y": specific_drag_y.as_dict(),
                "z": specific_drag_z.as_dict(),
            },
            "bridge_probe": bridge_probe,
        },
        "state_chain": state_chain,
        "control_effort": control_effort,
        "inertia": {
            "x": inertia_x.as_dict(),
            "y": inertia_y.as_dict(),
            "z": inertia_z.as_dict(),
        },
        "drag": {
            "x": drag_x.as_dict(),
            "y": drag_y.as_dict(),
            "z": drag_z.as_dict(),
        },
        "motor_model": {
            "time_constant_up_s": time_constant_up.as_dict(),
            "time_constant_down_s": time_constant_down.as_dict(),
            "max_rot_velocity_radps": max_rot_velocity.as_dict(),
            "motor_constant": motor_constant.as_dict(),
            "moment_constant": moment_constant.as_dict(),
            "rotor_drag_coefficient": rotor_drag_coefficient.as_dict(),
            "rolling_moment_coefficient": rolling_moment_coefficient.as_dict(),
            "rotor_velocity_slowdown_sim": rotor_velocity_slowdown.as_dict(),
        },
        "effective_actuator_dynamics": {
            "collective_up": effective_collective_up.as_dict(),
            "collective_down": effective_collective_down.as_dict(),
        },
        "warnings": warnings,
    }
