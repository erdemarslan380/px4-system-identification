"""Simple estimators for Gazebo SDF parameter identification from flight data."""

from __future__ import annotations

import csv
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable


GRAVITY_MPS2 = 9.80665


def load_numeric_csv(path: str | Path) -> list[dict[str, float]]:
    """Load a CSV file and keep only numeric fields for each row."""

    rows: list[dict[str, float]] = []
    with Path(path).open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for raw_row in reader:
            row: dict[str, float] = {}
            for key, value in raw_row.items():
                if value is None:
                    continue
                text = str(value).strip()
                if not text:
                    continue
                try:
                    row[key] = float(text)
                except ValueError:
                    continue
            if row:
                rows.append(row)
    return rows


def _mean(values: Iterable[float]) -> float:
    seq = list(values)
    if not seq:
        raise ValueError("at least one sample is required")
    return sum(seq) / len(seq)


def _std(values: Iterable[float], mean_value: float | None = None) -> float:
    seq = list(values)
    if len(seq) < 2:
        return 0.0
    mu = _mean(seq) if mean_value is None else float(mean_value)
    var = sum((value - mu) ** 2 for value in seq) / (len(seq) - 1)
    return math.sqrt(max(var, 0.0))


def _percentile(values: Iterable[float], q: float) -> float:
    seq = sorted(float(value) for value in values)
    if not seq:
        raise ValueError("at least one sample is required")
    quantile = min(1.0, max(0.0, float(q)))
    index = int(round((len(seq) - 1) * quantile))
    return seq[index]


def _rmse(prediction: Iterable[float], target: Iterable[float]) -> float:
    pred = list(prediction)
    truth = list(target)
    if len(pred) != len(truth) or not pred:
        raise ValueError("prediction and target must have equal non-zero length")
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(pred, truth)) / len(pred))


@dataclass
class HoverMassEstimate:
    mass_kg: float
    sample_count: int
    std_kg: float
    gravity_mps2: float = GRAVITY_MPS2

    def as_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass
class ThrustScaleEstimate:
    thrust_scale_n_per_cmd: float
    sample_count: int
    rmse_n: float
    gravity_mps2: float = GRAVITY_MPS2

    def as_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass
class AxisInertiaEstimate:
    axis: str
    inertia_kgm2: float
    sample_count: int
    rmse_nm: float

    def as_dict(self) -> dict[str, float | str]:
        return asdict(self)


@dataclass
class DiagonalInertiaTensorEstimate:
    x: AxisInertiaEstimate
    y: AxisInertiaEstimate
    z: AxisInertiaEstimate
    total_sample_count: int

    def as_dict(self) -> dict[str, dict[str, float | str] | int]:
        return {
            "x": self.x.as_dict(),
            "y": self.y.as_dict(),
            "z": self.z.as_dict(),
            "total_sample_count": self.total_sample_count,
        }


@dataclass
class DragEstimate:
    axis: str
    coefficient: float
    sample_count: int
    rmse_n: float

    def as_dict(self) -> dict[str, float | str]:
        return asdict(self)


@dataclass
class ScalarEstimate:
    value: float
    sample_count: int
    rmse: float

    def as_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass
class StepResponseDynamicsEstimate:
    delay_s: ScalarEstimate
    time_constant_s: ScalarEstimate

    def as_dict(self) -> dict[str, dict[str, float]]:
        return {
            "delay_s": self.delay_s.as_dict(),
            "time_constant_s": self.time_constant_s.as_dict(),
        }


def estimate_hover_mass(
    rows: Iterable[dict[str, float]],
    *,
    thrust_column: str = "thrust_n",
    accel_z_column: str = "az_world_mps2",
    roll_column: str = "roll",
    pitch_column: str = "pitch",
    gravity_mps2: float = GRAVITY_MPS2,
    min_total_accel_mps2: float = 0.5,
    low_torque_quantile: float = 0.5,
    min_thrust_fraction_of_max: float = 0.2,
) -> HoverMassEstimate:
    masses: list[float] = []
    torque_weighted: list[tuple[float, float, float]] = []
    thrust_values: list[float] = []
    for row in rows:
        thrust_n = row.get(thrust_column)
        accel_z = row.get(accel_z_column)
        if thrust_n is None or accel_z is None:
            continue
        thrust_vertical_n = float(thrust_n)
        roll = row.get(roll_column)
        pitch = row.get(pitch_column)
        if roll is not None and pitch is not None:
            roll = float(roll)
            pitch = float(pitch)
            if math.isfinite(roll) and math.isfinite(pitch):
                thrust_vertical_n *= math.cos(roll) * math.cos(pitch)
        total_accel = gravity_mps2 + accel_z
        if not math.isfinite(total_accel) or total_accel <= min_total_accel_mps2:
            continue
        mass_estimate = thrust_vertical_n / total_accel
        masses.append(mass_estimate)
        thrust_values.append(abs(thrust_vertical_n))
        torque_sum = 0.0
        torque_found = False
        for key in (
            "total_torque_body_x_nm",
            "total_torque_body_y_nm",
            "total_torque_body_z_nm",
            "tau_x_nm",
            "tau_y_nm",
            "tau_z_nm",
        ):
            value = row.get(key)
            if value is None:
                continue
            value = float(value)
            if not math.isfinite(value):
                continue
            torque_sum += abs(value)
            torque_found = True
        if torque_found:
            torque_weighted.append((abs(thrust_vertical_n), torque_sum, mass_estimate))
    if not masses:
        raise ValueError(f"no usable rows for hover-mass estimate using {thrust_column} and {accel_z_column}")

    fit_masses = list(masses)
    if thrust_values:
        max_thrust = max(thrust_values)
        thrust_floor = max_thrust * max(0.0, min(1.0, min_thrust_fraction_of_max))
        filtered = [mass for thrust, mass in zip(thrust_values, masses) if thrust >= thrust_floor]
        if len(filtered) >= max(10, len(masses) // 5):
            fit_masses = filtered
    if len(torque_weighted) >= 10:
        torque_weighted_filtered = list(torque_weighted)
        if thrust_values:
            max_thrust = max(thrust_values)
            thrust_floor = max_thrust * max(0.0, min(1.0, min_thrust_fraction_of_max))
            torque_weighted_filtered = [
                (thrust, torque, mass)
                for thrust, torque, mass in torque_weighted
                if thrust >= thrust_floor
            ] or torque_weighted
        torques = [torque for _, torque, _ in torque_weighted_filtered]
        candidate_quantiles = sorted(
            {
                max(0.1, min(0.9, low_torque_quantile - 0.2)),
                max(0.1, min(0.9, low_torque_quantile - 0.1)),
                max(0.1, min(0.9, low_torque_quantile)),
                max(0.1, min(0.9, low_torque_quantile + 0.1)),
            }
        )
        best_subset: list[float] | None = None
        best_cv = math.inf

        for quantile in candidate_quantiles:
            torque_cutoff = _percentile(torques, quantile)
            low_torque_masses = [mass for _, torque, mass in torque_weighted_filtered if torque <= torque_cutoff]
            if len(low_torque_masses) < max(10, len(masses) // 5):
                continue
            median_mass = _percentile(low_torque_masses, 0.5)
            allowed_delta = max(abs(median_mass) * 0.20, 0.05)
            clipped = [mass for mass in low_torque_masses if abs(mass - median_mass) <= allowed_delta]
            candidate = clipped if len(clipped) >= max(5, len(low_torque_masses) // 2) else low_torque_masses
            candidate_mean = _mean(candidate)
            candidate_std = _std(candidate, candidate_mean)
            candidate_cv = candidate_std / max(abs(candidate_mean), 1e-9)
            if candidate_cv < best_cv:
                best_cv = candidate_cv
                best_subset = candidate

        if best_subset:
            fit_masses = best_subset

    mean_mass = _mean(fit_masses)
    return HoverMassEstimate(
        mass_kg=mean_mass,
        sample_count=len(fit_masses),
        std_kg=_std(fit_masses, mean_mass),
        gravity_mps2=gravity_mps2,
    )


def estimate_thrust_scale(
    rows: Iterable[dict[str, float]],
    *,
    mass_kg: float,
    command_column: str = "thrust_cmd",
    accel_z_column: str = "az_world_mps2",
    gravity_mps2: float = GRAVITY_MPS2,
    min_command: float = 1e-6,
) -> ThrustScaleEstimate:
    commands: list[float] = []
    targets_n: list[float] = []
    for row in rows:
        command = row.get(command_column)
        accel_z = row.get(accel_z_column)
        if command is None or accel_z is None:
            continue
        if abs(command) <= min_command:
            continue
        commands.append(float(command))
        targets_n.append(float(mass_kg) * (gravity_mps2 + float(accel_z)))
    if not commands:
        raise ValueError(f"no usable rows for thrust-scale estimate using {command_column}")

    numerator = sum(cmd * tgt for cmd, tgt in zip(commands, targets_n))
    denominator = sum(cmd * cmd for cmd in commands)
    if denominator <= 0.0:
        raise ValueError("thrust command denominator is zero")
    scale = numerator / denominator
    prediction = [scale * cmd for cmd in commands]
    return ThrustScaleEstimate(
        thrust_scale_n_per_cmd=scale,
        sample_count=len(commands),
        rmse_n=_rmse(prediction, targets_n),
        gravity_mps2=gravity_mps2,
    )


def estimate_axis_inertia(
    rows: Iterable[dict[str, float]],
    *,
    axis: str,
    torque_column: str,
    angular_accel_column: str,
    min_angular_accel: float = 1e-4,
) -> AxisInertiaEstimate:
    torques: list[float] = []
    angular_accels: list[float] = []
    for row in rows:
        torque = row.get(torque_column)
        alpha = row.get(angular_accel_column)
        if torque is None or alpha is None:
            continue
        if abs(alpha) < min_angular_accel:
            continue
        torques.append(float(torque))
        angular_accels.append(float(alpha))
    if not torques:
        raise ValueError(f"no usable rows for inertia estimate on axis {axis}")
    numerator = sum(tau * alpha for tau, alpha in zip(torques, angular_accels))
    denominator = sum(alpha * alpha for alpha in angular_accels)
    if denominator <= 0.0:
        raise ValueError(f"degenerate angular-acceleration fit for axis {axis}")
    inertia = numerator / denominator
    prediction = [inertia * alpha for alpha in angular_accels]
    return AxisInertiaEstimate(
        axis=axis,
        inertia_kgm2=inertia,
        sample_count=len(torques),
        rmse_nm=_rmse(prediction, torques),
    )


def _solve_3x3_linear_system(a: list[list[float]], b: list[float]) -> list[float]:
    mat = [row[:] + [rhs] for row, rhs in zip(a, b)]
    n = 3
    for pivot in range(n):
        best = max(range(pivot, n), key=lambda row: abs(mat[row][pivot]))
        if abs(mat[best][pivot]) <= 1e-12:
            raise ValueError("singular inertia normal matrix")
        if best != pivot:
            mat[pivot], mat[best] = mat[best], mat[pivot]
        scale = mat[pivot][pivot]
        for col in range(pivot, n + 1):
            mat[pivot][col] /= scale
        for row in range(n):
            if row == pivot:
                continue
            factor = mat[row][pivot]
            if abs(factor) <= 1e-12:
                continue
            for col in range(pivot, n + 1):
                mat[row][col] -= factor * mat[pivot][col]
    return [mat[row][n] for row in range(n)]


def _solve_2x2_linear_system(a11: float, a12: float, a22: float, b1: float, b2: float) -> tuple[float, float]:
    det = a11 * a22 - a12 * a12
    if abs(det) <= 1e-12:
        raise ValueError("singular 2x2 normal matrix")
    x1 = (b1 * a22 - b2 * a12) / det
    x2 = (a11 * b2 - a12 * b1) / det
    return x1, x2


def estimate_axis_inertia_with_coupling(
    rows: Iterable[dict[str, float]],
    *,
    axis: str,
    torque_column: str,
    angular_accel_column: str,
    min_angular_accel: float = 1e-4,
) -> AxisInertiaEstimate:
    axis = str(axis).lower()
    if axis not in {"x", "y", "z"}:
        raise ValueError(f"unsupported axis {axis}")

    def _coupling_feature(row: dict[str, float]) -> float | None:
        if axis == "x":
            q = row.get("q")
            r = row.get("r")
            if q is None or r is None:
                return None
            return float(q) * float(r)
        if axis == "y":
            p = row.get("p")
            r = row.get("r")
            if p is None or r is None:
                return None
            return float(p) * float(r)
        p = row.get("p")
        q = row.get("q")
        if p is None or q is None:
            return None
        return float(p) * float(q)

    torques: list[float] = []
    angular_accels: list[float] = []
    coupling_terms: list[float] = []
    for row in rows:
        torque = row.get(torque_column)
        alpha = row.get(angular_accel_column)
        coupling = _coupling_feature(row)
        if torque is None or alpha is None or coupling is None:
            continue
        torque = float(torque)
        alpha = float(alpha)
        if not math.isfinite(torque) or not math.isfinite(alpha) or not math.isfinite(coupling):
            continue
        if abs(alpha) < min_angular_accel:
            continue
        torques.append(torque)
        angular_accels.append(alpha)
        coupling_terms.append(coupling)

    if not torques:
        raise ValueError(f"no usable rows for coupled inertia estimate on axis {axis}")

    a11 = sum(alpha * alpha for alpha in angular_accels)
    a12 = sum(alpha * coupling for alpha, coupling in zip(angular_accels, coupling_terms))
    a22 = sum(coupling * coupling for coupling in coupling_terms)
    b1 = sum(alpha * torque for alpha, torque in zip(angular_accels, torques))
    b2 = sum(coupling * torque for coupling, torque in zip(coupling_terms, torques))
    inertia, coupling_coeff = _solve_2x2_linear_system(a11, a12, a22, b1, b2)
    prediction = [
        inertia * alpha + coupling_coeff * coupling
        for alpha, coupling in zip(angular_accels, coupling_terms)
    ]
    return AxisInertiaEstimate(
        axis=axis,
        inertia_kgm2=inertia,
        sample_count=len(torques),
        rmse_nm=_rmse(prediction, torques),
    )


def estimate_diagonal_inertia_tensor(
    rows: Iterable[dict[str, float]],
    *,
    min_rate_abs: float = 1e-4,
    min_angular_accel_abs: float = 1e-4,
) -> DiagonalInertiaTensorEstimate:
    feature_rows: list[tuple[list[float], float, str]] = []
    axis_counts = {"x": 0, "y": 0, "z": 0}
    axis_truth: dict[str, list[float]] = {"x": [], "y": [], "z": []}
    axis_features: dict[str, list[list[float]]] = {"x": [], "y": [], "z": []}

    for row in rows:
        p = row.get("p")
        q = row.get("q")
        r = row.get("r")
        p_dot = row.get("p_dot_radps2")
        q_dot = row.get("q_dot_radps2")
        r_dot = row.get("r_dot_radps2")
        tau_x = row.get("tau_x_nm")
        tau_y = row.get("tau_y_nm")
        tau_z = row.get("tau_z_nm")
        values = (p, q, r, p_dot, q_dot, r_dot)
        if any(value is None or not math.isfinite(float(value)) for value in values):
            continue
        p = float(p)
        q = float(q)
        r = float(r)
        p_dot = float(p_dot)
        q_dot = float(q_dot)
        r_dot = float(r_dot)

        if tau_x is not None and math.isfinite(float(tau_x)) and abs(p_dot) >= min_angular_accel_abs:
            features = [p_dot, -q * r, q * r]
            feature_rows.append((features, float(tau_x), "x"))
            axis_truth["x"].append(float(tau_x))
            axis_features["x"].append(features)
            axis_counts["x"] += 1

        if tau_y is not None and math.isfinite(float(tau_y)) and abs(q_dot) >= min_angular_accel_abs:
            features = [p * r, q_dot, -p * r]
            feature_rows.append((features, float(tau_y), "y"))
            axis_truth["y"].append(float(tau_y))
            axis_features["y"].append(features)
            axis_counts["y"] += 1

        if tau_z is not None and math.isfinite(float(tau_z)) and abs(r_dot) >= min_angular_accel_abs:
            features = [-p * q, p * q, r_dot]
            feature_rows.append((features, float(tau_z), "z"))
            axis_truth["z"].append(float(tau_z))
            axis_features["z"].append(features)
            axis_counts["z"] += 1

    if not feature_rows:
        raise ValueError("no usable rows for joint inertia estimate")

    ata = [[0.0, 0.0, 0.0] for _ in range(3)]
    atb = [0.0, 0.0, 0.0]
    for features, target, _axis in feature_rows:
        for i in range(3):
            atb[i] += features[i] * target
            for j in range(3):
                ata[i][j] += features[i] * features[j]

    ixx, iyy, izz = _solve_3x3_linear_system(ata, atb)

    def _axis_estimate(axis: str, estimate_value: float) -> AxisInertiaEstimate:
        truths = axis_truth[axis]
        features = axis_features[axis]
        if not truths:
            raise ValueError(f"no usable rows for joint inertia estimate on axis {axis}")
        feature_index = {"x": 0, "y": 1, "z": 2}[axis]
        params = [ixx, iyy, izz]
        prediction = [
            row_features[0] * params[0] + row_features[1] * params[1] + row_features[2] * params[2]
            for row_features in features
        ]
        return AxisInertiaEstimate(
            axis=axis,
            inertia_kgm2=float(estimate_value),
            sample_count=axis_counts[axis],
            rmse_nm=_rmse(prediction, truths),
        )

    return DiagonalInertiaTensorEstimate(
        x=_axis_estimate("x", ixx),
        y=_axis_estimate("y", iyy),
        z=_axis_estimate("z", izz),
        total_sample_count=len(feature_rows),
    )


def estimate_quadratic_drag(
    rows: Iterable[dict[str, float]],
    *,
    axis: str,
    mass_kg: float,
    velocity_column: str,
    accel_column: str,
    min_speed_mps: float = 0.1,
) -> DragEstimate:
    features: list[float] = []
    forces_n: list[float] = []
    for row in rows:
        velocity = row.get(velocity_column)
        accel = row.get(accel_column)
        if velocity is None or accel is None:
            continue
        velocity = float(velocity)
        if abs(velocity) < min_speed_mps:
            continue
        features.append(abs(velocity) * velocity)
        forces_n.append(-float(mass_kg) * float(accel))
    if not features:
        raise ValueError(f"no usable rows for drag estimate on axis {axis}")
    numerator = sum(feature * force for feature, force in zip(features, forces_n))
    denominator = sum(feature * feature for feature in features)
    if denominator <= 0.0:
        raise ValueError(f"degenerate drag fit for axis {axis}")
    coefficient = numerator / denominator
    prediction = [coefficient * feature for feature in features]
    return DragEstimate(
        axis=axis,
        coefficient=coefficient,
        sample_count=len(features),
        rmse_n=_rmse(prediction, forces_n),
    )


def estimate_proportional_scale(
    rows: Iterable[dict[str, float]],
    *,
    feature_column: str,
    target_column: str,
    min_feature_abs: float = 1e-9,
) -> ScalarEstimate:
    features: list[float] = []
    targets: list[float] = []
    for row in rows:
        feature = row.get(feature_column)
        target = row.get(target_column)
        if feature is None or target is None:
            continue
        feature = float(feature)
        target = float(target)
        if not math.isfinite(feature) or not math.isfinite(target):
            continue
        if abs(feature) < min_feature_abs:
            continue
        features.append(feature)
        targets.append(target)
    if not features:
        raise ValueError(f"no usable rows for proportional estimate using {feature_column} -> {target_column}")
    numerator = sum(feature * target for feature, target in zip(features, targets))
    denominator = sum(feature * feature for feature in features)
    if denominator <= 0.0:
        raise ValueError("degenerate proportional fit")
    scale = numerator / denominator
    prediction = [scale * feature for feature in features]
    return ScalarEstimate(
        value=scale,
        sample_count=len(features),
        rmse=_rmse(prediction, targets),
    )


def estimate_ratio(
    rows: Iterable[dict[str, float]],
    *,
    numerator_column: str,
    denominator_column: str,
    min_denominator_abs: float = 1e-9,
) -> ScalarEstimate:
    ratios: list[float] = []
    for row in rows:
        numerator = row.get(numerator_column)
        denominator = row.get(denominator_column)
        if numerator is None or denominator is None:
            continue
        numerator = float(numerator)
        denominator = float(denominator)
        if not math.isfinite(numerator) or not math.isfinite(denominator):
            continue
        if abs(denominator) < min_denominator_abs:
            continue
        ratios.append(numerator / denominator)
    if not ratios:
        raise ValueError(f"no usable rows for ratio estimate using {numerator_column} / {denominator_column}")
    value = _mean(ratios)
    return ScalarEstimate(
        value=value,
        sample_count=len(ratios),
        rmse=_rmse(ratios, [value] * len(ratios)),
    )


def estimate_max_value(
    rows: Iterable[dict[str, float]],
    *,
    column: str,
) -> ScalarEstimate:
    values: list[float] = []
    for row in rows:
        value = row.get(column)
        if value is None:
            continue
        value = float(value)
        if math.isfinite(value):
            values.append(value)
    if not values:
        raise ValueError(f"no usable rows for max estimate using {column}")
    maximum = max(values)
    return ScalarEstimate(
        value=maximum,
        sample_count=len(values),
        rmse=0.0,
    )


def estimate_time_constant(
    rows: Iterable[dict[str, float]],
    *,
    time_column: str,
    command_column: str,
    actual_column: str,
    direction: str,
    min_step_abs: float = 5.0,
    min_step_fraction_of_range: float = 0.03,
    min_response_abs: float = 1.0,
    min_actual_fraction_of_max: float = 0.2,
    max_event_window_s: float = 0.5,
) -> ScalarEstimate:
    normalized_direction = str(direction).strip().lower()
    if normalized_direction not in {"up", "down"}:
        raise ValueError("direction must be 'up' or 'down'")

    seq = sorted(
        (
            {
                "t_s": float(row[time_column]),
                "command": float(row[command_column]),
                "actual": float(row[actual_column]),
            }
            for row in rows
            if row.get(time_column) is not None
            and row.get(command_column) is not None
            and row.get(actual_column) is not None
            and math.isfinite(float(row[time_column]))
            and math.isfinite(float(row[command_column]))
            and math.isfinite(float(row[actual_column]))
        ),
        key=lambda row: row["t_s"],
    )

    if len(seq) < 3:
        raise ValueError("at least two rows are required for time constant estimation")

    command_values = [row["command"] for row in seq]
    actual_values = [row["actual"] for row in seq]
    command_range = max(command_values) - min(command_values)
    step_threshold = max(float(min_step_abs), abs(command_range) * float(min_step_fraction_of_range))
    min_actual_abs = max(float(min_response_abs), max(abs(value) for value in actual_values) * float(min_actual_fraction_of_max))

    taus: list[float] = []
    event_rmses: list[float] = []
    index = 1

    while index < len(seq) - 1:
        prev = seq[index - 1]
        curr = seq[index]
        delta_command = curr["command"] - prev["command"]
        if normalized_direction == "up":
            if delta_command <= step_threshold:
                index += 1
                continue
        else:
            if delta_command >= -step_threshold:
                index += 1
                continue

        start_time = prev["t_s"]
        start_actual = prev["actual"]
        command_target = curr["command"]
        response_amplitude = command_target - start_actual

        if normalized_direction == "up":
            if response_amplitude <= min_response_abs:
                index += 1
                continue
        else:
            if response_amplitude >= -min_response_abs:
                index += 1
                continue

        target_actual = start_actual + 0.6321205588285577 * response_amplitude
        event_end = len(seq) - 1
        scan_limit = min(len(seq), index + 1)
        while scan_limit < len(seq):
            if seq[scan_limit]["t_s"] - start_time > max_event_window_s:
                break
            next_delta = seq[scan_limit]["command"] - seq[scan_limit - 1]["command"]
            if normalized_direction == "up" and next_delta > step_threshold:
                event_end = scan_limit - 1
                break
            if normalized_direction == "down" and next_delta < -step_threshold:
                event_end = scan_limit - 1
                break
            scan_limit += 1
        else:
            event_end = len(seq) - 1

        crossing_time: float | None = None
        event_samples = [seq[index - 1]]
        previous_sample = prev
        for sample_index in range(index, min(event_end + 1, len(seq))):
            sample = seq[sample_index]
            event_samples.append(sample)
            actual = sample["actual"]
            if normalized_direction == "up":
                if actual >= target_actual:
                    previous_actual = previous_sample["actual"]
                    previous_time = previous_sample["t_s"]
                    if actual > previous_actual:
                        alpha = (target_actual - previous_actual) / (actual - previous_actual)
                        alpha = min(1.0, max(0.0, alpha))
                        crossing_time = previous_time + alpha * (sample["t_s"] - previous_time)
                    else:
                        crossing_time = sample["t_s"]
                    break
            else:
                if actual <= target_actual:
                    previous_actual = previous_sample["actual"]
                    previous_time = previous_sample["t_s"]
                    if actual < previous_actual:
                        alpha = (target_actual - previous_actual) / (actual - previous_actual)
                        alpha = min(1.0, max(0.0, alpha))
                        crossing_time = previous_time + alpha * (sample["t_s"] - previous_time)
                    else:
                        crossing_time = sample["t_s"]
                    break
            previous_sample = sample

        if crossing_time is not None:
            tau = crossing_time - start_time
            if math.isfinite(tau) and tau > 0.0:
                taus.append(tau)
                prediction: list[float] = []
                observed: list[float] = []
                for sample in event_samples[1:]:
                    dt = sample["t_s"] - start_time
                    if dt < 0.0:
                        continue
                    predicted_actual = command_target - (command_target - start_actual) * math.exp(-dt / tau)
                    prediction.append(predicted_actual)
                    observed.append(sample["actual"])
                if prediction and observed:
                    event_rmses.append(_rmse(prediction, observed))

        index = max(index + 1, event_end + 1)

    if taus:
        tau_value = _mean(taus)
        return ScalarEstimate(
            value=tau_value,
            sample_count=len(taus),
            rmse=_mean(event_rmses) if event_rmses else 0.0,
        )

    tau_samples: list[float] = []
    for prev, curr in zip(seq[:-1], seq[1:]):
        dt = curr["t_s"] - prev["t_s"]
        if dt <= 1e-6:
            continue
        error = prev["command"] - prev["actual"]
        derivative = (curr["actual"] - prev["actual"]) / dt
        if abs(prev["actual"]) < min_actual_abs:
            continue
        if normalized_direction == "up":
            if error <= step_threshold or derivative <= (min_response_abs / dt):
                continue
        else:
            if error >= -step_threshold or derivative >= (-min_response_abs / dt):
                continue
        tau = error / derivative
        if math.isfinite(tau) and 1e-4 < tau <= max_event_window_s:
            tau_samples.append(tau)

    if not tau_samples:
        raise ValueError(f"no usable rows for {normalized_direction} time constant estimate")

    tau_samples.sort()
    lower_index = int(len(tau_samples) * 0.2)
    upper_index = max(lower_index + 1, int(math.ceil(len(tau_samples) * 0.8)))
    trimmed = tau_samples[lower_index:upper_index] or tau_samples
    tau_value = _percentile(trimmed, 0.5)
    return ScalarEstimate(
        value=tau_value,
        sample_count=len(trimmed),
        rmse=_rmse(trimmed, [tau_value] * len(trimmed)) if len(trimmed) > 1 else 0.0,
    )


def estimate_step_response_dynamics(
    rows: Iterable[dict[str, float]],
    *,
    time_column: str,
    command_column: str,
    response_column: str,
    direction: str,
    min_step_abs: float = 0.3,
    min_step_fraction_of_range: float = 0.2,
    min_response_abs: float = 0.5,
    max_event_window_s: float = 0.20,
    baseline_samples: int = 3,
    smooth_radius_samples: int = 1,
) -> StepResponseDynamicsEstimate:
    normalized_direction = str(direction).strip().lower()
    if normalized_direction not in {"up", "down"}:
        raise ValueError("direction must be 'up' or 'down'")

    seq = sorted(
        (
            {
                "t_s": float(row[time_column]),
                "command": float(row[command_column]),
                "response": float(row[response_column]),
            }
            for row in rows
            if row.get(time_column) is not None
            and row.get(command_column) is not None
            and row.get(response_column) is not None
            and math.isfinite(float(row[time_column]))
            and math.isfinite(float(row[command_column]))
            and math.isfinite(float(row[response_column]))
        ),
        key=lambda row: row["t_s"],
    )
    if len(seq) < max(6, baseline_samples + 3):
        raise ValueError("not enough rows for step-response dynamics estimate")

    if smooth_radius_samples > 0:
        smoothed: list[dict[str, float]] = []
        for index, row in enumerate(seq):
            start = max(0, index - smooth_radius_samples)
            stop = min(len(seq), index + smooth_radius_samples + 1)
            values = [seq[j]["response"] for j in range(start, stop)]
            smoothed.append(
                {
                    "t_s": row["t_s"],
                    "command": row["command"],
                    "response": sum(values) / len(values),
                }
            )
        seq = smoothed

    command_values = [row["command"] for row in seq]
    command_range = max(command_values) - min(command_values)
    step_threshold = max(float(min_step_abs), abs(command_range) * float(min_step_fraction_of_range))

    delays: list[float] = []
    taus: list[float] = []
    index = max(1, baseline_samples)
    while index < len(seq) - 2:
        prev_command = seq[index - 1]["command"]
        curr_command = seq[index]["command"]
        delta_command = curr_command - prev_command
        if normalized_direction == "up":
            if delta_command < step_threshold:
                index += 1
                continue
            expected_sign = 1.0
        else:
            if delta_command > -step_threshold:
                index += 1
                continue
            expected_sign = -1.0

        baseline = seq[index - baseline_samples:index]
        baseline_response = sum(row["response"] for row in baseline) / len(baseline)
        start_time = seq[index]["t_s"]

        event_end = len(seq) - 1
        for scan in range(index + 1, len(seq)):
            if seq[scan]["t_s"] - start_time > max_event_window_s:
                event_end = scan - 1
                break
            next_delta = seq[scan]["command"] - seq[scan - 1]["command"]
            if abs(next_delta) >= step_threshold:
                event_end = scan - 1
                break

        if event_end <= index:
            index += 1
            continue

        event = seq[index:event_end + 1]
        if normalized_direction == "up":
            peak_response = max(row["response"] for row in event)
            amplitude = peak_response - baseline_response
        else:
            peak_response = min(row["response"] for row in event)
            amplitude = baseline_response - peak_response
        if amplitude < min_response_abs:
            index = event_end + 1
            continue

        threshold_10 = baseline_response + expected_sign * (0.10 * amplitude)
        threshold_63 = baseline_response + expected_sign * (0.6321205588285577 * amplitude)
        t10: float | None = None
        t63: float | None = None
        previous = seq[index - 1]
        for sample in event:
            a = previous["response"]
            b = sample["response"]
            if (t10 is None) and (a - threshold_10) * (b - threshold_10) <= 0 and b != a:
                alpha = (threshold_10 - a) / (b - a)
                alpha = min(1.0, max(0.0, alpha))
                t10 = previous["t_s"] + alpha * (sample["t_s"] - previous["t_s"])
            if (t63 is None) and (a - threshold_63) * (b - threshold_63) <= 0 and b != a:
                alpha = (threshold_63 - a) / (b - a)
                alpha = min(1.0, max(0.0, alpha))
                t63 = previous["t_s"] + alpha * (sample["t_s"] - previous["t_s"])
            previous = sample

        if t10 is not None and t63 is not None:
            delay = max(0.0, t10 - start_time)
            tau = max(0.0, t63 - t10)
            delays.append(delay)
            taus.append(tau)

        index = max(index + 1, event_end + 1)

    if not delays or not taus:
        raise ValueError(f"no usable rows for {normalized_direction} step-response dynamics estimate")

    def _robust_scalar(values: list[float]) -> ScalarEstimate:
        values = sorted(float(value) for value in values if math.isfinite(float(value)))
        if not values:
            raise ValueError("no usable scalar values")
        lower_index = int(len(values) * 0.2)
        upper_index = max(lower_index + 1, int(math.ceil(len(values) * 0.8)))
        trimmed = values[lower_index:upper_index] or values
        median = _percentile(trimmed, 0.5)
        return ScalarEstimate(
            value=median,
            sample_count=len(trimmed),
            rmse=_rmse(trimmed, [median] * len(trimmed)) if len(trimmed) > 1 else 0.0,
        )

    return StepResponseDynamicsEstimate(
        delay_s=_robust_scalar(delays),
        time_constant_s=_robust_scalar(taus),
    )
