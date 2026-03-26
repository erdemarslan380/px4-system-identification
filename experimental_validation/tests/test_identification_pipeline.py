from __future__ import annotations

import csv
import math
import tempfile
import unittest
from pathlib import Path

from experimental_validation.identification import (
    estimate_parameters_from_identification_log,
    load_identification_csv,
    split_rows_by_profile,
)
from experimental_validation.identification_suite import build_identification_plan
from experimental_validation.qgc_params import parse_qgc_parameter_dump
from experimental_validation.estimators import estimate_diagonal_inertia_tensor


class IdentificationPipelineTests(unittest.TestCase):
    def test_parse_qgc_parameter_dump_supports_qgc_and_simple_formats(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            dump_path = Path(tmp) / "vehicle.params"
            dump_path.write_text(
                "# QGC parameter dump\n"
                "1\t1\tMPC_XY_P\t0.95\t6\n"
                "1\t1\tMPC_Z_P\t1.00\t6\n"
                "SYS_AUTOSTART,4001\n",
                encoding="utf-8",
            )
            params = parse_qgc_parameter_dump(dump_path)
            self.assertAlmostEqual(params["MPC_XY_P"], 0.95)
            self.assertAlmostEqual(params["MPC_Z_P"], 1.0)
            self.assertAlmostEqual(params["SYS_AUTOSTART"], 4001.0)

    def test_identification_log_can_be_split_by_profile(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            csv_path = Path(tmp) / "ident.csv"
            csv_path.write_text(
                "timestamp_us,profile,az_world_mps2,thrust_cmd,p_dot_radps2,roll_torque_proxy,vx_mps,ax_drag_mps2\n"
                "0,hover_thrust,0.1,0.52,0.0,0.0,0.0,0.0\n"
                "1,hover_thrust,0.2,0.55,0.0,0.0,0.0,0.0\n"
                "2,roll_sweep,0.0,0.50,4.0,0.08,0.0,0.0\n",
                encoding="utf-8",
            )
            rows = load_identification_csv(csv_path)
            grouped = split_rows_by_profile(rows)
            self.assertEqual(sorted(grouped.keys()), ["hover_thrust", "roll_sweep"])
            self.assertEqual(len(grouped["hover_thrust"]), 2)
            self.assertEqual(len(grouped["roll_sweep"]), 1)

    def test_estimate_parameters_from_identification_log(self) -> None:
        rows = []
        thrust_scale = 33.0
        mass_kg = 1.65
        ixx = 0.028
        iyy = 0.031
        izz = 0.045
        drag_x = 0.18
        drag_y = 0.21
        drag_z = 0.27
        gravity = 9.80665

        for i in range(1, 41):
            thrust_cmd = 0.42 + 0.003 * i
            thrust_n = thrust_scale * thrust_cmd
            az = (thrust_scale * thrust_cmd / mass_kg) - gravity
            rows.append({
                "timestamp_us": float(i),
                "profile": "hover_thrust",
                "thrust_cmd": thrust_cmd,
                "thrust_n": thrust_n,
                "az_world_mps2": az,
            })

        for i in range(1, 31):
            alpha = 0.4 + 0.02 * i
            rows.append({
                "timestamp_us": float(100 + i),
                "profile": "roll_sweep",
                "roll_torque_proxy": ixx * alpha,
                "p_dot_radps2": alpha,
            })
            rows.append({
                "timestamp_us": float(200 + i),
                "profile": "pitch_sweep",
                "pitch_torque_proxy": iyy * alpha,
                "q_dot_radps2": alpha,
            })
            rows.append({
                "timestamp_us": float(300 + i),
                "profile": "yaw_sweep",
                "yaw_torque_proxy": izz * alpha,
                "r_dot_radps2": alpha,
            })

        for i in range(1, 31):
            vx = 0.8 + 0.04 * i
            vy = 0.7 + 0.03 * i
            vz = 0.6 + 0.02 * i
            rows.append({
                "timestamp_us": float(400 + i),
                "profile": "drag_x",
                "vx_mps": vx,
                "ax_drag_mps2": -(drag_x / mass_kg) * abs(vx) * vx,
            })
            rows.append({
                "timestamp_us": float(500 + i),
                "profile": "drag_y",
                "vy_mps": vy,
                "ay_drag_mps2": -(drag_y / mass_kg) * abs(vy) * vy,
            })
            rows.append({
                "timestamp_us": float(600 + i),
                "profile": "drag_z",
                "vz_mps": vz,
                "az_drag_mps2": -(drag_z / mass_kg) * abs(vz) * vz,
            })

        report = estimate_parameters_from_identification_log(rows)
        self.assertAlmostEqual(report["thrust_scale"]["thrust_scale_n_per_cmd"], thrust_scale, places=5)
        self.assertAlmostEqual(report["mass"]["mass_kg"], mass_kg, places=5)
        self.assertAlmostEqual(report["inertia"]["x"]["inertia_kgm2"], ixx, places=5)
        self.assertAlmostEqual(report["inertia"]["y"]["inertia_kgm2"], iyy, places=5)
        self.assertAlmostEqual(report["inertia"]["z"]["inertia_kgm2"], izz, places=5)
        self.assertAlmostEqual(report["drag"]["x"]["coefficient"], drag_x, places=5)
        self.assertAlmostEqual(report["drag"]["y"]["coefficient"], drag_y, places=5)
        self.assertAlmostEqual(report["drag"]["z"]["coefficient"], drag_z, places=5)

    def test_joint_inertia_solver_recovers_coupled_diagonal_inertia(self) -> None:
        rows = []
        ixx = 0.02166666666666667
        iyy = 0.02166666666666667
        izz = 0.04000000000000001

        for i in range(1, 61):
            p_dot = 0.5 + 0.02 * i
            q = 0.8
            r = 1.2
            tau_x = ixx * p_dot + (izz - iyy) * q * r
            rows.append({
                "timestamp_us": float(100 + i),
                "profile": "roll_sweep",
                "p": 0.6,
                "q": q,
                "r": r,
                "p_dot_radps2": p_dot,
                "q_dot_radps2": 0.0,
                "r_dot_radps2": 0.0,
                "tau_x_nm": tau_x,
            })

        for i in range(1, 61):
            q_dot = 0.45 + 0.015 * i
            p = 0.9
            r = 0.7
            tau_y = iyy * q_dot + (ixx - izz) * p * r
            rows.append({
                "timestamp_us": float(200 + i),
                "profile": "pitch_sweep",
                "p": p,
                "q": 0.4,
                "r": r,
                "p_dot_radps2": 0.0,
                "q_dot_radps2": q_dot,
                "r_dot_radps2": 0.0,
                "tau_y_nm": tau_y,
            })

        for i in range(1, 61):
            r_dot = 0.35 + 0.02 * i
            p = 0.75
            q = 0.65
            tau_z = izz * r_dot + (iyy - ixx) * p * q
            rows.append({
                "timestamp_us": float(300 + i),
                "profile": "yaw_sweep",
                "p": p,
                "q": q,
                "r": 0.5,
                "p_dot_radps2": 0.0,
                "q_dot_radps2": 0.0,
                "r_dot_radps2": r_dot,
                "tau_z_nm": tau_z,
            })

        estimate = estimate_diagonal_inertia_tensor(rows)
        self.assertAlmostEqual(estimate.x.inertia_kgm2, ixx, places=6)
        self.assertAlmostEqual(estimate.y.inertia_kgm2, iyy, places=6)
        self.assertAlmostEqual(estimate.z.inertia_kgm2, izz, places=6)

    def test_unusable_inertia_profile_falls_back_to_zero_with_warning(self) -> None:
        rows = [
            {"timestamp_us": 1.0, "profile": "hover_thrust", "thrust_cmd": 0.5, "thrust_n": 16.0, "az_world_mps2": 0.0},
            {"timestamp_us": 2.0, "profile": "hover_thrust", "thrust_cmd": 0.52, "thrust_n": 16.5, "az_world_mps2": 0.2},
            {"timestamp_us": 3.0, "profile": "roll_sweep", "roll_torque_proxy": 0.0, "p_dot_radps2": 0.0},
            {"timestamp_us": 4.0, "profile": "roll_sweep", "roll_torque_proxy": 0.0, "p_dot_radps2": 0.0},
        ]
        report = estimate_parameters_from_identification_log(rows)
        self.assertEqual(report["inertia"]["x"]["inertia_kgm2"], 0.0)
        self.assertTrue(any("inertia estimate unusable" in warning for warning in report["warnings"]))

    def test_identification_log_can_merge_gazebo_truth_and_recover_motor_terms(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            ident_path = Path(tmp) / "ident.csv"
            truth_path = Path(tmp) / "truth.csv"

            ident_path.write_text(
                "timestamp_us,profile,thrust_cmd,az_world_mps2\n"
                "1000000,hover_thrust,0.50,0.0\n"
                "1020000,hover_thrust,0.60,0.0\n"
                "1040000,hover_thrust,0.30,0.0\n"
                "1060000,hover_thrust,0.55,0.0\n"
                "2000000,roll_sweep,0.55,0.0\n"
                "2020000,roll_sweep,0.55,0.0\n"
                "3000000,pitch_sweep,0.55,0.0\n"
                "3020000,pitch_sweep,0.55,0.0\n"
                "4000000,yaw_sweep,0.55,0.0\n"
                "4020000,yaw_sweep,0.55,0.0\n",
                encoding="utf-8",
            )

            headers = [
                "sim_time_us", "az_world_mps2", "total_prop_thrust_n",
                "total_torque_body_x_nm", "total_torque_body_y_nm", "total_torque_body_z_nm",
                "p_dot_body", "q_dot_body", "r_dot_body",
                "drag_force_body_x_n", "drag_force_body_y_n", "drag_force_body_z_n",
                "rolling_moment_body_x_nm", "rolling_moment_body_y_nm", "rolling_moment_body_z_nm",
                "drag_basis_body_x", "drag_basis_body_y", "drag_basis_body_z",
                "rolling_basis_body_x", "rolling_basis_body_y", "rolling_basis_body_z",
                "yaw_moment_basis_n", "observed_max_rot_velocity_radps", "rotor_velocity_slowdown_sim",
                "rotor_0_cmd_radps", "rotor_1_cmd_radps", "rotor_2_cmd_radps", "rotor_3_cmd_radps",
                "rotor_0_joint_vel_radps", "rotor_1_joint_vel_radps", "rotor_2_joint_vel_radps", "rotor_3_joint_vel_radps",
                "rotor_0_actual_radps", "rotor_1_actual_radps", "rotor_2_actual_radps", "rotor_3_actual_radps",
                "rotor_0_thrust_n", "rotor_1_thrust_n", "rotor_2_thrust_n", "rotor_3_thrust_n",
            ]
            truth_rows = [
                {"sim_time_us": 0, "az_world_mps2": -1.80665, "total_prop_thrust_n": 16.0, "drag_force_body_x_n": 0.008,
                 "rolling_moment_body_x_nm": 0.0001, "drag_basis_body_x": 100.0, "rolling_basis_body_x": 100.0,
                 "yaw_moment_basis_n": 1.0, "observed_max_rot_velocity_radps": 500.0, "rotor_velocity_slowdown_sim": 10.0,
                 "rotor_0_cmd_radps": 500.0, "rotor_1_cmd_radps": 500.0, "rotor_2_cmd_radps": 500.0, "rotor_3_cmd_radps": 500.0,
                 "rotor_0_joint_vel_radps": 50.0, "rotor_1_joint_vel_radps": 50.0, "rotor_2_joint_vel_radps": 50.0, "rotor_3_joint_vel_radps": 50.0,
                 "rotor_0_actual_radps": 500.0, "rotor_1_actual_radps": 500.0, "rotor_2_actual_radps": 500.0, "rotor_3_actual_radps": 500.0,
                 "rotor_0_thrust_n": 4.0, "rotor_1_thrust_n": 4.0, "rotor_2_thrust_n": 4.0, "rotor_3_thrust_n": 4.0},
                {"sim_time_us": 20000, "az_world_mps2": 2.39335, "total_prop_thrust_n": 24.4, "drag_force_body_x_n": 0.009,
                 "rolling_moment_body_x_nm": 0.00012, "drag_basis_body_x": 112.5, "rolling_basis_body_x": 120.0,
                 "yaw_moment_basis_n": 1.5, "observed_max_rot_velocity_radps": 600.0, "rotor_velocity_slowdown_sim": 10.0,
                 "rotor_0_cmd_radps": 600.0, "rotor_1_cmd_radps": 600.0, "rotor_2_cmd_radps": 600.0, "rotor_3_cmd_radps": 600.0,
                 "rotor_0_joint_vel_radps": 60.0, "rotor_1_joint_vel_radps": 60.0, "rotor_2_joint_vel_radps": 60.0, "rotor_3_joint_vel_radps": 60.0,
                 "rotor_0_actual_radps": 600.0, "rotor_1_actual_radps": 600.0, "rotor_2_actual_radps": 600.0, "rotor_3_actual_radps": 600.0,
                 "rotor_0_thrust_n": 6.1, "rotor_1_thrust_n": 6.1, "rotor_2_thrust_n": 6.1, "rotor_3_thrust_n": 6.1},
                {"sim_time_us": 40000, "az_world_mps2": -5.30665, "total_prop_thrust_n": 9.0, "drag_force_body_x_n": 0.004,
                 "rolling_moment_body_x_nm": 0.00004, "drag_basis_body_x": 50.0, "rolling_basis_body_x": 40.0,
                 "yaw_moment_basis_n": 0.75, "observed_max_rot_velocity_radps": 300.0, "rotor_velocity_slowdown_sim": 10.0,
                 "rotor_0_cmd_radps": 300.0, "rotor_1_cmd_radps": 300.0, "rotor_2_cmd_radps": 300.0, "rotor_3_cmd_radps": 300.0,
                 "rotor_0_joint_vel_radps": 30.0, "rotor_1_joint_vel_radps": 30.0, "rotor_2_joint_vel_radps": 30.0, "rotor_3_joint_vel_radps": 30.0,
                 "rotor_0_actual_radps": 300.0, "rotor_1_actual_radps": 300.0, "rotor_2_actual_radps": 300.0, "rotor_3_actual_radps": 300.0,
                 "rotor_0_thrust_n": 2.25, "rotor_1_thrust_n": 2.25, "rotor_2_thrust_n": 2.25, "rotor_3_thrust_n": 2.25},
                {"sim_time_us": 60000, "az_world_mps2": 0.60335, "total_prop_thrust_n": 20.82, "drag_force_body_x_n": 0.0088,
                 "rolling_moment_body_x_nm": 0.00011, "drag_basis_body_x": 110.0, "rolling_basis_body_x": 110.0,
                 "yaw_moment_basis_n": 1.375, "observed_max_rot_velocity_radps": 550.0, "rotor_velocity_slowdown_sim": 10.0,
                 "rotor_0_cmd_radps": 550.0, "rotor_1_cmd_radps": 550.0, "rotor_2_cmd_radps": 550.0, "rotor_3_cmd_radps": 550.0,
                 "rotor_0_joint_vel_radps": 55.0, "rotor_1_joint_vel_radps": 55.0, "rotor_2_joint_vel_radps": 55.0, "rotor_3_joint_vel_radps": 55.0,
                 "rotor_0_actual_radps": 550.0, "rotor_1_actual_radps": 550.0, "rotor_2_actual_radps": 550.0, "rotor_3_actual_radps": 550.0,
                 "rotor_0_thrust_n": 5.205, "rotor_1_thrust_n": 5.205, "rotor_2_thrust_n": 5.205, "rotor_3_thrust_n": 5.205},
                {"sim_time_us": 1000000, "total_torque_body_x_nm": 0.05, "p_dot_body": 2.0},
                {"sim_time_us": 1020000, "total_torque_body_x_nm": 0.075, "p_dot_body": 3.0},
                {"sim_time_us": 2000000, "total_torque_body_y_nm": 0.0625, "q_dot_body": 2.5},
                {"sim_time_us": 2020000, "total_torque_body_y_nm": 0.1, "q_dot_body": 4.0},
                {"sim_time_us": 3000000, "total_torque_body_z_nm": 0.054, "r_dot_body": 1.8, "yaw_moment_basis_n": 3.375},
                {"sim_time_us": 3020000, "total_torque_body_z_nm": 0.09, "r_dot_body": 3.0, "yaw_moment_basis_n": 5.625},
            ]
            with truth_path.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=headers)
                writer.writeheader()
                for row in truth_rows:
                    writer.writerow(row)

            rows = load_identification_csv(ident_path, truth_csv=truth_path)
            report = estimate_parameters_from_identification_log(rows)
            self.assertAlmostEqual(report["mass"]["mass_kg"], 2.0, places=5)
            self.assertAlmostEqual(report["inertia"]["x"]["inertia_kgm2"], 0.025, places=5)
            self.assertAlmostEqual(report["inertia"]["y"]["inertia_kgm2"], 0.025, places=5)
            self.assertAlmostEqual(report["inertia"]["z"]["inertia_kgm2"], 0.03, places=5)
            self.assertAlmostEqual(report["motor_model"]["motor_constant"]["value"], 1.7048014912261906e-5, places=9)
            self.assertAlmostEqual(report["motor_model"]["moment_constant"]["value"], 0.016, places=6)
            self.assertAlmostEqual(report["motor_model"]["rotor_drag_coefficient"]["value"], 8.0e-5, places=9)
            self.assertAlmostEqual(report["motor_model"]["rolling_moment_coefficient"]["value"], 1.0e-6, places=10)
            self.assertAlmostEqual(report["motor_model"]["rotor_velocity_slowdown_sim"]["value"], 10.0, places=6)
            self.assertAlmostEqual(report["motor_model"]["max_rot_velocity_radps"]["value"], 600.0, places=6)
            self.assertIn("motor_model", report)

    def test_load_identification_csv_derives_rotor_command_from_normalized_motor_output(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            ident_path = Path(tmp) / "ident.csv"
            truth_path = Path(tmp) / "truth.csv"
            ident_path.write_text(
                "timestamp_us,profile,motor_0,motor_1,motor_2,motor_3\n"
                "1000000,motor_step,0.25,0.25,0.25,0.25\n",
                encoding="utf-8",
            )
            truth_path.write_text(
                "sim_time_us,observed_max_rot_velocity_radps,rotor_0_actual_radps,rotor_1_actual_radps,rotor_2_actual_radps,rotor_3_actual_radps\n"
                "1000000,1000,500,500,500,500\n",
                encoding="utf-8",
            )
            rows = load_identification_csv(ident_path, truth_csv=truth_path)
            self.assertEqual(len(rows), 1)
            self.assertAlmostEqual(float(rows[0]["rotor_0_cmd_radps"]), 250.0, places=6)

    def test_load_identification_csv_backfills_rotor_command_when_truth_column_is_nan(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            ident_path = Path(tmp) / "ident.csv"
            truth_path = Path(tmp) / "truth.csv"
            ident_path.write_text(
                "timestamp_us,profile,motor_0,motor_1,motor_2,motor_3\n"
                "1000000,motor_step,0.25,0.25,0.25,0.25\n",
                encoding="utf-8",
            )
            truth_path.write_text(
                "sim_time_us,observed_max_rot_velocity_radps,rotor_0_cmd_radps,rotor_0_actual_radps,rotor_1_actual_radps,rotor_2_actual_radps,rotor_3_actual_radps\n"
                "1000000,1000,nan,500,500,500,500\n",
                encoding="utf-8",
            )
            rows = load_identification_csv(ident_path, truth_csv=truth_path)
            self.assertEqual(len(rows), 1)
            self.assertAlmostEqual(float(rows[0]["rotor_0_cmd_radps"]), 250.0, places=6)

    def test_load_identification_csv_truth_allowlist_limits_merged_columns(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            ident_path = Path(tmp) / "ident.csv"
            truth_path = Path(tmp) / "truth.csv"
            ident_path.write_text(
                "timestamp_us,profile,motor_0,motor_1,motor_2,motor_3\n"
                "1000000,motor_step,0.25,0.25,0.25,0.25\n",
                encoding="utf-8",
            )
            truth_path.write_text(
                "sim_time_us,thrust_n,rotor_0_actual_radps,rotor_1_actual_radps,rotor_2_actual_radps,rotor_3_actual_radps\n"
                "0,19.6133,500,500,500,500\n",
                encoding="utf-8",
            )
            rows = load_identification_csv(
                ident_path,
                truth_csv=truth_path,
                truth_field_allowlist={"rotor_0_actual_radps", "rotor_1_actual_radps", "rotor_2_actual_radps", "rotor_3_actual_radps"},
            )
            self.assertEqual(len(rows), 1)
            self.assertIn("rotor_0_actual_radps", rows[0])
            self.assertNotIn("thrust_n", rows[0])

    def test_load_identification_csv_overrides_zero_truth_rate_derivative_with_measured_rate_change(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            ident_path = Path(tmp) / "ident.csv"
            truth_path = Path(tmp) / "truth.csv"
            ident_path.write_text(
                "timestamp_us,profile,p,motor_0,motor_1,motor_2,motor_3\n"
                "1000000,roll_sweep,0.0,0.5,0.5,0.5,0.5\n"
                "1020000,roll_sweep,0.1,0.5,0.5,0.5,0.5\n",
                encoding="utf-8",
            )
            truth_path.write_text(
                "sim_time_us,p_dot_body,total_torque_body_x_nm\n"
                "0,0.0,0.01\n"
                "20000,0.0,0.02\n",
                encoding="utf-8",
            )
            rows = load_identification_csv(ident_path, truth_csv=truth_path)
            self.assertEqual(len(rows), 2)
            self.assertAlmostEqual(float(rows[1]["p_dot_radps2"]), 5.0, places=6)

    def test_load_identification_csv_prefers_absolute_truth_alignment_when_logs_share_clock(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            ident_path = Path(tmp) / "ident.csv"
            truth_path = Path(tmp) / "truth.csv"
            ident_path.write_text(
                "timestamp_us,profile,motor_0,motor_1,motor_2,motor_3\n"
                "1000000,motor_step,0.50,0.50,0.50,0.50\n"
                "1020000,motor_step,0.60,0.60,0.60,0.60\n",
                encoding="utf-8",
            )
            truth_path.write_text(
                "sim_time_us,rotor_0_actual_radps,rotor_1_actual_radps,rotor_2_actual_radps,rotor_3_actual_radps\n"
                "900000,50,50,50,50\n"
                "1000000,500,500,500,500\n"
                "1020000,600,600,600,600\n",
                encoding="utf-8",
            )
            rows = load_identification_csv(ident_path, truth_csv=truth_path)
            self.assertEqual(len(rows), 2)
            self.assertAlmostEqual(float(rows[0]["rotor_0_actual_radps"]), 500.0, places=6)
            self.assertAlmostEqual(float(rows[1]["rotor_0_actual_radps"]), 600.0, places=6)

    def test_load_identification_csv_assigns_truth_time_to_expanded_truth_rows(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            ident_path = Path(tmp) / "ident.csv"
            truth_path = Path(tmp) / "truth.csv"
            ident_path.write_text(
                "timestamp_us,profile,motor_0,motor_1,motor_2,motor_3\n"
                "1000000,motor_step,0.50,0.50,0.50,0.50\n",
                encoding="utf-8",
            )
            truth_path.write_text(
                "sim_time_us,rotor_0_actual_radps,rotor_1_actual_radps,rotor_2_actual_radps,rotor_3_actual_radps\n"
                "1000000,500,500,500,500\n"
                "1005000,520,520,520,520\n"
                "1010000,540,540,540,540\n",
                encoding="utf-8",
            )
            rows = load_identification_csv(ident_path, truth_csv=truth_path)
            expanded = [row for row in rows if "sim_time_us" in row]
            self.assertGreaterEqual(len(expanded), 2)
            self.assertTrue(all("t_s" in row for row in expanded))

    def test_load_identification_csv_derives_truth_body_rate_acceleration(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            ident_path = Path(tmp) / "ident.csv"
            truth_path = Path(tmp) / "truth.csv"
            ident_path.write_text(
                "timestamp_us,profile,motor_0,motor_1,motor_2,motor_3\n"
                "1000000,pitch_sweep,0.50,0.50,0.50,0.50\n"
                "1020000,pitch_sweep,0.50,0.50,0.50,0.50\n"
                "1040000,pitch_sweep,0.50,0.50,0.50,0.50\n",
                encoding="utf-8",
            )
            truth_path.write_text(
                "sim_time_us,q_body,total_torque_body_y_nm\n"
                "1000000,0.00,0.010\n"
                "1020000,0.10,0.012\n"
                "1040000,0.20,0.014\n",
                encoding="utf-8",
            )
            rows = load_identification_csv(ident_path, truth_csv=truth_path)
            self.assertEqual(len(rows), 3)
            self.assertAlmostEqual(float(rows[1]["q_dot_radps2"]), 5.0, places=6)

    def test_load_identification_csv_aliases_velocity_columns_without_vel_prefix(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            ident_path = Path(tmp) / "ident.csv"
            ident_path.write_text(
                "timestamp_us,profile,vx,vy,vz,ax,ay,az\n"
                "1000000,drag_x,1.5,-0.2,0.7,0.1,-0.3,0.05\n",
                encoding="utf-8",
            )
            rows = load_identification_csv(ident_path)
            self.assertEqual(len(rows), 1)
            self.assertAlmostEqual(float(rows[0]["vx_mps"]), 1.5, places=6)
            self.assertAlmostEqual(float(rows[0]["vy_mps"]), -0.2, places=6)
            self.assertAlmostEqual(float(rows[0]["vz_mps"]), 0.7, places=6)
            self.assertAlmostEqual(float(rows[0]["ax_drag_mps2"]), 0.1, places=6)

    def test_load_identification_csv_aliases_esc_rpm_to_rotor_speed(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            ident_path = Path(tmp) / "ident.csv"
            ident_path.write_text(
                "timestamp_us,profile,esc_0_rpm,esc_1_rpm,esc_2_rpm,esc_3_rpm,motor_0,motor_1,motor_2,motor_3\n"
                "1000000,motor_step,6000,6000,6000,6000,0.3,0.3,0.3,0.3\n",
                encoding="utf-8",
            )
            rows = load_identification_csv(ident_path)
            self.assertEqual(len(rows), 1)
            expected = 6000.0 * (2.0 * math.pi / 60.0)
            self.assertAlmostEqual(float(rows[0]["rotor_0_actual_radps"]), expected, places=6)

    def test_estimate_parameters_recovers_time_constants_from_motor_step_rows(self) -> None:
        rows = []
        tau_up = 0.0125
        tau_down = 0.025
        max_rot = 1000.0

        actual = 0.0
        t_s = 0.0
        dt = 0.005
        command = 0.0
        for k in range(240):
            rows.append({
                "timestamp_us": t_s * 1e6,
                "profile": "motor_step",
                "thrust_cmd": 0.55,
                "thrust_n": 2.0 * 9.80665,
                "az_world_mps2": 0.0,
                "rotor_0_cmd_radps": command,
                "rotor_1_cmd_radps": command,
                "rotor_2_cmd_radps": command,
                "rotor_3_cmd_radps": command,
                "rotor_0_actual_radps": actual,
                "rotor_1_actual_radps": actual,
                "rotor_2_actual_radps": actual,
                "rotor_3_actual_radps": actual,
                "rotor_0_joint_vel_radps": actual / 10.0,
                "rotor_1_joint_vel_radps": actual / 10.0,
                "rotor_2_joint_vel_radps": actual / 10.0,
                "rotor_3_joint_vel_radps": actual / 10.0,
                "observed_max_rot_velocity_radps": max_rot,
            })
            next_command = max_rot if k >= 5 else 0.0
            actual = next_command - (next_command - actual) * math.exp(-dt / tau_up)
            command = next_command
            t_s += dt

        for k in range(260):
            rows.append({
                "timestamp_us": t_s * 1e6,
                "profile": "motor_step",
                "thrust_cmd": 0.55,
                "thrust_n": 2.0 * 9.80665,
                "az_world_mps2": 0.0,
                "rotor_0_cmd_radps": command,
                "rotor_1_cmd_radps": command,
                "rotor_2_cmd_radps": command,
                "rotor_3_cmd_radps": command,
                "rotor_0_actual_radps": actual,
                "rotor_1_actual_radps": actual,
                "rotor_2_actual_radps": actual,
                "rotor_3_actual_radps": actual,
                "rotor_0_joint_vel_radps": actual / 10.0,
                "rotor_1_joint_vel_radps": actual / 10.0,
                "rotor_2_joint_vel_radps": actual / 10.0,
                "rotor_3_joint_vel_radps": actual / 10.0,
                "observed_max_rot_velocity_radps": max_rot,
            })
            next_command = 0.0
            actual = next_command - (next_command - actual) * math.exp(-dt / tau_down)
            command = next_command
            t_s += dt

        report = estimate_parameters_from_identification_log(rows)
        self.assertAlmostEqual(report["motor_model"]["time_constant_up_s"]["value"], tau_up, places=3)
        self.assertAlmostEqual(report["motor_model"]["time_constant_down_s"]["value"], tau_down, places=3)

    def test_estimate_parameters_recovers_time_constants_across_multiple_runs(self) -> None:
        tau_up = 0.0125
        tau_down = 0.025
        max_rot = 1000.0
        dt = 0.005
        rows: list[dict[str, float | str]] = []

        for run_index in (0.0, 1.0):
            actual = 0.0
            t_s = 0.0
            command = 0.0
            for k in range(140):
                rows.append({
                    "timestamp_us": t_s * 1e6,
                    "run_index": run_index,
                    "profile": "motor_step",
                    "thrust_cmd": 0.55,
                    "thrust_n": 2.0 * 9.80665,
                    "az_world_mps2": 0.0,
                    "rotor_0_cmd_radps": command,
                    "rotor_1_cmd_radps": command,
                    "rotor_2_cmd_radps": command,
                    "rotor_3_cmd_radps": command,
                    "rotor_0_actual_radps": actual,
                    "rotor_1_actual_radps": actual,
                    "rotor_2_actual_radps": actual,
                    "rotor_3_actual_radps": actual,
                    "rotor_0_joint_vel_radps": actual / 10.0,
                    "rotor_1_joint_vel_radps": actual / 10.0,
                    "rotor_2_joint_vel_radps": actual / 10.0,
                    "rotor_3_joint_vel_radps": actual / 10.0,
                    "observed_max_rot_velocity_radps": max_rot,
                })
                next_command = max_rot if k >= 5 else 0.0
                actual = next_command - (next_command - actual) * math.exp(-dt / tau_up)
                command = next_command
                t_s += dt

            for _ in range(170):
                rows.append({
                    "timestamp_us": t_s * 1e6,
                    "run_index": run_index,
                    "profile": "motor_step",
                    "thrust_cmd": 0.55,
                    "thrust_n": 2.0 * 9.80665,
                    "az_world_mps2": 0.0,
                    "rotor_0_cmd_radps": command,
                    "rotor_1_cmd_radps": command,
                    "rotor_2_cmd_radps": command,
                    "rotor_3_cmd_radps": command,
                    "rotor_0_actual_radps": actual,
                    "rotor_1_actual_radps": actual,
                    "rotor_2_actual_radps": actual,
                    "rotor_3_actual_radps": actual,
                    "rotor_0_joint_vel_radps": actual / 10.0,
                    "rotor_1_joint_vel_radps": actual / 10.0,
                    "rotor_2_joint_vel_radps": actual / 10.0,
                    "rotor_3_joint_vel_radps": actual / 10.0,
                    "observed_max_rot_velocity_radps": max_rot,
                })
                next_command = 0.0
                actual = next_command - (next_command - actual) * math.exp(-dt / tau_down)
                command = next_command
                t_s += dt

        report = estimate_parameters_from_identification_log(rows)
        self.assertAlmostEqual(report["motor_model"]["time_constant_up_s"]["value"], tau_up, places=3)
        self.assertAlmostEqual(report["motor_model"]["time_constant_down_s"]["value"], tau_down, places=3)

    def test_estimate_parameters_ignores_inferred_rotor_commands_for_time_constant_fit(self) -> None:
        tau_up = 0.0125
        dt = 0.005
        t_s = 0.0
        actual = 0.0
        rows: list[dict[str, float | str]] = []

        command = 0.0
        for _ in range(20):
            rows.append({
                "timestamp_us": t_s * 1e6,
                "profile": "motor_step",
                "thrust_cmd": 0.55,
                "thrust_n": 2.0 * 9.80665,
                "az_world_mps2": 0.0,
                "rotor_0_cmd_radps": command,
                "rotor_0_cmd_inferred": 1.0,
                "rotor_0_actual_radps": actual,
                "rotor_0_joint_vel_radps": actual / 10.0,
                "observed_max_rot_velocity_radps": 900.0,
            })
            next_command = 600.0
            actual = next_command - (next_command - actual) * math.exp(-dt / tau_up)
            command = next_command
            t_s += dt

        command = 0.0
        actual = 0.0
        for _ in range(5):
            rows.append({
                "timestamp_us": t_s * 1e6,
                "profile": "motor_step",
                "thrust_cmd": 0.55,
                "thrust_n": 2.0 * 9.80665,
                "az_world_mps2": 0.0,
                "rotor_0_cmd_radps": command,
                "rotor_0_cmd_inferred": 0.0,
                "rotor_0_actual_radps": actual,
                "rotor_0_joint_vel_radps": actual / 10.0,
                "observed_max_rot_velocity_radps": 900.0,
            })
            t_s += dt

        for _ in range(120):
            rows.append({
                "timestamp_us": t_s * 1e6,
                "profile": "motor_step",
                "thrust_cmd": 0.55,
                "thrust_n": 2.0 * 9.80665,
                "az_world_mps2": 0.0,
                "rotor_0_cmd_radps": command,
                "rotor_0_cmd_inferred": 0.0,
                "rotor_0_actual_radps": actual,
                "rotor_0_joint_vel_radps": actual / 10.0,
                "observed_max_rot_velocity_radps": 900.0,
            })
            next_command = 150.0
            actual = next_command - (next_command - actual) * math.exp(-dt / tau_up)
            command = next_command
            t_s += dt

        report = estimate_parameters_from_identification_log(rows)
        self.assertAlmostEqual(report["motor_model"]["time_constant_up_s"]["value"], tau_up, places=3)

    def test_estimate_parameters_falls_back_to_inferred_rows_when_explicit_rows_lack_down_event(self) -> None:
        tau_up = 0.0125
        tau_down = 0.025
        dt = 0.005
        t_s = 0.0
        actual = 0.0
        rows: list[dict[str, float | str]] = []

        command = 0.0
        for k in range(120):
            rows.append({
                "timestamp_us": t_s * 1e6,
                "profile": "motor_step",
                "thrust_cmd": 0.55,
                "thrust_n": 2.0 * 9.80665,
                "az_world_mps2": 0.0,
                "rotor_0_cmd_radps": command,
                "rotor_0_cmd_inferred": 0.0,
                "rotor_0_actual_radps": actual,
                "rotor_0_joint_vel_radps": actual / 10.0,
                "observed_max_rot_velocity_radps": 1000.0,
            })
            next_command = 700.0 if k >= 5 else 0.0
            actual = next_command - (next_command - actual) * math.exp(-dt / tau_up)
            command = next_command
            t_s += dt

        # The descent segment is only available through inferred commands. The
        # estimator should still recover tau_down by retrying with the full set.
        for _ in range(160):
            rows.append({
                "timestamp_us": t_s * 1e6,
                "profile": "motor_step",
                "thrust_cmd": 0.55,
                "thrust_n": 2.0 * 9.80665,
                "az_world_mps2": 0.0,
                "rotor_0_cmd_radps": command,
                "rotor_0_cmd_inferred": 1.0,
                "rotor_0_actual_radps": actual,
                "rotor_0_joint_vel_radps": actual / 10.0,
                "observed_max_rot_velocity_radps": 1000.0,
            })
            next_command = 0.0
            actual = next_command - (next_command - actual) * math.exp(-dt / tau_down)
            command = next_command
            t_s += dt

        report = estimate_parameters_from_identification_log(rows)
        self.assertAlmostEqual(report["motor_model"]["time_constant_up_s"]["value"], tau_up, places=3)
        self.assertAlmostEqual(report["motor_model"]["time_constant_down_s"]["value"], tau_down, places=3)

    def test_build_identification_plan_repeats_profiles(self) -> None:
        plan = build_identification_plan(
            plan_name="x500_identification_comprehensive",
            results_root="Tools/optimization/plan_runs/x500_identification_comprehensive",
            repeats={
                "mass_vertical": 2,
                "roll_sweep": 1,
                "motor_step": 3,
            },
        )
        task_ids = [task["id"] for task in plan["tasks"]]
        self.assertEqual(task_ids, [
            "x500_mass_vertical_r01",
            "x500_mass_vertical_r02",
            "x500_roll_sweep_r01",
            "x500_motor_step_r01",
            "x500_motor_step_r02",
            "x500_motor_step_r03",
        ])
        self.assertTrue(all(task["mission_mode"] == "identification" for task in plan["tasks"]))


if __name__ == "__main__":
    unittest.main()
