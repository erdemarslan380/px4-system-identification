from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

from experimental_validation.cli import main as cli_main
from experimental_validation.estimators import (
    estimate_axis_inertia,
    estimate_hover_mass,
    estimate_quadratic_drag,
    estimate_time_constant,
    estimate_thrust_scale,
)
from experimental_validation.sdf_export import build_inertial_snippet, build_parameter_summary


class ExperimentalValidationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.mass_kg = 1.82
        self.gravity = 9.80665
        self.thrust_scale = 34.5
        self.inertia = {"x": 0.028, "y": 0.031, "z": 0.049}
        self.drag = {"x": 0.19, "y": 0.22, "z": 0.31}
        self.rows = []
        for i in range(1, 41):
            thrust_cmd = 0.42 + 0.004 * i
            az_world = (self.thrust_scale * thrust_cmd / self.mass_kg) - self.gravity
            vx = 1.0 + 0.05 * i
            vy = 0.8 + 0.04 * i
            vz = 0.6 + 0.03 * i
            self.rows.append(
                {
                    "thrust_n": self.mass_kg * (self.gravity + az_world),
                    "thrust_cmd": thrust_cmd,
                    "az_world_mps2": az_world,
                    "tau_x_nm": self.inertia["x"] * (0.4 + 0.01 * i),
                    "tau_y_nm": self.inertia["y"] * (0.35 + 0.012 * i),
                    "tau_z_nm": self.inertia["z"] * (0.3 + 0.008 * i),
                    "p_dot_radps2": 0.4 + 0.01 * i,
                    "q_dot_radps2": 0.35 + 0.012 * i,
                    "r_dot_radps2": 0.3 + 0.008 * i,
                    "vx_mps": vx,
                    "vy_mps": vy,
                    "vz_mps": vz,
                    "ax_drag_mps2": -(self.drag["x"] / self.mass_kg) * abs(vx) * vx,
                    "ay_drag_mps2": -(self.drag["y"] / self.mass_kg) * abs(vy) * vy,
                    "az_drag_mps2": -(self.drag["z"] / self.mass_kg) * abs(vz) * vz,
                }
            )

    def test_estimate_hover_mass(self) -> None:
        result = estimate_hover_mass(self.rows)
        self.assertAlmostEqual(result.mass_kg, self.mass_kg, places=6)
        self.assertGreater(result.sample_count, 10)

    def test_estimate_hover_mass_rejects_high_torque_transients(self) -> None:
        rows = []
        for i in range(30):
            rows.append(
                {
                    "thrust_n": 2.0 * (self.gravity + 0.0),
                    "az_world_mps2": 0.0,
                    "total_torque_body_x_nm": 0.001 + 1e-5 * i,
                    "total_torque_body_y_nm": 0.001 + 1e-5 * i,
                    "total_torque_body_z_nm": 0.001 + 1e-5 * i,
                }
            )
        for i in range(30):
            rows.append(
                {
                    "thrust_n": 1.1 * (self.gravity + 0.0),
                    "az_world_mps2": 0.0,
                    "total_torque_body_x_nm": 0.10 + 0.001 * i,
                    "total_torque_body_y_nm": 0.10 + 0.001 * i,
                    "total_torque_body_z_nm": 0.10 + 0.001 * i,
                }
            )
        result = estimate_hover_mass(rows)
        self.assertAlmostEqual(result.mass_kg, 2.0, places=3)

    def test_estimate_thrust_scale(self) -> None:
        result = estimate_thrust_scale(self.rows, mass_kg=self.mass_kg)
        self.assertAlmostEqual(result.thrust_scale_n_per_cmd, self.thrust_scale, places=6)
        self.assertLess(result.rmse_n, 1e-9)

    def test_estimate_axis_inertia(self) -> None:
        rx = estimate_axis_inertia(self.rows, axis="x", torque_column="tau_x_nm", angular_accel_column="p_dot_radps2")
        ry = estimate_axis_inertia(self.rows, axis="y", torque_column="tau_y_nm", angular_accel_column="q_dot_radps2")
        rz = estimate_axis_inertia(self.rows, axis="z", torque_column="tau_z_nm", angular_accel_column="r_dot_radps2")
        self.assertAlmostEqual(rx.inertia_kgm2, self.inertia["x"], places=6)
        self.assertAlmostEqual(ry.inertia_kgm2, self.inertia["y"], places=6)
        self.assertAlmostEqual(rz.inertia_kgm2, self.inertia["z"], places=6)

    def test_estimate_quadratic_drag(self) -> None:
        dx = estimate_quadratic_drag(self.rows, axis="x", mass_kg=self.mass_kg, velocity_column="vx_mps", accel_column="ax_drag_mps2")
        dy = estimate_quadratic_drag(self.rows, axis="y", mass_kg=self.mass_kg, velocity_column="vy_mps", accel_column="ay_drag_mps2")
        dz = estimate_quadratic_drag(self.rows, axis="z", mass_kg=self.mass_kg, velocity_column="vz_mps", accel_column="az_drag_mps2")
        self.assertAlmostEqual(dx.coefficient, self.drag["x"], places=6)
        self.assertAlmostEqual(dy.coefficient, self.drag["y"], places=6)
        self.assertAlmostEqual(dz.coefficient, self.drag["z"], places=6)

    def test_sdf_export_helpers(self) -> None:
        inertial = build_inertial_snippet(
            mass_kg=self.mass_kg,
            ixx=self.inertia["x"],
            iyy=self.inertia["y"],
            izz=self.inertia["z"],
        )
        summary = build_parameter_summary(
            mass_kg=self.mass_kg,
            thrust_scale_n_per_cmd=self.thrust_scale,
            drag_coeff_x=self.drag["x"],
            drag_coeff_y=self.drag["y"],
            drag_coeff_z=self.drag["z"],
        )
        self.assertIn("<mass>1.820000</mass>", inertial)
        self.assertIn("thrust_scale_n_per_cmd: 34.500000", summary)

    def test_estimate_time_constant_uses_event_crossing(self) -> None:
        rows = []
        tau_up = 0.0125
        actual = 0.0
        dt = 0.005
        t_s = 0.0

        command = 0.0
        for k in range(120):
            rows.append(
                {
                    "t_s": t_s,
                    "command_radps": command,
                    "actual_radps": actual,
                }
            )
            next_command = 900.0 if k >= 5 else 0.0
            actual = next_command - (next_command - actual) * math.exp(-dt / tau_up)
            command = next_command
            t_s += dt

        estimate = estimate_time_constant(
            rows,
            time_column="t_s",
            command_column="command_radps",
            actual_column="actual_radps",
            direction="up",
        )
        self.assertAlmostEqual(estimate.value, tau_up, places=3)
        self.assertGreaterEqual(estimate.sample_count, 1)

    def test_cli_writes_outputs(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            csv_path = Path(tmp) / "experiment.csv"
            out_dir = Path(tmp) / "out"
            columns = list(self.rows[0].keys())
            csv_path.write_text(
                ",".join(columns)
                + "\n"
                + "\n".join(",".join(str(row[c]) for c in columns) for row in self.rows),
                encoding="utf-8",
            )
            exit_code = cli_main.__wrapped__() if hasattr(cli_main, "__wrapped__") else None
            self.assertIsNone(exit_code)

            # Re-run through the real CLI entry point.
            import sys

            argv_backup = sys.argv[:]
            try:
                sys.argv = [
                    "cli.py",
                    "--csv",
                    str(csv_path),
                    "--out-dir",
                    str(out_dir),
                ]
                rc = cli_main()
            finally:
                sys.argv = argv_backup
            self.assertEqual(rc, 0)
            payload = json.loads((out_dir / "identified_parameters.json").read_text(encoding="utf-8"))
            self.assertAlmostEqual(payload["mass"]["mass_kg"], self.mass_kg, places=6)
            self.assertTrue((out_dir / "candidate_inertial.sdf.xml").exists())
            self.assertTrue((out_dir / "candidate_vehicle_params.yaml").exists())


if __name__ == "__main__":
    unittest.main()
