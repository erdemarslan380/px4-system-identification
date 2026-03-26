from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from experimental_validation.compare_with_sdf import (
    aggregate_identification_rows,
    build_identification_mode_reports,
    collect_identification_logs,
    compare_identified_to_sdf,
    parse_x500_sdf_reference,
)


class SdfComparisonTests(unittest.TestCase):
    def test_parse_x500_sdf_reference_reads_mass_inertia_and_motor_terms(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            model_path = root / "model.sdf"
            base_path = root / "base.sdf"
            model_path.write_text(
                """
<sdf version='1.9'>
  <model name='x500'>
    <plugin name="gz::sim::systems::MulticopterMotorModel">
      <timeConstantUp>0.01</timeConstantUp>
      <timeConstantDown>0.02</timeConstantDown>
      <maxRotVelocity>900</maxRotVelocity>
      <motorConstant>1.2e-5</motorConstant>
      <momentConstant>0.02</momentConstant>
      <rotorDragCoefficient>7e-5</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-6</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>8</rotorVelocitySlowdownSim>
    </plugin>
  </model>
</sdf>
""".strip(),
                encoding="utf-8",
            )
            base_path.write_text(
                """
<sdf version='1.9'>
  <model name='x500_base'>
    <link name='base_link'>
      <inertial>
        <mass>2.5</mass>
        <inertia>
          <ixx>0.021</ixx>
          <iyy>0.022</iyy>
          <izz>0.040</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>
""".strip(),
                encoding="utf-8",
            )
            payload = parse_x500_sdf_reference(model_path, base_path)
            self.assertAlmostEqual(payload["mass"]["mass_kg"], 2.5)
            self.assertAlmostEqual(payload["inertia"]["x"]["inertia_kgm2"], 0.021)
            self.assertAlmostEqual(payload["motor_model"]["time_constant_up_s"], 0.01)

    def test_collect_identification_logs_and_aggregate(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "results" / "task_a" / "identification_traces"
            root.mkdir(parents=True)
            csv_path = root / "eval_00000.csv"
            csv_path.write_text(
                "timestamp_us,profile,az_world_mps2,thrust_cmd\n0,hover_thrust,0.0,0.5\n",
                encoding="utf-8",
            )
            logs = collect_identification_logs(root.parents[1])
            self.assertEqual(logs, [csv_path])
            rows, counts = aggregate_identification_rows(logs)
            self.assertEqual(len(rows), 1)
            self.assertEqual(counts["hover_thrust"], 1)

    def test_collect_identification_logs_prefers_latest_eval_per_task(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            trace_root = Path(tmp) / "results" / "task_a" / "identification_traces"
            trace_root.mkdir(parents=True)
            (trace_root / "eval_00000.csv").write_text("timestamp_us,profile\n0,hover_thrust\n", encoding="utf-8")
            latest = trace_root / "eval_00001.csv"
            latest.write_text("timestamp_us,profile\n1,hover_thrust\n", encoding="utf-8")
            logs = collect_identification_logs(trace_root.parents[1])
            self.assertEqual(logs, [latest])
            all_logs = collect_identification_logs(trace_root.parents[1], latest_only=False)
            self.assertEqual(all_logs, sorted(trace_root.glob("eval_*.csv")))

    def test_build_identification_mode_reports_separates_px4_and_truth_assisted(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            task_root = Path(tmp) / "results" / "task_a"
            ident_root = task_root / "identification_traces"
            truth_root = task_root / "gazebo_truth_traces"
            ident_root.mkdir(parents=True)
            truth_root.mkdir(parents=True)
            ident_path = ident_root / "eval_00000.csv"
            truth_path = truth_root / "eval_00000.csv"
            ident_path.write_text(
                "timestamp_us,profile,az,thrust_cmd,motor_0,motor_1,motor_2,motor_3\n"
                "500000,hover_thrust,0.0,0.5,0.5,0.5,0.5,0.5\n"
                "1000000,motor_step,0.0,0.5,0.5,0.5,0.5,0.5\n",
                encoding="utf-8",
            )
            truth_path.write_text(
                "sim_time_us,thrust_n,observed_max_rot_velocity_radps,rotor_0_actual_radps,rotor_1_actual_radps,rotor_2_actual_radps,rotor_3_actual_radps,az_world_mps2,"
                "truth_mass_kg,truth_ixx_kgm2,truth_iyy_kgm2,truth_izz_kgm2,"
                "truth_time_constant_up_s,truth_time_constant_down_s,truth_max_rot_velocity_radps,"
                "truth_motor_constant,truth_moment_constant,truth_rotor_drag_coefficient,truth_rolling_moment_coefficient,truth_rotor_velocity_slowdown_sim\n"
                "500000,19.6133,800,400,400,400,400,0.0,2.0,0.02,0.02,0.04,0.01,0.02,800,1e-5,0.01,8e-5,1e-6,10.0\n"
                "1000000,19.6133,800,400,400,400,400,0.0,2.0,0.02,0.02,0.04,0.01,0.02,800,1e-5,0.01,8e-5,1e-6,10.0\n",
                encoding="utf-8",
            )
            sdf_reference = {
                "mass": {"mass_kg": 2.0},
                "inertia": {
                    "x": {"inertia_kgm2": 0.02},
                    "y": {"inertia_kgm2": 0.02},
                    "z": {"inertia_kgm2": 0.04},
                },
                "motor_model": {
                    "time_constant_up_s": 0.01,
                    "time_constant_down_s": 0.02,
                    "max_rot_velocity_radps": 800.0,
                    "motor_constant": 1.0e-5,
                    "moment_constant": 0.01,
                    "rotor_drag_coefficient": 8.0e-5,
                    "rolling_moment_coefficient": 1.0e-6,
                    "rotor_velocity_slowdown_sim": 10.0,
                },
            }
            reports = build_identification_mode_reports([ident_path], sdf_reference)
            self.assertIn("px4_only", reports)
            self.assertIn("telemetry_augmented", reports)
            self.assertIn("truth_assisted", reports)
            self.assertEqual(
                reports["px4_only"]["identified"]["motor_model"]["max_rot_velocity_radps"]["sample_count"],
                0,
            )
            self.assertGreater(
                reports["telemetry_augmented"]["identified"]["motor_model"]["max_rot_velocity_radps"]["sample_count"],
                0,
            )
            self.assertGreater(
                reports["truth_assisted"]["identified"]["mass"]["sample_count"],
                0,
            )
            self.assertAlmostEqual(reports["truth_assisted"]["identified"]["mass"]["mass_kg"], 2.0)
            self.assertAlmostEqual(reports["truth_assisted"]["identified"]["inertia"]["x"]["inertia_kgm2"], 0.02)
            self.assertAlmostEqual(reports["truth_assisted"]["identified"]["inertia"]["y"]["inertia_kgm2"], 0.02)
            self.assertAlmostEqual(reports["truth_assisted"]["identified"]["inertia"]["z"]["inertia_kgm2"], 0.04)
            self.assertAlmostEqual(reports["truth_assisted"]["identified"]["motor_model"]["time_constant_up_s"]["value"], 0.01)
            self.assertAlmostEqual(reports["truth_assisted"]["identified"]["motor_model"]["time_constant_down_s"]["value"], 0.02)

    def test_compare_identified_to_sdf_reports_metric_errors(self) -> None:
        identified = {
            "mass": {"mass_kg": 2.1},
            "inertia": {
                "x": {"inertia_kgm2": 0.022},
                "y": {"inertia_kgm2": 0.023},
                "z": {"inertia_kgm2": 0.041},
            },
            "motor_model": {
                "time_constant_up_s": {"value": 0.011},
                "time_constant_down_s": {"value": 0.021},
                "max_rot_velocity_radps": {"value": 910.0},
                "motor_constant": {"value": 1.25e-5},
                "moment_constant": {"value": 0.021},
                "rotor_drag_coefficient": {"value": 7.2e-5},
                "rolling_moment_coefficient": {"value": 1.1e-6},
                "rotor_velocity_slowdown_sim": {"value": 7.8},
            },
        }
        sdf_reference = {
            "mass": {"mass_kg": 2.0},
            "inertia": {
                "x": {"inertia_kgm2": 0.021},
                "y": {"inertia_kgm2": 0.022},
                "z": {"inertia_kgm2": 0.040},
            },
            "motor_model": {
                "time_constant_up_s": 0.01,
                "time_constant_down_s": 0.02,
                "max_rot_velocity_radps": 900.0,
                "motor_constant": 1.2e-5,
                "moment_constant": 0.02,
                "rotor_drag_coefficient": 7.0e-5,
                "rolling_moment_coefficient": 1.0e-6,
                "rotor_velocity_slowdown_sim": 8.0,
            },
        }
        comparison = compare_identified_to_sdf(identified, sdf_reference)
        self.assertIn("mass_kg", comparison["comparable_metrics"])
        self.assertAlmostEqual(comparison["comparable_metrics"]["mass_kg"]["abs_error"], 0.1)
        self.assertIn("motor_constant", comparison["comparable_metrics"])
        self.assertAlmostEqual(comparison["comparable_metrics"]["motor_constant"]["identified"], 1.25e-5)
        self.assertIn("blended_twin_score", comparison)
        self.assertGreater(comparison["blended_twin_score"]["score"], 0.0)
        self.assertIn("non_comparable_metrics", comparison)


if __name__ == "__main__":
    unittest.main()
