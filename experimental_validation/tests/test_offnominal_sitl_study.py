from __future__ import annotations

import inspect
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import experimental_validation.offnominal_sitl_study as study
from experimental_validation.offnominal_sitl_study import (
    OffnominalPerturbation,
    PANEL_GROUPS,
    _runaway_cutoff_index,
    _trim_triplet,
    _trajectory_duration_s,
    offnominal_reference,
)
from experimental_validation.trajectory_catalog import DEFAULT_VALIDATION_TRAJECTORIES


class OffnominalSitlStudyTest(unittest.TestCase):
    def test_offnominal_reference_applies_expected_scales(self) -> None:
        reference = {
            "mass": {"mass_kg": 2.0},
            "inertia": {
                "x": {"inertia_kgm2": 0.02},
                "y": {"inertia_kgm2": 0.03},
                "z": {"inertia_kgm2": 0.04},
            },
            "motor_model": {
                "time_constant_up_s": 0.01,
                "time_constant_down_s": 0.02,
                "max_rot_velocity_radps": 900.0,
                "motor_constant": 8e-6,
                "moment_constant": 0.015,
                "rotor_drag_coefficient": 9e-5,
                "rolling_moment_coefficient": 1e-6,
                "rotor_velocity_slowdown_sim": 10.0,
            },
        }
        perturbation = OffnominalPerturbation()

        report = offnominal_reference(reference, perturbation)

        self.assertAlmostEqual(report["mass_kg"], 1.9)
        self.assertAlmostEqual(report["ixx_kgm2"], 0.019)
        self.assertAlmostEqual(report["iyy_kgm2"], 0.0285)
        self.assertAlmostEqual(report["izz_kgm2"], 0.038)
        self.assertAlmostEqual(report["time_constant_up_s"], 0.0105)
        self.assertAlmostEqual(report["time_constant_down_s"], 0.021)
        self.assertAlmostEqual(report["max_rot_velocity_radps"], 873.0)
        self.assertAlmostEqual(report["rotor_velocity_slowdown_sim"], 10.0)

    def test_duration_helper_reads_shipped_metadata(self) -> None:
        self.assertEqual(_trajectory_duration_s(DEFAULT_VALIDATION_TRAJECTORIES[0]), 23.0)
        self.assertEqual(_trajectory_duration_s(DEFAULT_VALIDATION_TRAJECTORIES[1]), 19.0)

    def test_panel_groups_cover_all_five_cases(self) -> None:
        flattened = [name for group in PANEL_GROUPS for name in group]
        self.assertEqual(flattened, [
            "circle",
            "hairpin",
            "lemniscate",
            "time_optimal_30s",
            "minimum_snap_50s",
        ])

    def test_ident_profile_is_set_before_identification_mode(self) -> None:
        source = inspect.getsource(study.run_identification_with_assets)
        self.assertLess(
            source.index('session.send(f"trajectory_reader set_ident_profile {profile}")'),
            source.index('session.send("trajectory_reader set_mode identification")'),
        )

    def test_tracking_log_is_waited_before_ident_log(self) -> None:
        source = inspect.getsource(study.run_identification_with_assets)
        self.assertLess(
            source.index('match_track = session.expect(TRACKING_LOG_START_RE, timeout_s=20)'),
            source.index('match_ident = session.expect(IDENT_LOG_START_RE, timeout_s=5)'),
        )

    def test_light_breeze_world_is_derived_from_default_world(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            px4_root = Path(tmp)
            worlds = px4_root / "Tools" / "simulation" / "gz" / "worlds"
            worlds.mkdir(parents=True)
            (worlds / "default.sdf").write_text(
                '<sdf version="1.9"><world name="default"><physics type="ode"/></world></sdf>',
                encoding="utf-8",
            )
            out = study.create_light_breeze_world(px4_root=px4_root, asset_root=px4_root, world_name="test_breeze")
            root = ET.parse(out).getroot()
            world = root.find("world")
            self.assertIsNotNone(world)
            self.assertEqual(world.attrib["name"], "test_breeze")
            self.assertEqual(world.findtext("wind/linear_velocity"), "0.6 0.2 0.0")

    def test_offnominal_model_keeps_motor_perturbations_after_truth_logger_injection(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            px4_root = Path(tmp)
            models = px4_root / "Tools" / "simulation" / "gz" / "models"
            stock_model = models / "x500"
            stock_base = models / "x500_base"
            stock_model.mkdir(parents=True)
            stock_base.mkdir(parents=True)
            (stock_base / "model.sdf").write_text(
                """
<sdf version="1.9">
  <model name="x500_base">
    <link name="base_link">
      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.02</ixx>
          <iyy>0.03</iyy>
          <izz>0.04</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>
""".strip(),
                encoding="utf-8",
            )
            (stock_base / "model.config").write_text("<model><name>x500_base</name></model>", encoding="utf-8")
            (stock_model / "model.sdf").write_text(
                """
<sdf version="1.9">
  <model name="x500">
    <include merge="true">
      <uri>model://x500_base</uri>
    </include>
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <timeConstantUp>0.01</timeConstantUp>
      <timeConstantDown>0.02</timeConstantDown>
      <maxRotVelocity>900</maxRotVelocity>
      <motorConstant>8e-06</motorConstant>
      <momentConstant>0.015</momentConstant>
      <rotorDragCoefficient>9e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    </plugin>
  </model>
</sdf>
""".strip(),
                encoding="utf-8",
            )
            (stock_model / "model.config").write_text("<model><name>x500</name></model>", encoding="utf-8")
            metadata = study.create_offnominal_model_assets(
                px4_root=px4_root,
                asset_root=px4_root / "assets",
                model_name="x500_offnominal",
                perturbation=study.OffnominalPerturbation(),
            )
            root = ET.parse(metadata["reference_model_sdf"]).getroot()
            plugin = root.find(".//plugin[@name='gz::sim::systems::MulticopterMotorModel']")
            self.assertIsNotNone(plugin)
            self.assertEqual(plugin.findtext("timeConstantUp"), "0.0105")
            self.assertEqual(plugin.findtext("timeConstantDown"), "0.021")

    def test_runaway_cutoff_detects_late_altitude_escape(self) -> None:
        ref = np.zeros((80, 3), dtype=float)
        pos = ref.copy()
        pos[55:, 2] = 10.0
        cutoff = _runaway_cutoff_index(ref, pos, z_error_threshold_m=6.0, consecutive_samples=5)
        self.assertEqual(cutoff, 55)

    def test_trim_triplet_uses_earliest_runaway_across_series(self) -> None:
        ref = np.zeros((100, 3), dtype=float)
        stock = ref.copy()
        real = ref.copy()
        stock[70:, 2] = 9.0
        real[60:, 2] = 9.0
        ref_out, stock_out, real_out = _trim_triplet(ref, stock, real)
        self.assertEqual(len(ref_out), 60)
        self.assertEqual(len(stock_out), 60)
        self.assertEqual(len(real_out), 60)


if __name__ == "__main__":
    unittest.main()
