from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree as ET

from experimental_validation.prepare_identified_model import prepare_identified_model


class PrepareIdentifiedModelTests(unittest.TestCase):
    def _write_stock_model(self, models_root: Path) -> None:
        x500 = models_root / "x500"
        x500_base = models_root / "x500_base"
        x500.mkdir(parents=True, exist_ok=True)
        x500_base.mkdir(parents=True, exist_ok=True)
        (x500 / "model.config").write_text("<model><name>x500</name></model>", encoding="utf-8")
        (x500_base / "model.config").write_text("<model><name>x500_base</name></model>", encoding="utf-8")
        (x500 / "model.sdf").write_text(
            """<?xml version='1.0'?>
<sdf version='1.9'>
  <model name='x500'>
    <include><uri>model://x500_base</uri></include>
    <plugin name='gz::sim::systems::MulticopterMotorModel' filename='gz-sim-multicopter-motor-model-system'>
      <timeConstantUp>0.01</timeConstantUp>
      <timeConstantDown>0.02</timeConstantDown>
      <maxRotVelocity>900</maxRotVelocity>
      <motorConstant>1.0e-05</motorConstant>
      <momentConstant>0.014</momentConstant>
      <rotorDragCoefficient>7e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    </plugin>
  </model>
</sdf>
""",
            encoding="utf-8",
        )
        (x500_base / "model.sdf").write_text(
            """<?xml version='1.0'?>
<sdf version='1.9'><model name='x500_base'><link name='base_link'><inertial><mass>1.0</mass><inertia><ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.01</iyy><iyz>0</iyz><izz>0.02</izz></inertia></inertial></link></model></sdf>
""",
            encoding="utf-8",
        )

    def _write_candidate(self, candidate_dir: Path) -> None:
        candidate_dir.mkdir(parents=True, exist_ok=True)
        (candidate_dir / "identified_parameters.json").write_text(
            json.dumps(
                {
                    "mass": {"mass_kg": 2.0},
                    "inertia": {
                        "x": {"inertia_kgm2": 0.021},
                        "y": {"inertia_kgm2": 0.022},
                        "z": {"inertia_kgm2": 0.041},
                    },
                    "motor_model": {
                        "time_constant_up_s": {"value": 0.0125},
                        "time_constant_down_s": {"value": 0.025},
                        "max_rot_velocity_radps": {"value": 1000.0},
                        "motor_constant": {"value": 8.5e-06},
                        "moment_constant": {"value": 0.016},
                        "rotor_drag_coefficient": {"value": 8.0e-05},
                        "rolling_moment_coefficient": {"value": 1.0e-06},
                        "rotor_velocity_slowdown_sim": {"value": 10.0},
                    },
                }
            ),
            encoding="utf-8",
        )
        (candidate_dir / "candidate_x500_base.sdf").write_text(
            """<?xml version='1.0'?>
<sdf version='1.9'><model name='x500_base'><link name='base_link'><inertial><mass>2.0</mass><inertia><ixx>0.021</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.022</iyy><iyz>0</iyz><izz>0.041</izz></inertia></inertial></link></model></sdf>
""",
            encoding="utf-8",
        )

    def test_prepare_identified_model_creates_expected_directories(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            px4_root = Path(tmp) / "PX4-Autopilot"
            models_root = px4_root / "Tools" / "simulation" / "gz" / "models"
            self._write_stock_model(models_root)
            candidate_dir = Path(tmp) / "candidate"
            self._write_candidate(candidate_dir)
            payload = prepare_identified_model(px4_root, candidate_dir, model_name="x500_identified")
            self.assertTrue((Path(payload["model_dir"]) / "model.sdf").exists())
            tree = ET.parse(Path(payload["model_dir"]) / "model.sdf")
            include_uri = tree.getroot().find("./model/include/uri")
            self.assertEqual(include_uri.text, "model://x500_identified_base")
            plugin = tree.getroot().find("./model/plugin")
            self.assertEqual(plugin.find("timeConstantUp").text, "0.0125")


if __name__ == "__main__":
    unittest.main()
