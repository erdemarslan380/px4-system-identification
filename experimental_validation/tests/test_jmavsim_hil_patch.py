from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from experimental_validation.jmavsim_hil_patch import (
    DEFAULT_HIL_STATE_INTERVAL_US,
    DISARM_GUARD_SNIPPET,
    HIL_STATE_ALT_NEW,
    patch_mavlink_hil_system_text,
    patch_px4_tree,
)


ORIGINAL_SNIPPET = """\
public class MAVLinkHILSystem extends MAVLinkHILSystemBase {
    private boolean gotHeartBeat = false;
    private boolean inited = false;
    private boolean stopped = false;
    private boolean gotHilActuatorControls = false;
    private long hilStateUpdateInterval = -1; //don't publish by default
    private long nextHilStatePub = 0;

    public void handleMessage(MAVLinkMessage msg) {
        if ("HIL_ACTUATOR_CONTROLS".equals(msg.getMsgName())) {
            List<Double> control = new ArrayList<Double>();
            boolean armed = true;
            simulator.advanceTime();

            vehicle.setControl(control);

        } else if ("HIL_CONTROLS".equals(msg.getMsgName()) &&
                   !gotHilActuatorControls) {
            List<Double> control = Arrays.asList(0.0, 0.0);
            boolean armed = true;

            vehicle.setControl(control);

        } else if ("COMMAND_LONG".equals(msg.getMsgName())) {
        }
    }

    public void update() {
        Sensors sensors = vehicle.getSensors();
        int alt = (int)(1000 * vehicle.position.z);
    }
}
"""


class JmavsimHilPatchTest(unittest.TestCase):
    def test_patch_text_enables_default_hil_state_interval(self) -> None:
        patched, changed = patch_mavlink_hil_system_text(ORIGINAL_SNIPPET)
        self.assertTrue(changed)
        self.assertIn(
            f"private static final long DEFAULT_HIL_STATE_UPDATE_INTERVAL_US = {DEFAULT_HIL_STATE_INTERVAL_US}L;",
            patched,
        )
        self.assertIn(
            "private long hilStateUpdateInterval = DEFAULT_HIL_STATE_UPDATE_INTERVAL_US;",
            patched,
        )
        self.assertNotIn("private long hilStateUpdateInterval = -1;", patched)

    def test_patch_text_is_idempotent(self) -> None:
        once, changed_once = patch_mavlink_hil_system_text(ORIGINAL_SNIPPET)
        twice, changed_twice = patch_mavlink_hil_system_text(once)
        self.assertTrue(changed_once)
        self.assertFalse(changed_twice)
        self.assertEqual(once, twice)

    def test_patch_text_zeroes_controls_when_disarmed(self) -> None:
        patched, _ = patch_mavlink_hil_system_text(ORIGINAL_SNIPPET)
        self.assertIn(DISARM_GUARD_SNIPPET.strip(), patched)
        self.assertEqual(patched.count("control.set(i, 0.0);"), 2)

    def test_patch_text_uses_global_altitude_for_hil_state(self) -> None:
        patched, _ = patch_mavlink_hil_system_text(ORIGINAL_SNIPPET)
        self.assertIn(HIL_STATE_ALT_NEW.strip(), patched)
        self.assertNotIn("int alt = (int)(1000 * vehicle.position.z);", patched)

    def test_patch_px4_tree_updates_target_file(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            px4_root = Path(tmpdir)
            target = px4_root / "Tools/simulation/jmavsim/jMAVSim/src/me/drton/jmavsim/MAVLinkHILSystem.java"
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(ORIGINAL_SNIPPET, encoding="utf-8")

            patched_target = patch_px4_tree(px4_root)

            self.assertEqual(patched_target, target)
            self.assertIn(
                "private long hilStateUpdateInterval = DEFAULT_HIL_STATE_UPDATE_INTERVAL_US;",
                target.read_text(encoding="utf-8"),
            )
            self.assertIn("control.set(i, 0.0);", target.read_text(encoding="utf-8"))
            self.assertIn(HIL_STATE_ALT_NEW.strip(), target.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
