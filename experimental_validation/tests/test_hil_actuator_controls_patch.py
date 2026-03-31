from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from experimental_validation.hil_actuator_controls_patch import (
    patch_hil_actuator_controls_text,
    patch_px4_tree,
)


ORIGINAL_SNIPPET = """\
class MavlinkStreamHILActuatorControls : public MavlinkStream
{
public:
\tunsigned get_size() override
\t{
\t\treturn _act_sub.advertised() ? MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
\t}

private:
\texplicit MavlinkStreamHILActuatorControls(Mavlink *mavlink) : MavlinkStream(mavlink)
\t{
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
\t\t_act_sub = uORB::Subscription{ORB_ID(actuator_outputs_sim)};
#endif
\t}

\tuORB::Subscription _act_sub{ORB_ID(actuator_outputs)};

\tbool send() override
\t{
\t\tactuator_outputs_s act;

\t\tif (_act_sub.update(&act)) {
\t\t\tmavlink_hil_actuator_controls_t msg{};
\t\t\tmsg.time_usec = act.timestamp;

\t\t\tfor (unsigned i = 0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; i++) {
\t\t\t\tmsg.controls[i] = act.output[i];
\t\t\t}
\t\t}
\t\treturn false;
\t}
};
"""


class HilActuatorControlsPatchTest(unittest.TestCase):
    def test_patch_text_prefers_sim_outputs(self) -> None:
        patched, changed = patch_hil_actuator_controls_text(ORIGINAL_SNIPPET)
        self.assertTrue(changed)
        self.assertIn("_act_sim_sub", patched)
        self.assertIn("_act_raw_sub", patched)
        self.assertIn("if (_act_sim_sub.update(&act))", patched)
        self.assertIn("} else if (_act_raw_sub.update(&act)) {", patched)

    def test_patch_text_normalizes_raw_outputs(self) -> None:
        patched, _ = patch_hil_actuator_controls_text(ORIGINAL_SNIPPET)
        self.assertIn("normalize_hil_control", patched)
        self.assertIn("(output - 1000.f) / 1000.f", patched)
        self.assertIn("using_sim_outputs ? act.output[i] : normalize_hil_control(act.output[i])", patched)

    def test_patch_text_is_idempotent(self) -> None:
        once, changed_once = patch_hil_actuator_controls_text(ORIGINAL_SNIPPET)
        twice, changed_twice = patch_hil_actuator_controls_text(once)
        self.assertTrue(changed_once)
        self.assertFalse(changed_twice)
        self.assertEqual(once, twice)

    def test_patch_px4_tree_updates_target_file(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            px4_root = Path(tmpdir)
            target = px4_root / "src/modules/mavlink/streams/HIL_ACTUATOR_CONTROLS.hpp"
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(ORIGINAL_SNIPPET, encoding="utf-8")

            patched_target = patch_px4_tree(px4_root)

            self.assertEqual(patched_target, target)
            content = target.read_text(encoding="utf-8")
            self.assertIn("_act_sim_sub", content)
            self.assertIn("normalize_hil_control", content)


if __name__ == "__main__":
    unittest.main()
