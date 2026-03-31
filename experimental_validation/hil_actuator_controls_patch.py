#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path


TARGET_RELATIVE_PATH = Path("src/modules/mavlink/streams/HIL_ACTUATOR_CONTROLS.hpp")

OLD_GET_SIZE = (
    "\t\treturn _act_sub.advertised() ? MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;\n"
)
NEW_GET_SIZE = (
    "\t\treturn (_act_sim_sub.advertised() || _act_raw_sub.advertised()) ?\n"
    "\t\t       MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;\n"
)

OLD_CONSTRUCTOR = """\
\texplicit MavlinkStreamHILActuatorControls(Mavlink *mavlink) : MavlinkStream(mavlink)
\t{
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
\t\t_act_sub = uORB::Subscription{ORB_ID(actuator_outputs_sim)};
#endif
\t}

\tuORB::Subscription _act_sub{ORB_ID(actuator_outputs)};
"""

NEW_CONSTRUCTOR = """\
\texplicit MavlinkStreamHILActuatorControls(Mavlink *mavlink) : MavlinkStream(mavlink) {}

\tstatic float normalize_hil_control(float output)
\t{
\t\t// actuator_outputs_sim already uses the HIL/SITL range [-1, 1].
\t\t// Hardware HIL can still publish PWM-like actuator_outputs in microseconds.
\t\t// Convert the common 1000..2000 us motor range back to [0, 1] before
\t\t// forwarding it to jMAVSim so the simulator doesn't see thousand-scale inputs.
\t\tif (output > 1.5f || output < -1.5f) {
\t\t\tif (output >= 900.f && output <= 2100.f) {
\t\t\t\toutput = (output - 1000.f) / 1000.f;
\t\t\t}
\t\t}

\t\tif (output > 1.f) {
\t\t\treturn 1.f;
\t\t}

\t\tif (output < -1.f) {
\t\t\treturn -1.f;
\t\t}

\t\treturn output;
\t}

\tuORB::Subscription _act_raw_sub{ORB_ID(actuator_outputs)};
\tuORB::Subscription _act_sim_sub{ORB_ID(actuator_outputs_sim)};
"""

OLD_SEND = """\
\t\tactuator_outputs_s act;

\t\tif (_act_sub.update(&act)) {
\t\t\tmavlink_hil_actuator_controls_t msg{};
\t\t\tmsg.time_usec = act.timestamp;

\t\t\tfor (unsigned i = 0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; i++) {
\t\t\t\tmsg.controls[i] = act.output[i];
\t\t\t}
"""

NEW_SEND = """\
\t\tactuator_outputs_s act;
\t\tbool got_outputs = false;
\t\tbool using_sim_outputs = false;

\t\tif (_act_sim_sub.update(&act)) {
\t\t\tgot_outputs = true;
\t\t\tusing_sim_outputs = true;

\t\t} else if (_act_raw_sub.update(&act)) {
\t\t\tgot_outputs = true;
\t\t}

\t\tif (got_outputs) {
\t\t\tmavlink_hil_actuator_controls_t msg{};
\t\t\tmsg.time_usec = act.timestamp;

\t\t\tfor (unsigned i = 0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; i++) {
\t\t\t\tmsg.controls[i] = using_sim_outputs ? act.output[i] : normalize_hil_control(act.output[i]);
\t\t\t}
"""


def patch_hil_actuator_controls_text(text: str) -> tuple[str, bool]:
    changed = False

    if "normalize_hil_control" not in text:
        if OLD_CONSTRUCTOR not in text:
            raise RuntimeError("Could not find constructor patch point in HIL_ACTUATOR_CONTROLS.hpp")
        text = text.replace(OLD_CONSTRUCTOR, NEW_CONSTRUCTOR, 1)
        changed = True

    if "_act_raw_sub.advertised()" not in text:
        if OLD_GET_SIZE not in text:
            raise RuntimeError("Could not find get_size() patch point in HIL_ACTUATOR_CONTROLS.hpp")
        text = text.replace(OLD_GET_SIZE, NEW_GET_SIZE, 1)
        changed = True

    if "using_sim_outputs ? act.output[i] : normalize_hil_control(act.output[i])" not in text:
        if OLD_SEND not in text:
            raise RuntimeError("Could not find send() patch point in HIL_ACTUATOR_CONTROLS.hpp")
        text = text.replace(OLD_SEND, NEW_SEND, 1)
        changed = True

    return text, changed


def patch_px4_tree(px4_root: Path) -> Path:
    target = px4_root / TARGET_RELATIVE_PATH
    if not target.is_file():
        raise FileNotFoundError(target)

    original = target.read_text(encoding="utf-8")
    patched, changed = patch_hil_actuator_controls_text(original)
    if changed:
        target.write_text(patched, encoding="utf-8")
    return target


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Patch PX4 HIL_ACTUATOR_CONTROLS so hardware HIL prefers normalized actuator_outputs_sim."
    )
    parser.add_argument("--px4-root", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    target = patch_px4_tree(args.px4_root)
    print(target)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
