#!/usr/bin/env python3

from __future__ import annotations

import argparse
import re
from pathlib import Path


TARGET_RELATIVE_PATH = Path(
    "Tools/simulation/jmavsim/jMAVSim/src/me/drton/jmavsim/MAVLinkHILSystem.java"
)
DEFAULT_HIL_STATE_INTERVAL_US = 20_000
DEFAULT_INTERVAL_DECLARATION = (
    f"    private static final long DEFAULT_HIL_STATE_UPDATE_INTERVAL_US = {DEFAULT_HIL_STATE_INTERVAL_US}L;\n"
)
DEFAULT_INTERVAL_LINE = (
    "    private long hilStateUpdateInterval = DEFAULT_HIL_STATE_UPDATE_INTERVAL_US; "
    "// publish ground truth by default for hardware HIL\n"
)
DISARM_GUARD_SNIPPET = """\
            if (!armed) {
                for (int i = 0; i < control.size(); ++i) {
                    control.set(i, 0.0);
                }
            }

"""
HIL_STATE_ALT_PATTERN = re.compile(
    r"(?m)^(?P<indent>[ \t]*)int alt = \(int\)\(1000 \* vehicle\.position\.z\);$"
)
HIL_STATE_ALT_NEW = "int alt = (int)(1000 * sensors.getGlobalPosition().alt);"


def patch_mavlink_hil_system_text(text: str) -> tuple[str, bool]:
    changed = False

    if DEFAULT_INTERVAL_DECLARATION in text and DEFAULT_INTERVAL_LINE in text:
        pass
    else:
        old_line = "    private long hilStateUpdateInterval = -1; //don't publish by default\n"
        new_block = DEFAULT_INTERVAL_DECLARATION + DEFAULT_INTERVAL_LINE

        if old_line in text:
            text = text.replace(old_line, new_block, 1)
            changed = True
        else:
            anchor = "    private boolean gotHilActuatorControls = false;\n"
            if anchor in text:
                text = text.replace(anchor, anchor + new_block, 1)
                changed = True
            else:
                raise RuntimeError("Could not find a patch point for MAVLinkHILSystem.java")

    if DISARM_GUARD_SNIPPET not in text:
        hil_actuator_anchor = "            simulator.advanceTime();\n\n            vehicle.setControl(control);\n"
        if hil_actuator_anchor in text:
            text = text.replace(
                hil_actuator_anchor,
                f"            simulator.advanceTime();\n\n{DISARM_GUARD_SNIPPET}            vehicle.setControl(control);\n",
                1,
            )
            changed = True
        else:
            raise RuntimeError("Could not find HIL_ACTUATOR_CONTROLS patch point in MAVLinkHILSystem.java")

        hil_controls_anchor = "            vehicle.setControl(control);\n\n        } else if (\"COMMAND_LONG\".equals(msg.getMsgName())) {\n"
        if hil_controls_anchor in text:
            text = text.replace(
                hil_controls_anchor,
                f"{DISARM_GUARD_SNIPPET}            vehicle.setControl(control);\n\n        }} else if (\"COMMAND_LONG\".equals(msg.getMsgName())) {{\n",
                1,
            )
            changed = True
        else:
            raise RuntimeError("Could not find HIL_CONTROLS patch point in MAVLinkHILSystem.java")

    if HIL_STATE_ALT_NEW in text:
        pass
    else:
        def repl(match: re.Match[str]) -> str:
            return f"{match.group('indent')}{HIL_STATE_ALT_NEW}"

        patched_text, count = HIL_STATE_ALT_PATTERN.subn(repl, text, count=1)
        if count == 0:
            raise RuntimeError("Could not find HIL_STATE_QUATERNION altitude patch point in MAVLinkHILSystem.java")
        text = patched_text
        changed = True

    return text, changed


def patch_px4_tree(px4_root: Path) -> Path:
    target = px4_root / TARGET_RELATIVE_PATH
    if not target.is_file():
        raise FileNotFoundError(target)

    original = target.read_text(encoding="utf-8")
    patched, changed = patch_mavlink_hil_system_text(original)
    if changed:
        target.write_text(patched, encoding="utf-8")
    return target


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Patch jMAVSim so hardware HIL always receives HIL_STATE_QUATERNION ground truth."
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
