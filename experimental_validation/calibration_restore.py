"""Generate reusable calibration-restore files from a QGC parameter dump."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

from experimental_validation.qgc_params import parse_qgc_parameter_dump


_EXACT_NAMES = {
    "SENS_BOARD_ROT",
    "SENS_BOARD_X_OFF",
    "SENS_BOARD_Y_OFF",
    "SENS_BOARD_Z_OFF",
    "SYS_AUTOSTART",
}

_PREFIXES = (
    "CAL_",
    "SENS_",
    "RC",
)

_RC_PATTERNS = (
    re.compile(r"^RC\d+_(MIN|MAX|TRIM|REV|DZ|DEADZ)$"),
    re.compile(r"^RC_MAP_[A-Z0-9_]+$"),
    re.compile(r"^RC_[A-Z0-9_]+$"),
)


def select_calibration_params(params: dict[str, float]) -> dict[str, float]:
    selected: dict[str, float] = {}
    for name, value in sorted(params.items()):
        if name in _EXACT_NAMES:
            selected[name] = value
            continue
        if any(name.startswith(prefix) for prefix in ("CAL_", "SENS_")):
            selected[name] = value
            continue
        if any(pattern.match(name) for pattern in _RC_PATTERNS):
            selected[name] = value
            continue
    return selected


def write_restore_outputs(params: dict[str, float], out_dir: str | Path) -> dict[str, str]:
    out = Path(out_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)

    json_path = out / "selected_calibration_params.json"
    qgc_path = out / "restore_calibration.params"
    nsh_path = out / "restore_calibration.nsh"

    json_path.write_text(json.dumps(params, indent=2), encoding="utf-8")
    qgc_lines = ["# Selected calibration parameters"]
    qgc_lines.extend(f"{name},{value}" for name, value in sorted(params.items()))
    qgc_path.write_text("\n".join(qgc_lines) + "\n", encoding="utf-8")

    nsh_lines = [
        "#!/bin/sh",
        "# Generated calibration restore script",
        "set +e",
    ]
    nsh_lines.extend(f"param set {name} {value}" for name, value in sorted(params.items()))
    nsh_lines.extend(["param save", "reboot"])
    nsh_path.write_text("\n".join(nsh_lines) + "\n", encoding="utf-8")
    nsh_path.chmod(0o755)

    return {
        "json": str(json_path),
        "params": str(qgc_path),
        "nsh": str(nsh_path),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Generate calibration restore files from a QGroundControl parameter dump.")
    ap.add_argument("--input", required=True, help="QGroundControl parameter dump path.")
    ap.add_argument("--out-dir", required=True, help="Output directory for restore artifacts.")
    args = ap.parse_args()

    input_path = Path(args.input).resolve()
    if not input_path.exists():
        raise RuntimeError(f"parameter dump not found: {input_path}")

    params = parse_qgc_parameter_dump(input_path)
    selected = select_calibration_params(params)
    if not selected:
        raise RuntimeError(f"no calibration-like parameters found in: {input_path}")
    outputs = write_restore_outputs(selected, args.out_dir)
    print(json.dumps({
        "ok": True,
        "input": str(input_path),
        "selected_count": len(selected),
        "outputs": outputs,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
