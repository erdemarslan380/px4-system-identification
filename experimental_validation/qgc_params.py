"""Helpers to parse QGroundControl parameter exports."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Dict

_PARAM_NAME_RE = re.compile(r"^[A-Z0-9_]+$")


def _try_float(text: str) -> float | None:
    try:
        return float(str(text).strip())
    except (TypeError, ValueError):
        return None


def parse_qgc_parameter_dump(path: str | Path) -> Dict[str, float]:
    """Parse a QGC-exported parameter file.

    Supported formats:
    - QGC tab-separated lines: SYSID COMPID NAME VALUE TYPE
    - Simple CSV/TSV/TXT lines: NAME,VALUE or NAME VALUE
    """

    params: Dict[str, float] = {}
    for raw_line in Path(path).read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue

        if "\t" in line:
            parts = [part.strip() for part in line.split("\t") if part.strip()]
            if len(parts) >= 4 and _PARAM_NAME_RE.match(parts[2]):
                value = _try_float(parts[3])
                if value is not None:
                    params[parts[2]] = value
                    continue

        if "," in line:
            parts = [part.strip() for part in line.split(",") if part.strip()]
            if len(parts) >= 2 and _PARAM_NAME_RE.match(parts[0]):
                value = _try_float(parts[1])
                if value is not None:
                    params[parts[0]] = value
                    continue

        parts = [part.strip() for part in line.split() if part.strip()]
        if len(parts) >= 2 and _PARAM_NAME_RE.match(parts[0]):
            value = _try_float(parts[1])
            if value is not None:
                params[parts[0]] = value

    return params
