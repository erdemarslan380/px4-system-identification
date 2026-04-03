#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

TRAJ_ID_TO_CASE = {
    100: "hairpin",
    101: "lemniscate",
    102: "circle",
    103: "time_optimal_30s",
    104: "minimum_snap_50s",
}


def latest_match(paths: list[Path]) -> Path:
    return max(paths, key=lambda p: (p.stat().st_mtime_ns, p.name))


def build_staged_tracking_root(source_root: Path, out_root: Path) -> dict[str, str]:
    source_tracking = source_root / "tracking_logs"
    if not source_tracking.exists():
        raise FileNotFoundError(f"tracking_logs not found under {source_root}")

    out_tracking = out_root / "tracking_logs"
    out_tracking.mkdir(parents=True, exist_ok=True)

    staged: dict[str, str] = {}
    for traj_id, case_name in TRAJ_ID_TO_CASE.items():
        canonical = source_tracking / f"{case_name}.csv"
        if canonical.exists():
            src = canonical
        else:
            matches = sorted(source_tracking.glob(f"t{traj_id}r*.csv"))
            if not matches:
                raise FileNotFoundError(f"No HIL tracking log found for trajectory {traj_id} ({case_name}) in {source_tracking}")
            src = latest_match(matches)

        dst = out_tracking / f"{case_name}.csv"
        shutil.copy2(src, dst)
        staged[case_name] = str(dst.resolve())

    return staged


def main() -> int:
    ap = argparse.ArgumentParser(description="Stage HIL trajectory tracking CSVs into canonical case names for figure generation.")
    ap.add_argument("--source-root", required=True, help="Root containing tracking_logs/*.csv from HIL pulls.")
    ap.add_argument("--out-root", required=True, help="Destination root to create tracking_logs/{case}.csv under.")
    args = ap.parse_args()

    staged = build_staged_tracking_root(
        source_root=Path(args.source_root).expanduser().resolve(),
        out_root=Path(args.out_root).expanduser().resolve(),
    )
    for case_name, path in staged.items():
        print(f"{case_name}: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
