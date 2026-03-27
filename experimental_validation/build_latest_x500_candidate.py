#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.compare_with_sdf import (
    build_identification_mode_reports,
    choose_primary_identification_mode,
    parse_x500_sdf_reference,
    write_comparison_outputs,
)

REQUIRED_PROFILES = (
    "hover_thrust",
    "mass_vertical",
    "roll_sweep",
    "pitch_sweep",
    "yaw_sweep",
    "drag_x",
    "drag_y",
    "drag_z",
    "motor_step",
)


def latest_profile_logs(ident_root: Path) -> tuple[dict[str, Path], list[str]]:
    selected: dict[str, Path] = {}
    missing: list[str] = []
    for profile in REQUIRED_PROFILES:
        matches = sorted(ident_root.glob(f"{profile}*.csv"), key=lambda path: path.stat().st_mtime)
        if not matches:
            missing.append(profile)
            continue
        selected[profile] = matches[-1].resolve()
    return selected, missing


def main() -> int:
    ap = argparse.ArgumentParser(description="Build one x500 candidate from the latest complete identification log set.")
    ap.add_argument("--rootfs", default=str(Path.home() / "PX4-Autopilot-Identification" / "build" / "px4_sitl_default" / "rootfs"))
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--truth-csv", default="", help="Optional explicit Gazebo truth CSV. If omitted, the latest file under sysid_truth_logs is used.")
    ap.add_argument("--sdf-model", default="Tools/simulation/gz/models/x500/model.sdf")
    ap.add_argument("--sdf-base-model", default="Tools/simulation/gz/models/x500_base/model.sdf")
    args = ap.parse_args()

    rootfs = Path(args.rootfs).expanduser().resolve()
    ident_root = rootfs / "identification_logs"
    truth_root = rootfs / "sysid_truth_logs"

    if not ident_root.exists():
        raise SystemExit(f"identification_logs not found: {ident_root}")

    logs_by_profile, missing = latest_profile_logs(ident_root)
    if missing:
        print(json.dumps({
            "ok": False,
            "missing_profiles": missing,
            "identification_logs": str(ident_root),
        }, indent=2))
        return 2

    truth_csvs: list[Path] = []
    if args.truth_csv:
        truth_csvs = [Path(args.truth_csv).expanduser().resolve()]
    elif truth_root.exists():
        matches = sorted(truth_root.glob("*.csv"), key=lambda path: path.stat().st_mtime)
        if matches:
            truth_csvs = [matches[-1].resolve()]

    csv_paths = [logs_by_profile[profile] for profile in REQUIRED_PROFILES]
    sdf_reference = parse_x500_sdf_reference(args.sdf_model, args.sdf_base_model)
    reports_by_mode = build_identification_mode_reports(csv_paths, sdf_reference, explicit_truth_csvs=truth_csvs)
    primary_mode = choose_primary_identification_mode(reports_by_mode)

    out_dir = Path(args.out_dir).expanduser().resolve()
    write_comparison_outputs(
        out_dir,
        reports_by_mode=reports_by_mode,
        sdf_reference=sdf_reference,
        csv_paths=csv_paths,
        primary_mode=primary_mode,
    )

    print(json.dumps({
        "ok": True,
        "rootfs": str(rootfs),
        "out_dir": str(out_dir),
        "primary_mode": primary_mode,
        "truth_csvs": reports_by_mode[primary_mode]["truth_csvs"],
        "selected_logs": {profile: str(path) for profile, path in logs_by_profile.items()},
        "comparison": reports_by_mode[primary_mode]["comparison"]["comparable_metrics"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
