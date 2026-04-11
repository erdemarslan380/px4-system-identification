#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Split a full-session jMAVSim truth log into per-profile CSVs.")
    ap.add_argument("--ident-root", required=True, help="Directory containing pulled identification CSVs")
    ap.add_argument("--truth-csv", required=True, help="Full-session jMAVSim truth CSV")
    ap.add_argument("--summary-json", required=True, help="single_session_summary.json from the helper")
    ap.add_argument("--out-dir", required=True, help="Destination directory for split truth CSVs")
    ap.add_argument("--padding-ms", type=float, default=250.0, help="Extra host-time padding around each maneuver")
    return ap.parse_args()


def _load_summary(summary_path: Path) -> list[dict]:
    payload = json.loads(summary_path.read_text(encoding="utf-8"))
    profiles = payload.get("profiles", [])
    if not isinstance(profiles, list) or not profiles:
        raise RuntimeError(f"No profile summaries found in {summary_path}")
    return profiles


def _load_ident_map(ident_root: Path) -> dict[str, Path]:
    mapping: dict[str, Path] = {}
    for path in sorted(ident_root.glob("*.csv")):
        stem = path.stem
        profile = stem
        match = re.match(r"^(?P<profile>.+)_s\d+_r\d+_[0-9a-fA-F]+$", stem)
        if match:
            profile = match.group("profile")
        elif "_r" in stem:
            profile = stem.split("_r", 1)[0]
        mapping[profile] = path.resolve()
    if not mapping:
        raise RuntimeError(f"No identification CSVs found under {ident_root}")
    return mapping


def _select_host_column(fieldnames: list[str]) -> str:
    for candidate in ("host_mono_ns", "host_wall_time_ns"):
        if candidate in fieldnames:
            return candidate
    raise RuntimeError("Truth CSV is missing both host_mono_ns and host_wall_time_ns columns")


def _truth_window_rows(
    rows: list[dict[str, str]],
    *,
    host_column: str,
    start_ns: int,
    end_ns: int,
) -> list[dict[str, str]]:
    selected: list[dict[str, str]] = []
    for row in rows:
        raw = row.get(host_column, "").strip()
        if not raw:
            continue
        sample_ns = int(raw)
        if start_ns <= sample_ns <= end_ns:
            selected.append(row)
    return selected


def main() -> int:
    args = parse_args()
    ident_root = Path(args.ident_root).expanduser().resolve()
    truth_csv = Path(args.truth_csv).expanduser().resolve()
    summary_json = Path(args.summary_json).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    ident_map = _load_ident_map(ident_root)
    profile_summaries = _load_summary(summary_json)

    with truth_csv.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        fieldnames = list(reader.fieldnames or [])
        if not fieldnames:
            raise RuntimeError(f"Truth CSV has no header: {truth_csv}")
        host_column = _select_host_column(fieldnames)
        truth_rows = list(reader)

    padding_ns = int(args.padding_ms * 1_000_000.0)
    manifest: list[dict[str, object]] = []
    for segment in profile_summaries:
        profile = str(segment["profile"])
        ident_csv = ident_map.get(profile)
        if ident_csv is None:
            raise RuntimeError(f"No identification CSV found for profile {profile}")

        if host_column == "host_mono_ns":
            start_ns = int(segment["host_mono_time_start_ns"]) - padding_ns
            end_ns = int(segment["host_mono_time_end_ns"]) + padding_ns
        else:
            start_ns = int(segment["host_wall_time_start_ns"]) - padding_ns
            end_ns = int(segment["host_wall_time_end_ns"]) + padding_ns

        split_rows = _truth_window_rows(
            truth_rows,
            host_column=host_column,
            start_ns=start_ns,
            end_ns=end_ns,
        )
        if not split_rows:
            raise RuntimeError(
                f"No truth samples matched profile {profile} in {truth_csv.name} using {host_column} "
                f"window [{start_ns}, {end_ns}]"
            )

        output_csv = out_dir / ident_csv.name
        with output_csv.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(split_rows)

        manifest.append(
            {
                "profile": profile,
                "ident_csv": str(ident_csv),
                "truth_csv": str(output_csv),
                "rows": len(split_rows),
                "host_column": host_column,
            }
        )

    (out_dir / "split_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
