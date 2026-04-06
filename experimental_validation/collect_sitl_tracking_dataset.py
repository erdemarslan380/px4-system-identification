#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.run_sitl_validation import (  # noqa: E402
    DEFAULT_MODEL_SPECS,
    DEFAULT_VALIDATION_TRAJECTORIES,
    ValidationModelSpec,
    ValidationTrajectory,
    _resolve_trajectories,
    run_validation_model,
)


def _latest_tracking_log(run_rootfs: Path) -> Path | None:
    tracking_root = run_rootfs / "tracking_logs"
    if not tracking_root.exists():
        return None
    csvs = sorted(tracking_root.glob("*.csv"), key=lambda path: path.stat().st_mtime)
    return csvs[-1] if csvs else None


def _copy(path: Path, dst: Path) -> str:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(path, dst)
    return str(dst.resolve())


def collect_tracking_dataset(
    *,
    px4_root: str | Path,
    out_root: str | Path,
    candidate_dir: str | Path,
    model_spec: ValidationModelSpec,
    trajectories: tuple[ValidationTrajectory, ...] = DEFAULT_VALIDATION_TRAJECTORIES,
    sitl_esc_min: int = 0,
    sitl_esc_max: int | None = None,
    sitl_hover_thrust: float | None = None,
    headless: bool = True,
    show_console: bool = False,
) -> dict:
    px4_root = Path(px4_root).expanduser().resolve()
    out_root = Path(out_root).expanduser().resolve()
    candidate_dir = Path(candidate_dir).expanduser().resolve()
    dataset_root = out_root / model_spec.label
    tracking_root = dataset_root / "tracking_logs"
    tracking_root.mkdir(parents=True, exist_ok=True)

    mode_label = "visual" if not headless else "headless"
    print(
        f"Collecting {len(trajectories)} trajectory runs for {model_spec.display_name} in {mode_label} mode.",
        flush=True,
    )
    if not headless:
        print(
            "Gazebo will reopen sequentially for each trajectory in the suite.",
            flush=True,
        )

    cases: list[dict[str, object]] = []
    for case_index, entry in enumerate(trajectories, start=1):
        case_root = out_root / "cases" / entry.name
        error_text: str | None = None
        copied_log: str | None = None
        print(
            f"[{case_index}/{len(trajectories)}] Starting trajectory: {entry.name}",
            flush=True,
        )

        try:
            manifest = run_validation_model(
                px4_root=px4_root,
                out_root=case_root,
                model_spec=model_spec,
                candidate_dir=candidate_dir,
                trajectories=(entry,),
                sitl_esc_min=sitl_esc_min,
                sitl_esc_max=sitl_esc_max,
                sitl_hover_thrust=sitl_hover_thrust,
                headless=headless,
                show_console=show_console,
            )
            if manifest.get("results"):
                copied_log = _copy(Path(manifest["results"][0]["tracking_log"]), tracking_root / f"{entry.name}.csv")
                print(
                    f"[{case_index}/{len(trajectories)}] Completed: {entry.name}",
                    flush=True,
                )
        except Exception as exc:
            error_text = str(exc)
            print(
                f"[{case_index}/{len(trajectories)}] Finished with gate/error: {entry.name}: {error_text}",
                flush=True,
            )
            run_rootfs = case_root / "runtime" / model_spec.label / "rootfs"
            latest = _latest_tracking_log(run_rootfs)
            if latest is not None:
                copied_log = _copy(latest, tracking_root / f"{entry.name}.csv")

        cases.append(
            {
                "name": entry.name,
                "traj_id": entry.traj_id,
                "tracking_log": copied_log,
                "ok": error_text is None,
                "error": error_text,
            }
        )

    summary = {
        "px4_root": str(px4_root),
        "candidate_dir": str(candidate_dir),
        "model_label": model_spec.label,
        "model_name": model_spec.gz_model,
        "dataset_root": str(dataset_root.resolve()),
        "cases": cases,
    }
    (dataset_root / "collection_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def resolve_model_spec_arg(
    *,
    model_label: str,
    gz_model: str = "",
    display_name: str = "",
) -> ValidationModelSpec:
    known = {item.label: item for item in DEFAULT_MODEL_SPECS}
    if model_label in known and not gz_model and not display_name:
        return known[model_label]
    if not gz_model:
        raise ValueError("--gz-model is required when --model-label is not one of the built-in defaults")
    return ValidationModelSpec(
        label=model_label,
        gz_model=gz_model,
        display_name=display_name or model_label,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="Collect one tracking CSV per trajectory using the canonical SITL runner.")
    ap.add_argument("--px4-root", default="~/PX4-Autopilot-Identification")
    ap.add_argument("--out-root", required=True)
    ap.add_argument("--candidate-dir", default="~/px4-system-identification/examples/paper_assets/candidates/x500_truth_assisted_sitl_v1")
    ap.add_argument("--model-label", required=True, help="Dataset label. Built-in values: stock_sitl_placeholder, digital_twin.")
    ap.add_argument("--gz-model", default="", help="Gazebo model name for non-default model labels.")
    ap.add_argument("--display-name", default="", help="Human-readable name for non-default model labels.")
    ap.add_argument("--trajectory-names", default="", help="Comma-separated subset of trajectory names. Default: all five.")
    ap.add_argument("--sitl-esc-max", type=int, default=None)
    ap.add_argument("--sitl-esc-min", type=int, default=0)
    ap.add_argument("--sitl-hover-thrust", type=float, default=None)
    ap.add_argument("--visual", action="store_true")
    ap.add_argument("--show-console", action="store_true")
    args = ap.parse_args()

    trajectory_names = [item.strip() for item in args.trajectory_names.split(",") if item.strip()] or None
    model_spec = resolve_model_spec_arg(
        model_label=args.model_label,
        gz_model=args.gz_model,
        display_name=args.display_name,
    )
    summary = collect_tracking_dataset(
        px4_root=args.px4_root,
        out_root=args.out_root,
        candidate_dir=args.candidate_dir,
        model_spec=model_spec,
        trajectories=_resolve_trajectories(trajectory_names),
        sitl_esc_min=args.sitl_esc_min,
        sitl_esc_max=args.sitl_esc_max,
        sitl_hover_thrust=args.sitl_hover_thrust,
        headless=not args.visual,
        show_console=args.show_console,
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
