#!/usr/bin/env python3
"""Evaluate multiple bridge candidates against repeated HITL logs and pick the best tube overlap."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.build_hitl_sitl_bridge_review_bundle import build_bundle
from experimental_validation.collect_sitl_tracking_dataset import collect_tracking_dataset
from experimental_validation.trajectory_catalog import validation_trajectory_id_map
from experimental_validation.run_sitl_validation import ValidationModelSpec

MODEL_SPEC = ValidationModelSpec(
    label="bridge_calibration",
    gz_model="x500_bridge_calibrated",
    display_name="Bridge calibration candidate",
    truth_state_bridge=False,
)


def _sha256(path: Path) -> str:
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Select the best bridge candidate using HITL-vs-SITL tube overlap.")
    ap.add_argument("--px4-root", default="~/PX4-Autopilot-Identification")
    ap.add_argument("--hitl-tracking-dir", required=True)
    ap.add_argument("--candidate-root", default="")
    ap.add_argument("--candidate-dir", action="append", default=[])
    ap.add_argument("--candidate-glob", default="*")
    ap.add_argument("--out-root", required=True)
    ap.add_argument("--case-name", default="lemniscate")
    ap.add_argument("--repetitions", type=int, default=5)
    ap.add_argument("--repeat-attempts", type=int, default=3)
    return ap.parse_args()


def _resolve_candidates(args: argparse.Namespace) -> list[Path]:
    candidates: list[Path] = [Path(item).expanduser().resolve() for item in args.candidate_dir]
    if args.candidate_root.strip():
        root = Path(args.candidate_root).expanduser().resolve()
        candidates.extend(sorted(path for path in root.glob(args.candidate_glob) if path.is_dir()))
    deduped: list[Path] = []
    seen: set[Path] = set()
    for item in candidates:
        if item not in seen:
            seen.add(item)
            deduped.append(item)
    return deduped


def _collect_repeats(
    *,
    px4_root: Path,
    candidate_dir: Path,
    case_name: str,
    repetitions: int,
    repeat_attempts: int,
    out_root: Path,
) -> Path:
    entry = next(value for value in validation_trajectory_id_map().values() if value.name == case_name)
    sitl_dir = out_root / "tracking_logs"
    sitl_dir.mkdir(parents=True, exist_ok=True)

    for repeat_idx in range(1, repetitions + 1):
        dst = sitl_dir / f"repeat_{repeat_idx:02d}.csv"
        if dst.exists() and dst.stat().st_size > 0:
            continue

        repeat_root = out_root / "runs" / f"repeat_{repeat_idx:02d}"
        last_case: dict | None = None
        for attempt in range(1, repeat_attempts + 1):
            print(
                f"[run] {candidate_dir.name}/{case_name}/repeat_{repeat_idx:02d} "
                f"(attempt {attempt}/{repeat_attempts})",
                flush=True,
            )
            dataset = collect_tracking_dataset(
                px4_root=px4_root,
                out_root=repeat_root,
                candidate_dir=candidate_dir,
                model_spec=MODEL_SPEC,
                trajectories=(entry,),
                headless=True,
                show_console=False,
                skip_landing_after_trajectories=True,
            )
            case = dataset["cases"][0]
            last_case = case
            if case.get("tracking_log"):
                src = Path(str(case["tracking_log"]))
                shutil.copy2(src, dst)
                break
        else:
            raise RuntimeError(
                f"missing tracking log for {candidate_dir.name}/{case_name}/repeat_{repeat_idx:02d}: {last_case}"
            )
    return sitl_dir


def _score_from_bundle(summary: dict[str, object]) -> dict[str, float]:
    pair = next(
        item
        for item in summary["overlap_pairs"]
        if item["pair_key"] == "hitl_repeatability__sitl_identified"
    )
    return {
        "mean_overlap_pct": float(pair["mean_overlap_pct"]),
        "contact_pct": float(pair["contact_pct"]),
        "mean_center_distance_m": float(pair["mean_center_distance_m"]),
        "max_center_distance_m": float(pair["max_center_distance_m"]),
    }


def main() -> int:
    args = parse_args()
    px4_root = Path(args.px4_root).expanduser().resolve()
    out_root = Path(args.out_root).expanduser().resolve()
    hitl_tracking_dir = Path(args.hitl_tracking_dir).expanduser().resolve()
    candidates = _resolve_candidates(args)
    if not candidates:
        raise SystemExit("No candidate directories resolved.")

    out_root.mkdir(parents=True, exist_ok=True)
    results: list[dict[str, object]] = []

    for candidate_dir in candidates:
        candidate_root = out_root / candidate_dir.name
        sitl_tracking_dir = _collect_repeats(
            px4_root=px4_root,
            candidate_dir=candidate_dir,
            case_name=args.case_name,
            repetitions=args.repetitions,
            repeat_attempts=args.repeat_attempts,
            out_root=candidate_root,
        )
        review_dir = candidate_root / "review"
        bundle = build_bundle(
            hitl_tracking_dir=hitl_tracking_dir,
            sitl_tracking_dir=sitl_tracking_dir,
            prior_tracking_dir=None,
            out_dir=review_dir,
            case_name=args.case_name,
            samples=700,
            max_points=700,
        )
        score = _score_from_bundle(bundle)
        results.append(
            {
                "candidate_dir": str(candidate_dir),
                "candidate_name": candidate_dir.name,
                "review_dir": str(review_dir),
                "tracking_dir": str(sitl_tracking_dir),
                "score": score,
                "bitwise_identical": len({_sha256(path) for path in sitl_tracking_dir.glob('*.csv')}) == 1,
            }
        )

    results.sort(
        key=lambda item: (
            -float(item["score"]["mean_overlap_pct"]),
            -float(item["score"]["contact_pct"]),
            float(item["score"]["mean_center_distance_m"]),
            float(item["score"]["max_center_distance_m"]),
        )
    )
    best = results[0]

    best_review_link = out_root / "best_review"
    if best_review_link.exists() or best_review_link.is_symlink():
        if best_review_link.is_dir() and not best_review_link.is_symlink():
            shutil.rmtree(best_review_link)
        else:
            best_review_link.unlink()
    best_review_link.symlink_to(Path(best["review_dir"]))

    summary = {
        "case_name": args.case_name,
        "hitl_tracking_dir": str(hitl_tracking_dir),
        "repetitions": args.repetitions,
        "results": results,
        "best": best,
        "best_review_dir": str(best_review_link.resolve()),
    }
    (out_root / "selection_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
