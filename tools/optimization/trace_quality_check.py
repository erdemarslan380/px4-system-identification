#!/usr/bin/env python3
"""Post-run trace smoothness and quality checks for tuned controller results."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

from generate_controller_suite_report import collect_cases, load_manifest, pct_improvement


def _trace_arrays(trace: dict | None) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    if not trace:
        return None
    try:
        t = np.asarray(trace.get("t", []), dtype=float)
        ref_map = trace.get("ref", {}) or {}
        act_map = trace.get("act", {}) or {}
        ref = np.column_stack([
            np.asarray(ref_map.get("x", []), dtype=float),
            np.asarray(ref_map.get("y", []), dtype=float),
            np.asarray(ref_map.get("z", []), dtype=float),
        ])
        act = np.column_stack([
            np.asarray(act_map.get("x", []), dtype=float),
            np.asarray(act_map.get("y", []), dtype=float),
            np.asarray(act_map.get("z", []), dtype=float),
        ])
    except Exception:
        return None
    if t.ndim != 1 or ref.ndim != 2 or act.ndim != 2:
        return None
    if len(t) < 6 or ref.shape[0] != len(t) or act.shape[0] != len(t):
        return None
    if ref.shape[1] != 3 or act.shape[1] != 3:
        return None
    if not np.all(np.isfinite(t)) or not np.all(np.isfinite(ref)) or not np.all(np.isfinite(act)):
        return None
    return t, ref, act


def _safe_gradient(values: np.ndarray, t: np.ndarray) -> np.ndarray:
    if values.shape[0] < 3:
        return np.zeros_like(values)
    t_use = np.asarray(t, dtype=float)
    if len(t_use) != values.shape[0]:
        t_use = np.linspace(0.0, float(values.shape[0] - 1), values.shape[0], dtype=float)
    if np.any(~np.isfinite(t_use)) or np.any(np.diff(t_use) <= 1e-9):
        end_t = float(np.nanmax(t_use)) if np.any(np.isfinite(t_use)) else float(values.shape[0] - 1)
        if end_t <= 0.0:
            end_t = float(values.shape[0] - 1)
        t_use = np.linspace(0.0, end_t, values.shape[0], dtype=float)
    return np.gradient(values, t_use, axis=0, edge_order=1)


def derive_quality(trace: dict | None) -> Dict[str, Any]:
    arrays = _trace_arrays(trace)
    if arrays is None:
        return {
            "samples": 0,
            "jerk_rms": math.nan,
            "step_std": math.nan,
            "terminal_spike_ratio": math.nan,
            "terminal_error_mean": math.nan,
            "z_escape_m": math.nan,
            "warnings": ["trace unavailable"],
        }

    t, ref, act = arrays
    vel = _safe_gradient(act, t)
    acc = _safe_gradient(vel, t)
    jerk = _safe_gradient(acc, t)

    jerk_norm = np.linalg.norm(jerk, axis=1)
    jerk_rms = float(np.sqrt(np.mean(jerk_norm * jerk_norm)))

    step = np.linalg.norm(np.diff(act, axis=0), axis=1)
    step_std = float(np.std(step)) if len(step) else math.nan

    err = np.linalg.norm(ref - act, axis=1)
    tail_n = max(5, int(0.1 * len(err)))
    terminal_error_mean = float(np.mean(err[-tail_n:]))
    head_mean = float(np.mean(jerk_norm[:-tail_n])) if len(jerk_norm) > tail_n else jerk_rms
    tail_mean = float(np.mean(jerk_norm[-tail_n:]))
    terminal_spike_ratio = float(tail_mean / max(1e-9, head_mean))

    ref_z = ref[:, 2]
    act_z = act[:, 2]
    z_lo = float(np.min(ref_z) - 0.75)
    z_hi = float(np.max(ref_z) + 0.75)
    z_escape_m = 0.0
    if len(act_z):
        below = max(0.0, z_lo - float(np.min(act_z)))
        above = max(0.0, float(np.max(act_z)) - z_hi)
        z_escape_m = max(below, above)

    warnings: List[str] = []
    if terminal_spike_ratio > 2.5:
        warnings.append("terminal jerk spike")
    if z_escape_m > 0.25:
        warnings.append("z excursion outside reference band")
    if terminal_error_mean > 0.8:
        warnings.append("large terminal position error")

    return {
        "samples": int(len(t)),
        "jerk_rms": jerk_rms,
        "step_std": step_std,
        "terminal_spike_ratio": terminal_spike_ratio,
        "terminal_error_mean": terminal_error_mean,
        "z_escape_m": z_escape_m,
        "warnings": warnings,
    }


def summarize_case(case: dict) -> dict:
    start_q = derive_quality(case.get("start_trace"))
    best_q = derive_quality(case.get("best_trace"))
    return {
        "task_id": case.get("task_id"),
        "controller": case.get("controller"),
        "optimizer": case.get("optimizer"),
        "traj_id": case.get("traj_id"),
        "status": case.get("status"),
        "start": start_q,
        "best": best_q,
        "jerk_gain_pct": pct_improvement(start_q.get("jerk_rms"), best_q.get("jerk_rms")),
        "terminal_error_gain_pct": pct_improvement(start_q.get("terminal_error_mean"), best_q.get("terminal_error_mean")),
        "start_fitness": case.get("start_fitness"),
        "best_fitness": case.get("best_fitness"),
        "start_error_area": case.get("start_error_area"),
        "best_error_area": case.get("best_error_area"),
        "error_area_gain_pct": case.get("error_area_gain_pct"),
        "start_energy_area": case.get("start_energy_area"),
        "best_energy_area": case.get("best_energy_area"),
        "energy_area_gain_pct": case.get("energy_area_gain_pct"),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Check finished task traces for smoothness and terminal anomalies")
    ap.add_argument("--results-root", required=True)
    ap.add_argument("--out-json", default="")
    ap.add_argument("--out-md", default="")
    args = ap.parse_args()

    results_root = Path(args.results_root).resolve()
    manifest = load_manifest(results_root) if results_root.exists() else None
    cases = collect_cases(results_root, manifest=manifest)
    summaries = [summarize_case(case) for case in cases]

    payload = {
        "results_root": str(results_root),
        "cases": summaries,
        "summary": {
            "total": len(summaries),
            "ok": sum(1 for item in summaries if item.get("status") == "ok"),
            "warnings": sum(
                1 for item in summaries
                if item["best"].get("warnings") or item["start"].get("warnings")
            ),
        },
    }

    if args.out_json:
        Path(args.out_json).write_text(json.dumps(payload, indent=2), encoding="utf-8")

    if args.out_md:
        lines = [
            "# Trace Quality Summary",
            "",
            f"- total cases: {payload['summary']['total']}",
            f"- ok cases: {payload['summary']['ok']}",
            f"- cases with warnings: {payload['summary']['warnings']}",
            "",
            "| Task | Controller | Traj | Status | Start jerk RMS | Best jerk RMS | Jerk Δ% | Best terminal err | Best spike ratio | Best z escape | Warnings |",
            "| --- | --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
        ]
        for item in summaries:
            warnings = ", ".join(item["best"].get("warnings") or []) or "-"
            lines.append(
                "| {task} | {controller} | {traj} | {status} | {sjerk:.3f} | {bjerk:.3f} | {jg} | {berr:.3f} | {bspike:.2f} | {bz:.3f} | {warn} |".format(
                    task=item.get("task_id", ""),
                    controller=str(item.get("controller", "")).upper(),
                    traj=int(item.get("traj_id", -1)),
                    status=item.get("status", ""),
                    sjerk=float(item["start"].get("jerk_rms") or math.nan),
                    bjerk=float(item["best"].get("jerk_rms") or math.nan),
                    jg=("n/a" if item.get("jerk_gain_pct") is None else f"{float(item['jerk_gain_pct']):+.2f}%"),
                    berr=float(item["best"].get("terminal_error_mean") or math.nan),
                    bspike=float(item["best"].get("terminal_spike_ratio") or math.nan),
                    bz=float(item["best"].get("z_escape_m") or math.nan),
                    warn=warnings,
                )
            )
        Path(args.out_md).write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(json.dumps(payload["summary"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
