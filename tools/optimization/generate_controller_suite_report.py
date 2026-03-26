#!/usr/bin/env python3
"""Generate a static HTML report for multi-controller trajectory tuning runs."""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

from controller_profiles import get_controller_profile
from optimizer_registry import get_optimizer_spec
from plan_runtime import normalize_manifest_runtime

REPORT_TITLE = "Controller Tuning Suite Report"
CONTROLLER_ORDER = {"pid": 0, "dfbc": 1, "indi": 2, "mpc": 3, "cmpc": 4, "sysid": 5}
STATUS_CLASS = {
    "ok": "ok",
    "failed": "bad",
    "running": "run",
    "pending": "pend",
}
ALGO_ORDER = {"bayes": 0, "random": 1, "anneal": 2}


def pct_improvement(start: float, best: float) -> float | None:
    if not math.isfinite(start) or abs(start) < 1e-12:
        return None
    return 100.0 * (start - best) / start


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def load_manifest(results_root: Path) -> dict | None:
    for name in ("plan_manifest.json", "suite_manifest.json"):
        path = results_root / name
        if path.exists():
            return load_json(path)
    return None


def load_history(results_dir: Path) -> List[dict]:
    path = results_dir / "history.jsonl"
    rows: List[dict] = []
    if not path.exists():
        return rows
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            raw = line.strip()
            if raw:
                rows.append(json.loads(raw))
    return rows


def find_start_record(history: List[dict]) -> dict | None:
    for row in history:
        if row.get("phase") == "seed" and row.get("candidate") == "start_params":
            return row
    return history[0] if history else None


def find_best_record(history: List[dict]) -> dict | None:
    good = [row for row in history if not row.get("failed", False)]
    if not good:
        return None
    return min(good, key=lambda row: float(row.get("fitness", math.inf)))


def resolve_trace_path(eval_meta: dict) -> Path | None:
    for key in ("tracking_log_archive", "tracking_log"):
        raw = str(eval_meta.get(key, "")).strip()
        if raw:
            path = Path(raw)
            if path.exists():
                return path
    return None


def load_trace(eval_meta: dict, controller: str) -> dict | None:
    trace_path = resolve_trace_path(eval_meta)
    if trace_path is None:
        return None
    try:
        trace_name = get_controller_profile(controller).trace_name
    except Exception:
        trace_name = controller

    t: List[float] = []
    ref_x: List[float] = []
    ref_y: List[float] = []
    ref_z: List[float] = []
    act_x: List[float] = []
    act_y: List[float] = []
    act_z: List[float] = []
    t0_us = None

    with trace_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("controller", "").strip().lower() != trace_name:
                continue
            ts_us = int(float(row["timestamp_us"]))
            if t0_us is None:
                t0_us = ts_us
            t.append((ts_us - t0_us) * 1e-6)
            ref_x.append(float(row["ref_x"]))
            ref_y.append(float(row["ref_y"]))
            ref_z.append(float(row["ref_z"]))
            act_x.append(float(row["pos_x"]))
            act_y.append(float(row["pos_y"]))
            act_z.append(float(row["pos_z"]))

    if not t:
        return None

    return {
        "t": t,
        "ref": {"x": ref_x, "y": ref_y, "z": ref_z},
        "act": {"x": act_x, "y": act_y, "z": act_z},
    }


def fmt_metric(value: float | None, digits: int = 6) -> str:
    if value is None or not math.isfinite(value):
        return "n/a"
    return f"{value:.{digits}f}"


def fmt_pct(value: float | None) -> str:
    if value is None or not math.isfinite(value):
        return "n/a"
    sign = "+" if value >= 0 else ""
    return f"{sign}{value:.2f}%"


def trapz(y: List[float], t: List[float]) -> float:
    if len(y) < 2 or len(t) < 2:
        return 0.0
    n = min(len(y), len(t))
    total = 0.0
    for i in range(1, n):
        dt = max(1e-6, float(t[i]) - float(t[i - 1]))
        total += 0.5 * (float(y[i]) + float(y[i - 1])) * dt
    return total


def derive_trace_metrics(trace: dict | None) -> dict:
    if not trace:
        return {"err_area": math.nan, "energy_area": math.nan}

    t = [float(v) for v in trace.get("t", [])]
    rx = [float(v) for v in trace.get("ref", {}).get("x", [])]
    ry = [float(v) for v in trace.get("ref", {}).get("y", [])]
    rz = [float(v) for v in trace.get("ref", {}).get("z", [])]
    ax = [float(v) for v in trace.get("act", {}).get("x", [])]
    ay = [float(v) for v in trace.get("act", {}).get("y", [])]
    az = [float(v) for v in trace.get("act", {}).get("z", [])]
    n = min(len(t), len(rx), len(ry), len(rz), len(ax), len(ay), len(az))
    if n < 2:
        return {"err_area": math.nan, "energy_area": math.nan}

    t = t[:n]
    err: List[float] = []
    power: List[float] = [0.0] * n

    for i in range(n):
        dx = rx[i] - ax[i]
        dy = ry[i] - ay[i]
        dz = rz[i] - az[i]
        err.append(math.sqrt(dx * dx + dy * dy + dz * dz))
        if i > 0:
            dt = max(1e-6, t[i] - t[i - 1])
            vx = (ax[i] - ax[i - 1]) / dt
            vy = (ay[i] - ay[i - 1]) / dt
            vz = (az[i] - az[i - 1]) / dt
            speed = math.sqrt(vx * vx + vy * vy + vz * vz)
            power[i] = speed * speed

    return {
        "err_area": trapz(err, t),
        "energy_area": trapz(power, t),
    }


def monospace_params(params: Dict[str, float]) -> str:
    lines = []
    for key in sorted(params):
        val = params[key]
        lines.append(f"{key}={val:.2f}")
    return "\n".join(lines)


def merged_params(primary: Dict[str, float], fixed: Dict[str, float]) -> Dict[str, float]:
    out = dict(fixed)
    out.update(primary)
    return out


def merge_bounds(traces: Iterable[dict], key_x: str, key_y: str, ref_or_act: str) -> Tuple[float, float, float, float]:
    xs: List[float] = []
    ys: List[float] = []
    for trace in traces:
        if not trace:
            continue
        xs.extend(trace[ref_or_act][key_x])
        ys.extend(trace[ref_or_act][key_y])
        xs.extend(trace["ref"][key_x])
        ys.extend(trace["ref"][key_y])
    if not xs or not ys:
        return (-1.0, 1.0, -1.0, 1.0)
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)
    span_x = max(1e-6, max_x - min_x)
    span_y = max(1e-6, max_y - min_y)
    pad_x = 0.08 * span_x
    pad_y = 0.08 * span_y
    return min_x - pad_x, max_x + pad_x, min_y - pad_y, max_y + pad_y


def _path_from_series(xs: List[float], ys: List[float], bounds: Tuple[float, float, float, float], width: int, height: int) -> str:
    if not xs or not ys:
        return ""
    min_x, max_x, min_y, max_y = bounds
    left, top, plot_w, plot_h = 34, 16, width - 50, height - 34
    def px(x: float) -> float:
        return left + ((x - min_x) / max(1e-9, max_x - min_x)) * plot_w
    def py(y: float) -> float:
        return top + (1.0 - ((y - min_y) / max(1e-9, max_y - min_y))) * plot_h
    return " ".join(
        f"{'M' if i == 0 else 'L'} {px(x):.2f} {py(y):.2f}"
        for i, (x, y) in enumerate(zip(xs, ys))
    )


def svg_xy(trace: dict | None, bounds: Tuple[float, float, float, float], title: str) -> str:
    width, height = 320, 230
    if not trace:
        return f"<div class='plot-empty'>{html.escape(title)} unavailable</div>"
    ref_path = _path_from_series(trace["ref"]["x"], trace["ref"]["y"], bounds, width, height)
    act_path = _path_from_series(trace["act"]["x"], trace["act"]["y"], bounds, width, height)
    min_x, max_x, min_y, max_y = bounds
    return f"""
    <svg viewBox="0 0 {width} {height}" class="plot-svg" role="img" aria-label="{html.escape(title)}">
      <rect x="0" y="0" width="{width}" height="{height}" rx="14" fill="#f5efe0" stroke="#cdbf99" />
      <line x1="34" y1="{height-18}" x2="{width-16}" y2="{height-18}" stroke="#b8aa82" stroke-width="1" />
      <line x1="34" y1="16" x2="34" y2="{height-18}" stroke="#b8aa82" stroke-width="1" />
      <path d="{ref_path}" fill="none" stroke="#1d4ed8" stroke-width="2.2" />
      <path d="{act_path}" fill="none" stroke="#d97706" stroke-width="2.2" />
      <text x="18" y="16" class="axis-label">y</text>
      <text x="{width-20}" y="{height-4}" class="axis-label">x</text>
      <text x="38" y="18" class="plot-label">{html.escape(title)}</text>
      <text x="38" y="{height-4}" class="axis-value">x:[{min_x:.2f},{max_x:.2f}] y:[{min_y:.2f},{max_y:.2f}]</text>
    </svg>
    """


def svg_zt(trace: dict | None, bounds: Tuple[float, float, float, float], title: str) -> str:
    width, height = 320, 230
    if not trace:
        return f"<div class='plot-empty'>{html.escape(title)} unavailable</div>"
    ref_path = _path_from_series(trace["t"], trace["ref"]["z"], bounds, width, height)
    act_path = _path_from_series(trace["t"], trace["act"]["z"], bounds, width, height)
    min_t, max_t, min_z, max_z = bounds
    return f"""
    <svg viewBox="0 0 {width} {height}" class="plot-svg" role="img" aria-label="{html.escape(title)}">
      <rect x="0" y="0" width="{width}" height="{height}" rx="14" fill="#f5efe0" stroke="#cdbf99" />
      <line x1="34" y1="{height-18}" x2="{width-16}" y2="{height-18}" stroke="#b8aa82" stroke-width="1" />
      <line x1="34" y1="16" x2="34" y2="{height-18}" stroke="#b8aa82" stroke-width="1" />
      <path d="{ref_path}" fill="none" stroke="#1d4ed8" stroke-width="2.2" />
      <path d="{act_path}" fill="none" stroke="#d97706" stroke-width="2.2" />
      <text x="10" y="16" class="axis-label">z</text>
      <text x="{width-18}" y="{height-4}" class="axis-label">t</text>
      <text x="38" y="18" class="plot-label">{html.escape(title)}</text>
      <text x="38" y="{height-4}" class="axis-value">t:[{min_t:.1f},{max_t:.1f}] z:[{min_z:.2f},{max_z:.2f}]</text>
    </svg>
    """


def case_sort_key(case: dict) -> Tuple[int, int]:
    return (
        CONTROLLER_ORDER.get(case["controller"], 99),
        int(case["traj_id"]),
        ALGO_ORDER.get(str(case.get("optimizer", "bayes")), 99),
    )


def default_case(controller: str, traj_id: int, results_dir: Path) -> dict:
    optimizer = get_optimizer_spec("bayes")
    return {
        "results_dir": str(results_dir),
        "controller": controller,
        "traj_id": int(traj_id),
        "task_group_id": f"{controller}_traj{int(traj_id)}",
        "optimizer": optimizer.name,
        "optimizer_display_name": optimizer.display_name,
        "optimizer_short_label": optimizer.short_label,
        "optimizer_color": optimizer.color,
        "iterations": 0,
        "global_iters": 0,
        "local_iters": 0,
        "status": "pending",
        "status_detail": "waiting to start",
        "start_eval": None,
        "best_eval": None,
        "start_params": {},
        "best_params": {},
        "start_fitness": math.nan,
        "best_fitness": math.nan,
        "fitness_gain_pct": None,
        "start_track_rmse": math.nan,
        "best_track_rmse": math.nan,
        "track_gain_pct": None,
        "start_energy": math.nan,
        "best_energy": math.nan,
        "energy_gain_pct": None,
        "start_error_area": math.nan,
        "best_error_area": math.nan,
        "error_area_gain_pct": None,
        "start_energy_area": math.nan,
        "best_energy_area": math.nan,
        "energy_area_gain_pct": None,
        "start_trace": None,
        "best_trace": None,
    }


def build_case(results_dir: Path, controller: str | None = None, traj_id: int | None = None) -> dict | None:
    run_config_path = results_dir / "run_config.json"
    pool_status_path = results_dir / "pool_status.json"
    if not run_config_path.exists() and not pool_status_path.exists():
        if controller is None or traj_id is None:
            return None
        return default_case(controller, traj_id, results_dir)

    run_config = load_json(run_config_path) if run_config_path.exists() else {}
    history = load_history(results_dir)
    start_record = find_start_record(history)
    best_record = find_best_record(history)
    best_summary = load_json(results_dir / "best.json") if (results_dir / "best.json").exists() else {}
    fixed_params = dict(run_config.get("fixed_params", {}))

    controller_name = str(run_config.get("controller", controller or "dfbc"))
    optimizer = get_optimizer_spec(run_config.get("optimizer", "bayes"))
    case = default_case(controller_name, int(run_config.get("traj_id", -1 if traj_id is None else traj_id)), results_dir)
    case.update({
        "iterations": int(run_config.get("iterations", 0)),
        "global_iters": int(run_config.get("global_iters", 0)),
        "local_iters": int(run_config.get("local_iters", 0)),
        "task_group_id": str(run_config.get("task_group_id", case["task_group_id"])),
        "optimizer": optimizer.name,
        "optimizer_display_name": str(run_config.get("optimizer_display_name") or optimizer.display_name),
        "optimizer_short_label": str(run_config.get("optimizer_short_label") or optimizer.short_label),
        "optimizer_color": str(run_config.get("optimizer_color") or optimizer.color),
    })

    pool_status = load_json(pool_status_path) if pool_status_path.exists() else {}
    completed = int(pool_status.get("completed_evals", 0))
    total = int(pool_status.get("total_expected_evals", 0))
    workers_busy = int(pool_status.get("workers_busy", 0))
    progress_ratio = float(pool_status.get("progress_ratio", 0.0)) if pool_status else 0.0
    eta_s = pool_status.get("eta_s", None)

    if best_record is not None:
        case["status"] = "ok" if total == 0 or completed >= total else "running"
    elif workers_busy > 0 or (total > 0 and completed < total):
        case["status"] = "running"
    elif run_config_path.exists() or pool_status_path.exists():
        case["status"] = "failed" if total > 0 and completed >= total else "pending"

    if case["status"] == "running":
        eta_txt = f", eta={eta_s:.0f}s" if isinstance(eta_s, (int, float)) and math.isfinite(eta_s) else ""
        case["status_detail"] = f"progress={completed}/{total} ({progress_ratio * 100.0:.1f}%), workers_busy={workers_busy}{eta_txt}"
    elif case["status"] == "ok":
        case["status_detail"] = f"completed {completed}/{total} evals"
    elif case["status"] == "failed":
        case["status_detail"] = f"run prepared but no valid best result"

    start_eval = None
    best_eval = None
    start_meta = None
    best_meta = None
    if start_record is not None:
        start_eval = int(start_record.get("eval_index", -1))
        path = results_dir / f"eval_{start_eval:05d}.json"
        if path.exists():
            start_meta = load_json(path)
    if best_record is not None:
        best_eval = int(best_record.get("eval_index", -1))
        path = results_dir / f"eval_{best_eval:05d}.json"
        if path.exists():
            best_meta = load_json(path)

    case["start_eval"] = start_eval
    case["best_eval"] = best_eval
    case["fixed_params"] = fixed_params
    case["start_params"] = merged_params(start_record.get("params", {}) if start_record else {}, fixed_params)
    case["best_params"] = merged_params(best_summary.get("best_params", best_record.get("params", {}) if best_record else {}), fixed_params)

    start_fit = float(start_meta.get("fitness", math.nan)) if start_meta else math.nan
    best_fit = float(best_meta.get("fitness", math.nan)) if best_meta else math.nan
    start_track = float(start_meta.get("track_rmse", math.nan)) if start_meta else math.nan
    best_track = float(best_meta.get("track_rmse", math.nan)) if best_meta else math.nan
    start_energy = float(start_meta.get("energy_term", math.nan)) if start_meta else math.nan
    best_energy = float(best_meta.get("energy_term", math.nan)) if best_meta else math.nan

    case["start_fitness"] = start_fit
    case["best_fitness"] = best_fit
    case["fitness_gain_pct"] = pct_improvement(start_fit, best_fit)
    case["start_track_rmse"] = start_track
    case["best_track_rmse"] = best_track
    case["track_gain_pct"] = pct_improvement(start_track, best_track)
    case["start_energy"] = start_energy
    case["best_energy"] = best_energy
    case["energy_gain_pct"] = pct_improvement(start_energy, best_energy)

    start_trace = load_trace(start_meta, controller_name) if start_meta else None
    best_trace = load_trace(best_meta, controller_name) if best_meta else None
    start_derived = derive_trace_metrics(start_trace)
    best_derived = derive_trace_metrics(best_trace)
    case["start_trace"] = start_trace
    case["best_trace"] = best_trace
    case["start_error_area"] = float(start_derived["err_area"])
    case["best_error_area"] = float(best_derived["err_area"])
    case["error_area_gain_pct"] = pct_improvement(case["start_error_area"], case["best_error_area"])
    case["start_energy_area"] = float(start_derived["energy_area"])
    case["best_energy_area"] = float(best_derived["energy_area"])
    case["energy_area_gain_pct"] = pct_improvement(case["start_energy_area"], case["best_energy_area"])
    return case


def render_case(case: dict) -> str:
    traces = [case.get("start_trace"), case.get("best_trace")]
    xy_bounds = merge_bounds(traces, "x", "y", "act")

    z_values: List[float] = []
    t_values: List[float] = []
    for trace in traces:
        if not trace:
            continue
        t_values.extend(trace["t"])
        z_values.extend(trace["ref"]["z"])
        z_values.extend(trace["act"]["z"])
    if t_values and z_values:
        t0, t1 = min(t_values), max(t_values)
        z0, z1 = min(z_values), max(z_values)
        dt = max(1e-6, t1 - t0)
        dz = max(1e-6, z1 - z0)
        z_bounds = (t0, t1 + 0.08 * dt, z0 - 0.08 * dz, z1 + 0.08 * dz)
    else:
        z_bounds = (0.0, 1.0, -1.0, 1.0)

    status_class = STATUS_CLASS.get(str(case["status"]), "pend")
    return f"""
    <section class="case-row">
      <div class="case-header">
        <div>
          <h2>{html.escape(case['controller'].upper())} / Traj {case['traj_id']}</h2>
          <p class="subline">optimizer={html.escape(str(case.get('optimizer_display_name', case.get('optimizer', 'bayes'))))} | iter={case['iterations']} global={case['global_iters']} local={case['local_iters']} start_eval={case['start_eval']} best_eval={case['best_eval']} | {html.escape(str(case.get('status_detail', '')))}</p>
          <p class="subline">fixed_params={len(case.get('fixed_params', {}))}</p>
        </div>
        <div class="status-pill {status_class}">{html.escape(case['status'])}</div>
      </div>
      <div class="case-grid">
        <div class="metric-card">
          <div class="metric-block">
            <span class="metric-label">Fitness</span>
            <span class="metric-pair">{fmt_metric(case['start_fitness'])} → {fmt_metric(case['best_fitness'])}</span>
            <span class="metric-gain">{fmt_pct(case['fitness_gain_pct'])}</span>
          </div>
          <div class="metric-block">
            <span class="metric-label">Tracking RMSE</span>
            <span class="metric-pair">{fmt_metric(case['start_track_rmse'])} → {fmt_metric(case['best_track_rmse'])}</span>
            <span class="metric-gain">{fmt_pct(case['track_gain_pct'])}</span>
          </div>
          <div class="metric-block">
            <span class="metric-label">Energy</span>
            <span class="metric-pair">{fmt_metric(case['start_energy'])} → {fmt_metric(case['best_energy'])}</span>
            <span class="metric-gain">{fmt_pct(case['energy_gain_pct'])}</span>
          </div>
        </div>
        <div class="params-card">
          <h3>Start Params</h3>
          <pre>{html.escape(monospace_params(case['start_params']))}</pre>
        </div>
        <div class="params-card">
          <h3>Optimized Params</h3>
          <pre>{html.escape(monospace_params(case['best_params']))}</pre>
        </div>
        <div class="plot-card">
          <h3>Before XY</h3>
          {svg_xy(case.get('start_trace'), xy_bounds, 'Ref vs actual XY')}
        </div>
        <div class="plot-card">
          <h3>After XY</h3>
          {svg_xy(case.get('best_trace'), xy_bounds, 'Ref vs actual XY')}
        </div>
        <div class="plot-card">
          <h3>Before Z(t)</h3>
          {svg_zt(case.get('start_trace'), z_bounds, 'Ref vs actual z(t)')}
        </div>
        <div class="plot-card">
          <h3>After Z(t)</h3>
          {svg_zt(case.get('best_trace'), z_bounds, 'Ref vs actual z(t)')}
        </div>
      </div>
    </section>
    """


def pct_heat_style(value: float | None, scale: float) -> str:
    if value is None or not math.isfinite(value) or scale <= 1e-9:
        return ""
    intensity = min(1.0, abs(float(value)) / scale)
    if value >= 0:
        return f"background: rgba(22, 101, 52, {0.14 + 0.34 * intensity:.3f}); color: #0f3d21;"
    return f"background: rgba(185, 28, 28, {0.14 + 0.34 * intensity:.3f}); color: #671313;"


def render_integral_tables(cases: List[dict]) -> str:
    by_traj: Dict[int, List[dict]] = {}
    for case in cases:
        by_traj.setdefault(int(case["traj_id"]), []).append(case)

    sections = []
    for traj_id in sorted(by_traj):
        traj_cases = by_traj[traj_id]
        by_algo: Dict[str, List[dict]] = {}
        for case in traj_cases:
            by_algo.setdefault(str(case.get("optimizer", "bayes")), []).append(case)

        tables = []
        for optimizer in sorted(by_algo, key=lambda key: ALGO_ORDER.get(key, 99)):
            group = sorted(by_algo[optimizer], key=lambda case: CONTROLLER_ORDER.get(str(case.get("controller", "")), 99))
            pct_values = []
            for case in group:
                for key in ("error_area_gain_pct", "energy_area_gain_pct"):
                    value = case.get(key)
                    if isinstance(value, (int, float)) and math.isfinite(float(value)):
                        pct_values.append(abs(float(value)))
            scale = max(pct_values) if pct_values else 1.0
            algo_name = str(group[0].get("optimizer_display_name", optimizer)) if group else optimizer
            rows = []
            for case in group:
                err_style = pct_heat_style(case.get("error_area_gain_pct"), scale)
                eng_style = pct_heat_style(case.get("energy_area_gain_pct"), scale)
                rows.append(
                    f"""
                    <tr>
                      <td>{html.escape(str(case.get('controller', '')).upper())}</td>
                      <td>{html.escape(str(case.get('status', 'n/a')))}</td>
                      <td>{fmt_metric(case.get('start_error_area'), 3)}</td>
                      <td>{fmt_metric(case.get('best_error_area'), 3)}</td>
                      <td style="{err_style}">{fmt_pct(case.get('error_area_gain_pct'))}</td>
                      <td>{fmt_metric(case.get('start_energy_area'), 3)}</td>
                      <td>{fmt_metric(case.get('best_energy_area'), 3)}</td>
                      <td style="{eng_style}">{fmt_pct(case.get('energy_area_gain_pct'))}</td>
                    </tr>
                    """
                )
            tables.append(
                f"""
                <div class="optimizer-table-block">
                  <h3>{html.escape(algo_name)}</h3>
                  <div class="table-scroll">
                    <table class="summary-table">
                      <thead>
                        <tr>
                          <th>Controller</th>
                          <th>Status</th>
                          <th>Start ∫|e|dt</th>
                          <th>Best ∫|e|dt</th>
                          <th>Error Improve / Regress %</th>
                          <th>Start ∫Pdt</th>
                          <th>Best ∫Pdt</th>
                          <th>Energy Improve / Regress %</th>
                        </tr>
                      </thead>
                      <tbody>
                        {''.join(rows)}
                      </tbody>
                    </table>
                  </div>
                </div>
                """
            )
        sections.append(
            f"""
            <section class="summary-table-wrap">
              <h2>Trajectory {traj_id}</h2>
              <p class="subline">Only the percentage columns are heat-colored. Green means improvement, red means regression.</p>
              {''.join(tables)}
            </section>
            """
        )

    return "".join(sections)


def render_report(cases: List[dict], out_path: Path, manifest: dict | None = None) -> None:
    sections = "\n".join(render_case(case) for case in cases)
    summary_table = render_integral_tables(cases)
    total = len(cases)
    counts = {
        "ok": sum(1 for case in cases if case.get("status") == "ok"),
        "running": sum(1 for case in cases if case.get("status") == "running"),
        "pending": sum(1 for case in cases if case.get("status") == "pending"),
        "failed": sum(1 for case in cases if case.get("status") == "failed"),
    }
    manifest_note = ""
    if manifest:
        ctrls = ",".join(str(x) for x in manifest.get("controllers", []))
        trajs = ",".join(str(x) for x in manifest.get("traj_ids", []))
        manifest_note = f" Plan: controllers=[{html.escape(ctrls)}], traj_ids=[{html.escape(trajs)}]."
    html_text = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{REPORT_TITLE}</title>
  <style>
    :root {{
      --bg: #eee8d8;
      --card: #f9f4e9;
      --ink: #1d2b4f;
      --muted: #6b6f67;
      --line: #cdbf99;
      --good: #0f766e;
      --bad: #b42318;
      --accent: #d97706;
      --blue: #1d4ed8;
    }}
    * {{ box-sizing: border-box; }}
    body {{ margin: 0; background: radial-gradient(circle at top, #f6f0df 0%, var(--bg) 60%); color: var(--ink); font-family: "IBM Plex Sans", "Segoe UI", sans-serif; }}
    main {{ width: min(1540px, calc(100vw - 32px)); margin: 24px auto 56px; }}
    header {{ margin-bottom: 18px; }}
    h1 {{ margin: 0 0 8px; font-size: 40px; letter-spacing: -0.04em; }}
    .lead {{ margin: 0; color: var(--muted); font-size: 16px; }}
    .case-row {{ background: rgba(249,244,233,0.9); border: 1px solid var(--line); border-radius: 22px; padding: 18px; margin-top: 18px; box-shadow: 0 14px 30px rgba(72, 61, 31, 0.08); }}
    .case-header {{ display: flex; align-items: start; justify-content: space-between; gap: 16px; margin-bottom: 14px; }}
    .case-header h2 {{ margin: 0; font-size: 26px; letter-spacing: -0.03em; }}
    .subline {{ margin: 6px 0 0; color: var(--muted); font-size: 14px; }}
    .status-pill {{ padding: 8px 14px; border-radius: 999px; font-weight: 700; text-transform: uppercase; font-size: 12px; letter-spacing: 0.08em; }}
    .status-pill.ok {{ color: var(--good); background: rgba(15,118,110,0.12); }}
    .status-pill.bad {{ color: var(--bad); background: rgba(180,35,24,0.12); }}
    .status-pill.run {{ color: #9a3412; background: rgba(217,119,6,0.12); }}
    .status-pill.pend {{ color: #475467; background: rgba(71,84,103,0.10); }}
    .summary-bar {{ display: flex; flex-wrap: wrap; gap: 10px; margin-top: 14px; }}
    .summary-chip {{ padding: 8px 12px; border-radius: 999px; border: 1px solid var(--line); background: rgba(249,244,233,0.9); font: 600 13px/1.2 "IBM Plex Mono", monospace; }}
    .summary-table-wrap {{ background: rgba(249,244,233,0.9); border: 1px solid var(--line); border-radius: 22px; padding: 18px; margin-top: 22px; box-shadow: 0 14px 30px rgba(72, 61, 31, 0.08); }}
    .summary-table-wrap h2 {{ margin: 0 0 12px; font-size: 26px; letter-spacing: -0.03em; }}
    .optimizer-table-block + .optimizer-table-block {{ margin-top: 16px; padding-top: 16px; border-top: 1px dashed var(--line); }}
    .optimizer-table-block h3 {{ margin: 0 0 10px; font-size: 18px; }}
    .table-scroll {{ overflow-x: auto; }}
    .summary-table {{ width: 100%; border-collapse: collapse; font: 13px/1.4 "IBM Plex Mono", monospace; }}
    .summary-table th, .summary-table td {{ padding: 10px 12px; border-bottom: 1px solid var(--line); text-align: left; white-space: nowrap; }}
    .summary-table th {{ color: var(--muted); text-transform: uppercase; letter-spacing: 0.06em; font-size: 12px; }}
    .case-grid {{ display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)) 1.1fr 1.1fr 1.1fr 1.1fr; gap: 14px; align-items: start; }}
    .metric-card, .params-card, .plot-card {{ background: var(--card); border: 1px solid var(--line); border-radius: 18px; padding: 14px; min-height: 100%; }}
    .metric-block + .metric-block {{ margin-top: 14px; padding-top: 14px; border-top: 1px dashed var(--line); }}
    .metric-label {{ display: block; color: var(--muted); font-size: 13px; text-transform: uppercase; letter-spacing: 0.08em; }}
    .metric-pair {{ display: block; margin-top: 6px; font-size: 20px; font-weight: 700; letter-spacing: -0.03em; }}
    .metric-gain {{ display: block; margin-top: 4px; font-size: 28px; font-weight: 800; color: var(--good); }}
    .params-card h3, .plot-card h3 {{ margin: 0 0 10px; font-size: 16px; }}
    pre {{ margin: 0; white-space: pre-wrap; word-break: break-word; font: 12px/1.5 "IBM Plex Mono", "SFMono-Regular", monospace; color: #21304d; max-height: 330px; overflow: auto; }}
    .plot-svg {{ width: 100%; height: auto; display: block; }}
    .plot-empty {{ height: 230px; border-radius: 14px; display: grid; place-items: center; border: 1px dashed var(--line); color: var(--muted); font-size: 14px; }}
    .plot-label {{ font: 600 12px "IBM Plex Mono", monospace; fill: #24334f; }}
    .axis-label {{ font: 600 12px "IBM Plex Mono", monospace; fill: #6b6f67; }}
    .axis-value {{ font: 500 10px "IBM Plex Mono", monospace; fill: #6b6f67; }}
    @media (max-width: 1440px) {{
      .case-grid {{ grid-template-columns: repeat(2, minmax(0, 1fr)); }}
    }}
    @media (max-width: 800px) {{
      main {{ width: min(100vw - 20px, 900px); margin: 16px auto 40px; }}
      h1 {{ font-size: 30px; }}
      .case-header {{ flex-direction: column; }}
      .case-grid {{ grid-template-columns: 1fr; }}
      .summary-table th, .summary-table td {{ padding: 8px 10px; }}
    }}
  </style>
</head>
<body>
  <main>
    <header>
      <h1>{REPORT_TITLE}</h1>
      <p class="lead">Results are grouped by trajectory first and optimizer second. Each comparison table shows start vs best integral metrics and percentage change. Positive percentages mean improvement vs start, negative percentages mean regression vs start. The detailed sections keep parameter sets and before/after trajectory plots together.{manifest_note}</p>
      <div class="summary-bar">
        <div class="summary-chip">total={total}</div>
        <div class="summary-chip">ok={counts['ok']}</div>
        <div class="summary-chip">running={counts['running']}</div>
        <div class="summary-chip">pending={counts['pending']}</div>
        <div class="summary-chip">failed={counts['failed']}</div>
      </div>
    </header>
    {sections}
    {summary_table}
  </main>
</body>
</html>
"""
    out_path.write_text(html_text, encoding="utf-8")


def collect_cases(results_root: Path, manifest: dict | None = None) -> List[dict]:
    cases: List[dict] = []
    planned = []
    manifest_task_map: Dict[str, dict] = {}
    if manifest:
        manifest, _ = normalize_manifest_runtime(manifest)
        for task in manifest.get("tasks", []):
            controller = str(task.get("controller", ""))
            traj_id = int(task.get("traj_id", -1))
            task_id = str(task.get("task_id") or f"{controller}_traj{traj_id}")
            manifest_task_map[task_id] = task
            planned.append((controller, traj_id, task_id))
        if not planned:
            for controller in manifest.get("controllers", []):
                for traj_id in manifest.get("traj_ids", []):
                    planned.append((str(controller), int(traj_id), f"{controller}_traj{traj_id}"))

    if planned:
        for controller, traj_id, task_id in planned:
            results_dir = results_root / task_id
            case = build_case(results_dir, controller=controller, traj_id=traj_id)
            if case is not None:
                case["task_id"] = task_id
                task_meta = manifest_task_map.get(task_id)
                if task_meta is not None:
                    case["status"] = str(task_meta.get("status", case.get("status", "pending")))
                    case["status_detail"] = str(task_meta.get("status_detail", case.get("status_detail", "")))
                    case["iterations"] = int(task_meta.get("iterations", case.get("iterations", 0)))
                    case["global_iters"] = int(task_meta.get("global_iters", case.get("global_iters", 0)))
                    case["task_group_id"] = str(task_meta.get("task_group_id", case.get("task_group_id", task_id)))
                    case["optimizer"] = str(task_meta.get("optimizer", case.get("optimizer", "bayes")))
                    case["optimizer_display_name"] = str(task_meta.get("optimizer_display_name", case.get("optimizer_display_name", "Bayesian + Central Diff")))
                    case["optimizer_short_label"] = str(task_meta.get("optimizer_short_label", case.get("optimizer_short_label", "Bayes")))
                    case["optimizer_color"] = str(task_meta.get("optimizer_color", case.get("optimizer_color", "#ef476f")))
                    case["started_at"] = task_meta.get("started_at")
                    case["finished_at"] = task_meta.get("finished_at")
                cases.append(case)
    elif results_root.exists():
        for child in sorted(results_root.iterdir()):
            if child.is_dir():
                case = build_case(child)
                if case is not None:
                    case["task_id"] = child.name
                    cases.append(case)

    cases.sort(key=case_sort_key)
    return cases


def main() -> int:
    ap = argparse.ArgumentParser(description="Generate static HTML report for controller suite results")
    ap.add_argument("--results-root", required=True)
    ap.add_argument("--out", default="Tools/optimization/controller_suite_report.html")
    args = ap.parse_args()

    results_root = Path(args.results_root).resolve()
    out_path = Path(args.out).resolve()
    manifest = load_manifest(results_root) if results_root.exists() else None

    cases = collect_cases(results_root, manifest=manifest)
    render_report(cases, out_path, manifest=manifest)
    print(json.dumps({"cases": len(cases), "report": str(out_path)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
