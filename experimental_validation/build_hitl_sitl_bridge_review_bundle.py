from __future__ import annotations

import argparse
import base64
import csv
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.build_hitl_review_bundle import PLOTLY_ASSET_PATH
from experimental_validation.repeatability_tube_figures import (
    QUERY_SAMPLES,
    _build_model_payload,
    _render_case_figure,
)
from experimental_validation.trajectory_comparison_figures import (
    _align_to_reference_start,
    _load_tracking,
    _trajectory_rmse,
    _trim_dataset,
)

DEFAULT_MAX_POINTS = 1800
REF_COLOR = "#222222"
HITL_COLOR = "#b45309"
SITL_COLOR = "#0f766e"
PRIOR_COLOR = "#1d4ed8"


def _plotly_script_tag() -> str:
    if PLOTLY_ASSET_PATH.exists():
        return f"<script>{PLOTLY_ASSET_PATH.read_text(encoding='utf-8')}</script>"
    return '<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>'


def _sanitize_tracking_csv(src: Path, dst: Path) -> Path:
    dst.parent.mkdir(parents=True, exist_ok=True)
    with src.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fieldnames = reader.fieldnames or []
        rows: list[dict[str, str]] = []
        required = ("timestamp_us", "ref_x", "ref_y", "ref_z", "pos_x", "pos_y", "pos_z")
        for row in reader:
            try:
                if any(row.get(key, "") is None or row.get(key, "") == "" for key in required):
                    continue
                for key in required:
                    float(row[key])
            except (TypeError, ValueError):
                continue
            rows.append({name: row.get(name, "") for name in fieldnames})

    with dst.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return dst


def _decimation_indices(length: int, max_points: int) -> list[int]:
    if length <= 0:
        return []
    if length <= max_points:
        return list(range(length))
    step = max(1, math.ceil(length / max_points))
    reduced = list(range(0, length, step))
    if reduced[-1] != length - 1:
        reduced.append(length - 1)
    return reduced


def _circle_intersection_area(r1: float, r2: float, d: float) -> float:
    r1 = max(float(r1), 0.0)
    r2 = max(float(r2), 0.0)
    d = max(float(d), 0.0)
    if r1 <= 1e-12 and r2 <= 1e-12:
        return 0.0
    if d >= r1 + r2:
        return 0.0
    if d <= abs(r1 - r2):
        return math.pi * min(r1, r2) ** 2
    if d <= 1e-12:
        return math.pi * min(r1, r2) ** 2
    term1 = r1 * r1 * math.acos(max(-1.0, min(1.0, (d * d + r1 * r1 - r2 * r2) / (2.0 * d * r1))))
    term2 = r2 * r2 * math.acos(max(-1.0, min(1.0, (d * d + r2 * r2 - r1 * r1) / (2.0 * d * r2))))
    term3 = 0.5 * math.sqrt(
        max(
            0.0,
            (-d + r1 + r2)
            * (d + r1 - r2)
            * (d - r1 + r2)
            * (d + r1 + r2),
        )
    )
    return term1 + term2 - term3


def _circle_iou(r1: float, r2: float, d: float) -> float:
    area1 = math.pi * max(r1, 0.0) ** 2
    area2 = math.pi * max(r2, 0.0) ** 2
    union = area1 + area2
    if union <= 1e-12:
        return 1.0 if d <= 1e-9 else 0.0
    inter = _circle_intersection_area(r1, r2, d)
    denom = union - inter
    if denom <= 1e-12:
        return 1.0
    return inter / denom


def _safe_float_list(values: np.ndarray, *, digits: int = 6) -> list[float]:
    return [round(float(value), digits) for value in values.tolist()]


def _copy_raw_csv(src: Path, dst: Path) -> str:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return str(dst)


def _run_payload(case_name: str, csv_path: Path, sanitized_csv_path: Path, *, out_dir: Path, channel: str, max_points: int) -> dict[str, object]:
    timestamps_s, ref_raw, pos_raw = _load_tracking(sanitized_csv_path)
    ref_trimmed, pos_trimmed = _trim_dataset(case_name, timestamps_s, ref_raw, pos_raw)
    ref_plot, pos_plot = _align_to_reference_start(ref_trimmed, pos_trimmed)
    rmse_raw, errors = _trajectory_rmse(ref_trimmed, pos_trimmed)
    idx = _decimation_indices(len(ref_plot), max_points=max_points)
    ref_plot_decimated = ref_plot[idx].tolist() if idx else []
    pos_plot_decimated = pos_plot[idx].tolist() if idx else []
    progress = [0.0 if len(ref_plot) <= 1 else float(i) / float(len(ref_plot) - 1) for i in idx]
    raw_rel = Path("raw") / channel / csv_path.name
    _copy_raw_csv(csv_path, out_dir / raw_rel)

    return {
        "name": csv_path.stem,
        "channel": channel,
        "csv_name": csv_path.name,
        "csv": str(raw_rel),
        "csv_b64": base64.b64encode(csv_path.read_bytes()).decode("ascii"),
        "samples": int(len(ref_trimmed)),
        "rmse_m": float(rmse_raw),
        "max_error_m": float(np.max(errors)) if len(errors) else 0.0,
        "reference_plot": ref_plot_decimated,
        "position_plot": pos_plot_decimated,
        "progress": progress,
    }


def _reference_vs_model_payload(model_payload: dict[str, object], *, model_key: str, label: str) -> dict[str, object]:
    ref_xy = np.column_stack(
        [
            np.asarray(model_payload["reference_centerline"]["x"], dtype=float),
            np.asarray(model_payload["reference_centerline"]["y"], dtype=float),
        ]
    )
    pos_xy = np.column_stack(
        [
            np.asarray(model_payload["centerline"]["x"], dtype=float),
            np.asarray(model_payload["centerline"]["y"], dtype=float),
        ]
    )
    progress = np.asarray(model_payload["progress"], dtype=float)
    radius_xy = np.asarray(model_payload["std"]["xy_radius"], dtype=float)
    center_distance = np.linalg.norm(pos_xy - ref_xy, axis=1)
    inside = center_distance <= radius_xy
    safe_radius = np.maximum(radius_xy, 1e-9)
    normalized_overlap = np.where(
        radius_xy > 1e-9,
        np.clip(1.0 - (center_distance / safe_radius), 0.0, 1.0),
        np.where(center_distance <= 1e-9, 1.0, 0.0),
    )
    excess_gap = np.maximum(center_distance - radius_xy, 0.0)
    focus_idx = int(np.argmax(excess_gap)) if len(excess_gap) else 0
    return {
        "pair_key": f"reference__{model_key}",
        "left_key": "reference",
        "right_key": model_key,
        "left_label": "Reference",
        "right_label": label,
        "mode": "reference_to_tube",
        "mean_overlap_pct": round(float(np.mean(normalized_overlap) * 100.0), 3),
        "contact_pct": round(float(np.mean(inside) * 100.0), 3),
        "mean_center_distance_m": round(float(np.mean(center_distance)), 6),
        "max_center_distance_m": round(float(np.max(center_distance)), 6),
        "sample_overlap_pct": [round(float(value) * 100.0, 4) for value in normalized_overlap.tolist()],
        "sample_contact": [bool(value) for value in inside.tolist()],
        "sample_center_distance_m": [round(float(value), 6) for value in center_distance.tolist()],
        "sample_excess_gap_m": [round(float(value), 6) for value in excess_gap.tolist()],
        "focus_progress_pct": round(float(progress[focus_idx] * 100.0), 3) if len(progress) else 0.0,
        "comment": "Reference line versus repeatability tube containment score.",
    }


def _pair_overlap_payload(left_model: dict[str, object], right_model: dict[str, object]) -> dict[str, object]:
    left_xy = np.column_stack(
        [
            np.asarray(left_model["centerline"]["x"], dtype=float),
            np.asarray(left_model["centerline"]["y"], dtype=float),
        ]
    )
    right_xy = np.column_stack(
        [
            np.asarray(right_model["centerline"]["x"], dtype=float),
            np.asarray(right_model["centerline"]["y"], dtype=float),
        ]
    )
    left_radius = np.asarray(left_model["std"]["xy_radius"], dtype=float)
    right_radius = np.asarray(right_model["std"]["xy_radius"], dtype=float)
    progress = np.asarray(left_model["progress"], dtype=float)
    center_distance = np.linalg.norm(left_xy - right_xy, axis=1)
    sample_iou = np.asarray([
        _circle_iou(lr, rr, dd) for lr, rr, dd in zip(left_radius, right_radius, center_distance)
    ])
    sample_contact = center_distance <= (left_radius + right_radius)
    overlap_gap = np.maximum(center_distance - (left_radius + right_radius), 0.0)
    focus_idx = int(np.argmax(overlap_gap)) if len(overlap_gap) else 0
    return {
        "pair_key": f"{left_model['key']}__{right_model['key']}",
        "left_key": left_model["key"],
        "right_key": right_model["key"],
        "left_label": left_model["label"],
        "right_label": right_model["label"],
        "mode": "tube_vs_tube",
        "mean_overlap_pct": round(float(np.mean(sample_iou) * 100.0), 3),
        "contact_pct": round(float(np.mean(sample_contact) * 100.0), 3),
        "mean_center_distance_m": round(float(np.mean(center_distance)), 6),
        "max_center_distance_m": round(float(np.max(center_distance)), 6),
        "sample_overlap_pct": [round(float(value) * 100.0, 4) for value in sample_iou.tolist()],
        "sample_contact": [bool(value) for value in sample_contact.tolist()],
        "sample_center_distance_m": [round(float(value), 6) for value in center_distance.tolist()],
        "sample_excess_gap_m": [round(float(value), 6) for value in overlap_gap.tolist()],
        "focus_progress_pct": round(float(progress[focus_idx] * 100.0), 3) if len(progress) else 0.0,
        "comment": "Tube-vs-tube IoU across progress. Higher is better and directly reflects digital-twin overlap.",
    }


def _build_html(bundle: dict[str, object], plotly_script: str) -> str:
    data_json = json.dumps(bundle, ensure_ascii=True)
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>HITL vs SITL Hairpin Bridge Review</title>
  {plotly_script}
  <style>
    :root {{
      --bg: #f3ede3;
      --panel: #fffaf4;
      --ink: #17202a;
      --muted: #5d6d7e;
      --border: #d8cfc0;
      --chip: #f6efe5;
      --ref: {REF_COLOR};
      --hitl: {HITL_COLOR};
      --sitl: {SITL_COLOR};
      --prior: {PRIOR_COLOR};
    }}
    body {{
      margin: 0;
      font-family: "IBM Plex Sans", "Segoe UI", sans-serif;
      color: var(--ink);
      background: linear-gradient(180deg, #eee5d7 0%, var(--bg) 100%);
    }}
    .layout {{ display: block; min-height: 100vh; }}
    .sidebar {{
      padding: 24px;
      border-bottom: 1px solid var(--border);
      background: rgba(255, 250, 244, 0.94);
    }}
    .content {{
      padding: 24px;
      display: grid;
      gap: 20px;
    }}
    .card {{
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 18px;
      padding: 18px;
      box-shadow: 0 10px 28px rgba(23, 32, 42, 0.06);
    }}
    h1, h2, h3, h4 {{ margin: 0 0 12px 0; }}
    p, li {{ line-height: 1.5; color: var(--muted); }}
    .meta-grid {{
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 12px;
    }}
    .meta {{
      border: 1px solid var(--border);
      border-radius: 14px;
      padding: 12px;
      background: white;
    }}
    .meta span {{
      display: block;
      font-size: 12px;
      color: var(--muted);
      margin-bottom: 6px;
      text-transform: uppercase;
      letter-spacing: 0.04em;
    }}
    .meta strong {{ font-size: 18px; font-weight: 600; }}
    .controls-grid {{
      display: grid;
      grid-template-columns: 1.2fr 1fr;
      gap: 18px;
    }}
    .control-block {{
      border: 1px solid var(--border);
      border-radius: 14px;
      padding: 14px;
      background: white;
    }}
    .chip-list {{ display: flex; flex-wrap: wrap; gap: 10px; }}
    .chip {{
      display: inline-flex;
      align-items: center;
      gap: 8px;
      padding: 8px 10px;
      border-radius: 999px;
      background: var(--chip);
      border: 1px solid var(--border);
      font-size: 14px;
    }}
    .slider-row {{
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 12px;
      align-items: center;
    }}
    .slider-row input[type="range"] {{ width: 100%; }}
    .selection-grid {{
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
    }}
    .plot-grid {{
      display: grid;
      grid-template-columns: 1.1fr 1fr;
      gap: 18px;
    }}
    .plot {{ min-height: 560px; }}
    table {{ width: 100%; border-collapse: collapse; }}
    th, td {{
      padding: 10px 12px;
      border-bottom: 1px solid var(--border);
      text-align: left;
      vertical-align: top;
    }}
    .pair-grid {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
    }}
    select {{
      width: 100%;
      border: 1px solid var(--border);
      border-radius: 10px;
      padding: 8px 10px;
      background: #fff;
      font: inherit;
    }}
    .footer-links {{ display: flex; flex-wrap: wrap; gap: 10px; }}
    .footer-links a {{ color: #0f766e; text-decoration: none; font-weight: 600; }}
    @media (max-width: 1100px) {{
      .controls-grid, .plot-grid {{ grid-template-columns: 1fr; }}
      .meta-grid {{ grid-template-columns: repeat(2, minmax(0, 1fr)); }}
      .selection-grid {{ grid-template-columns: 1fr; }}
    }}
    @media (max-width: 720px) {{
      .sidebar, .content {{ padding: 14px; }}
      .card {{ padding: 14px; }}
      .meta-grid {{ grid-template-columns: 1fr; }}
      .slider-row {{ grid-template-columns: 1fr; }}
      .pair-grid {{ grid-template-columns: 1fr; }}
    }}
  </style>
</head>
<body>
  <div class="layout">
    <aside class="sidebar">
      <h1>HITL vs SITL Hairpin Bridge</h1>
      <p id="sidebarIntro">Reference trajectory, HITL repeatability tube, identified-SDF SITL tube, and the exact jMAVSim-prior SITL tube are shown together. The main question here is shape similarity and tube overlap, not just RMSE.</p>
      <div class="card">
        <h3>Bundle</h3>
        <p id="bundleSummary"></p>
      </div>
    </aside>
    <main class="content">
      <section class="card">
        <h2 id="title"></h2>
        <p id="subtitle"></p>
        <div class="meta-grid">
          <div class="meta"><span>Trajectory</span><strong id="metaCase"></strong></div>
          <div class="meta"><span>HITL runs</span><strong id="metaHitlRuns"></strong></div>
          <div class="meta"><span>Ident SITL runs</span><strong id="metaSitlRuns"></strong></div>
          <div class="meta" id="metaPriorCard"><span>Exact prior runs</span><strong id="metaPriorRuns"></strong></div>
          <div class="meta"><span>Output root</span><strong id="metaOutDir"></strong></div>
        </div>
      </section>

      <section class="card">
        <h3>Inspection Controls</h3>
        <div class="controls-grid">
          <div class="control-block">
            <h4>Visible layers</h4>
            <div id="layerChecks" class="chip-list"></div>
            <p>Reference, all mean centerlines, all uncertainty tubes, and optional individual repeats are controlled here.</p>
          </div>
          <div class="control-block">
            <h4>Track target</h4>
            <div class="chip-list" id="trackTargetChecks"></div>
            <p>The slider follows the selected target. Hovering a trace also moves the scrubber to that progress point.</p>
          </div>
        </div>
        <div class="slider-row" style="margin-top:18px">
          <input id="progressSlider" type="range" min="0" max="1000" step="1" value="0" />
          <strong id="progressText">0%</strong>
        </div>
      </section>

      <section class="card">
        <h3>Selected Point</h3>
        <div class="selection-grid">
          <div class="meta"><span>Tracked layer</span><strong id="selLayer"></strong></div>
          <div class="meta"><span>Progress</span><strong id="selProgress"></strong></div>
          <div class="meta"><span>Reference error</span><strong id="selError"></strong></div>
          <div class="meta"><span>Tracked point</span><strong id="selPoint"></strong></div>
          <div class="meta"><span>Reference point</span><strong id="selRef"></strong></div>
          <div class="meta"><span>Tube radius</span><strong id="selTubeRadius"></strong></div>
        </div>
      </section>

      <section class="card">
        <h3>3D tube view</h3>
        <p>The plots are clipped to the current slider position so the overlap between the two repeatability tubes is easier to inspect. Both individual run families are also available as faint overlays.</p>
        <div class="plot-grid">
          <div class="card">
            <h3>3D centerline + uncertainty</h3>
            <div id="tubePlot" class="plot"></div>
          </div>
          <div class="card">
            <h3>Top-down tube overlap</h3>
            <div id="tubeZoomPlot" class="plot"></div>
          </div>
        </div>
        <div class="meta-grid" style="margin-top:14px">
          <div class="meta"><span>Selected pair</span><strong id="pairName">-</strong></div>
          <div class="meta"><span>Mean overlap</span><strong id="pairOverlap">-</strong></div>
          <div class="meta"><span>Current overlap</span><strong id="pairCurrentOverlap">-</strong></div>
          <div class="meta"><span>Contact</span><strong id="pairContact">-</strong></div>
          <div class="meta"><span>Mean center dist.</span><strong id="pairDistance">-</strong></div>
          <div class="meta"><span>HITL included</span><strong id="pairHitlRuns">-</strong></div>
          <div class="meta"><span>Ident SITL included</span><strong id="pairSitlRuns">-</strong></div>
          <div class="meta" id="pairPriorCard"><span>Exact prior included</span><strong id="pairPriorRuns">-</strong></div>
        </div>
        <p>Assets: <a id="tubePngLink" href="#" target="_blank" rel="noopener">PNG</a> / <a id="tubeSvgLink" href="#" target="_blank" rel="noopener">SVG</a></p>
        <div class="slider-row" style="margin-top:14px">
          <input id="progressSliderBottom" type="range" min="0" max="1000" step="1" value="0" />
          <strong id="progressTextBottom">0%</strong>
        </div>
      </section>

      <section class="card">
        <h3>Overlap comparison module</h3>
        <div class="pair-grid">
          <label>
            <strong>Katman A</strong>
            <select id="leftPair"></select>
          </label>
          <label>
            <strong>Katman B</strong>
            <select id="rightPair"></select>
          </label>
        </div>
        <table style="margin-top:12px">
          <thead>
            <tr>
              <th>Pair</th>
              <th>Mean overlap [%]</th>
              <th>Current overlap [%]</th>
              <th>Contact [%]</th>
              <th>Mean center dist. [m]</th>
              <th>Comment</th>
            </tr>
          </thead>
          <tbody id="pairTableBody"></tbody>
        </table>
      </section>

      <section class="card">
        <h3>Run tables</h3>
        <div class="plot-grid">
          <div class="card">
            <h4>HITL runs</h4>
            <table>
              <thead>
                <tr><th>Run</th><th>Samples</th><th>RMSE [m]</th><th>Max error [m]</th><th>CSV</th></tr>
              </thead>
              <tbody id="hitlRunsBody"></tbody>
            </table>
          </div>
          <div class="card">
            <h4>Identified SITL runs</h4>
            <table>
              <thead>
                <tr><th>Run</th><th>Samples</th><th>RMSE [m]</th><th>Max error [m]</th><th>CSV</th></tr>
              </thead>
              <tbody id="sitlRunsBody"></tbody>
            </table>
          </div>
          <div class="card" id="priorRunsCard">
            <h4>Exact prior SITL runs</h4>
            <table>
              <thead>
                <tr><th>Run</th><th>Samples</th><th>RMSE [m]</th><th>Max error [m]</th><th>CSV</th></tr>
              </thead>
              <tbody id="priorRunsBody"></tbody>
            </table>
          </div>
        </div>
      </section>

      <section class="card">
        <div class="footer-links">
          <a id="hitlCsvLink" href="#" target="_blank" rel="noopener">Open first HITL CSV</a>
          <a id="sitlCsvLink" href="#" target="_blank" rel="noopener">Open first SITL CSV</a>
          <a id="priorCsvLink" href="#" target="_blank" rel="noopener">Open first exact-prior CSV</a>
        </div>
      </section>
    </main>
  </div>

  <script>
    const bundle = {data_json};
    const hasPrior = Boolean(bundle.has_prior && bundle.models.sitl_exact_prior);

    const colors = {{
      reference: '{REF_COLOR}',
      hitl_repeatability: '{HITL_COLOR}',
      sitl_identified: '{SITL_COLOR}',
      ...(hasPrior ? {{ sitl_exact_prior: '{PRIOR_COLOR}' }} : {{}}),
    }};

    const channelMeta = {{
      reference: {{ label: 'Reference', key: 'reference' }},
      hitl_repeatability: {{ label: bundle.models.hitl_repeatability.label, key: 'hitl_repeatability' }},
      sitl_identified: {{ label: bundle.models.sitl_identified.label, key: 'sitl_identified' }},
      ...(hasPrior ? {{ sitl_exact_prior: {{ label: bundle.models.sitl_exact_prior.label, key: 'sitl_exact_prior' }} }} : {{}}),
    }};

    const state = {{
      progress: 0,
      visible: {{
        reference: true,
        hitl_tube: true,
        hitl_mean: true,
        hitl_repeats: true,
        sitl_tube: true,
        sitl_mean: true,
        sitl_repeats: true,
        ...(hasPrior ? {{
          prior_tube: true,
          prior_mean: true,
          prior_repeats: true,
        }} : {{}}),
      }},
      trackKey: 'hitl_repeatability',
      pairKey: 'hitl_repeatability__sitl_identified',
    }};

    const el = {{
      sidebarIntro: document.getElementById('sidebarIntro'),
      bundleSummary: document.getElementById('bundleSummary'),
      title: document.getElementById('title'),
      subtitle: document.getElementById('subtitle'),
      metaCase: document.getElementById('metaCase'),
      metaHitlRuns: document.getElementById('metaHitlRuns'),
      metaSitlRuns: document.getElementById('metaSitlRuns'),
      metaPriorRuns: document.getElementById('metaPriorRuns'),
      metaPriorCard: document.getElementById('metaPriorCard'),
      metaOutDir: document.getElementById('metaOutDir'),
      layerChecks: document.getElementById('layerChecks'),
      trackTargetChecks: document.getElementById('trackTargetChecks'),
      progressSlider: document.getElementById('progressSlider'),
      progressSliderBottom: document.getElementById('progressSliderBottom'),
      progressText: document.getElementById('progressText'),
      progressTextBottom: document.getElementById('progressTextBottom'),
      selLayer: document.getElementById('selLayer'),
      selProgress: document.getElementById('selProgress'),
      selError: document.getElementById('selError'),
      selPoint: document.getElementById('selPoint'),
      selRef: document.getElementById('selRef'),
      selTubeRadius: document.getElementById('selTubeRadius'),
      pairName: document.getElementById('pairName'),
      pairOverlap: document.getElementById('pairOverlap'),
      pairCurrentOverlap: document.getElementById('pairCurrentOverlap'),
      pairContact: document.getElementById('pairContact'),
      pairDistance: document.getElementById('pairDistance'),
      pairHitlRuns: document.getElementById('pairHitlRuns'),
      pairSitlRuns: document.getElementById('pairSitlRuns'),
      pairPriorRuns: document.getElementById('pairPriorRuns'),
      pairPriorCard: document.getElementById('pairPriorCard'),
      tubePngLink: document.getElementById('tubePngLink'),
      tubeSvgLink: document.getElementById('tubeSvgLink'),
      leftPair: document.getElementById('leftPair'),
      rightPair: document.getElementById('rightPair'),
      pairTableBody: document.getElementById('pairTableBody'),
      hitlRunsBody: document.getElementById('hitlRunsBody'),
      sitlRunsBody: document.getElementById('sitlRunsBody'),
      priorRunsBody: document.getElementById('priorRunsBody'),
      priorRunsCard: document.getElementById('priorRunsCard'),
      hitlCsvLink: document.getElementById('hitlCsvLink'),
      sitlCsvLink: document.getElementById('sitlCsvLink'),
      priorCsvLink: document.getElementById('priorCsvLink'),
    }};

    function fmt(value, digits = 3, suffix = '') {{
      if (value === undefined || value === null || Number.isNaN(value)) return 'n/a';
      return `${{Number(value).toFixed(digits)}}${{suffix}}`;
    }}

    function csvHref(csvB64) {{
      return csvB64 ? `data:text/csv;base64,${{csvB64}}` : '#';
    }}

    function rgba(hex, alpha) {{
      const normalized = hex.replace('#', '');
      const value = parseInt(normalized.length === 3 ? normalized.split('').map((item) => item + item).join('') : normalized, 16);
      const r = (value >> 16) & 255;
      const g = (value >> 8) & 255;
      const b = value & 255;
      return `rgba(${{r}}, ${{g}}, ${{b}}, ${{alpha}})`;
    }}

    function valuesUntil(values, progress) {{
      if (!values?.length) return [];
      const idx = Math.max(0, Math.min(values.length - 1, Math.round(progress * (values.length - 1))));
      return values.slice(0, idx + 1);
    }}

    function pointsUntil(points, progress) {{
      if (!points?.length) return [];
      const idx = Math.max(0, Math.min(points.length - 1, Math.round(progress * (points.length - 1))));
      return points.slice(0, idx + 1);
    }}

    function nearestIndex(length, progress) {{
      if (!length) return 0;
      return Math.max(0, Math.min(length - 1, Math.round(progress * (length - 1))));
    }}

    function currentPair() {{
      return bundle.overlap_pairs.find((entry) => entry.pair_key === state.pairKey) || bundle.overlap_pairs[0];
    }}

    function setPairFromSelectors() {{
      const left = el.leftPair.value;
      const right = el.rightPair.value;
      if (!left || !right || left === right) return;
      const direct = `${{left}}__${{right}}`;
      const reverse = `${{right}}__${{left}}`;
      const resolved = bundle.overlap_pairs.find((entry) => entry.pair_key === direct || entry.pair_key === reverse);
      if (resolved) {{
        state.pairKey = resolved.pair_key;
      }}
      render();
    }}

    function buildControls() {{
      el.layerChecks.innerHTML = '';
      const layerEntries = [
        ['reference', 'Reference'],
        ['hitl_tube', 'HITL tube'],
        ['hitl_mean', 'HITL mean'],
        ['hitl_repeats', 'HITL repeats'],
        ['sitl_tube', 'Ident SITL tube'],
        ['sitl_mean', 'Ident SITL mean'],
        ['sitl_repeats', 'Ident SITL repeats'],
      ];
      if (hasPrior) {{
        layerEntries.push(
          ['prior_tube', 'Exact prior tube'],
          ['prior_mean', 'Exact prior mean'],
          ['prior_repeats', 'Exact prior repeats'],
        );
      }}
      layerEntries.forEach(([key, label]) => {{
        const wrapper = document.createElement('label');
        wrapper.className = 'chip';
        wrapper.innerHTML = `<input type="checkbox" ${{state.visible[key] ? 'checked' : ''}}> <span>${{label}}</span>`;
        wrapper.querySelector('input').onchange = (event) => {{
          state.visible[key] = event.target.checked;
          render();
        }};
        el.layerChecks.appendChild(wrapper);
      }});

      el.trackTargetChecks.innerHTML = '';
      const trackEntries = [
        ['reference', 'Reference'],
        ['hitl_repeatability', 'HITL mean'],
        ['sitl_identified', 'SITL mean'],
      ];
      if (hasPrior) {{
        trackEntries.push(['sitl_exact_prior', 'Exact prior mean']);
      }}
      trackEntries.forEach(([key, label]) => {{
        const wrapper = document.createElement('label');
        wrapper.className = 'chip';
        wrapper.innerHTML = `<input type="radio" name="trackTarget" ${{state.trackKey === key ? 'checked' : ''}}> <span>${{label}}</span>`;
        wrapper.querySelector('input').onchange = () => {{
          state.trackKey = key;
          render();
        }};
        el.trackTargetChecks.appendChild(wrapper);
      }});

      const options = [
        ['reference', 'Reference'],
        ['hitl_repeatability', 'HITL repeatability'],
        ['sitl_identified', 'SITL identified twin'],
      ];
      if (hasPrior) {{
        options.push(['sitl_exact_prior', 'SITL exact jMAVSim prior']);
      }}
      el.leftPair.innerHTML = '';
      el.rightPair.innerHTML = '';
      options.forEach(([value, label]) => {{
        const optA = document.createElement('option');
        optA.value = value;
        optA.textContent = label;
        el.leftPair.appendChild(optA);
        const optB = document.createElement('option');
        optB.value = value;
        optB.textContent = label;
        el.rightPair.appendChild(optB);
      }});
      el.leftPair.value = 'hitl_repeatability';
      el.rightPair.value = 'sitl_identified';
      el.leftPair.onchange = setPairFromSelectors;
      el.rightPair.onchange = setPairFromSelectors;
    }}

    function renderRunTable(target, runs) {{
      target.innerHTML = '';
      runs.forEach((run) => {{
        const tr = document.createElement('tr');
        tr.innerHTML = `
          <td><strong>${{run.name}}</strong></td>
          <td>${{run.samples}}</td>
          <td>${{fmt(run.rmse_m, 4)}}</td>
          <td>${{fmt(run.max_error_m, 4)}}</td>
          <td><a href="${{csvHref(run.csv_b64)}}" download="${{run.csv_name}}" target="_blank" rel="noopener">open CSV</a></td>
        `;
        target.appendChild(tr);
      }});
    }}

    function renderSelection() {{
      const ref = bundle.reference;
      const track = state.trackKey === 'reference' ? bundle.reference : bundle.models[state.trackKey];
      const idx = nearestIndex(ref.x.length, state.progress);
      const refPoint = {{ x: ref.x[idx], y: ref.y[idx], z: ref.z[idx] }};
      let posPoint = refPoint;
      let radius = 0.0;
      if (state.trackKey !== 'reference') {{
        posPoint = {{
          x: track.centerline.x[idx],
          y: track.centerline.y[idx],
          z: track.centerline.z[idx],
        }};
        radius = track.std.xy_radius[idx];
      }}
      const err = Math.sqrt(
        Math.pow(posPoint.x - refPoint.x, 2) +
        Math.pow(posPoint.y - refPoint.y, 2) +
        Math.pow(posPoint.z - refPoint.z, 2)
      );
      el.selLayer.textContent = channelMeta[state.trackKey].label;
      el.selProgress.textContent = `${{fmt(state.progress * 100, 1)}} %`;
      el.selError.textContent = `${{fmt(err, 3)}} m`;
      el.selPoint.textContent = `(${{fmt(posPoint.x, 2)}}, ${{fmt(posPoint.y, 2)}}, ${{fmt(posPoint.z, 2)}})`;
      el.selRef.textContent = `(${{fmt(refPoint.x, 2)}}, ${{fmt(refPoint.y, 2)}}, ${{fmt(refPoint.z, 2)}})`;
      el.selTubeRadius.textContent = `${{fmt(radius, 3)}} m`;
    }}

    function renderPairModule() {{
      const pair = currentPair();
      const idx = nearestIndex(pair.sample_overlap_pct.length, state.progress);
      el.pairName.textContent = `${{pair.left_label}} vs ${{pair.right_label}}`;
      el.pairOverlap.textContent = `${{fmt(pair.mean_overlap_pct, 1)}} %`;
      el.pairCurrentOverlap.textContent = `${{fmt(pair.sample_overlap_pct[idx], 1)}} %`;
      el.pairContact.textContent = `${{fmt(pair.contact_pct, 1)}} %`;
      el.pairDistance.textContent = `${{fmt(pair.mean_center_distance_m, 3)}} m`;
      el.pairHitlRuns.textContent = `${{bundle.models.hitl_repeatability.included_source_count}} / ${{bundle.models.hitl_repeatability.source_count}}`;
      el.pairSitlRuns.textContent = `${{bundle.models.sitl_identified.included_source_count}} / ${{bundle.models.sitl_identified.source_count}}`;
      el.pairPriorRuns.textContent = hasPrior
        ? `${{bundle.models.sitl_exact_prior.included_source_count}} / ${{bundle.models.sitl_exact_prior.source_count}}`
        : 'n/a';
      el.pairTableBody.innerHTML = `
        <tr style="background:#eff6ff">
          <td><strong>${{pair.left_label}}</strong><br><span>${{pair.right_label}}</span></td>
          <td>${{fmt(pair.mean_overlap_pct, 1)}}</td>
          <td>${{fmt(pair.sample_overlap_pct[idx], 1)}}</td>
          <td>${{fmt(pair.contact_pct, 1)}}</td>
          <td>${{fmt(pair.mean_center_distance_m, 3)}}</td>
          <td>${{pair.comment}}</td>
        </tr>
      `;
    }}

    function addRuns(traces3d, traces2d, runs, progress, color, visibleKey, namePrefix) {{
      if (!state.visible[visibleKey]) return;
      runs.forEach((run) => {{
        const points3d = pointsUntil(run.position_plot, progress);
        const xs = points3d.map((p) => p[0]);
        const ys = points3d.map((p) => p[1]);
        const zs = points3d.map((p) => p[2]);
        traces3d.push({{
          type: 'scatter3d',
          mode: xs.length <= 1 ? 'markers' : 'lines',
          name: `${{namePrefix}} ${{run.name}}`,
          x: xs, y: ys, z: zs,
          line: {{ color: rgba(color, 0.25), width: 2 }},
          marker: {{ color: rgba(color, 0.25), size: 3 }},
          opacity: 0.25,
          showlegend: false,
        }});
        traces2d.push({{
          type: 'scatter',
          mode: xs.length <= 1 ? 'markers' : 'lines',
          name: `${{namePrefix}} ${{run.name}}`,
          x: xs, y: ys,
          line: {{ color: rgba(color, 0.20), width: 1.2 }},
          opacity: 0.24,
          showlegend: false,
        }});
      }});
    }}

    function addTubeAndMean(traces3d, traces2d, model, progress, color, showTube, showMean, label) {{
      const idx = nearestIndex(model.progress.length, progress);
      const meanX = valuesUntil(model.centerline.x, progress);
      const meanY = valuesUntil(model.centerline.y, progress);
      const meanZ = valuesUntil(model.centerline.z, progress);
      const upperX = valuesUntil(model.tube_xy.upper_x, progress);
      const upperY = valuesUntil(model.tube_xy.upper_y, progress);
      const lowerX = valuesUntil(model.tube_xy.lower_x, progress);
      const lowerY = valuesUntil(model.tube_xy.lower_y, progress);
      const polyX = upperX.length >= 2 ? [...upperX, ...lowerX.slice().reverse(), upperX[0]] : [];
      const polyY = upperY.length >= 2 ? [...upperY, ...lowerY.slice().reverse(), upperY[0]] : [];
      const radii = valuesUntil(model.std.xy_radius, progress);
      const maxRadius = Math.max(...model.std.xy_radius, 1e-6);
      if (showTube && polyX.length >= 2) {{
        traces2d.push({{
          type: 'scatter',
          mode: 'lines',
          name: `${{label}} 1σ tube`,
          x: polyX, y: polyY,
          fill: 'toself',
          fillcolor: rgba(color, 0.16),
          line: {{ color: rgba(color, 0.30), width: 1 }},
          hoverinfo: 'skip',
        }});
      }}
      if (showMean) {{
        traces3d.push({{
          type: 'scatter3d',
          mode: meanX.length <= 1 ? 'markers' : 'lines',
          name: label,
          x: meanX, y: meanY, z: meanZ,
          line: {{ color: color, width: 6 }},
          marker: {{ color: color, size: 4 }},
        }});
        traces3d.push({{
          type: 'scatter3d',
          mode: 'markers',
          name: `${{label}} sigma`,
          x: meanX, y: meanY, z: meanZ,
          marker: {{
            color: color,
            size: radii.map((value) => 4 + (12 * Math.max(0, value)) / maxRadius),
            opacity: 0.16,
            sizemode: 'diameter',
          }},
          showlegend: false,
          hoverinfo: 'skip',
        }});
        traces2d.push({{
          type: 'scatter',
          mode: meanX.length <= 1 ? 'markers' : 'lines',
          name: label,
          x: meanX, y: meanY,
          line: {{ color: color, width: 3.2 }},
        }});
      }}
      return idx;
    }}

    function renderPlots() {{
      const progress = state.progress;
      const ref = bundle.reference;
      const idx = nearestIndex(ref.x.length, progress);
      const traces3d = [];
      const traces2d = [];
      const refX = valuesUntil(ref.x, progress);
      const refY = valuesUntil(ref.y, progress);
      const refZ = valuesUntil(ref.z, progress);

      if (state.visible.reference) {{
        traces3d.push({{
          type: 'scatter3d',
          mode: refX.length <= 1 ? 'markers' : 'lines',
          name: 'Reference',
          x: refX, y: refY, z: refZ,
          line: {{ color: colors.reference, width: 6, dash: 'dash' }},
        }});
        traces2d.push({{
          type: 'scatter',
          mode: refX.length <= 1 ? 'markers' : 'lines',
          name: 'Reference',
          x: refX, y: refY,
          line: {{ color: colors.reference, width: 2.2, dash: 'dash' }},
        }});
      }}

      addRuns(traces3d, traces2d, bundle.hitl_runs, progress, colors.hitl_repeatability, 'hitl_repeats', 'HITL');
      addRuns(traces3d, traces2d, bundle.sitl_runs, progress, colors.sitl_identified, 'sitl_repeats', 'SITL');
      addTubeAndMean(traces3d, traces2d, bundle.models.hitl_repeatability, progress, colors.hitl_repeatability, state.visible.hitl_tube, state.visible.hitl_mean, 'HITL mean');
      addTubeAndMean(traces3d, traces2d, bundle.models.sitl_identified, progress, colors.sitl_identified, state.visible.sitl_tube, state.visible.sitl_mean, 'SITL mean');
      if (hasPrior) {{
        addRuns(traces3d, traces2d, bundle.prior_runs, progress, colors.sitl_exact_prior, 'prior_repeats', 'Exact prior');
        addTubeAndMean(traces3d, traces2d, bundle.models.sitl_exact_prior, progress, colors.sitl_exact_prior, state.visible.prior_tube, state.visible.prior_mean, 'Exact prior mean');
      }}

      const refPoint = {{ x: ref.x[idx], y: ref.y[idx], z: ref.z[idx] }};
      const hitlIdx = nearestIndex(bundle.models.hitl_repeatability.progress.length, progress);
      const sitlIdx = nearestIndex(bundle.models.sitl_identified.progress.length, progress);
      const hitlPoint = {{
        x: bundle.models.hitl_repeatability.centerline.x[hitlIdx],
        y: bundle.models.hitl_repeatability.centerline.y[hitlIdx],
        z: bundle.models.hitl_repeatability.centerline.z[hitlIdx],
      }};
      const sitlPoint = {{
        x: bundle.models.sitl_identified.centerline.x[sitlIdx],
        y: bundle.models.sitl_identified.centerline.y[sitlIdx],
        z: bundle.models.sitl_identified.centerline.z[sitlIdx],
      }};
      const marker3dX = [refPoint.x, hitlPoint.x, sitlPoint.x];
      const marker3dY = [refPoint.y, hitlPoint.y, sitlPoint.y];
      const marker3dZ = [refPoint.z, hitlPoint.z, sitlPoint.z];
      const marker2dX = [refPoint.x, hitlPoint.x, sitlPoint.x];
      const marker2dY = [refPoint.y, hitlPoint.y, sitlPoint.y];
      const markerText = ['Ref', 'HITL', 'Ident'];
      const markerColors = [colors.reference, colors.hitl_repeatability, colors.sitl_identified];
      if (hasPrior) {{
        const priorIdx = nearestIndex(bundle.models.sitl_exact_prior.progress.length, progress);
        const priorPoint = {{
          x: bundle.models.sitl_exact_prior.centerline.x[priorIdx],
          y: bundle.models.sitl_exact_prior.centerline.y[priorIdx],
          z: bundle.models.sitl_exact_prior.centerline.z[priorIdx],
        }};
        marker3dX.push(priorPoint.x);
        marker3dY.push(priorPoint.y);
        marker3dZ.push(priorPoint.z);
        marker2dX.push(priorPoint.x);
        marker2dY.push(priorPoint.y);
        markerText.push('Prior');
        markerColors.push(colors.sitl_exact_prior);
      }}

      traces3d.push({{
        type: 'scatter3d',
        mode: 'markers+text',
        name: 'Selected progress',
        x: marker3dX,
        y: marker3dY,
        z: marker3dZ,
        text: markerText,
        textposition: 'bottom center',
        marker: {{ color: markerColors, size: 6, symbol: 'x' }},
        showlegend: false,
        hoverinfo: 'skip',
      }});

      traces2d.push({{
        type: 'scatter',
        mode: 'markers+text',
        name: 'Selected progress',
        x: marker2dX,
        y: marker2dY,
        text: markerText,
        textposition: 'bottom center',
        marker: {{ color: markerColors, size: 10, symbol: 'x' }},
        showlegend: false,
        hoverinfo: 'skip',
      }});

      Plotly.newPlot('tubePlot', traces3d, {{
        margin: {{ l: 0, r: 0, t: 10, b: 0 }},
        paper_bgcolor: '#ffffff',
        plot_bgcolor: '#ffffff',
        uirevision: 'bridge-tube-3d',
        legend: {{ orientation: 'h', y: 1.08 }},
        scene: {{
          dragmode: 'orbit',
          xaxis: {{ title: 'x [m]' }},
          yaxis: {{ title: 'y [m]' }},
          zaxis: {{ title: 'z [m]' }},
          aspectmode: 'data',
          camera: {{ eye: {{ x: 1.25, y: 1.20, z: 0.75 }} }},
        }},
      }}, {{ responsive: true }});

      const pair = currentPair();
      Plotly.newPlot('tubeZoomPlot', traces2d, {{
        margin: {{ l: 50, r: 20, t: 10, b: 45 }},
        paper_bgcolor: '#ffffff',
        plot_bgcolor: '#ffffff',
        xaxis: {{ title: 'x [m]', scaleanchor: 'y', scaleratio: 1 }},
        yaxis: {{ title: 'y [m]' }},
        legend: {{ orientation: 'h', y: 1.08 }},
        annotations: [{{
          xref: 'paper',
          yref: 'paper',
          x: 0.02,
          y: 0.98,
          xanchor: 'left',
          yanchor: 'top',
          align: 'left',
          bgcolor: '#fff7ed',
          bordercolor: '#fed7aa',
          text: `${{pair.left_label}} vs ${{pair.right_label}}<br>Overlap ${{fmt(pair.mean_overlap_pct, 1)}}%<br>Contact ${{fmt(pair.contact_pct, 1)}}%`,
          showarrow: false,
        }}],
      }}, {{ responsive: true }});

      const tubePlot = document.getElementById('tubePlot');
      if (tubePlot) {{
        tubePlot.on?.('plotly_hover', (event) => {{
          const point = event?.points?.[0];
          if (!point || point.pointNumber === undefined) return;
          const len = bundle.reference.x.length || 1;
          const localIdx = Math.max(0, Math.min(len - 1, point.pointNumber));
          syncSlider(String(Math.round((localIdx / Math.max(1, len - 1)) * 1000)));
        }});
      }}
      const tubeZoomPlot = document.getElementById('tubeZoomPlot');
      if (tubeZoomPlot) {{
        tubeZoomPlot.on?.('plotly_hover', (event) => {{
          const point = event?.points?.[0];
          if (!point || point.pointNumber === undefined) return;
          const len = bundle.reference.x.length || 1;
          const localIdx = Math.max(0, Math.min(len - 1, point.pointNumber));
          syncSlider(String(Math.round((localIdx / Math.max(1, len - 1)) * 1000)));
        }});
      }}
    }}

    function syncSlider(value) {{
      el.progressSlider.value = value;
      el.progressSliderBottom.value = value;
      state.progress = Number(value) / 1000;
      const label = `${{fmt(state.progress * 100, 1)}}%`;
      el.progressText.textContent = label;
      el.progressTextBottom.textContent = label;
      render();
    }}

    function render() {{
      renderSelection();
      renderPairModule();
      renderPlots();
    }}

    function init() {{
      el.bundleSummary.textContent = hasPrior
        ? `${{bundle.hitl_runs.length}} HITL CSV + ${{bundle.sitl_runs.length}} identified-SITL CSV + ${{bundle.prior_runs.length}} exact-prior CSV loaded`
        : `${{bundle.hitl_runs.length}} HITL CSV + ${{bundle.sitl_runs.length}} identified-SITL CSV loaded`;
      el.title.textContent = 'Hairpin x5 bridge review';
      el.subtitle.textContent = hasPrior
        ? 'The goal is visual shape similarity between HITL and both SITL candidates. Use the overlap module to inspect whether the tubes occupy the same corridor, and whether the exact-prior ceiling is substantially better than the identified twin.'
        : 'The goal is visual shape similarity between HITL and the calibrated SITL twin. Use the overlap module to inspect whether the two tubes occupy the same corridor.';
      el.sidebarIntro.textContent = hasPrior
        ? 'Reference trajectory, HITL repeatability tube, identified-SDF SITL tube, and the exact jMAVSim-prior SITL tube are shown together. The main question here is shape similarity and tube overlap, not just RMSE.'
        : 'Reference trajectory, HITL repeatability tube, and the calibrated identified-SDF SITL tube are shown together. The main question here is shape similarity and tube overlap, not just RMSE.';
      el.metaCase.textContent = bundle.case_name;
      el.metaHitlRuns.textContent = String(bundle.hitl_runs.length);
      el.metaSitlRuns.textContent = String(bundle.sitl_runs.length);
      el.metaPriorRuns.textContent = hasPrior ? String(bundle.prior_runs.length) : 'n/a';
      el.metaOutDir.textContent = bundle.out_dir;
      el.tubePngLink.href = bundle.figures?.png || '#';
      el.tubeSvgLink.href = bundle.figures?.svg || '#';
      if (!hasPrior) {{
        el.metaPriorCard.hidden = true;
        el.pairPriorCard.hidden = true;
        el.priorRunsCard.hidden = true;
        el.priorCsvLink.hidden = true;
      }}
      buildControls();
      renderRunTable(el.hitlRunsBody, bundle.hitl_runs);
      renderRunTable(el.sitlRunsBody, bundle.sitl_runs);
      if (hasPrior) {{
        renderRunTable(el.priorRunsBody, bundle.prior_runs);
      }}
      el.hitlCsvLink.href = csvHref(bundle.hitl_runs[0]?.csv_b64 || '');
      el.sitlCsvLink.href = csvHref(bundle.sitl_runs[0]?.csv_b64 || '');
      if (hasPrior) {{
        el.priorCsvLink.href = csvHref(bundle.prior_runs[0]?.csv_b64 || '');
      }}
      el.progressSlider.oninput = (event) => syncSlider(event.target.value);
      el.progressSliderBottom.oninput = (event) => syncSlider(event.target.value);
      syncSlider(0);
    }}

    init();
  </script>
</body>
</html>
"""


def build_bundle(
    *,
    hitl_tracking_dir: Path,
    sitl_tracking_dir: Path,
    prior_tracking_dir: Path | None,
    out_dir: Path,
    case_name: str,
    samples: int,
    max_points: int,
) -> dict[str, object]:
    hitl_csv_paths = sorted(hitl_tracking_dir.glob("*.csv"))
    sitl_csv_paths = sorted(sitl_tracking_dir.glob("*.csv"))
    prior_csv_paths = sorted(prior_tracking_dir.glob("*.csv")) if prior_tracking_dir else []
    if not hitl_csv_paths:
        raise FileNotFoundError(f"no HITL tracking CSV files found under {hitl_tracking_dir}")
    if not sitl_csv_paths:
        raise FileNotFoundError(f"no SITL tracking CSV files found under {sitl_tracking_dir}")
    if prior_tracking_dir and not prior_csv_paths:
        raise FileNotFoundError(f"no exact-prior SITL tracking CSV files found under {prior_tracking_dir}")

    out_dir.mkdir(parents=True, exist_ok=True)
    sanitized_hitl_dir = out_dir / "sanitized" / "hitl"
    sanitized_sitl_dir = out_dir / "sanitized" / "sitl"
    sanitized_prior_dir = out_dir / "sanitized" / "prior"
    sanitized_hitl = [_sanitize_tracking_csv(path, sanitized_hitl_dir / path.name) for path in hitl_csv_paths]
    sanitized_sitl = [_sanitize_tracking_csv(path, sanitized_sitl_dir / path.name) for path in sitl_csv_paths]
    sanitized_prior = [_sanitize_tracking_csv(path, sanitized_prior_dir / path.name) for path in prior_csv_paths]

    hitl_runs = [
        _run_payload(case_name, csv_path, sanitized_csv, out_dir=out_dir, channel="hitl", max_points=max_points)
        for csv_path, sanitized_csv in zip(hitl_csv_paths, sanitized_hitl)
    ]
    sitl_runs = [
        _run_payload(case_name, csv_path, sanitized_csv, out_dir=out_dir, channel="sitl", max_points=max_points)
        for csv_path, sanitized_csv in zip(sitl_csv_paths, sanitized_sitl)
    ]
    prior_runs = [
        _run_payload(case_name, csv_path, sanitized_csv, out_dir=out_dir, channel="prior", max_points=max_points)
        for csv_path, sanitized_csv in zip(prior_csv_paths, sanitized_prior)
    ]

    query = np.linspace(0.0, 1.0, samples)
    hitl_model = _build_model_payload(case_name, sanitized_hitl, query, "hitl_repeatability", "HITL repeatability")
    sitl_model = _build_model_payload(case_name, sanitized_sitl, query, "sitl_identified", "SITL identified twin")
    prior_model = (
        _build_model_payload(case_name, sanitized_prior, query, "sitl_exact_prior", "SITL exact jMAVSim prior")
        if sanitized_prior
        else None
    )
    models_payload = {
        hitl_model["key"]: hitl_model,
        sitl_model["key"]: sitl_model,
    }
    if prior_model is not None:
        models_payload[prior_model["key"]] = prior_model
    case_payload = {
        "reference": hitl_model["reference_centerline"],
        "models": models_payload,
    }
    figures = _render_case_figure(case_name, case_payload, out_dir / "figures")
    overlaps = [
        _pair_overlap_payload(hitl_model, sitl_model),
        _reference_vs_model_payload(hitl_model, model_key=hitl_model["key"], label=hitl_model["label"]),
        _reference_vs_model_payload(sitl_model, model_key=sitl_model["key"], label=sitl_model["label"]),
    ]
    if prior_model is not None:
        overlaps.extend(
            [
                _pair_overlap_payload(hitl_model, prior_model),
                _pair_overlap_payload(sitl_model, prior_model),
                _reference_vs_model_payload(prior_model, model_key=prior_model["key"], label=prior_model["label"]),
            ]
        )

    bundle = {
        "hitl_log_root": str(hitl_tracking_dir.resolve()),
        "sitl_log_root": str(sitl_tracking_dir.resolve()),
        "prior_log_root": str(prior_tracking_dir.resolve()) if prior_tracking_dir else "",
        "out_dir": str(out_dir.resolve()),
        "case_name": case_name,
        "has_prior": prior_model is not None,
        "reference": hitl_model["reference_centerline"],
        "hitl_runs": hitl_runs,
        "sitl_runs": sitl_runs,
        "prior_runs": prior_runs,
        "models": models_payload,
        "overlap_pairs": overlaps,
        "figures": figures,
    }
    (out_dir / "summary.json").write_text(json.dumps(bundle, indent=2), encoding="utf-8")
    (out_dir / "index.html").write_text(_build_html(bundle, _plotly_script_tag()), encoding="utf-8")
    return bundle


def main() -> int:
    parser = argparse.ArgumentParser(description="Build a self-contained HITL vs SITL hairpin bridge review page.")
    parser.add_argument("--hitl-tracking-dir", required=True, help="Directory containing repeated HITL tracking CSV files.")
    parser.add_argument("--sitl-tracking-dir", required=True, help="Directory containing repeated SITL tracking CSV files.")
    parser.add_argument(
        "--prior-tracking-dir",
        default="",
        help="Optional directory containing repeated exact-prior SITL tracking CSV files.",
    )
    parser.add_argument("--out-dir", required=True, help="Output directory for index.html, summary.json, figures, and raw CSVs.")
    parser.add_argument("--case-name", default="hairpin", help="Trajectory case name used for trimming/alignment.")
    parser.add_argument("--samples", type=int, default=QUERY_SAMPLES, help="Progress-aligned samples used for the tube payload.")
    parser.add_argument("--max-points", type=int, default=DEFAULT_MAX_POINTS, help="Maximum plotted points per individual run after decimation.")
    args = parser.parse_args()

    bundle = build_bundle(
        hitl_tracking_dir=Path(args.hitl_tracking_dir).expanduser().resolve(),
        sitl_tracking_dir=Path(args.sitl_tracking_dir).expanduser().resolve(),
        prior_tracking_dir=Path(args.prior_tracking_dir).expanduser().resolve() if args.prior_tracking_dir.strip() else None,
        out_dir=Path(args.out_dir).expanduser().resolve(),
        case_name=args.case_name,
        samples=args.samples,
        max_points=args.max_points,
    )
    print(
        json.dumps(
            {
                "ok": True,
                "out_dir": bundle["out_dir"],
                "hitl_runs": len(bundle["hitl_runs"]),
                "sitl_runs": len(bundle["sitl_runs"]),
                "prior_runs": len(bundle["prior_runs"]),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
