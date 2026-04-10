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


def _plotly_script_tag() -> str:
    if PLOTLY_ASSET_PATH.exists():
        return f"<script>{PLOTLY_ASSET_PATH.read_text(encoding='utf-8')}</script>"
    return '<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>'


def _copy_raw_csv(src: Path, dst: Path) -> str:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return str(dst)


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


def _reference_overlap_payload(model_payload: dict[str, object]) -> dict[str, object]:
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
        "pair_key": "reference__hitl_repeatability",
        "left_key": "reference",
        "right_key": "hitl_repeatability",
        "mean_iou_pct": round(float(np.mean(normalized_overlap) * 100.0), 3),
        "median_iou_pct": round(float(np.median(normalized_overlap) * 100.0), 3),
        "max_iou_pct": round(float(np.max(normalized_overlap) * 100.0), 3),
        "contact_pct": round(float(np.mean(inside) * 100.0), 3),
        "containment_pct": round(float(np.mean(inside) * 100.0), 3),
        "mean_center_distance_m": round(float(np.mean(center_distance)), 6),
        "max_center_distance_m": round(float(np.max(center_distance)), 6),
        "mean_excess_gap_m": round(float(np.mean(excess_gap)), 6),
        "max_excess_gap_m": round(float(np.max(excess_gap)), 6),
        "mean_radius_m": round(float(np.mean(radius_xy)), 6),
        "max_radius_m": round(float(np.max(radius_xy)), 6),
        "focus_progress_pct": round(float(progress[focus_idx] * 100.0), 3) if len(progress) else 0.0,
        "sample_iou_pct": [round(float(value) * 100.0, 4) for value in normalized_overlap.tolist()],
        "sample_contact": [bool(value) for value in inside.tolist()],
        "sample_center_distance_m": [round(float(value), 6) for value in center_distance.tolist()],
        "sample_radius_m": [round(float(value), 6) for value in radius_xy.tolist()],
        "sample_excess_gap_m": [round(float(value), 6) for value in excess_gap.tolist()],
        "comment": (
            "Single-channel HITL mode: overlap is a normalized reference-to-tube containment score. "
            "100% means the reference lies on the tube centerline, 0% means it sits outside the tube."
        ),
    }


def _run_payload(case_name: str, csv_path: Path, sanitized_csv_path: Path, *, out_dir: Path, max_points: int) -> dict[str, object]:
    timestamps_s, ref_raw, pos_raw = _load_tracking(sanitized_csv_path)
    ref_trimmed, pos_trimmed = _trim_dataset(case_name, timestamps_s, ref_raw, pos_raw)
    ref_plot, pos_plot = _align_to_reference_start(ref_trimmed, pos_trimmed)
    rmse_raw, errors = _trajectory_rmse(ref_trimmed, pos_trimmed)
    idx = _decimation_indices(len(ref_plot), max_points=max_points)
    ref_plot_decimated = ref_plot[idx].tolist() if idx else []
    pos_plot_decimated = pos_plot[idx].tolist() if idx else []
    progress = [0.0 if len(ref_plot) <= 1 else float(i) / float(len(ref_plot) - 1) for i in idx]
    raw_rel = Path("raw") / "tracking_logs" / csv_path.name
    _copy_raw_csv(csv_path, out_dir / raw_rel)

    return {
        "name": csv_path.stem,
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


def _build_html(bundle: dict[str, object], plotly_script: str) -> str:
    data_json = json.dumps(bundle, ensure_ascii=True)
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>HITL Hairpin Repeatability Review</title>
  {plotly_script}
  <style>
    :root {{
      --bg: #f3ede3;
      --panel: #fffaf4;
      --ink: #17202a;
      --muted: #5d6d7e;
      --accent: #0f766e;
      --tube: {HITL_COLOR};
      --border: #d8cfc0;
      --chip: #f6efe5;
    }}
    body {{
      margin: 0;
      font-family: "IBM Plex Sans", "Segoe UI", sans-serif;
      color: var(--ink);
      background: linear-gradient(180deg, #eee5d7 0%, var(--bg) 100%);
    }}
    .layout {{
      display: block;
      min-height: 100vh;
    }}
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
    h1, h2, h3 {{
      margin: 0 0 12px 0;
    }}
    .card {{
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 18px;
      padding: 18px;
      box-shadow: 0 10px 28px rgba(23, 32, 42, 0.06);
    }}
    p, li {{
      line-height: 1.5;
      color: var(--muted);
    }}
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
    .meta strong {{
      font-size: 18px;
      font-weight: 600;
    }}
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
    .chip-list {{
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
    }}
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
    .slider-row input[type="range"] {{
      width: 100%;
    }}
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
    .plot {{
      min-height: 520px;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
    }}
    th, td {{
      padding: 10px 12px;
      border-bottom: 1px solid var(--border);
      text-align: left;
    }}
    .tube-compare-card {{
      margin-top: 12px;
      padding: 14px;
      border: 1px solid var(--border);
      border-radius: 14px;
      background: white;
    }}
    .tube-compare-controls {{
      display: grid;
      gap: 12px;
    }}
    .tube-compare-picks {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
    }}
    .tube-compare-field {{
      display: grid;
      gap: 8px;
    }}
    .tube-compare-field select {{
      width: 100%;
      border: 1px solid var(--border);
      border-radius: 10px;
      padding: 8px 10px;
      background: #fff;
      font: inherit;
    }}
    .tube-table-wrap {{
      margin-top: 12px;
    }}
    code {{
      background: #f3ede3;
      padding: 2px 6px;
      border-radius: 6px;
    }}
    .footer-links {{
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
    }}
    .footer-links a {{
      color: var(--accent);
      text-decoration: none;
      font-weight: 600;
    }}
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
      .tube-compare-picks {{ grid-template-columns: 1fr; }}
    }}
  </style>
</head>
<body>
  <div class="layout">
    <aside class="sidebar">
      <h1>HITL Hairpin Repeatability</h1>
      <p>Separate HITL review page with the same repeatability language as the SITL inspector. The main plot shows the reference path, the 5-run mean centerline, the repeatability tube, and optional individual repeats.</p>
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
          <div class="meta"><span>Runs</span><strong id="metaRuns"></strong></div>
          <div class="meta"><span>Included</span><strong id="metaIncluded"></strong></div>
          <div class="meta"><span>Output root</span><strong id="metaOutDir"></strong></div>
        </div>
      </section>

      <section class="card">
        <h3>Inspection Controls</h3>
        <div class="controls-grid">
          <div class="control-block">
            <h4>Visible layers</h4>
            <div id="layerChecks" class="chip-list"></div>
            <p>Reference, HITL mean centerline, uncertainty tube, and individual repeats are controlled here.</p>
          </div>
          <div class="control-block">
            <h4>Track target</h4>
            <div class="chip-list" id="trackTargetChecks"></div>
            <p>The slider follows the selected target. Hovering a trace also jumps the slider to that progress location.</p>
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
          <div class="meta"><span>Ref error</span><strong id="selError"></strong></div>
          <div class="meta"><span>Tracked point</span><strong id="selPoint"></strong></div>
          <div class="meta"><span>Reference point</span><strong id="selRef"></strong></div>
          <div class="meta"><span>Tube radius</span><strong id="selTubeRadius"></strong></div>
        </div>
      </section>

      <section class="card">
        <h3>Hairpin tube view</h3>
        <p>Reference line and HITL repeatability tube are clipped to the current slider position. Individual runs are shown as faint overlays to make local spread easy to inspect.</p>
        <div class="plot-grid">
          <div class="card">
            <h3>3D centerline + uncertainty</h3>
            <div id="tubePlot" class="plot"></div>
          </div>
          <div class="card">
            <h3>XY tube overlap zoom</h3>
            <div id="tubeZoomPlot" class="plot"></div>
          </div>
        </div>
        <div class="meta-grid" style="margin-top:14px">
          <div class="meta"><span>Selected pair</span><strong id="tubeSelectedPair">-</strong></div>
          <div class="meta"><span>Mean overlap</span><strong id="tubeOverlap">-</strong></div>
          <div class="meta"><span>Current overlap</span><strong id="tubeCurrentOverlap">-</strong></div>
          <div class="meta"><span>Contact</span><strong id="tubeContact">-</strong></div>
          <div class="meta"><span>Mean center dist.</span><strong id="tubeDistance">-</strong></div>
          <div class="meta"><span>Runs in tube</span><strong id="tubeRuns">-</strong></div>
        </div>
        <p id="tubeFilterNote" style="margin-top:12px"></p>
        <p>Assets: <a id="tubePngLink" href="#" target="_blank" rel="noopener">PNG</a> / <a id="tubeSvgLink" href="#" target="_blank" rel="noopener">SVG</a></p>
        <div class="slider-row" style="margin-top:14px">
          <input id="progressSliderBottom" type="range" min="0" max="1000" step="1" value="0" />
          <strong id="progressTextBottom">0%</strong>
        </div>
        <p>The lower slider is synchronized with the main slider and keeps the tube scrubber immediately under the tube plots.</p>
        <div class="tube-compare-card">
          <h3>Overlap comparison module</h3>
          <p>This single-channel HITL view fixes the pair as <code>Reference vs HITL repeatability tube</code>. When a second HITL channel is added later, the same module can compare any two channels.</p>
          <div id="tubeCompareControls" class="tube-compare-controls"></div>
        </div>
        <div class="tube-table-wrap">
          <table>
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
            <tbody id="tubeOverlapBody"></tbody>
          </table>
        </div>
      </section>

      <section class="card">
        <div class="footer-links">
          <a id="currentCsvLink" href="#" target="_blank" rel="noopener">Open selected raw CSV</a>
        </div>
      </section>

      <section class="card">
        <h3>5-run decision table</h3>
        <table>
          <thead>
            <tr>
              <th>Run</th>
              <th>Samples</th>
              <th>RMSE [m]</th>
              <th>Max error [m]</th>
              <th>CSV</th>
            </tr>
          </thead>
          <tbody id="metricsBody"></tbody>
        </table>
      </section>
    </main>
  </div>

  <script>
    const bundle = {data_json};

    const state = {{
      progress: 0,
      visible: {{
        reference: true,
        tube: true,
        mean: true,
        repeats: true,
      }},
      trackKey: 'mean',
    }};

    const el = {{
      bundleSummary: document.getElementById('bundleSummary'),
      title: document.getElementById('title'),
      subtitle: document.getElementById('subtitle'),
      metaCase: document.getElementById('metaCase'),
      metaRuns: document.getElementById('metaRuns'),
      metaIncluded: document.getElementById('metaIncluded'),
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
      tubeSelectedPair: document.getElementById('tubeSelectedPair'),
      tubeOverlap: document.getElementById('tubeOverlap'),
      tubeCurrentOverlap: document.getElementById('tubeCurrentOverlap'),
      tubeContact: document.getElementById('tubeContact'),
      tubeDistance: document.getElementById('tubeDistance'),
      tubeRuns: document.getElementById('tubeRuns'),
      tubeFilterNote: document.getElementById('tubeFilterNote'),
      tubePngLink: document.getElementById('tubePngLink'),
      tubeSvgLink: document.getElementById('tubeSvgLink'),
      tubeCompareControls: document.getElementById('tubeCompareControls'),
      tubeOverlapBody: document.getElementById('tubeOverlapBody'),
      currentCsvLink: document.getElementById('currentCsvLink'),
      metricsBody: document.getElementById('metricsBody'),
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
      const value = parseInt(normalized.length === 3 ? normalized.split('').map(item => item + item).join('') : normalized, 16);
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

    function buildCheckboxes() {{
      el.layerChecks.innerHTML = '';
      const labels = [
        ['reference', 'Reference'],
        ['tube', 'Repeatability tube'],
        ['mean', 'HITL mean centerline'],
        ['repeats', 'Individual repeats'],
      ];
      labels.forEach(([key, label]) => {{
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
      [['reference', 'Reference'], ['mean', 'HITL mean']].forEach(([key, label]) => {{
        const wrapper = document.createElement('label');
        wrapper.className = 'chip';
        wrapper.innerHTML = `<input type="radio" name="trackTarget" ${{state.trackKey === key ? 'checked' : ''}}> <span>${{label}}</span>`;
        wrapper.querySelector('input').onchange = () => {{
          state.trackKey = key;
          render();
        }};
        el.trackTargetChecks.appendChild(wrapper);
      }});

      el.tubeCompareControls.innerHTML = `
        <div class="tube-compare-picks">
          <label class="tube-compare-field">
            <strong>Katman A</strong>
            <select disabled><option>Reference</option></select>
          </label>
          <label class="tube-compare-field">
            <strong>Katman B</strong>
            <select disabled><option>HITL repeatability</option></select>
          </label>
        </div>
      `;
    }}

    function renderRunMetrics() {{
      el.metricsBody.innerHTML = '';
      bundle.runs.forEach((run) => {{
        const tr = document.createElement('tr');
        tr.innerHTML = `
          <td><strong>${{run.name}}</strong></td>
          <td>${{run.samples}}</td>
          <td>${{fmt(run.rmse_m, 4)}}</td>
          <td>${{fmt(run.max_error_m, 4)}}</td>
          <td><a href="${{csvHref(run.csv_b64)}}" download="${{run.csv_name}}" target="_blank" rel="noopener">open CSV</a></td>
        `;
        el.metricsBody.appendChild(tr);
      }});
    }}

    function renderSelection() {{
      const overlap = bundle.reference_overlap;
      const idx = nearestIndex(bundle.model.progress.length, state.progress);
      const refPoint = {{
        x: bundle.model.reference_centerline.x[idx],
        y: bundle.model.reference_centerline.y[idx],
        z: bundle.model.reference_centerline.z[idx],
      }};
      const posPoint = {{
        x: bundle.model.centerline.x[idx],
        y: bundle.model.centerline.y[idx],
        z: bundle.model.centerline.z[idx],
      }};
      const trackedPoint = state.trackKey === 'reference' ? refPoint : posPoint;
      const err = overlap.sample_center_distance_m[idx];
      const radius = overlap.sample_radius_m[idx];
      el.selLayer.textContent = state.trackKey === 'reference' ? 'Reference' : 'HITL mean';
      el.selProgress.textContent = `${{fmt(state.progress * 100, 1)}} %`;
      el.selError.textContent = `${{fmt(err, 3)}} m`;
      el.selPoint.textContent = `(${{fmt(trackedPoint.x, 2)}}, ${{fmt(trackedPoint.y, 2)}}, ${{fmt(trackedPoint.z, 2)}})`;
      el.selRef.textContent = `(${{fmt(refPoint.x, 2)}}, ${{fmt(refPoint.y, 2)}}, ${{fmt(refPoint.z, 2)}})`;
      el.selTubeRadius.textContent = `${{fmt(radius, 3)}} m`;
      el.currentCsvLink.href = csvHref(bundle.runs[0]?.csv_b64 || '');
      el.currentCsvLink.textContent = 'Open first raw repeat CSV';
    }}

    function renderOverlapModule() {{
      const overlap = bundle.reference_overlap;
      const idx = nearestIndex(overlap.sample_iou_pct.length, state.progress);
      const included = bundle.model.included_source_count;
      const total = bundle.model.source_count;
      el.tubeSelectedPair.textContent = 'Reference vs HITL repeatability tube';
      el.tubeOverlap.textContent = `${{fmt(overlap.mean_iou_pct, 1)}} %`;
      el.tubeCurrentOverlap.textContent = `${{fmt(overlap.sample_iou_pct[idx], 1)}} %`;
      el.tubeContact.textContent = `${{fmt(overlap.contact_pct, 1)}} %`;
      el.tubeDistance.textContent = `${{fmt(overlap.mean_center_distance_m, 3)}} m`;
      el.tubeRuns.textContent = `${{included}} / ${{total}}`;
      el.tubeFilterNote.textContent = overlap.comment;
      el.tubePngLink.href = bundle.figures?.png || '#';
      el.tubeSvgLink.href = bundle.figures?.svg || '#';
      el.tubeOverlapBody.innerHTML = `
        <tr style="background:#eff6ff">
          <td><strong>Reference</strong><br><span>HITL repeatability tube</span></td>
          <td>${{fmt(overlap.mean_iou_pct, 1)}}</td>
          <td>${{fmt(overlap.sample_iou_pct[idx], 1)}}</td>
          <td>${{fmt(overlap.contact_pct, 1)}}</td>
          <td>${{fmt(overlap.mean_center_distance_m, 3)}}</td>
          <td>${{overlap.comment}}</td>
        </tr>
      `;
    }}

    function renderTubePlots() {{
      const model = bundle.model;
      const overlap = bundle.reference_overlap;
      const idx = nearestIndex(model.progress.length, state.progress);
      const refX = valuesUntil(model.reference_centerline.x, state.progress);
      const refY = valuesUntil(model.reference_centerline.y, state.progress);
      const refZ = valuesUntil(model.reference_centerline.z, state.progress);
      const meanX = valuesUntil(model.centerline.x, state.progress);
      const meanY = valuesUntil(model.centerline.y, state.progress);
      const meanZ = valuesUntil(model.centerline.z, state.progress);
      const upperX = valuesUntil(model.tube_xy.upper_x, state.progress);
      const upperY = valuesUntil(model.tube_xy.upper_y, state.progress);
      const lowerX = valuesUntil(model.tube_xy.lower_x, state.progress);
      const lowerY = valuesUntil(model.tube_xy.lower_y, state.progress);
      const polyX = upperX.length >= 2 ? [...upperX, ...lowerX.slice().reverse(), upperX[0]] : [];
      const polyY = upperY.length >= 2 ? [...upperY, ...lowerY.slice().reverse(), upperY[0]] : [];
      const radii = valuesUntil(model.std.xy_radius, state.progress);
      const maxRadius = Math.max(...model.std.xy_radius, 1e-6);
      const traces3d = [];
      const traces2d = [];

      if (state.visible.reference) {{
        traces3d.push({{
          type: 'scatter3d',
          mode: refX.length <= 1 ? 'markers' : 'lines',
          name: 'Reference',
          x: refX, y: refY, z: refZ,
          line: {{ color: '{REF_COLOR}', width: 6, dash: 'dash' }},
        }});
        traces2d.push({{
          type: 'scatter',
          mode: refX.length <= 1 ? 'markers' : 'lines',
          name: 'Reference',
          x: refX, y: refY,
          line: {{ color: '{REF_COLOR}', width: 2.2, dash: 'dash' }},
          showlegend: false,
        }});
      }}

      if (state.visible.repeats) {{
        bundle.runs.forEach((run) => {{
          const points3d = pointsUntil(run.position_plot, state.progress);
          const xs = points3d.map((p) => p[0]);
          const ys = points3d.map((p) => p[1]);
          const zs = points3d.map((p) => p[2]);
          traces3d.push({{
            type: 'scatter3d',
            mode: xs.length <= 1 ? 'markers' : 'lines',
            name: run.name,
            x: xs, y: ys, z: zs,
            line: {{ color: rgba('{HITL_COLOR}', 0.30), width: 2 }},
            marker: {{ color: rgba('{HITL_COLOR}', 0.30), size: 3 }},
            opacity: 0.30,
            showlegend: false,
            hovertemplate: `<b>${{run.name}}</b><extra></extra>`,
          }});
          traces2d.push({{
            type: 'scatter',
            mode: xs.length <= 1 ? 'markers' : 'lines',
            name: run.name,
            x: xs, y: ys,
            line: {{ color: rgba('{HITL_COLOR}', 0.25), width: 1.5 }},
            opacity: 0.28,
            showlegend: false,
            hovertemplate: `<b>${{run.name}}</b><extra></extra>`,
          }});
        }});
      }}

      if (state.visible.tube && polyX.length >= 2) {{
        traces2d.push({{
          type: 'scatter',
          mode: 'lines',
          name: 'HITL 1σ tube',
          x: polyX, y: polyY,
          fill: 'toself',
          fillcolor: rgba('{HITL_COLOR}', 0.18),
          line: {{ color: rgba('{HITL_COLOR}', 0.36), width: 1 }},
          hoverinfo: 'skip',
          showlegend: false,
        }});
      }}

      if (state.visible.mean) {{
        traces3d.push({{
          type: 'scatter3d',
          mode: meanX.length <= 1 ? 'markers' : 'lines',
          name: 'HITL mean centerline',
          x: meanX, y: meanY, z: meanZ,
          line: {{ color: '{HITL_COLOR}', width: 6 }},
          marker: {{ color: '{HITL_COLOR}', size: 4 }},
          hovertemplate: '<b>HITL mean</b><br>x=%{{x:.2f}}<br>y=%{{y:.2f}}<br>z=%{{z:.2f}}<extra></extra>',
        }});
        traces3d.push({{
          type: 'scatter3d',
          mode: 'markers',
          name: 'Tube sigma',
          x: meanX, y: meanY, z: meanZ,
          marker: {{
            color: '{HITL_COLOR}',
            size: radii.map((value) => 4 + (12 * Math.max(0, value)) / maxRadius),
            opacity: 0.18,
            sizemode: 'diameter',
          }},
          showlegend: false,
          hovertemplate: '<b>Tube sigma</b><extra></extra>',
        }});
        traces2d.push({{
          type: 'scatter',
          mode: meanX.length <= 1 ? 'markers' : 'lines',
          name: 'HITL mean centerline',
          x: meanX, y: meanY,
          line: {{ color: '{HITL_COLOR}', width: 3.4 }},
          hovertemplate: '<b>HITL mean</b><br>x=%{{x:.2f}}<br>y=%{{y:.2f}}<extra></extra>',
          showlegend: false,
        }});
      }}

      const refPoint = {{
        x: model.reference_centerline.x[idx],
        y: model.reference_centerline.y[idx],
        z: model.reference_centerline.z[idx],
      }};
      const meanPoint = {{
        x: model.centerline.x[idx],
        y: model.centerline.y[idx],
        z: model.centerline.z[idx],
      }};

      traces3d.push({{
        type: 'scatter3d',
        mode: 'markers+text',
        name: 'Selected progress',
        x: [refPoint.x, meanPoint.x],
        y: [refPoint.y, meanPoint.y],
        z: [refPoint.z, meanPoint.z],
        text: ['Ref', 'Mean'],
        textposition: 'bottom center',
        marker: {{ color: ['{REF_COLOR}', '{HITL_COLOR}'], size: 6, symbol: 'x' }},
        showlegend: false,
        hoverinfo: 'skip',
      }});

      traces2d.push({{
        type: 'scatter',
        mode: 'markers+text',
        name: 'Selected progress',
        x: [refPoint.x, meanPoint.x],
        y: [refPoint.y, meanPoint.y],
        text: ['Ref', 'Mean'],
        textposition: 'bottom center',
        marker: {{ color: ['{REF_COLOR}', '{HITL_COLOR}'], size: 10, symbol: 'x' }},
        showlegend: false,
        hoverinfo: 'skip',
      }});

      Plotly.newPlot('tubePlot', traces3d, {{
        margin: {{ l: 0, r: 0, t: 10, b: 0 }},
        paper_bgcolor: '#ffffff',
        plot_bgcolor: '#ffffff',
        uirevision: 'hitl-tube-3d',
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

      Plotly.newPlot('tubeZoomPlot', traces2d, {{
        margin: {{ l: 50, r: 20, t: 10, b: 45 }},
        paper_bgcolor: '#ffffff',
        plot_bgcolor: '#ffffff',
        xaxis: {{ title: 'x [m]', scaleanchor: 'y', scaleratio: 1 }},
        yaxis: {{ title: 'y [m]' }},
        showlegend: false,
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
          text: `Reference vs HITL tube<br>Overlap ${{fmt(overlap.mean_iou_pct, 1)}}%<br>Contact ${{fmt(overlap.contact_pct, 1)}}%`,
          showarrow: false,
        }}],
      }}, {{ responsive: true }});
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
      renderOverlapModule();
      renderTubePlots();
    }}

    function init() {{
      el.bundleSummary.textContent = `${{bundle.runs.length}} HITL tracking logs loaded from ${{bundle.log_root}}`;
      el.title.textContent = 'Hairpin x5 repeatability review';
      el.subtitle.textContent = 'Reference path plus the 5-run HITL repeatability tube. The overlap module is pinned to reference-versus-tube until a second HITL channel is added.';
      el.metaCase.textContent = bundle.case_name;
      el.metaRuns.textContent = String(bundle.runs.length);
      el.metaIncluded.textContent = `${{bundle.model.included_source_count}} / ${{bundle.model.source_count}}`;
      el.metaOutDir.textContent = bundle.out_dir;
      buildCheckboxes();
      renderRunMetrics();
      el.progressSlider.oninput = (event) => syncSlider(event.target.value);
      el.progressSliderBottom.oninput = (event) => syncSlider(event.target.value);
      syncSlider(0);

      document.getElementById('tubePlot').on && document.getElementById('tubePlot').on('plotly_hover', () => null);
      const tubePlot = document.getElementById('tubePlot');
      if (tubePlot) {{
        tubePlot.on?.('plotly_hover', (event) => {{
          const point = event?.points?.[0];
          if (!point || point.pointNumber === undefined) return;
          const len = bundle.model.progress.length || 1;
          const idx = Math.max(0, Math.min(len - 1, point.pointNumber));
          syncSlider(String(Math.round((idx / Math.max(1, len - 1)) * 1000)));
        }});
      }}
      const tubeZoomPlot = document.getElementById('tubeZoomPlot');
      if (tubeZoomPlot) {{
        tubeZoomPlot.on?.('plotly_hover', (event) => {{
          const point = event?.points?.[0];
          if (!point || point.pointNumber === undefined) return;
          const len = bundle.model.progress.length || 1;
          const idx = Math.max(0, Math.min(len - 1, point.pointNumber));
          syncSlider(String(Math.round((idx / Math.max(1, len - 1)) * 1000)));
        }});
      }}
    }}

    init();
  </script>
</body>
</html>
"""


def build_bundle(
    *,
    tracking_dir: Path,
    out_dir: Path,
    case_name: str,
    label: str,
    max_runs: int | None,
    samples: int,
    max_points: int,
) -> dict[str, object]:
    csv_paths = sorted(tracking_dir.glob("*.csv"))
    if max_runs is not None:
        csv_paths = csv_paths[:max_runs]
    if not csv_paths:
        raise FileNotFoundError(f"no tracking CSV files found under {tracking_dir}")

    out_dir.mkdir(parents=True, exist_ok=True)
    sanitized_dir = out_dir / "sanitized"
    sanitized_paths = [_sanitize_tracking_csv(csv_path, sanitized_dir / csv_path.name) for csv_path in csv_paths]
    run_payloads = [
        _run_payload(case_name, csv_path, sanitized_csv_path, out_dir=out_dir, max_points=max_points)
        for csv_path, sanitized_csv_path in zip(csv_paths, sanitized_paths)
    ]

    query = np.linspace(0.0, 1.0, samples)
    model_payload = _build_model_payload(case_name, sanitized_paths, query, "hitl_repeatability", label)
    case_payload = {
        "reference": model_payload["reference_centerline"],
        "models": {
            "hitl_repeatability": model_payload,
        },
    }
    figures = _render_case_figure(case_name, case_payload, out_dir / "figures")
    reference_overlap = _reference_overlap_payload(model_payload)

    bundle = {
        "log_root": str(tracking_dir.resolve()),
        "out_dir": str(out_dir.resolve()),
        "case_name": case_name,
        "label": label,
        "runs": run_payloads,
        "model": model_payload,
        "reference_overlap": reference_overlap,
        "figures": figures,
    }

    (out_dir / "summary.json").write_text(json.dumps(bundle, indent=2), encoding="utf-8")
    (out_dir / "index.html").write_text(_build_html(bundle, _plotly_script_tag()), encoding="utf-8")
    return bundle


def main() -> int:
    parser = argparse.ArgumentParser(description="Build a self-contained HITL repeatability review page.")
    parser.add_argument("--tracking-dir", required=True, help="Directory containing repeated HITL tracking CSV files.")
    parser.add_argument("--out-dir", required=True, help="Output directory for index.html, summary.json, figures, and raw CSVs.")
    parser.add_argument("--case-name", default="hairpin", help="Trajectory case name used for trimming/alignment.")
    parser.add_argument("--label", default="HITL repeatability", help="Human-readable label for the aggregated HITL channel.")
    parser.add_argument("--max-runs", type=int, default=None, help="Optional cap on the number of CSV files to include.")
    parser.add_argument("--samples", type=int, default=QUERY_SAMPLES, help="Progress-aligned samples used for the tube payload.")
    parser.add_argument("--max-points", type=int, default=DEFAULT_MAX_POINTS, help="Maximum plotted points per individual run after decimation.")
    args = parser.parse_args()

    bundle = build_bundle(
        tracking_dir=Path(args.tracking_dir).expanduser().resolve(),
        out_dir=Path(args.out_dir).expanduser().resolve(),
        case_name=args.case_name,
        label=args.label,
        max_runs=args.max_runs,
        samples=args.samples,
        max_points=args.max_points,
    )
    print(json.dumps({"ok": True, "out_dir": bundle["out_dir"], "num_runs": len(bundle["runs"])}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
