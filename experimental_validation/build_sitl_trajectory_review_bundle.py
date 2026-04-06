from __future__ import annotations

import argparse
import base64
import json
import math
import re
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.trajectory_comparison_figures import (
    PANEL_GROUPS,
    _align_to_reference_start,
    _load_tracking,
    _shape_rmse,
    _trajectory_rmse,
    _trim_dataset,
)

DEFAULT_MAX_POINTS = 1800
REF_COLOR = "#222222"
PLOTLY_ASSET_NAME = "plotly-2.35.2.min.js"
PLOTLY_ASSET_PATH = Path(__file__).resolve().parent / "assets" / PLOTLY_ASSET_NAME


@dataclass(frozen=True)
class DatasetSpec:
    key: str
    label: str
    root: Path
    color: str
    dash: str


def _plotly_script_tag() -> str:
    if PLOTLY_ASSET_PATH.exists():
        return f"<script>{PLOTLY_ASSET_PATH.read_text(encoding='utf-8')}</script>"
    return '<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>'


def _safe_key(label: str) -> str:
    cleaned = re.sub(r"[^a-z0-9]+", "_", label.lower()).strip("_")
    return cleaned or "dataset"


def _case_order() -> list[str]:
    ordered: list[str] = []
    for group in PANEL_GROUPS:
        ordered.extend(group)
    return ordered


def _decimate_points(points: list[list[float]], max_points: int) -> list[list[float]]:
    if len(points) <= max_points:
        return points
    step = max(1, math.ceil(len(points) / max_points))
    reduced = points[::step]
    if reduced[-1] != points[-1]:
        reduced.append(points[-1])
    return reduced


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


def _copy_raw_csv(src: Path, dst: Path) -> str:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return str(dst)


def _dataset_payload(case: str, spec: DatasetSpec, *, out_dir: Path, max_points: int) -> dict[str, object]:
    csv_path = spec.root / "tracking_logs" / f"{case}.csv"
    timestamps_s, ref_raw, pos_raw = _load_tracking(csv_path)
    ref_trimmed, pos_trimmed = _trim_dataset(case, timestamps_s, ref_raw, pos_raw)
    ref_plot, pos_plot = _align_to_reference_start(ref_trimmed, pos_trimmed)
    rmse_raw, _ = _trajectory_rmse(ref_trimmed, pos_trimmed)
    shape_rmse, _ = _shape_rmse(ref_trimmed, pos_trimmed)
    decimation_idx = _decimation_indices(len(ref_trimmed), max_points=max_points)
    progress = [0.0 if len(ref_trimmed) <= 1 else float(idx) / float(len(ref_trimmed) - 1) for idx in decimation_idx]
    ref_plot_decimated = ref_plot[decimation_idx].tolist() if decimation_idx else []
    pos_plot_decimated = pos_plot[decimation_idx].tolist() if decimation_idx else []
    delta_decimated = (pos_plot[decimation_idx] - ref_plot[decimation_idx]).tolist() if decimation_idx else []
    error_norm = [
        math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
        for dx, dy, dz in delta_decimated
    ]

    rel_raw = Path("raw") / spec.key / f"{case}.csv"
    _copy_raw_csv(csv_path, out_dir / rel_raw)
    csv_b64 = base64.b64encode(csv_path.read_bytes()).decode("ascii")

    return {
        "key": spec.key,
        "label": spec.label,
        "color": spec.color,
        "dash": spec.dash,
        "csv": str(rel_raw),
        "csv_name": csv_path.name,
        "csv_b64": csv_b64,
        "samples": len(ref_trimmed),
        "rmse_m": rmse_raw,
        "shape_rmse_m": shape_rmse,
        "reference_plot": ref_plot_decimated,
        "position_plot": pos_plot_decimated,
        "progress": progress,
        "error_xyz": delta_decimated,
        "error_norm_m": error_norm,
    }


def _build_html(bundle: dict[str, object], plotly_script: str) -> str:
    data_json = json.dumps(bundle, ensure_ascii=True)
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>SITL Trajectory Review</title>
  {plotly_script}
  <style>
    :root {{
      --bg: #f3ede3;
      --panel: #fffaf4;
      --ink: #17202a;
      --muted: #5d6d7e;
      --accent: #0f766e;
      --accent-2: #a16207;
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
      display: grid;
      grid-template-columns: 320px 1fr;
      min-height: 100vh;
    }}
    .sidebar {{
      padding: 24px;
      border-right: 1px solid var(--border);
      background: rgba(255, 250, 244, 0.94);
      overflow-y: auto;
    }}
    .content {{
      padding: 24px;
      display: grid;
      gap: 20px;
    }}
    h1, h2, h3 {{
      margin: 0 0 12px 0;
    }}
    p {{
      line-height: 1.5;
    }}
    .card {{
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 18px;
      padding: 18px;
      box-shadow: 0 10px 28px rgba(23, 32, 42, 0.06);
    }}
    .case-list button {{
      width: 100%;
      text-align: left;
      margin: 0 0 10px 0;
      padding: 12px 14px;
      border-radius: 12px;
      border: 1px solid var(--border);
      background: white;
      cursor: pointer;
      color: var(--ink);
    }}
    .case-list button.active {{
      border-color: var(--accent);
      background: #e6fffb;
    }}
    .case-tabs {{
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
    }}
    .case-tab {{
      display: inline-flex;
      align-items: center;
      justify-content: center;
      padding: 10px 14px;
      border-radius: 999px;
      border: 1px solid var(--border);
      background: white;
      cursor: pointer;
      font-size: 14px;
      color: var(--ink);
      min-height: 42px;
    }}
    .case-tab.active {{
      border-color: var(--accent);
      background: #e6fffb;
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
    .control-block h4 {{
      margin: 0 0 10px 0;
      font-size: 15px;
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
    .chip input {{
      margin: 0;
    }}
    .trackbar-wrap {{
      display: grid;
      gap: 10px;
    }}
    .trackbar-row {{
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 12px;
      align-items: center;
    }}
    .trackbar-row input[type="range"] {{
      width: 100%;
    }}
    .selection-grid {{
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
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
    .meta .label {{
      display: block;
      font-size: 12px;
      color: var(--muted);
      margin-bottom: 6px;
      text-transform: uppercase;
      letter-spacing: 0.04em;
    }}
    .meta .value {{
      font-size: 18px;
      font-weight: 600;
    }}
    .dataset-table {{
      width: 100%;
      border-collapse: collapse;
    }}
    .dataset-table th, .dataset-table td {{
      padding: 10px 12px;
      border-bottom: 1px solid var(--border);
      text-align: left;
    }}
    .dataset-table code {{
      background: #f3ede3;
      padding: 2px 6px;
      border-radius: 6px;
    }}
    .legend-note {{
      color: var(--muted);
      font-size: 13px;
      margin-top: 8px;
    }}
    #plot3d {{
      min-height: 72vh;
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
      .layout {{ grid-template-columns: 1fr; }}
      .meta-grid {{ grid-template-columns: repeat(2, minmax(0, 1fr)); }}
      .controls-grid {{ grid-template-columns: 1fr; }}
      .selection-grid {{ grid-template-columns: 1fr; }}
      #plot3d {{ min-height: 55vh; }}
    }}
    @media (max-width: 720px) {{
      .sidebar, .content {{ padding: 14px; }}
      .card {{ padding: 14px; }}
      .meta-grid {{ grid-template-columns: 1fr; }}
      .trackbar-row {{ grid-template-columns: 1fr; }}
      .chip-list {{ flex-direction: column; }}
    }}
  </style>
</head>
<body>
  <div class="layout">
    <aside class="sidebar">
      <h1>SITL Trajectory Review</h1>
      <p>Interactive 3D comparison for the validation trajectories. Use the layer toggles, track target selector, and progress slider to inspect where a selected point sits relative to the main reference path.</p>
      <div class="card">
        <h3>Bundle</h3>
        <p id="bundleSummary"></p>
      </div>
      <div class="card">
        <h3>Trajectories</h3>
        <div id="caseList" class="case-list"></div>
      </div>
    </aside>
    <main class="content">
      <section class="card">
        <h2 id="caseTitle"></h2>
        <p id="caseSubtitle"></p>
        <div class="meta-grid">
          <div class="meta"><span class="label">Reference</span><span class="value">Planned path</span></div>
          <div class="meta"><span class="label">Datasets</span><span class="value" id="metaDatasetCount"></span></div>
          <div class="meta"><span class="label">Max samples</span><span class="value" id="metaMaxSamples"></span></div>
          <div class="meta"><span class="label">Review root</span><span class="value" id="metaOutDir"></span></div>
        </div>
      </section>
      <section class="card">
        <h3>Trajectory Switcher</h3>
        <p class="legend-note">All five validation trajectories are available here. Use these tabs if the left sidebar is out of view.</p>
        <div id="caseTabs" class="case-tabs"></div>
      </section>
      <section class="card">
        <h3>Inspection Controls</h3>
        <div class="controls-grid">
          <div class="control-block">
            <h4>Visible layers</h4>
            <div id="layerControls" class="chip-list"></div>
            <p class="legend-note">Each layer keeps its own legend entry in the 3D view. Visibility is driven by the checkboxes here so the mobile and desktop behavior stays consistent.</p>
          </div>
          <div class="control-block">
            <h4>Track target</h4>
            <div id="trackTargetControls" class="chip-list"></div>
            <p class="legend-note">The slider follows the selected target. Hovering any curve also jumps the slider to the hovered progress location.</p>
          </div>
        </div>
        <div class="trackbar-wrap" style="margin-top:18px">
          <div class="trackbar-row">
            <input id="progressSlider" type="range" min="0" max="1000" step="1" value="0" />
            <strong id="progressLabel">0%</strong>
          </div>
        </div>
      </section>
      <section class="card">
        <h3>Selected Point</h3>
        <div class="selection-grid">
          <div class="meta"><span class="label">Tracked layer</span><span class="value" id="selectionLayer"></span></div>
          <div class="meta"><span class="label">Sample</span><span class="value" id="selectionSample"></span></div>
          <div class="meta"><span class="label">Ref error</span><span class="value" id="selectionError"></span></div>
          <div class="meta"><span class="label">Tracked point</span><span class="value" id="selectionPoint"></span></div>
          <div class="meta"><span class="label">Reference point</span><span class="value" id="selectionRefPoint"></span></div>
          <div class="meta"><span class="label">Error vector</span><span class="value" id="selectionErrorVec"></span></div>
        </div>
      </section>
      <section class="card">
        <h3>3D Path View</h3>
        <div id="plot3d"></div>
        <p class="legend-note">Start markers are circles, end markers are marked with a triangle glyph, and the current slider/hover selection is shown for every visible layer.</p>
      </section>
      <section class="card">
        <div class="footer-links">
          <a id="currentCsvLink" href="#" target="_blank" rel="noopener">Open selected raw CSV</a>
        </div>
      </section>
      <section class="card">
        <h3>Dataset Metrics</h3>
        <table class="dataset-table">
          <thead>
            <tr>
              <th>Dataset</th>
              <th>Ref RMSE [m]</th>
              <th>Shape RMSE [m]</th>
              <th>Samples</th>
              <th>Raw CSV</th>
            </tr>
          </thead>
          <tbody id="datasetRows"></tbody>
        </table>
      </section>
    </main>
  </div>
  <script>
    const bundle = {data_json};
    const caseList = document.getElementById('caseList');
    const caseTabs = document.getElementById('caseTabs');
    const bundleSummary = document.getElementById('bundleSummary');
    const caseTitle = document.getElementById('caseTitle');
    const caseSubtitle = document.getElementById('caseSubtitle');
    const metaDatasetCount = document.getElementById('metaDatasetCount');
    const metaMaxSamples = document.getElementById('metaMaxSamples');
    const metaOutDir = document.getElementById('metaOutDir');
    const datasetRows = document.getElementById('datasetRows');
    const layerControls = document.getElementById('layerControls');
    const trackTargetControls = document.getElementById('trackTargetControls');
    const progressSlider = document.getElementById('progressSlider');
    const progressLabel = document.getElementById('progressLabel');
    const selectionLayer = document.getElementById('selectionLayer');
    const selectionSample = document.getElementById('selectionSample');
    const selectionError = document.getElementById('selectionError');
    const selectionPoint = document.getElementById('selectionPoint');
    const selectionRefPoint = document.getElementById('selectionRefPoint');
    const selectionErrorVec = document.getElementById('selectionErrorVec');
    const currentCsvLink = document.getElementById('currentCsvLink');
    const plotId = 'plot3d';
    const state = {{
      caseName: null,
      visible: {{}},
      trackKey: null,
      sliderIndex: 0,
      layers: [],
      traceMeta: [],
    }};

    function fmt(value, digits = 3) {{
      if (value === null || value === undefined || Number.isNaN(value)) return 'n/a';
      return Number(value).toFixed(digits);
    }}

    function fmtPoint(point) {{
      if (!point || point.length < 3) return 'n/a';
      return `(${{
        fmt(point[0], 2)
      }}, ${{
        fmt(point[1], 2)
      }}, ${{
        fmt(point[2], 2)
      }})`;
    }}

    function fmtErrorVec(values) {{
      if (!values || values.length < 3) return 'n/a';
      return `dx=${{fmt(values[0], 3)}} dy=${{fmt(values[1], 3)}} dz=${{fmt(values[2], 3)}}`;
    }}

    function csvHref(csvB64) {{
      return csvB64 ? `data:text/csv;base64,${{csvB64}}` : '#';
    }}

    function buildLayers(caseData) {{
      const referenceLayer = {{
        key: 'reference',
        label: 'Reference',
        color: '{REF_COLOR}',
        dash: 'dash',
        csv: '',
        samples: caseData.reference_plot.length,
        rmse_m: 0,
        shape_rmse_m: 0,
        points: caseData.reference_plot,
        ref_points: caseData.reference_plot,
        progress: caseData.reference_progress,
        error_xyz: caseData.reference_plot.map(() => [0, 0, 0]),
        error_norm_m: caseData.reference_plot.map(() => 0),
        csv_name: '',
        csv_b64: '',
      }};
      return [referenceLayer].concat(caseData.datasets.map((dataset) => ({{
        key: dataset.key,
        label: dataset.label,
        color: dataset.color,
        dash: dataset.dash,
        csv: dataset.csv,
        samples: dataset.samples,
        rmse_m: dataset.rmse_m,
        shape_rmse_m: dataset.shape_rmse_m,
        points: dataset.position_plot,
        ref_points: dataset.reference_plot,
        progress: dataset.progress,
        error_xyz: dataset.error_xyz,
        error_norm_m: dataset.error_norm_m,
        csv_name: dataset.csv_name,
        csv_b64: dataset.csv_b64,
      }})));
    }}

    function currentCase() {{
      return bundle.cases.find((item) => item.name === state.caseName) || bundle.cases[0];
    }}

    function setActiveCaseButtons(caseName) {{
      [...caseList.children].forEach(btn => btn.classList.toggle('active', btn.dataset.name === caseName));
      [...caseTabs.children].forEach(btn => btn.classList.toggle('active', btn.dataset.name === caseName));
    }}

    function layerByKey(key) {{
      return state.layers.find((item) => item.key === key) || state.layers[0];
    }}

    function nearestPoint(layer, normalizedProgress) {{
      const count = layer.points.length;
      if (!count) {{
        return {{
          index: 0,
          point: null,
          refPoint: null,
          progress: 0,
          errorVec: [0, 0, 0],
          errorNorm: 0,
        }};
      }}
      const idx = Math.max(0, Math.min(count - 1, Math.round(normalizedProgress * Math.max(0, count - 1))));
      return {{
        index: idx,
        point: layer.points[idx],
        refPoint: layer.ref_points[idx],
        progress: layer.progress[idx] || 0,
        errorVec: layer.error_xyz[idx] || [0, 0, 0],
        errorNorm: layer.error_norm_m[idx] || 0,
      }};
    }}

    function trackedProgress() {{
      const target = layerByKey(state.trackKey);
      const maxIndex = Math.max(0, target.points.length - 1);
      const idx = Math.max(0, Math.min(maxIndex, state.sliderIndex));
      if (target.points.length <= 1) return 0;
      return idx / maxIndex;
    }}

    bundleSummary.textContent = `${{bundle.cases.length}} trajectories, ${{bundle.dataset_labels.join(', ')}}`;
    metaDatasetCount.textContent = String(bundle.dataset_labels.length);
    metaMaxSamples.textContent = String(bundle.max_points);
    metaOutDir.textContent = bundle.out_dir;

    function refreshControls(caseData) {{
      layerControls.innerHTML = '';
      trackTargetControls.innerHTML = '';
      state.layers.forEach((layer) => {{
        if (!(layer.key in state.visible)) {{
          state.visible[layer.key] = true;
        }}

        const visibleLabel = document.createElement('label');
        visibleLabel.className = 'chip';
        visibleLabel.innerHTML = `
          <input type="checkbox" data-layer-key="${{layer.key}}" ${{state.visible[layer.key] ? 'checked' : ''}} />
          <span>${{layer.label}}</span>
        `;
        visibleLabel.querySelector('input').addEventListener('change', (event) => {{
          state.visible[layer.key] = Boolean(event.target.checked);
          render(caseData);
        }});
        layerControls.appendChild(visibleLabel);

        const radioLabel = document.createElement('label');
        radioLabel.className = 'chip';
        radioLabel.innerHTML = `
          <input type="radio" name="trackTarget" value="${{layer.key}}" ${{state.trackKey === layer.key ? 'checked' : ''}} />
          <span>${{layer.label}}</span>
        `;
        radioLabel.querySelector('input').addEventListener('change', () => {{
          state.trackKey = layer.key;
          const target = layerByKey(state.trackKey);
          state.sliderIndex = Math.min(state.sliderIndex, Math.max(0, target.points.length - 1));
          progressSlider.max = String(Math.max(0, target.points.length - 1));
          progressSlider.value = String(state.sliderIndex);
          render(caseData);
        }});
        trackTargetControls.appendChild(radioLabel);
      }});
    }}

    function buildTraces(caseData) {{
      const progress = trackedProgress();
      const traces = [];
      state.traceMeta = [];

      state.layers.forEach((layer) => {{
        const selected = nearestPoint(layer, progress);
        const visible = Boolean(state.visible[layer.key]);
        const customData = layer.points.map((point, idx) => ([
          layer.progress[idx] || 0,
          point[0], point[1], point[2],
          layer.ref_points[idx][0], layer.ref_points[idx][1], layer.ref_points[idx][2],
          layer.error_xyz[idx][0], layer.error_xyz[idx][1], layer.error_xyz[idx][2],
          layer.error_norm_m[idx] || 0,
          idx,
        ]));

        traces.push({{
          type: 'scatter3d',
          mode: 'lines',
          name: layer.label,
          legendgroup: layer.key,
          showlegend: true,
          visible: visible,
          x: layer.points.map((p) => p[0]),
          y: layer.points.map((p) => p[1]),
          z: layer.points.map((p) => p[2]),
          customdata: customData,
          hovertemplate:
            '<b>%{{fullData.name}}</b><br>' +
            'Progress: %{{customdata[0]:.1%}}<br>' +
            'Tracked: (%{{customdata[1]:.2f}}, %{{customdata[2]:.2f}}, %{{customdata[3]:.2f}})<br>' +
            'Reference: (%{{customdata[4]:.2f}}, %{{customdata[5]:.2f}}, %{{customdata[6]:.2f}})<br>' +
            'Error: %{{customdata[10]:.3f}} m<extra></extra>',
          line: {{ color: layer.color, width: layer.key === 'reference' ? 7 : 6, dash: layer.dash }},
        }});
        state.traceMeta.push({{ role: 'line', layerKey: layer.key }});

        traces.push({{
          type: 'scatter3d',
          mode: 'markers',
          name: `${{layer.label}} start`,
          legendgroup: layer.key,
          showlegend: false,
          visible: visible,
          x: layer.points.length ? [layer.points[0][0]] : [],
          y: layer.points.length ? [layer.points[0][1]] : [],
          z: layer.points.length ? [layer.points[0][2]] : [],
          marker: {{ size: 8, color: layer.color, symbol: 'circle' }},
          hovertemplate: '<b>%{{fullData.name}}</b><extra></extra>',
        }});
        state.traceMeta.push({{ role: 'start', layerKey: layer.key }});

        traces.push({{
          type: 'scatter3d',
          mode: 'markers+text',
          name: `${{layer.label}} end`,
          legendgroup: layer.key,
          showlegend: false,
          visible: visible,
          x: layer.points.length ? [layer.points[layer.points.length - 1][0]] : [],
          y: layer.points.length ? [layer.points[layer.points.length - 1][1]] : [],
          z: layer.points.length ? [layer.points[layer.points.length - 1][2]] : [],
          text: layer.points.length ? ['▲'] : [],
          textposition: 'top center',
          textfont: {{ color: layer.color, size: 16 }},
          marker: {{ size: 6, color: layer.color, symbol: 'circle-open' }},
          hovertemplate: '<b>%{{fullData.name}}</b><extra></extra>',
        }});
        state.traceMeta.push({{ role: 'end', layerKey: layer.key }});

        traces.push({{
          type: 'scatter3d',
          mode: 'markers',
          name: `${{layer.label}} track`,
          legendgroup: layer.key,
          showlegend: false,
          visible: visible,
          x: selected.point ? [selected.point[0]] : [],
          y: selected.point ? [selected.point[1]] : [],
          z: selected.point ? [selected.point[2]] : [],
          marker: {{ size: 10, color: layer.color, symbol: 'diamond-open', line: {{ color: layer.color, width: 3 }} }},
          hovertemplate: '<b>%{{fullData.name}}</b><extra></extra>',
        }});
        state.traceMeta.push({{ role: 'track', layerKey: layer.key }});
      }});

      const target = layerByKey(state.trackKey);
      if (target.key !== 'reference') {{
        const selected = nearestPoint(target, progress);
        traces.push({{
          type: 'scatter3d',
          mode: 'lines',
          name: 'Reference error segment',
          showlegend: false,
          visible: state.visible[target.key] && state.visible.reference,
          x: selected.point ? [selected.refPoint[0], selected.point[0]] : [],
          y: selected.point ? [selected.refPoint[1], selected.point[1]] : [],
          z: selected.point ? [selected.refPoint[2], selected.point[2]] : [],
          line: {{ color: '#b91c1c', width: 5, dash: 'dot' }},
          hovertemplate: '<b>Reference error segment</b><extra></extra>',
        }});
        state.traceMeta.push({{ role: 'error-bridge', layerKey: target.key }});
      }}
      return traces;
    }}

    function updateSelectionCard(caseData) {{
      const target = layerByKey(state.trackKey);
      const selected = nearestPoint(target, trackedProgress());
      selectionLayer.textContent = target.label;
      selectionSample.textContent = `${{selected.index + 1}} / ${{target.points.length}}`;
      selectionError.textContent = `${{fmt(selected.errorNorm, 3)}} m`;
      selectionPoint.textContent = fmtPoint(selected.point);
      selectionRefPoint.textContent = fmtPoint(selected.refPoint);
      selectionErrorVec.textContent = fmtErrorVec(selected.errorVec);
      progressLabel.textContent = `${{fmt(100.0 * selected.progress, 1)}}%`;
      currentCsvLink.style.display = target.csv ? 'inline-flex' : 'none';
      currentCsvLink.href = target.csv_b64 ? csvHref(target.csv_b64) : '#';
      currentCsvLink.download = target.csv_name || '';
      currentCsvLink.textContent = target.csv_b64 ? `Open ${{target.label}} raw CSV` : 'Reference has no raw CSV';
    }}

    function render(caseData) {{
      state.caseName = caseData.name;
      state.layers = buildLayers(caseData);
      if (!state.trackKey || !state.layers.some((layer) => layer.key === state.trackKey)) {{
        state.trackKey = state.layers.find((layer) => layer.key !== 'reference')?.key || 'reference';
      }}

      refreshControls(caseData);
      const target = layerByKey(state.trackKey);
      const maxIndex = Math.max(0, target.points.length - 1);
      state.sliderIndex = Math.min(state.sliderIndex, maxIndex);
      progressSlider.max = String(maxIndex);
      progressSlider.value = String(state.sliderIndex);

      caseTitle.textContent = caseData.name;
      caseSubtitle.textContent = 'Reference plus available SITL outputs for this trajectory. Hover a curve or drag the slider to see the matching reference location and per-point error.';
      datasetRows.innerHTML = '';
      caseData.datasets.forEach((dataset) => {{
        const row = document.createElement('tr');
        row.innerHTML = `
          <td><strong>${{dataset.label}}</strong></td>
          <td>${{fmt(dataset.rmse_m)}}</td>
          <td>${{fmt(dataset.shape_rmse_m)}}</td>
          <td>${{dataset.samples}}</td>
          <td><a href="${{csvHref(dataset.csv_b64)}}" download="${{dataset.csv_name}}" target="_blank" rel="noopener"><code>${{dataset.csv_name}}</code></a></td>
        `;
        datasetRows.appendChild(row);
      }});

      updateSelectionCard(caseData);
      Plotly.react(plotId, buildTraces(caseData), {{
        margin: {{ l: 0, r: 0, t: 20, b: 0 }},
        paper_bgcolor: 'rgba(0,0,0,0)',
        plot_bgcolor: 'rgba(0,0,0,0)',
        legend: {{ orientation: 'h', itemclick: false, itemdoubleclick: false }},
        hovermode: 'closest',
        uirevision: caseData.name,
        scene: {{
          xaxis: {{ title: 'X [m]' }},
          yaxis: {{ title: 'Y [m]' }},
          zaxis: {{ title: 'Z (up) [m]' }},
          aspectmode: 'data',
          camera: {{ eye: {{ x: 1.35, y: 1.25, z: 0.95 }} }}
        }},
      }}, {{responsive: true}});

      setActiveCaseButtons(caseData.name);
    }}

    progressSlider.addEventListener('input', () => {{
      state.sliderIndex = Number(progressSlider.value);
      render(currentCase());
    }});

    function appendCaseButton(container, caseData, className = '') {{
      const button = document.createElement('button');
      button.dataset.name = caseData.name;
      button.textContent = caseData.name;
      if (className) button.className = className;
      button.addEventListener('click', () => {{
        state.sliderIndex = 0;
        render(caseData);
      }});
      container.appendChild(button);
    }}

    bundle.cases.forEach((caseData, index) => {{
      appendCaseButton(caseList, caseData);
      appendCaseButton(caseTabs, caseData, 'case-tab');
      if (index === 0) render(caseData);
    }});

    document.getElementById(plotId).on('plotly_hover', (event) => {{
      const point = event.points && event.points[0];
      if (!point) return;
      const meta = state.traceMeta[point.curveNumber];
      if (!meta || meta.role !== 'line') return;
      const caseData = currentCase();
      const layer = layerByKey(meta.layerKey);
      if (!layer.points.length) return;
      state.trackKey = meta.layerKey;
      state.sliderIndex = point.pointNumber;
      render(caseData);
    }});
  </script>
</body>
</html>
"""


def build_bundle(
    *,
    stock_root: Path,
    out_dir: Path,
    stock_label: str,
    compare_root: Path | None = None,
    compare_label: str | None = None,
    compare_root_2: Path | None = None,
    compare_label_2: str | None = None,
    max_points: int = DEFAULT_MAX_POINTS,
) -> dict[str, object]:
    if (compare_root is None) != (compare_label is None):
        raise ValueError("--compare-root and --compare-label must be provided together")
    if (compare_root_2 is None) != (compare_label_2 is None):
        raise ValueError("--compare-root-2 and --compare-label-2 must be provided together")
    if compare_root_2 is not None and compare_root is None:
        raise ValueError("second comparison dataset requires the first comparison dataset")

    out_dir.mkdir(parents=True, exist_ok=True)
    plotly_script = _plotly_script_tag()

    datasets = [
        DatasetSpec(key="stock", label=stock_label, root=stock_root, color="#1f77b4", dash="solid"),
    ]
    if compare_root is not None and compare_label is not None:
        datasets.append(
            DatasetSpec(
                key=_safe_key(compare_label),
                label=compare_label,
                root=compare_root,
                color="#d95f02",
                dash="dash",
            )
        )
    if compare_root_2 is not None and compare_label_2 is not None:
        datasets.append(
            DatasetSpec(
                key=_safe_key(compare_label_2),
                label=compare_label_2,
                root=compare_root_2,
                color="#6a3d9a",
                dash="dot",
            )
        )

    cases_payload: list[dict[str, object]] = []
    for case in _case_order():
        dataset_payloads = [
            _dataset_payload(case, spec, out_dir=out_dir, max_points=max_points)
            for spec in datasets
        ]
        cases_payload.append(
            {
                "name": case,
                "reference_plot": dataset_payloads[0]["reference_plot"],
                "reference_progress": dataset_payloads[0]["progress"],
                "datasets": [
                    {
                        key: value
                        for key, value in payload.items()
                        if key not in {"reference_plot"}
                    }
                    for payload in dataset_payloads
                ],
            }
        )

    bundle = {
        "out_dir": str(out_dir.resolve()),
        "max_points": max_points,
        "dataset_labels": [spec.label for spec in datasets],
        "cases": cases_payload,
    }
    (out_dir / "summary.json").write_text(json.dumps(bundle, indent=2), encoding="utf-8")
    (out_dir / "index.html").write_text(_build_html(bundle, plotly_script), encoding="utf-8")
    return bundle


def main() -> int:
    parser = argparse.ArgumentParser(description="Build an interactive SITL trajectory review bundle.")
    parser.add_argument("--stock-root", required=True)
    parser.add_argument("--stock-label", default="Stock x500 SITL")
    parser.add_argument("--compare-root", default="")
    parser.add_argument("--compare-label", default="")
    parser.add_argument("--compare-root-2", default="")
    parser.add_argument("--compare-label-2", default="")
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--max-points", type=int, default=DEFAULT_MAX_POINTS)
    args = parser.parse_args()

    bundle = build_bundle(
        stock_root=Path(args.stock_root).expanduser().resolve(),
        out_dir=Path(args.out_dir).expanduser().resolve(),
        stock_label=args.stock_label,
        compare_root=Path(args.compare_root).expanduser().resolve() if args.compare_root else None,
        compare_label=args.compare_label or None,
        compare_root_2=Path(args.compare_root_2).expanduser().resolve() if args.compare_root_2 else None,
        compare_label_2=args.compare_label_2 or None,
        max_points=args.max_points,
    )
    print(json.dumps({"ok": True, "out_dir": bundle["out_dir"], "cases": len(bundle["cases"])}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
