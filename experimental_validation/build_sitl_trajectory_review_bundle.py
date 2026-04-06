from __future__ import annotations

import argparse
import json
import math
import re
import shutil
from dataclasses import dataclass
from pathlib import Path

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


@dataclass(frozen=True)
class DatasetSpec:
    key: str
    label: str
    root: Path
    color: str
    dash: str


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

    rel_raw = Path("raw") / spec.key / f"{case}.csv"
    _copy_raw_csv(csv_path, out_dir / rel_raw)

    return {
        "key": spec.key,
        "label": spec.label,
        "color": spec.color,
        "dash": spec.dash,
        "csv": str(rel_raw),
        "samples": len(ref_trimmed),
        "rmse_m": rmse_raw,
        "shape_rmse_m": shape_rmse,
        "reference_plot": _decimate_points(ref_plot.tolist(), max_points=max_points),
        "position_plot": _decimate_points(pos_plot.tolist(), max_points=max_points),
    }


def _build_html(bundle: dict[str, object]) -> str:
    data_json = json.dumps(bundle, ensure_ascii=True)
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>SITL Trajectory Review</title>
  <script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
  <style>
    :root {{
      --bg: #f3ede3;
      --panel: #fffaf4;
      --ink: #17202a;
      --muted: #5d6d7e;
      --accent: #0f766e;
      --border: #d8cfc0;
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
    #plot3d {{
      min-height: 700px;
    }}
    @media (max-width: 1100px) {{
      .layout {{ grid-template-columns: 1fr; }}
      .meta-grid {{ grid-template-columns: repeat(2, minmax(0, 1fr)); }}
    }}
  </style>
</head>
<body>
  <div class="layout">
    <aside class="sidebar">
      <h1>SITL Trajectory Review</h1>
      <p>Interactive 3D comparison for the five validation trajectories. Rotate, zoom, and pan directly in the browser.</p>
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
        <h3>3D Path View</h3>
        <div id="plot3d"></div>
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
    const bundleSummary = document.getElementById('bundleSummary');
    const caseTitle = document.getElementById('caseTitle');
    const caseSubtitle = document.getElementById('caseSubtitle');
    const metaDatasetCount = document.getElementById('metaDatasetCount');
    const metaMaxSamples = document.getElementById('metaMaxSamples');
    const metaOutDir = document.getElementById('metaOutDir');
    const datasetRows = document.getElementById('datasetRows');

    function fmt(value, digits = 3) {{
      if (value === null || value === undefined || Number.isNaN(value)) return 'n/a';
      return Number(value).toFixed(digits);
    }}

    bundleSummary.textContent = `${{bundle.cases.length}} trajectories, ${{bundle.dataset_labels.join(', ')}}`;
    metaDatasetCount.textContent = String(bundle.dataset_labels.length);
    metaMaxSamples.textContent = String(bundle.max_points);
    metaOutDir.textContent = bundle.out_dir;

    function render(caseData) {{
      caseTitle.textContent = caseData.name;
      caseSubtitle.textContent = 'Reference plus available SITL outputs for this trajectory.';
      datasetRows.innerHTML = '';

      const traces = [{{
        type: 'scatter3d',
        mode: 'lines',
        name: 'Reference',
        x: caseData.reference_plot.map(p => p[0]),
        y: caseData.reference_plot.map(p => p[1]),
        z: caseData.reference_plot.map(p => p[2]),
        line: {{ color: '{REF_COLOR}', width: 7, dash: 'dash' }},
      }}];

      caseData.datasets.forEach((dataset) => {{
        traces.push({{
          type: 'scatter3d',
          mode: 'lines',
          name: dataset.label,
          x: dataset.position_plot.map(p => p[0]),
          y: dataset.position_plot.map(p => p[1]),
          z: dataset.position_plot.map(p => p[2]),
          line: {{ color: dataset.color, width: 6, dash: dataset.dash }},
        }});

        const row = document.createElement('tr');
        row.innerHTML = `
          <td><strong>${{dataset.label}}</strong></td>
          <td>${{fmt(dataset.rmse_m)}}</td>
          <td>${{fmt(dataset.shape_rmse_m)}}</td>
          <td>${{dataset.samples}}</td>
          <td><a href="${{dataset.csv}}" target="_blank" rel="noopener"><code>${{dataset.csv}}</code></a></td>
        `;
        datasetRows.appendChild(row);
      }});

      Plotly.newPlot('plot3d', traces, {{
        margin: {{ l: 0, r: 0, t: 20, b: 0 }},
        paper_bgcolor: 'rgba(0,0,0,0)',
        plot_bgcolor: 'rgba(0,0,0,0)',
        legend: {{ orientation: 'h' }},
        scene: {{
          xaxis: {{ title: 'X [m]' }},
          yaxis: {{ title: 'Y [m]' }},
          zaxis: {{ title: 'Z (up) [m]' }},
          aspectmode: 'data',
          camera: {{ eye: {{ x: 1.35, y: 1.25, z: 0.95 }} }}
        }},
      }}, {{responsive: true}});

      [...caseList.children].forEach(btn => btn.classList.toggle('active', btn.dataset.name === caseData.name));
    }}

    bundle.cases.forEach((caseData, index) => {{
      const button = document.createElement('button');
      button.dataset.name = caseData.name;
      button.textContent = caseData.name;
      button.addEventListener('click', () => render(caseData));
      caseList.appendChild(button);
      if (index === 0) render(caseData);
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
    (out_dir / "index.html").write_text(_build_html(bundle), encoding="utf-8")
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
