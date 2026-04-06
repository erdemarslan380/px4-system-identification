from __future__ import annotations

import argparse
import csv
import json
import math
import re
import shutil
from pathlib import Path


DEFAULT_MAX_POINTS = 4000
SORTIE_PREFIX_RE = re.compile(r"^\d+[_-]+(.+)$")


def _sortie_profile_from_path(csv_path: Path) -> str:
    parent = csv_path.parent
    if parent.name not in {"tracking_logs", "identification_logs", "identification_traces"}:
        return ""
    sortie_name = parent.parent.name
    match = SORTIE_PREFIX_RE.match(sortie_name)
    return match.group(1) if match else ""


def _run_display_name(csv_path: Path, kind: str, profile: str) -> str:
    if profile:
        return f"{profile} [{kind}]"
    parent_name = csv_path.parent.parent.name if csv_path.parent.parent != csv_path.parent else ""
    stem = csv_path.stem
    if parent_name:
        return f"{parent_name} / {stem}"
    return stem


def _read_float(row: dict[str, str], key: str) -> float | None:
    value = row.get(key, "")
    if value is None or value == "":
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def _decimate(values: list[dict[str, float | None]], max_points: int) -> list[dict[str, float | None]]:
    if len(values) <= max_points:
        return values
    step = max(1, math.ceil(len(values) / max_points))
    reduced = values[::step]
    if reduced[-1] is not values[-1]:
        reduced.append(values[-1])
    return reduced


def _load_run(csv_path: Path, kind: str, max_points: int) -> dict[str, object]:
    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)

    samples: list[dict[str, float | None]] = []
    controller = ""
    profile = ""

    for row in rows:
        timestamp_us = _read_float(row, "timestamp_us")
        if timestamp_us is None:
            continue

        controller = row.get("controller", controller) or controller
        profile = row.get("profile", profile) or profile

        samples.append(
            {
                "t": timestamp_us * 1e-6,
                "ref_x": _read_float(row, "ref_x"),
                "ref_y": _read_float(row, "ref_y"),
                "ref_z": _read_float(row, "ref_z"),
                "pos_x": _read_float(row, "pos_x"),
                "pos_y": _read_float(row, "pos_y"),
                "pos_z": _read_float(row, "pos_z"),
            }
        )

    profile = profile or _sortie_profile_from_path(csv_path)
    decimated = _decimate(samples, max_points=max_points)

    duration_s = 0.0
    if samples:
        duration_s = samples[-1]["t"] - samples[0]["t"]  # type: ignore[operator]

    point_pairs = []
    for sample in samples:
        if None not in (sample["ref_x"], sample["ref_y"], sample["ref_z"], sample["pos_x"], sample["pos_y"], sample["pos_z"]):
            dx = float(sample["pos_x"]) - float(sample["ref_x"])
            dy = float(sample["pos_y"]) - float(sample["ref_y"])
            dz = float(sample["pos_z"]) - float(sample["ref_z"])
            point_pairs.append(dx * dx + dy * dy + dz * dz)

    rmse_position_m = math.sqrt(sum(point_pairs) / len(point_pairs)) if point_pairs else None

    return {
        "name": _run_display_name(csv_path, kind, profile),
        "kind": kind,
        "source_csv": str(csv_path),
        "controller": controller,
        "profile": profile,
        "duration_s": duration_s,
        "num_rows": len(samples),
        "rmse_position_m": rmse_position_m,
        "samples": decimated,
    }


def _find_logs(log_root: Path) -> tuple[list[Path], list[Path]]:
    tracking = sorted((log_root / "tracking_logs").glob("*.csv")) if (log_root / "tracking_logs").exists() else []
    ident = []
    if (log_root / "identification_logs").exists():
        ident.extend(sorted((log_root / "identification_logs").glob("*.csv")))
    if (log_root / "identification_traces").exists():
        ident.extend(sorted((log_root / "identification_traces").glob("*.csv")))

    if not tracking and not ident:
        tracking = sorted(log_root.rglob("tracking_logs/*.csv"))
        ident = sorted(log_root.rglob("identification_logs/*.csv")) + sorted(log_root.rglob("identification_traces/*.csv"))

    return tracking, ident


def _copy_raw_logs(
    tracking_logs: list[Path],
    identification_logs: list[Path],
    out_dir: Path,
    *,
    log_root: Path,
) -> dict[str, str]:
    raw_root = out_dir / "raw"
    tracking_out = raw_root / "tracking_logs"
    identification_out = raw_root / "identification_logs"
    identification_trace_out = raw_root / "identification_traces"
    tracking_out.mkdir(parents=True, exist_ok=True)
    identification_out.mkdir(parents=True, exist_ok=True)
    identification_trace_out.mkdir(parents=True, exist_ok=True)

    mapping: dict[str, str] = {}
    for src in tracking_logs:
        try:
            rel = src.relative_to(log_root)
        except ValueError:
            rel = Path("tracking_logs") / src.name
        dst = raw_root / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        mapping[str(src)] = str(dst.relative_to(out_dir))
    for src in identification_logs:
        try:
            rel = src.relative_to(log_root)
        except ValueError:
            dst_dir = identification_trace_out if src.parent.name == "identification_traces" else identification_out
            rel = dst_dir.relative_to(raw_root) / src.name
        dst = raw_root / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        mapping[str(src)] = str(dst.relative_to(out_dir))
    return mapping


def _build_html(bundle: dict[str, object]) -> str:
    data_json = json.dumps(bundle, ensure_ascii=True)
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>HITL Log Review</title>
  <script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
  <style>
    :root {{
      --bg: #f4efe7;
      --panel: #fffaf4;
      --ink: #17202a;
      --muted: #5d6d7e;
      --accent: #0f766e;
      --accent-2: #d97706;
      --border: #d8cfc0;
    }}
    body {{
      margin: 0;
      font-family: "IBM Plex Sans", "Segoe UI", sans-serif;
      color: var(--ink);
      background: linear-gradient(180deg, #efe7dc 0%, var(--bg) 100%);
    }}
    .layout {{
      display: grid;
      grid-template-columns: 320px 1fr;
      min-height: 100vh;
    }}
    .sidebar {{
      padding: 24px;
      border-right: 1px solid var(--border);
      background: rgba(255, 250, 244, 0.92);
      backdrop-filter: blur(6px);
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
    p, li {{
      color: var(--muted);
      line-height: 1.45;
    }}
    .card {{
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 18px;
      padding: 18px;
      box-shadow: 0 10px 30px rgba(23, 32, 42, 0.06);
    }}
    .run-list button {{
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
    .run-list button.active {{
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
    .plots {{
      display: grid;
      grid-template-columns: 1.2fr 1fr;
      gap: 20px;
    }}
    #plot3d, #plotxyz {{
      min-height: 520px;
    }}
    code {{
      background: #f3ede3;
      padding: 2px 6px;
      border-radius: 6px;
    }}
    @media (max-width: 1100px) {{
      .layout {{ grid-template-columns: 1fr; }}
      .plots {{ grid-template-columns: 1fr; }}
      .meta-grid {{ grid-template-columns: repeat(2, minmax(0, 1fr)); }}
    }}
  </style>
</head>
<body>
  <div class="layout">
    <aside class="sidebar">
      <h1>HITL Log Review</h1>
      <p>Interactive 3D inspection for <code>tracking_logs</code>, <code>identification_logs</code>, and <code>identification_traces</code>. Rotate, zoom, and pan directly in the browser.</p>
      <div class="card">
        <h3>Bundle</h3>
        <p id="bundleSummary"></p>
      </div>
      <div class="card">
        <h3>Runs</h3>
        <div id="runList" class="run-list"></div>
      </div>
    </aside>
    <main class="content">
      <section class="card">
        <h2 id="runTitle"></h2>
        <p id="runSubtitle"></p>
        <div class="meta-grid">
          <div class="meta"><span class="label">Type</span><span class="value" id="metaKind"></span></div>
          <div class="meta"><span class="label">Rows</span><span class="value" id="metaRows"></span></div>
          <div class="meta"><span class="label">Duration</span><span class="value" id="metaDuration"></span></div>
          <div class="meta"><span class="label">RMSE</span><span class="value" id="metaRmse"></span></div>
        </div>
      </section>
      <section class="plots">
        <div class="card">
          <h3>3D Path View</h3>
          <div id="plot3d"></div>
        </div>
        <div class="card">
          <h3>X/Y/Z Versus Time</h3>
          <div id="plotxyz"></div>
        </div>
      </section>
      <section class="card">
        <h3>Raw CSV</h3>
        <p id="rawCsvPath"></p>
      </section>
    </main>
  </div>
  <script>
    const bundle = {data_json};
    const runList = document.getElementById('runList');
    const bundleSummary = document.getElementById('bundleSummary');
    const runTitle = document.getElementById('runTitle');
    const runSubtitle = document.getElementById('runSubtitle');
    const metaKind = document.getElementById('metaKind');
    const metaRows = document.getElementById('metaRows');
    const metaDuration = document.getElementById('metaDuration');
    const metaRmse = document.getElementById('metaRmse');
    const rawCsvPath = document.getElementById('rawCsvPath');

    bundleSummary.textContent = `${{bundle.runs.length}} runs loaded from ${{bundle.log_root}}`;

    function fmt(value, digits = 3, suffix = '') {{
      if (value === null || value === undefined || Number.isNaN(value)) return 'n/a';
      return `${{Number(value).toFixed(digits)}}${{suffix}}`;
    }}

    function render(run) {{
      runTitle.textContent = run.name;
      runSubtitle.textContent = run.profile
        ? `${{run.kind}} run, profile ${{run.profile}}`
        : `${{run.kind}} run${{run.controller ? `, controller ${{run.controller}}` : ''}}`;
      metaKind.textContent = run.kind;
      metaRows.textContent = String(run.num_rows);
      metaDuration.textContent = fmt(run.duration_s, 2, ' s');
      metaRmse.textContent = run.rmse_position_m === null ? 'n/a' : fmt(run.rmse_position_m, 3, ' m');
      rawCsvPath.innerHTML = `Raw file: <code>${{run.bundle_csv}}</code>`;

      const refX = [];
      const refY = [];
      const refZ = [];
      const posX = [];
      const posY = [];
      const posZ = [];
      const time = [];

      for (const sample of run.samples) {{
        time.push(sample.t);
        if (sample.ref_x !== null && sample.ref_y !== null && sample.ref_z !== null) {{
          refX.push(sample.ref_x); refY.push(sample.ref_y); refZ.push(sample.ref_z);
        }}
        if (sample.pos_x !== null && sample.pos_y !== null && sample.pos_z !== null) {{
          posX.push(sample.pos_x); posY.push(sample.pos_y); posZ.push(sample.pos_z);
        }}
      }}

      const plot3dData = [];
      if (refX.length) {{
        plot3dData.push({{
          type: 'scatter3d',
          mode: 'lines',
          name: 'Reference',
          x: refX, y: refY, z: refZ,
          line: {{ color: '#0f766e', width: 6 }}
        }});
      }}
      if (posX.length) {{
        plot3dData.push({{
          type: 'scatter3d',
          mode: 'lines',
          name: 'Vehicle',
          x: posX, y: posY, z: posZ,
          line: {{ color: '#d97706', width: 6 }}
        }});
      }}

      Plotly.newPlot('plot3d', plot3dData, {{
        margin: {{ l: 0, r: 0, t: 20, b: 0 }},
        paper_bgcolor: 'rgba(0,0,0,0)',
        plot_bgcolor: 'rgba(0,0,0,0)',
        scene: {{
          xaxis: {{ title: 'X [m]' }},
          yaxis: {{ title: 'Y [m]' }},
          zaxis: {{ title: 'Z [m]' }},
          aspectmode: 'data',
          camera: {{ eye: {{ x: 1.35, y: 1.35, z: 0.9 }} }}
        }},
        legend: {{ orientation: 'h' }}
      }}, {{responsive: true}});

      const xyzData = [];
      const components = [
        ['x', 'X', '#0f766e'],
        ['y', 'Y', '#2563eb'],
        ['z', 'Z', '#b45309'],
      ];
      components.forEach(([axis, label, color], index) => {{
        xyzData.push({{
          type: 'scatter',
          mode: 'lines',
          name: `Ref ${{label}}`,
          x: time,
          y: run.samples.map(s => s[`ref_${{axis}}`]),
          line: {{ color, width: 2, dash: 'dot' }},
          xaxis: `x${{index + 1}}`,
          yaxis: `y${{index + 1}}`,
        }});
        xyzData.push({{
          type: 'scatter',
          mode: 'lines',
          name: `Pos ${{label}}`,
          x: time,
          y: run.samples.map(s => s[`pos_${{axis}}`]),
          line: {{ color, width: 3 }},
          xaxis: `x${{index + 1}}`,
          yaxis: `y${{index + 1}}`,
        }});
      }});

      Plotly.newPlot('plotxyz', xyzData, {{
        margin: {{ l: 50, r: 20, t: 20, b: 40 }},
        paper_bgcolor: 'rgba(0,0,0,0)',
        plot_bgcolor: 'rgba(0,0,0,0)',
        grid: {{ rows: 3, columns: 1, pattern: 'independent' }},
        legend: {{ orientation: 'h' }},
        xaxis3: {{ title: 'Time [s]' }},
        yaxis: {{ title: 'X [m]' }},
        yaxis2: {{ title: 'Y [m]' }},
        yaxis3: {{ title: 'Z [m]' }},
      }}, {{responsive: true}});

      [...runList.children].forEach(btn => btn.classList.toggle('active', btn.dataset.name === run.name));
    }}

    bundle.runs.forEach((run, index) => {{
      const button = document.createElement('button');
      button.dataset.name = run.name;
      button.textContent = `${{run.name}} (${{run.kind}})`;
      button.addEventListener('click', () => render(run));
      runList.appendChild(button);
      if (index === 0) render(run);
    }});
  </script>
</body>
</html>
"""


def build_bundle(log_root: Path, out_dir: Path, max_points: int) -> dict[str, object]:
    tracking_logs, identification_logs = _find_logs(log_root)
    if not tracking_logs and not identification_logs:
        raise FileNotFoundError(
            f"No tracking_logs, identification_logs, or identification_traces CSV files found under {log_root}"
        )

    out_dir.mkdir(parents=True, exist_ok=True)
    raw_map = _copy_raw_logs(tracking_logs, identification_logs, out_dir, log_root=log_root)

    runs = []
    for path in tracking_logs:
        run = _load_run(path, kind="tracking", max_points=max_points)
        run["bundle_csv"] = raw_map[str(path)]
        runs.append(run)
    for path in identification_logs:
        run = _load_run(path, kind="identification", max_points=max_points)
        run["bundle_csv"] = raw_map[str(path)]
        runs.append(run)

    runs.sort(key=lambda item: (item["kind"], item["name"]))  # type: ignore[index]

    bundle = {
        "log_root": str(log_root),
        "out_dir": str(out_dir),
        "runs": runs,
    }

    (out_dir / "summary.json").write_text(json.dumps(bundle, indent=2), encoding="utf-8")
    (out_dir / "index.html").write_text(_build_html(bundle), encoding="utf-8")
    return bundle


def main() -> int:
    parser = argparse.ArgumentParser(description="Build an interactive HITL log review bundle from SD-card CSV logs.")
    parser.add_argument(
        "--log-root",
        required=True,
        help="Directory containing tracking_logs/ and optionally identification_logs/ or identification_traces/.",
    )
    parser.add_argument("--out-dir", required=True, help="Output directory for index.html, summary.json, and copied raw CSVs.")
    parser.add_argument("--max-points", type=int, default=DEFAULT_MAX_POINTS, help="Maximum plotted samples per run after decimation.")
    args = parser.parse_args()

    bundle = build_bundle(Path(args.log_root).expanduser().resolve(), Path(args.out_dir).expanduser().resolve(), args.max_points)
    print(json.dumps({"ok": True, "out_dir": bundle["out_dir"], "num_runs": len(bundle["runs"])}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
