#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

PANEL_GROUPS = (
    ("circle", "hairpin", "lemniscate"),
    ("time_optimal_30s", "minimum_snap_50s"),
)


def build_pending_group_figures(*, out_dir: Path, compare_label: str, pending_reason: str) -> dict[str, str]:
    out_dir.mkdir(parents=True, exist_ok=True)
    figures: dict[str, str] = {}

    for group_index, cases in enumerate(PANEL_GROUPS, start=1):
        fig, axes = plt.subplots(1, len(cases), figsize=(7.8 * len(cases), 7.0))
        axes = np.atleast_1d(axes).ravel().tolist()

        for ax, case in zip(axes, cases):
            ax.axis("off")
            ax.set_title(case, fontsize=22, pad=18)
            ax.add_patch(plt.Rectangle((0.12, 0.16), 0.76, 0.68, fill=False, linewidth=2.0, transform=ax.transAxes))
            ax.text(
                0.5,
                0.64,
                "Pending comparison",
                ha="center",
                va="center",
                fontsize=22,
                fontweight="bold",
                transform=ax.transAxes,
            )
            ax.text(
                0.5,
                0.48,
                "Reference vs SITL vs "
                f"{compare_label}",
                ha="center",
                va="center",
                fontsize=16,
                transform=ax.transAxes,
            )
            ax.text(
                0.5,
                0.28,
                pending_reason,
                ha="center",
                va="center",
                fontsize=13,
                wrap=True,
                transform=ax.transAxes,
            )

        fig.suptitle(f"SITL vs {compare_label}", fontsize=26)
        fig.subplots_adjust(top=0.82, bottom=0.08, wspace=0.26)
        name = "group_1_circle_hairpin_lemniscate" if group_index == 1 else "group_2_time_optimal_minimum_snap"
        out_path = out_dir / f"{name}.png"
        fig.savefig(out_path, bbox_inches="tight", dpi=300)
        plt.close(fig)
        figures[name] = str(out_path.resolve())

    summary = {
        "status": "pending_data",
        "compare_label": compare_label,
        "pending_reason": pending_reason,
        "figures": figures,
    }
    (out_dir / "comparison_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return figures


def main() -> int:
    ap = argparse.ArgumentParser(description="Generate placeholder grouped figures for a comparison block whose data is not ready yet.")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--compare-label", required=True)
    ap.add_argument("--pending-reason", required=True)
    args = ap.parse_args()

    figures = build_pending_group_figures(
        out_dir=Path(args.out_dir).expanduser().resolve(),
        compare_label=args.compare_label,
        pending_reason=args.pending_reason,
    )
    print(json.dumps(figures, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
