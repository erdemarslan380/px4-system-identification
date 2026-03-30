#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import subprocess
from pathlib import Path
from typing import Iterable

from pyulog import ULog


ISSUE_PATTERNS = {
    "ram_usage_too_high": "RAM usage too high",
    "low_on_stack": "low on stack",
    "parameter_verify_failed": "verify: failed",
    "baro_stale": "BARO #0 failed",
    "failsafe_activated": "Failsafe activated",
}


def summarize_percentages(values: Iterable[float]) -> dict[str, float | int | None]:
    items = [float(v) for v in values]
    if not items:
        return {
            "samples": 0,
            "min_pct": None,
            "mean_pct": None,
            "max_pct": None,
        }

    return {
        "samples": len(items),
        "min_pct": min(items),
        "mean_pct": sum(items) / len(items),
        "max_pct": max(items),
    }


def summarize_ram_usage(ram_usage_pct: Iterable[float]) -> dict[str, float | int | str | None]:
    items = [float(v) for v in ram_usage_pct]
    summary = summarize_percentages(items)
    valid_items = [value for value in items if 0.0 <= value <= 100.0]

    status = "unavailable"
    if items:
        status = "valid" if len(valid_items) == len(items) else "invalid_spike_detected"

    summary.update(
        {
            "status": status,
            "valid_samples": len(valid_items),
            "valid_min_pct": min(valid_items) if valid_items else None,
            "valid_mean_pct": (sum(valid_items) / len(valid_items)) if valid_items else None,
            "valid_max_pct": max(valid_items) if valid_items else None,
        }
    )
    return summary


def extract_issue_counts(messages: Iterable[str]) -> dict[str, int]:
    materialized = list(messages)
    return {
        name: sum(1 for message in materialized if pattern in message)
        for name, pattern in ISSUE_PATTERNS.items()
    }


def host_process_snapshot(pid: int) -> dict[str, float | int | str]:
    result = subprocess.run(
        ["ps", "-p", str(pid), "-o", "pid=,etime=,%cpu=,%mem=,rss=,vsz=,args="],
        capture_output=True,
        text=True,
        check=True,
    )
    line = result.stdout.strip()
    if not line:
        raise RuntimeError(f"PID {pid} not found")
    pid_s, etime, cpu_s, mem_s, rss_s, vsz_s, args = line.split(maxsplit=6)
    return {
        "pid": int(pid_s),
        "elapsed": etime,
        "cpu_pct": float(cpu_s),
        "mem_pct": float(mem_s),
        "rss_kib": int(rss_s),
        "vsz_kib": int(vsz_s),
        "command": args,
    }


def analyze_ulg(path: Path) -> dict[str, object]:
    ulog = ULog(str(path))
    cpuload = next((dataset for dataset in ulog.data_list if dataset.name == "cpuload"), None)
    if cpuload is None:
        raise RuntimeError(f"{path} does not contain a cpuload topic")

    cpu_load_pct = [float(value) * 100.0 for value in cpuload.data["load"]]
    ram_usage_pct = [float(value) * 100.0 for value in cpuload.data["ram_usage"]]
    logged_messages = [message.message for message in ulog.logged_messages]

    return {
        "source": str(path),
        "cpu_load": summarize_percentages(cpu_load_pct),
        "ram_usage": summarize_ram_usage(ram_usage_pct),
        "issue_counts": extract_issue_counts(logged_messages),
        "logged_messages_sample": logged_messages[:12],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize HIL RAM/CPU usage from PX4 ULog files.")
    parser.add_argument(
        "--ulg",
        dest="ulgs",
        action="append",
        required=True,
        help="Path to a PX4 ULog file. Pass multiple times for multiple HIL runs.",
    )
    parser.add_argument(
        "--host-pid",
        type=int,
        default=None,
        help="Optional host-side process PID (for example the jMAVSim Java PID).",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Optional JSON output path.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = {
        "hil_runs": [analyze_ulg(Path(path).expanduser().resolve()) for path in args.ulgs],
    }

    if args.host_pid is not None:
        report["host_process"] = host_process_snapshot(args.host_pid)

    serialized = json.dumps(report, indent=2)
    print(serialized)

    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(serialized + "\n", encoding="utf-8")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
