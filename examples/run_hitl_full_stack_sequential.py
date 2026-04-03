#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.hitl_catalog import CAMPAIGNS
from pull_sdcard_logs_over_mavftp import REMOTE_GROUPS, collect_remote_files, download_file_via_port


LOGGER_RE = re.compile(r"/fs/microsd/log/\d{4}-\d{2}-\d{2}/[0-9_]+\.ulg")


@dataclass
class SegmentResult:
    name: str
    kind: str
    started_at: str
    finished_at: str | None = None
    return_code: int | None = None
    success: bool = False
    ulg_paths: list[str] = field(default_factory=list)
    new_tracking_logs: list[str] = field(default_factory=list)
    new_identification_logs: list[str] = field(default_factory=list)
    log_file: str | None = None


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Run the full 9-ident + 5-trajectory HIL stack sequentially with clean GUI restarts."
    )
    p.add_argument("--campaign", choices=sorted(CAMPAIGNS), default="full_stack")
    p.add_argument("--px4-root", default="/home/earsub/PX4-Autopilot-Identification")
    p.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument(
        "--output-root",
        default=str(REPO_ROOT / "hitl_runs" / f"full_stack_{datetime.now().strftime('%Y%m%d_%H%M%S')}"),
    )
    p.add_argument("--custom-hold-seconds", type=float, default=15.0)
    p.add_argument("--trajectory-tail-seconds", type=float, default=10.0)
    p.add_argument("--ident-tail-seconds", type=float, default=8.0)
    p.add_argument("--continue-on-error", action="store_true")
    return p.parse_args()


def run_and_tee(cmd: list[str], *, cwd: str, log_path: Path) -> tuple[int, list[str]]:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as logf:
        proc = subprocess.Popen(
            cmd,
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        ulg_paths: list[str] = []
        assert proc.stdout is not None
        for line in proc.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
            logf.write(line)
            for match in LOGGER_RE.findall(line):
                if match not in ulg_paths:
                    ulg_paths.append(match)
        proc.wait()
        return proc.returncode, ulg_paths


def list_local_csvs(root: Path) -> tuple[set[str], set[str]]:
    tracking = set(str(p.relative_to(root)) for p in (root / "tracking_logs").glob("*.csv")) if (root / "tracking_logs").exists() else set()
    ident = set(str(p.relative_to(root)) for p in (root / "identification_logs").glob("*.csv")) if (root / "identification_logs").exists() else set()
    return tracking, ident


def remote_csv_state(endpoint: str, baud: int = 57600) -> dict[str, dict[str, int | None]]:
    grouped = collect_remote_files(
        endpoint,
        baud,
        heartbeat_timeout=10.0,
        remote_groups=REMOTE_GROUPS,
        suffixes=(".csv",),
    )
    return {group: {name: size for name, size in files} for group, files in grouped.items()}


def download_remote_diff(
    *,
    endpoint: str,
    destination: Path,
    before: dict[str, dict[str, int | None]],
    after: dict[str, dict[str, int | None]],
    baud: int = 57600,
) -> tuple[list[str], list[str]]:
    pulled_tracking: list[str] = []
    pulled_ident: list[str] = []
    destination.mkdir(parents=True, exist_ok=True)

    for local_group, remote_dir in REMOTE_GROUPS.items():
        local_dir = destination / local_group
        local_dir.mkdir(parents=True, exist_ok=True)
        before_group = before.get(local_group, {})
        after_group = after.get(local_group, {})
        target_list = pulled_tracking if local_group == "tracking_logs" else pulled_ident

        for file_name, expected_size in sorted(after_group.items()):
            if before_group.get(file_name) == expected_size:
                continue
            remote_path = f"{remote_dir}/{file_name}"
            local_path = local_dir / file_name
            download_file_via_port(
                port=endpoint,
                baud=baud,
                heartbeat_timeout=10.0,
                remote_path=remote_path,
                local_path=local_path,
                expected_size=expected_size,
                retries=2,
                timeout=30.0,
            )
            target_list.append(str(local_path.relative_to(destination)))

    return pulled_tracking, pulled_ident


def restart_clean(px4_root: str, baud: int) -> None:
    cmd = [
        str(REPO_ROOT / "examples" / "restart_hitl_px4_clean_gui.sh"),
        px4_root,
        str(baud),
    ]
    subprocess.run(cmd, cwd=str(REPO_ROOT), check=True)
    time.sleep(3.0)


def log_implies_success(log_path: Path, kind: str) -> bool:
    if not log_path.exists():
        return False
    text = log_path.read_text(encoding="utf-8", errors="replace")
    if kind == "ident":
        return "Built-in identification summary:" in text and "Built-in identification unstable" not in text
    if kind == "trajectory":
        return "Built-in trajectory summary:" in text and "Built-in trajectory unstable" not in text
    return False


def ident_cmd(endpoint: str, profile: str, custom_hold_seconds: float, ident_tail_seconds: float) -> list[str]:
    return [
        sys.executable,
        str(REPO_ROOT / "examples" / "run_hitl_px4_builtin_ident_minimal.py"),
        "--endpoint",
        endpoint,
        "--profile",
        profile,
        "--custom-hold-seconds",
        str(custom_hold_seconds),
        "--ident-tail-seconds",
        str(ident_tail_seconds),
    ]


def traj_cmd(endpoint: str, traj_id: int, custom_hold_seconds: float, trajectory_tail_seconds: float) -> list[str]:
    return [
        sys.executable,
        str(REPO_ROOT / "examples" / "run_hitl_px4_builtin_trajectory_minimal.py"),
        "--endpoint",
        endpoint,
        "--traj-id",
        str(traj_id),
        "--custom-hold-seconds",
        str(custom_hold_seconds),
        "--trajectory-tail-seconds",
        str(trajectory_tail_seconds),
    ]


def write_manifest(path: Path, results: list[SegmentResult]) -> None:
    path.write_text(json.dumps([asdict(r) for r in results], indent=2), encoding="utf-8")


def main() -> int:
    args = parse_args()
    output_root = Path(args.output_root).expanduser()
    output_root.mkdir(parents=True, exist_ok=True)
    results: list[SegmentResult] = []
    manifest_path = output_root / "manifest.json"

    plan: list[tuple[str, str, list[str]]] = []
    for profile in CAMPAIGNS[args.campaign]["ident_profiles"]:
        plan.append((f"ident_{profile}", "ident", ident_cmd(args.endpoint, profile, args.custom_hold_seconds, args.ident_tail_seconds)))
    for traj_id in CAMPAIGNS[args.campaign]["trajectory_ids"]:
        plan.append((f"traj_{traj_id}", "trajectory", traj_cmd(args.endpoint, traj_id, args.custom_hold_seconds, args.trajectory_tail_seconds)))

    print(f"Running {len(plan)} segments into {output_root}", flush=True)

    for name, kind, cmd in plan:
        result = SegmentResult(name=name, kind=kind, started_at=datetime.now().isoformat())
        results.append(result)
        write_manifest(manifest_path, results)

        restart_clean(args.px4_root, args.baud)
        try:
            remote_before = remote_csv_state(args.endpoint)
        except Exception as exc:
            print(f"Warning: remote CSV listing failed before {name}: {exc}", flush=True)
            remote_before = {group: {} for group in REMOTE_GROUPS}

        log_path = output_root / "console_logs" / f"{name}.log"
        result.log_file = str(log_path)

        rc, ulg_paths = run_and_tee(cmd, cwd=str(REPO_ROOT), log_path=log_path)
        result.return_code = rc
        result.ulg_paths = ulg_paths

        try:
            restart_clean(args.px4_root, args.baud)
            remote_after = remote_csv_state(args.endpoint)
            pulled_tracking, pulled_ident = download_remote_diff(
                endpoint=args.endpoint,
                destination=output_root,
                before=remote_before,
                after=remote_after,
            )
        except Exception as exc:
            print(f"Warning: remote diff pull failed after {name}: {exc}", flush=True)
            pulled_tracking, pulled_ident = [], []

        result.new_tracking_logs = pulled_tracking
        result.new_identification_logs = pulled_ident
        result.finished_at = datetime.now().isoformat()
        result.success = (rc == 0) or log_implies_success(log_path, kind)
        write_manifest(manifest_path, results)

        if not result.success and not args.continue_on_error:
            print(f"Stopping on failure in {name}", flush=True)
            return rc if rc != 0 else 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
