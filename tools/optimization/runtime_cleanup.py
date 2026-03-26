#!/usr/bin/env python3
"""Helpers for cleaning stale PX4 and simulator runtime slots."""

from __future__ import annotations

import os
import re
import signal
import subprocess
import time
from pathlib import Path
from typing import Iterable


def _ps_rows() -> list[tuple[int, str]]:
    out = subprocess.check_output(["ps", "-eo", "pid,args"], text=True)
    rows: list[tuple[int, str]] = []
    for line in out.splitlines()[1:]:
        parts = line.strip().split(None, 1)
        if len(parts) != 2:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        rows.append((pid, parts[1]))
    return rows


def _kill_pid_or_group(pid: int, sig: int) -> None:
    try:
        pgid = os.getpgid(pid)
    except ProcessLookupError:
        return
    try:
        if pgid > 0:
            os.killpg(pgid, sig)
        else:
            os.kill(pid, sig)
    except ProcessLookupError:
        return


def _wait_gone(pids: set[int], timeout_s: float) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        alive = [pid for pid in pids if os.path.exists(f"/proc/{pid}")]
        if not alive:
            return
        time.sleep(0.15)


def _listener_pids_for_ports(ports: Iterable[int]) -> set[int]:
    ports = {int(port) for port in ports}
    if not ports:
        return set()
    try:
        out = subprocess.check_output(["ss", "-ltnp"], text=True, stderr=subprocess.DEVNULL)
    except Exception:
        return set()
    pids: set[int] = set()
    for line in out.splitlines():
        if not any(re.search(rf":{port}\b", line) for port in ports):
            continue
        for raw_pid in re.findall(r"pid=(\d+)", line):
            try:
                pids.add(int(raw_pid))
            except ValueError:
                continue
    return pids


def _jmavsim_launcher_pids(repo_root: Path, ports: Iterable[int]) -> set[int]:
    port_set = {int(port) for port in ports}
    if not port_set:
        return set()
    rows = _ps_rows()
    pids: set[int] = set()
    script_hint = str((repo_root / "Tools" / "simulation" / "jmavsim" / "jmavsim_run.sh").resolve())
    for pid, cmd in rows:
        if script_hint not in cmd:
            continue
        if any(re.search(rf"(?:^|\s)-p\s+{port}(?:\s|$)", cmd) for port in port_set):
            pids.add(pid)
    return pids


def _px4_instance_pids(instance_ids: Iterable[int]) -> set[int]:
    instances = {int(instance_id) for instance_id in instance_ids}
    if not instances:
        return set()
    rows = _ps_rows()
    pids: set[int] = set()
    for pid, cmd in rows:
        if "/bin/px4" not in cmd:
            continue
        if any(re.search(rf"(?:^|\s)-i\s+{instance_id}(?:\s|$)", cmd) for instance_id in instances):
            pids.add(pid)
    return pids


def _cmdline_hint_pids(hints: Iterable[str], exclude_pids: Iterable[int] = ()) -> set[int]:
    hint_list = [str(item).strip() for item in hints if str(item).strip()]
    if not hint_list:
        return set()
    excluded = {int(pid) for pid in exclude_pids if int(pid) > 0}
    pids: set[int] = set()
    for pid, cmd in _ps_rows():
        if pid in excluded:
            continue
        if any(hint in cmd for hint in hint_list):
            pids.add(pid)
    return pids


def cleanup_runtime_slots(
    repo_root: Path,
    instance_ids: Iterable[int],
    ports: Iterable[int],
    process_hints: Iterable[str] = (),
    wait_timeout_s: float = 6.0,
) -> dict:
    instance_ids = [int(value) for value in instance_ids]
    ports = [int(value) for value in ports]

    targets = (
        _px4_instance_pids(instance_ids)
        | _jmavsim_launcher_pids(repo_root, ports)
        | _listener_pids_for_ports(ports)
        | _cmdline_hint_pids(process_hints, exclude_pids=(os.getpid(), os.getppid()))
    )

    if not targets:
        return {"killed": 0, "instance_ids": instance_ids, "ports": ports}

    for sig in (signal.SIGTERM, signal.SIGKILL):
        for pid in sorted(targets):
            _kill_pid_or_group(pid, sig)
        _wait_gone(targets, wait_timeout_s if sig == signal.SIGTERM else 2.0)

    return {"killed": len(targets), "instance_ids": instance_ids, "ports": ports}
