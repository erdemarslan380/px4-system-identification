#!/usr/bin/env python3
"""Run a one-shot visual or headless replay for manual inspection from the dashboard."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, Optional

from controller_profiles import (
    ControllerProfile,
    ensure_default_param_file,
    get_controller_profile,
    parse_param_file,
    write_param_file,
)
from px4_eval import Px4Ctl, get_local_position_sample
from simulator_backend import (
    DEFAULT_GZ_VEHICLE,
    DEFAULT_GZ_WORLD,
    build_simulator_launch_spec,
    prepare_rootfs_simulator_assets,
    simulator_state_fields,
    wait_for_simulator_ready,
)


def install_signal_handlers() -> None:
    def _handler(_signum, _frame) -> None:
        raise SystemExit(130)
    signal.signal(signal.SIGTERM, _handler)
    signal.signal(signal.SIGINT, _handler)


def write_state(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.tmp")
    tmp.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    tmp.replace(path)


def read_json(path: Path) -> dict | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def append_fallback_trace(px4: Px4Ctl,
                          live_trace_json: Path,
                          sample_trace: dict,
                          takeoff_alt: float,
                          done: bool,
                          message: str) -> bool:
    existing = read_json(live_trace_json) or {}
    existing_t = existing.get("t", [])
    if existing.get("source") != "fallback_live" and isinstance(existing_t, list) and len(existing_t) > 0:
        return True

    sample = get_local_position_sample(px4)
    now_s = time.time() - float(sample_trace["t0"])
    last_t = sample_trace["t"][-1] if sample_trace["t"] else None
    if last_t is None or now_s - last_t >= 0.22:
        sample_trace["t"].append(now_s)
        sample_trace["ref"]["x"].append(0.0)
        sample_trace["ref"]["y"].append(0.0)
        sample_trace["ref"]["z"].append(-abs(float(takeoff_alt)))
        sample_trace["act"]["x"].append(float(sample["x"]))
        sample_trace["act"]["y"].append(float(sample["y"]))
        sample_trace["act"]["z"].append(float(sample["z"]))

        keep = 1200
        if len(sample_trace["t"]) > keep:
            for key in ("t",):
                sample_trace[key] = sample_trace[key][-keep:]
            for axis in ("x", "y", "z"):
                sample_trace["ref"][axis] = sample_trace["ref"][axis][-keep:]
                sample_trace["act"][axis] = sample_trace["act"][axis][-keep:]

    write_state(live_trace_json, {
        "eval_id": 0,
        "worker_id": -1,
        "done": bool(done),
        "message": message,
        "fitness": None,
        "source": "fallback_live",
        "t": list(sample_trace["t"]),
        "ref": {
            "x": list(sample_trace["ref"]["x"]),
            "y": list(sample_trace["ref"]["y"]),
            "z": list(sample_trace["ref"]["z"]),
        },
        "act": {
            "x": list(sample_trace["act"]["x"]),
            "y": list(sample_trace["act"]["y"]),
            "z": list(sample_trace["act"]["z"]),
        },
    })
    return False


def write_placeholder_trace(live_trace_json: Path, message: str, done: bool = False) -> None:
    write_state(live_trace_json, {
        "eval_id": 0,
        "worker_id": -1,
        "done": bool(done),
        "message": message,
        "fitness": None,
        "source": "strict_wait",
        "t": [],
        "ref": {"x": [], "y": [], "z": []},
        "act": {"x": [], "y": [], "z": []},
    })


def copy_tree_contents(src: Path, dst: Path, ignore=None) -> None:
    if not src.exists():
        return
    for item in src.iterdir():
        if ignore is not None and item.name in ignore(src, [item.name]):
            continue
        target = dst / item.name
        if item.is_dir():
            shutil.copytree(item, target, dirs_exist_ok=True, ignore=ignore)
        elif item.is_symlink():
            if target.exists() or target.is_symlink():
                target.unlink()
            target.symlink_to(item.resolve())
        else:
            shutil.copy2(item, target)


def resolve_trajectory_source(repo_root: Path, base_rootfs: Path) -> Path | None:
    if (base_rootfs / "trajectories").exists():
        return base_rootfs / "trajectories"

    fallback = repo_root / "build" / "px4_sitl_default" / "rootfs" / "trajectories"
    if fallback.exists():
        return fallback

    return None


def prepare_rootfs(
    repo_root: Path,
    build_dir: Path,
    base_rootfs: Path,
    rootfs: Path,
    profile: ControllerProfile,
    simulator: str,
) -> Path:
    shutil.rmtree(rootfs, ignore_errors=True)
    rootfs.mkdir(parents=True, exist_ok=True)
    (rootfs / "log").mkdir(parents=True, exist_ok=True)
    (rootfs / "eeprom").mkdir(parents=True, exist_ok=True)
    (rootfs / "parameters").mkdir(parents=True, exist_ok=True)
    (rootfs / "trajectories" / "tracking_logs").mkdir(parents=True, exist_ok=True)

    etc_link = rootfs / "etc"
    if etc_link.exists() or etc_link.is_symlink():
        etc_link.unlink()
    etc_link.symlink_to((build_dir / "etc").resolve())

    prepare_rootfs_simulator_assets(repo_root, build_dir, base_rootfs, rootfs, simulator)

    if (base_rootfs / "dataman").exists():
        shutil.copy2(base_rootfs / "dataman", rootfs / "dataman")
    if (base_rootfs / "parameters").exists():
        copy_tree_contents(base_rootfs / "parameters", rootfs / "parameters")
    trajectory_src = resolve_trajectory_source(repo_root, base_rootfs)
    if trajectory_src is not None:
        shutil.copytree(
            trajectory_src,
            rootfs / "trajectories",
            dirs_exist_ok=True,
            ignore=shutil.ignore_patterns("tracking_logs"),
        )
        (rootfs / "trajectories" / "tracking_logs").mkdir(parents=True, exist_ok=True)

    params_path = rootfs / "parameters" / profile.param_file_name
    if not params_path.exists():
        ensure_default_param_file(rootfs, profile)
    return params_path


def terminate_proc(proc: Optional[subprocess.Popen], timeout_s: float = 8.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=timeout_s)
    except Exception:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=3.0)
        except Exception:
            pass


def wait_px4_ready(rootfs: Path, instance_id: int, timeout_s: float = 50.0) -> None:
    bin_dir = rootfs.parent / "bin"
    cmd = [str(bin_dir / "px4-commander"), "--instance", str(instance_id), "status"]
    t0 = time.time()
    last_error = ""
    ready_hits = 0
    while time.time() - t0 < timeout_s:
        try:
            proc = subprocess.run(
                cmd,
                cwd=str(rootfs),
                capture_output=True,
                text=True,
                timeout=4.0,
            )
            out = (proc.stdout or "") + (proc.stderr or "")
            if proc.returncode == 0 and ("navigation mode" in out or "armed:" in out):
                ready_hits += 1
                if ready_hits >= 2:
                    time.sleep(0.8)
                    return
            else:
                ready_hits = 0
            last_error = out.strip()
        except Exception as exc:
            ready_hits = 0
            last_error = str(exc)
        time.sleep(0.6)
    raise RuntimeError(f"PX4 not ready in {timeout_s:.1f}s: {last_error}")


def compute_replay_runtime_profile(*,
                                   simulator: str,
                                   mission_mode: str,
                                   strict_requested: bool,
                                   effective_headless: bool,
                                   takeoff_timeout: float,
                                   trajectory_timeout: float) -> dict:
    simulator = str(simulator or "gz").strip().lower()
    mission_mode = str(mission_mode or "trajectory").strip().lower()
    strict_effective = bool(strict_requested)
    takeoff_budget = float(takeoff_timeout)
    trajectory_budget = float(trajectory_timeout)
    note = ""

    # GUI Gazebo replay is used for visual inspection while other workers may
    # still be running. Inflate budgets so rendering and transport jitter does
    # not cause spurious takeoff failures.
    if simulator == "gz" and not effective_headless:
        takeoff_budget = max(takeoff_budget, 60.0)
        trajectory_budget = max(trajectory_budget, 240.0 if mission_mode == "identification" else 180.0)

    # Identification replay is primarily used to inspect the full motion in the
    # simulator. Under Gazebo GUI, strict OFFBOARD-only matching is more brittle
    # than the underlying optimizer worker path because rendering load changes
    # timing. Prefer a relaxed visual replay for reliability.
    if simulator == "gz" and not effective_headless and mission_mode == "identification" and strict_requested:
        strict_effective = False
        note = "Strict identification replay relaxed under Gazebo GUI for visual stability."

    return {
        "strict_effective": strict_effective,
        "takeoff_timeout": takeoff_budget,
        "trajectory_timeout": trajectory_budget,
        "note": note,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Run one visual replay for a chosen parameter set")
    ap.add_argument("--rootfs", default="build/px4_sitl_default/rootfs")
    ap.add_argument("--build-dir", default="")
    ap.add_argument("--controller", choices=("dfbc", "pid", "indi", "mpc", "cmpc", "sysid"), required=True)
    ap.add_argument("--traj-id", type=int, required=True)
    ap.add_argument("--params-json", required=True)
    ap.add_argument("--fixed-params-json", default="{}")
    ap.add_argument("--state-json", required=True)
    ap.add_argument("--result-json", required=True)
    ap.add_argument("--live-trace-json", required=True)
    ap.add_argument("--takeoff-alt", type=float, default=2.0)
    ap.add_argument("--takeoff-timeout", type=float, default=40.0)
    ap.add_argument("--trajectory-timeout", type=float, default=180.0)
    ap.add_argument("--w-track", type=float, default=1.0)
    ap.add_argument("--w-energy", type=float, default=0.05)
    ap.add_argument("--instance-id", type=int, default=40)
    ap.add_argument("--tcp-port", type=int, default=4600)
    ap.add_argument("--sim-speed-factor", type=float, default=1.0)
    ap.add_argument("--simulator", choices=("gz", "jmavsim"), default="gz")
    ap.add_argument("--simulator-vehicle", default=DEFAULT_GZ_VEHICLE)
    ap.add_argument("--simulator-world", default=DEFAULT_GZ_WORLD)
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--strict-eval", action="store_true")
    ap.add_argument("--auto-close-after-finish", action="store_true")
    ap.add_argument("--landing-mode", choices=("land", "rtl"), default="land")
    ap.add_argument("--trace-window", choices=("offboard",), default="offboard")
    ap.add_argument("--engagement-dwell-s", type=float, default=2.0)
    ap.add_argument("--mission-mode", choices=("trajectory", "identification"), default="trajectory")
    ap.add_argument("--ident-profile", default="hover_thrust")
    ap.add_argument("--base-param-file", default="")
    args = ap.parse_args()
    install_signal_handlers()

    repo_root = Path(__file__).resolve().parents[2]
    tool_root = Path(__file__).resolve().parent
    base_rootfs = (repo_root / args.rootfs).resolve() if not Path(args.rootfs).is_absolute() else Path(args.rootfs).resolve()
    if args.build_dir:
        build_dir = ((repo_root / args.build_dir).resolve()
                     if not Path(args.build_dir).is_absolute()
                     else Path(args.build_dir).resolve())
    else:
        build_dir = base_rootfs.parent.resolve()
    replay_root = build_dir / f"visual_replay_{args.instance_id:03d}"
    profile = get_controller_profile(args.controller)
    state_json = Path(args.state_json).resolve()
    result_json = Path(args.result_json).resolve()
    live_trace_json = Path(args.live_trace_json).resolve()

    params = json.loads(args.params_json)
    fixed_params = json.loads(args.fixed_params_json)
    requested_headless = bool(args.headless)
    effective_headless = requested_headless or not bool(os.environ.get("DISPLAY"))
    display_mode = "headless" if effective_headless else "gui"
    if not requested_headless and effective_headless:
        display_mode = "headless_fallback"
    runtime_profile = compute_replay_runtime_profile(
        simulator=args.simulator,
        mission_mode=args.mission_mode,
        strict_requested=bool(args.strict_eval),
        effective_headless=effective_headless,
        takeoff_timeout=float(args.takeoff_timeout),
        trajectory_timeout=float(args.trajectory_timeout),
    )

    sim_proc: Optional[subprocess.Popen] = None
    gui_proc: Optional[subprocess.Popen] = None
    px4_proc: Optional[subprocess.Popen] = None
    eval_proc: Optional[subprocess.Popen] = None
    eval_fp = None
    sample_trace = {
        "t0": time.time(),
        "t": [],
        "ref": {"x": [], "y": [], "z": []},
        "act": {"x": [], "y": [], "z": []},
    }
    saw_real_trace = False
    try:
        for artifact in (result_json, live_trace_json):
            try:
                artifact.unlink()
            except FileNotFoundError:
                pass

        params_file = prepare_rootfs(repo_root, build_dir, base_rootfs, replay_root, profile, args.simulator)
        merged_params = parse_param_file(params_file, profile)
        for name, value in params.items():
            merged_params[str(name)] = float(value)
        write_state(state_json, {
            "status": "starting",
            "controller": args.controller,
            "traj_id": args.traj_id,
            "started_at": time.time(),
            "headless": effective_headless,
            "headless_requested": requested_headless,
            "display_mode": display_mode,
            "sim_speed_factor": float(args.sim_speed_factor),
            "simulator": args.simulator,
            "simulator_vehicle": args.simulator_vehicle,
            "simulator_world": args.simulator_world,
            "strict_requested": bool(args.strict_eval),
            "strict_effective": bool(runtime_profile["strict_effective"]),
            "runtime_note": runtime_profile["note"],
            "mission_mode": args.mission_mode,
            "ident_profile": args.ident_profile,
            "base_param_file": args.base_param_file,
            "params": merged_params,
            "fixed_params": fixed_params,
        })
        write_param_file(params_file, profile, merged_params, header="Manual replay parameters")

        server_headless = effective_headless or (args.simulator == "gz" and not effective_headless)
        sim_spec = build_simulator_launch_spec(
            repo_root=repo_root,
            build_dir=build_dir,
            base_rootfs=base_rootfs,
            rootfs=replay_root,
            simulator=args.simulator,
            instance_id=args.instance_id,
            tcp_port=args.tcp_port,
            sim_speed_factor=float(args.sim_speed_factor),
            headless=server_headless,
            slot_name=f"replay_{args.instance_id:03d}",
            simulator_vehicle=args.simulator_vehicle,
            simulator_world=args.simulator_world,
        )
        sim_log = tool_root / "results" / f"replay_{args.simulator}.log"
        px4_log = tool_root / "results" / "replay_px4.log"
        sim_proc = subprocess.Popen(
            sim_spec.command,
            cwd=str(replay_root),
            stdout=sim_log.open("w", encoding="utf-8"),
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env=sim_spec.environment,
        )
        wait_for_simulator_ready(sim_spec, timeout_s=30.0)
        if args.simulator == "gz" and not effective_headless:
            gui_log = tool_root / "results" / "replay_gz_gui.log"
            gui_proc = subprocess.Popen(
                ["gz", "sim", "-g"],
                cwd=str(replay_root),
                stdout=gui_log.open("w", encoding="utf-8"),
                stderr=subprocess.STDOUT,
                start_new_session=True,
                env=sim_spec.environment,
            )

        px4_env = os.environ.copy()
        for key, value in sim_spec.px4_environment.items():
            if value == "":
                continue
            px4_env[key] = str(value)
        px4_cmd = [
            str(build_dir / "bin" / "px4"),
            "-i", str(args.instance_id),
            "-d", str(build_dir / "etc"),
        ]
        px4_proc = subprocess.Popen(
            px4_cmd,
            cwd=str(replay_root),
            stdout=px4_log.open("w", encoding="utf-8"),
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env=px4_env,
        )
        wait_px4_ready(replay_root, args.instance_id)
        replay_px4 = Px4Ctl(replay_root, instance_id=args.instance_id)

        write_state(state_json, {
            "status": "running",
            "controller": args.controller,
            "traj_id": args.traj_id,
            "started_at": time.time(),
            "headless": effective_headless,
            "headless_requested": requested_headless,
            "display_mode": display_mode,
            "sim_speed_factor": float(args.sim_speed_factor),
            "simulator": args.simulator,
            "simulator_vehicle": args.simulator_vehicle,
            "simulator_world": args.simulator_world,
            "strict_requested": bool(args.strict_eval),
            "strict_effective": bool(runtime_profile["strict_effective"]),
            "runtime_note": runtime_profile["note"],
            "mission_mode": args.mission_mode,
            "ident_profile": args.ident_profile,
            "base_param_file": args.base_param_file,
            "params": merged_params,
            "fixed_params": fixed_params,
            "px4_pid": px4_proc.pid,
            "gui_pid": gui_proc.pid if gui_proc else None,
            **simulator_state_fields(args.simulator, sim_proc.pid),
        })

        sample_trace["t0"] = time.time()
        sample_trace["t"].clear()
        for axis in ("x", "y", "z"):
            sample_trace["ref"][axis].clear()
            sample_trace["act"][axis].clear()
        saw_real_trace = False
        if runtime_profile["strict_effective"]:
            write_placeholder_trace(
                live_trace_json,
                runtime_profile["note"] or "Strict replay is waiting for OFFBOARD tracking to begin. The simulator may still show takeoff or landing outside the mission trace.",
                done=False,
            )

        eval_cmd = [
            sys.executable,
            str(tool_root / "px4_eval.py"),
            "--rootfs", str(replay_root),
            "--params-file", str(params_file),
            "--controller", args.controller,
            "--traj-id", str(args.traj_id),
            "--takeoff-alt", str(args.takeoff_alt),
            "--takeoff-timeout", str(runtime_profile["takeoff_timeout"]),
            "--trajectory-timeout", str(runtime_profile["trajectory_timeout"]),
            "--w-track", str(args.w_track),
            "--w-energy", str(args.w_energy),
            "--landing-mode", str(args.landing_mode),
            "--trace-window", str(args.trace_window),
            "--engagement-dwell-s", str(args.engagement_dwell_s),
            "--mission-mode", str(args.mission_mode),
            "--ident-profile", str(args.ident_profile),
            "--result-json", str(result_json),
            "--live-trace-json", str(live_trace_json),
            "--eval-id", "0",
            "--worker-id", "-1",
            "--instance-id", str(args.instance_id),
        ]
        if str(args.base_param_file).strip():
            eval_cmd.extend(["--base-param-file", str(args.base_param_file)])
        if not runtime_profile["strict_effective"]:
            eval_cmd.append("--relaxed-replay")
        for name, value in sorted(fixed_params.items()):
            eval_cmd.extend(["--fixed-param", f"{name}={value}"])
        eval_log = tool_root / "results" / "replay_eval.log"
        eval_log.parent.mkdir(parents=True, exist_ok=True)
        eval_fp = eval_log.open("w", encoding="utf-8")
        eval_proc = subprocess.Popen(
            eval_cmd,
            cwd=str(repo_root),
            stdout=eval_fp,
            stderr=subprocess.STDOUT,
            text=True,
        )
        deadline = time.time() + max(120.0, float(runtime_profile["trajectory_timeout"]) + float(runtime_profile["takeoff_timeout"]) + 60.0)
        while True:
            if time.time() > deadline:
                raise RuntimeError("visual replay timed out")
            if sim_proc.poll() is not None:
                raise RuntimeError(f"{sim_spec.display_name} exited during replay")
            if px4_proc.poll() is not None:
                raise RuntimeError("PX4 exited during replay")
            if gui_proc is not None and gui_proc.poll() is not None:
                gui_proc = None
            if not runtime_profile["strict_effective"]:
                try:
                    saw_real_trace = append_fallback_trace(
                        replay_px4,
                        live_trace_json,
                        sample_trace,
                        args.takeoff_alt,
                        done=False,
                        message="replay running",
                    ) or saw_real_trace
                except Exception:
                    pass
            if eval_proc.poll() is not None:
                break
            time.sleep(0.25)
        eval_fp.flush()
        eval_fp.close()
        eval_fp = None
        if eval_proc.returncode != 0:
            err_text = eval_log.read_text(encoding="utf-8").strip() if eval_log.exists() else ""
            if not saw_real_trace and not runtime_profile["strict_effective"]:
                try:
                    append_fallback_trace(
                        replay_px4,
                        live_trace_json,
                        sample_trace,
                        args.takeoff_alt,
                        done=True,
                        message="replay failed",
                    )
                except Exception:
                    pass
            elif runtime_profile["strict_effective"]:
                write_placeholder_trace(
                    live_trace_json,
                    runtime_profile["note"] or "Strict replay failed before OFFBOARD trace became available.",
                    done=True,
                )
            raise RuntimeError(err_text or "visual replay failed")
        payload = json.loads(result_json.read_text(encoding="utf-8")) if result_json.exists() else {}
        write_state(state_json, {
            "status": "finished",
            "controller": args.controller,
            "traj_id": args.traj_id,
            "started_at": time.time(),
            "headless": effective_headless,
            "headless_requested": requested_headless,
            "display_mode": display_mode,
            "sim_speed_factor": float(args.sim_speed_factor),
            "simulator": args.simulator,
            "simulator_vehicle": args.simulator_vehicle,
            "simulator_world": args.simulator_world,
            "strict_requested": bool(args.strict_eval),
            "strict_effective": bool(runtime_profile["strict_effective"]),
            "runtime_note": runtime_profile["note"],
            "mission_mode": args.mission_mode,
            "ident_profile": args.ident_profile,
            "base_param_file": args.base_param_file,
            "params": merged_params,
            "fixed_params": fixed_params,
            "px4_pid": px4_proc.pid,
            "gui_pid": gui_proc.pid if gui_proc else None,
            **simulator_state_fields(args.simulator, sim_proc.pid),
            "result": payload,
            "finished_at": time.time(),
        })
        if effective_headless or args.auto_close_after_finish:
            return 0
        while True:
            if sim_proc.poll() is not None:
                break
            if px4_proc.poll() is not None:
                break
            if gui_proc is not None and gui_proc.poll() is not None:
                gui_proc = None
            if not runtime_profile["strict_effective"]:
                try:
                    if not saw_real_trace:
                        saw_real_trace = append_fallback_trace(
                            replay_px4,
                            live_trace_json,
                            sample_trace,
                            args.takeoff_alt,
                            done=True,
                            message="replay finished, waiting for stop",
                        ) or saw_real_trace
                except Exception:
                    pass
            time.sleep(0.35)
        return 0
    except Exception as exc:
        if px4_proc is not None and px4_proc.poll() is None:
            try:
                replay_px4 = Px4Ctl(replay_root, instance_id=args.instance_id)
                if runtime_profile["strict_effective"]:
                    write_placeholder_trace(
                        live_trace_json,
                        runtime_profile["note"] or "Strict replay failed before OFFBOARD trace became available.",
                        done=True,
                    )
                else:
                    append_fallback_trace(
                        replay_px4,
                        live_trace_json,
                        sample_trace,
                        args.takeoff_alt,
                        done=True,
                        message="replay failed",
                    )
            except Exception:
                pass
        write_state(state_json, {
            "status": "failed",
            "controller": args.controller,
            "traj_id": args.traj_id,
            "headless": effective_headless,
            "headless_requested": requested_headless,
            "display_mode": display_mode,
            "sim_speed_factor": float(args.sim_speed_factor),
            "simulator": args.simulator,
            "simulator_vehicle": args.simulator_vehicle,
            "simulator_world": args.simulator_world,
            "strict_requested": bool(args.strict_eval),
            "strict_effective": bool(runtime_profile["strict_effective"]),
            "runtime_note": runtime_profile["note"],
            "mission_mode": args.mission_mode,
            "ident_profile": args.ident_profile,
            "base_param_file": args.base_param_file,
            "params": params,
            "fixed_params": fixed_params,
            "px4_pid": px4_proc.pid if px4_proc else None,
            "gui_pid": gui_proc.pid if gui_proc else None,
            **(simulator_state_fields(args.simulator, sim_proc.pid) if sim_proc else {"simulator": args.simulator}),
            "error": str(exc),
            "failed_at": time.time(),
        })
        if not effective_headless and sim_proc is not None and px4_proc is not None:
            while True:
                if sim_proc.poll() is not None:
                    break
                if px4_proc.poll() is not None:
                    break
                if gui_proc is not None and gui_proc.poll() is not None:
                    gui_proc = None
                time.sleep(0.35)
        return 1
    finally:
        try:
            if eval_fp is not None:
                eval_fp.close()
        except Exception:
            pass
        terminate_proc(eval_proc)
        terminate_proc(px4_proc)
        terminate_proc(gui_proc)
        terminate_proc(sim_proc)


if __name__ == "__main__":
    raise SystemExit(main())
