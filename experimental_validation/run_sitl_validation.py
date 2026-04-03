#!/usr/bin/env python3
"""Run the five validation trajectories in PX4 SITL for stock and identified Gazebo models."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time
import threading
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import pexpect
from pymavlink import mavutil

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
EXAMPLES_ROOT = REPO_ROOT / "examples"
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from experimental_validation.prepare_identified_model import prepare_identified_model
from experimental_validation.validation_trajectories import (
    COMMON_LOGGED_START,
    DEFAULT_VALIDATION_TRAJECTORIES,
    ValidationTrajectory,
    export_validation_trajectories,
)
from run_hitl_px4_builtin_trajectory_minimal import TRAJ_DURATION_S, observe_builtin_trajectory
from run_hitl_px4_trajectory_reader_position_step_minimal import (
    observe_armed_ground_hold,
    observe_hold_phase,
    send_position_target_local_ned,
)
from run_hitl_udp_sequence import (
    arm_with_retries,
    request_message_interval,
    robust_ground_baseline,
    set_anchor_from_position,
    set_param,
    set_position_target_absolute,
    set_position_target_relative,
    set_offboard,
    wait_for_ground_quiet,
    wait_for_sim_ready,
)

PX4_PROMPT = "pxh> "
TRACKING_LOG_START_RE = re.compile(r"Tracking log started: (\./tracking_logs/[^\r\n]+\.csv)")
TRAJECTORY_DONE_TEXT = "Trajectory EOF reached, tracking log closed"


@dataclass(frozen=True)
class ValidationModelSpec:
    label: str
    gz_model: str
    display_name: str


DEFAULT_MODEL_SPECS: tuple[ValidationModelSpec, ...] = (
    ValidationModelSpec("stock_sitl_placeholder", "x500", "Stock x500 SITL"),
    ValidationModelSpec("digital_twin", "x500_identified", "Identified digital twin SITL"),
)

PX4_BIN_PATTERN = re.compile(r".*/PX4-Autopilot(?:-Identification)?/build/px4_sitl_default/bin/px4(?:\s|$)")
GZ_SIM_PATTERN = re.compile(r".*\bgz sim\b.*")
SITL_HOVER_Z = -5.0
SITL_HOVER_TIMEOUT_SECONDS = 45.0
SITL_PRE_OFFBOARD_SECONDS = 2.0
SITL_RAMP_SECONDS = 8.0
SITL_SETPOINT_PERIOD = 0.05
SITL_DIRECT_HOLD_SECONDS = 3.0
SITL_CUSTOM_HOLD_SECONDS = 5.0
SITL_REPORT_PERIOD = 0.5
SITL_ARM_ATTEMPTS = 3
SITL_GROUND_XY_WINDOW = 0.25
SITL_GROUND_Z_WINDOW = 0.25
SITL_DIRECT_XY_LIMIT = 0.25
SITL_DIRECT_Z_TOLERANCE = 0.35
SITL_DIRECT_TILT_LIMIT_DEG = 8.0
SITL_CUSTOM_XY_LIMIT = 0.20
SITL_CUSTOM_Z_TOLERANCE = 0.20
SITL_CUSTOM_TILT_LIMIT_DEG = 6.0
SITL_TRAJECTORY_XY_ENVELOPE_LIMIT = 3.50
SITL_TRAJECTORY_Z_TOLERANCE = 0.50
SITL_TRAJECTORY_TILT_LIMIT_DEG = 20.0
SITL_TRAJECTORY_TAIL_SECONDS = 10.0
SITL_ESC_MIN_DEFAULT = 0
SITL_START_ATTEMPTS = 3
SITL_START_RETRY_DELAY_SECONDS = 3.0
VISUAL_FOLLOW_OFFSET_X = 0.0
VISUAL_FOLLOW_OFFSET_Y = 0.0
VISUAL_FOLLOW_OFFSET_Z = 10.0
SITL_CONSOLE_WINDOW_TITLE = "PX4 SITL Log (read-only)"


def _apply_x500_esc_scaling(session: "Px4SitlSession", *, min_value: int, max_value: int) -> None:
    for motor_idx in range(1, 5):
        session.send(f"param set SIM_GZ_EC_MIN{motor_idx} {min_value}")
        session.send(f"param set SIM_GZ_EC_MAX{motor_idx} {max_value}")


def _start_session_with_retries(
    *,
    px4_root: Path,
    run_rootfs: Path,
    env: dict[str, str],
    attempts: int = SITL_START_ATTEMPTS,
) -> "Px4SitlSession":
    last_error: Exception | None = None

    for attempt in range(1, max(1, attempts) + 1):
        session = Px4SitlSession(px4_root, run_rootfs, env)

        try:
            session.start()
            return session

        except Exception as exc:
            last_error = exc
            session.shutdown()
            _cleanup_stale_sitl_processes()

            if attempt >= max(1, attempts):
                raise

            time.sleep(SITL_START_RETRY_DELAY_SECONDS)

    assert last_error is not None
    raise last_error


def _resolve_trajectories(names: Iterable[str] | None) -> tuple[ValidationTrajectory, ...]:
    if names is None:
        return DEFAULT_VALIDATION_TRAJECTORIES

    by_name = {entry.name: entry for entry in DEFAULT_VALIDATION_TRAJECTORIES}
    resolved: list[ValidationTrajectory] = []

    for name in names:
        entry = by_name.get(name)
        if entry is None:
            known = ", ".join(sorted(by_name))
            raise ValueError(f"unknown trajectory '{name}' (known: {known})")
        resolved.append(entry)

    return tuple(resolved)


def _resolve_model_specs(labels: Iterable[str] | None) -> tuple[ValidationModelSpec, ...]:
    if labels is None:
        return DEFAULT_MODEL_SPECS

    by_label = {entry.label: entry for entry in DEFAULT_MODEL_SPECS}
    resolved: list[ValidationModelSpec] = []

    for label in labels:
        entry = by_label.get(label)
        if entry is None:
            known = ", ".join(sorted(by_label))
            raise ValueError(f"unknown model label '{label}' (known: {known})")
        resolved.append(entry)

    return tuple(resolved)


def _append_path_list(existing: str, *entries: Path) -> str:
    parts = [part for part in str(existing or "").split(":") if part]
    for entry in entries:
        text = str(entry.resolve())
        if text not in parts:
            parts.insert(0, text)
    return ":".join(parts)


def _indent(elem: ET.Element, level: int = 0) -> None:
    text = "\n" + ("  " * level)
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = text + "  "
        for child in elem:
            _indent(child, level + 1)
        if not elem[-1].tail or not elem[-1].tail.strip():
            elem[-1].tail = text
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = text


def _inject_truth_logger_plugin(model_sdf_path: Path) -> None:
    tree = ET.parse(model_sdf_path)
    root = tree.getroot()
    model = root.find(".//model")
    if model is None:
        raise RuntimeError(f"missing <model> in {model_sdf_path}")

    for plugin in list(model.findall("plugin")):
        if (
            plugin.attrib.get("filename") == "SystemIdentificationLoggerPlugin"
            or plugin.attrib.get("name") == "SystemIdentificationLoggerPlugin"
        ):
            model.remove(plugin)

    rotor_plugins = [
        plugin for plugin in model.findall("plugin")
        if plugin.attrib.get("name") == "gz::sim::systems::MulticopterMotorModel"
    ]
    if not rotor_plugins:
        return

    logger = ET.SubElement(
        model,
        "plugin",
        {
            "filename": "SystemIdentificationLoggerPlugin",
            "name": "SystemIdentificationLoggerPlugin",
        },
    )
    ET.SubElement(logger, "enabled").text = "true"
    ET.SubElement(logger, "base_link_name").text = "base_link"
    ET.SubElement(logger, "sample_period_s").text = "0.005"
    ET.SubElement(logger, "command_sub_topic").text = "command/motor_speed"

    def _copy(src_plugin: ET.Element, src_name: str, dst_parent: ET.Element, dst_name: str) -> None:
        node = src_plugin.find(src_name)
        if node is not None and node.text is not None:
            ET.SubElement(dst_parent, dst_name).text = node.text.strip()

    for plugin in rotor_plugins:
        rotor = ET.SubElement(logger, "rotor")
        _copy(plugin, "motorNumber", rotor, "motor_number")
        _copy(plugin, "jointName", rotor, "joint_name")
        _copy(plugin, "linkName", rotor, "link_name")
        _copy(plugin, "turningDirection", rotor, "turning_direction")
        _copy(plugin, "timeConstantUp", rotor, "time_constant_up_s")
        _copy(plugin, "timeConstantDown", rotor, "time_constant_down_s")
        _copy(plugin, "maxRotVelocity", rotor, "max_rot_velocity_radps")
        _copy(plugin, "motorConstant", rotor, "motor_constant")
        _copy(plugin, "momentConstant", rotor, "moment_constant")
        _copy(plugin, "rotorDragCoefficient", rotor, "rotor_drag_coefficient")
        _copy(plugin, "rollingMomentCoefficient", rotor, "rolling_moment_coefficient")
        _copy(plugin, "rotorVelocitySlowdownSim", rotor, "rotor_velocity_slowdown_sim")

    _indent(root)
    tree.write(model_sdf_path, encoding="utf-8", xml_declaration=True)


def _prepare_run_rootfs(base_rootfs: Path, run_rootfs: Path) -> None:
    if run_rootfs.exists():
        shutil.rmtree(run_rootfs)
    shutil.copytree(base_rootfs, run_rootfs, symlinks=True)
    for name in (
        "tracking_logs",
        "identification_logs",
        "sysid_truth_logs",
        "log",
        "mavlink",
    ):
        shutil.rmtree(run_rootfs / name, ignore_errors=True)
    for name in ("parameters.bson", "parameters_backup.bson", "dataman"):
        path = run_rootfs / name
        if path.exists() or path.is_symlink():
            path.unlink()


def _kill_matching_processes(pattern: re.Pattern[str]) -> None:
    proc = subprocess.run(
        ["ps", "-eo", "pid=,args="],
        capture_output=True,
        text=True,
        check=False,
    )
    for line in proc.stdout.splitlines():
        line = line.strip()
        if not line:
            continue
        pid_text, _, cmdline = line.partition(" ")
        try:
            pid = int(pid_text)
        except ValueError:
            continue
        if pid == os.getpid():
            continue
        if pattern.match(cmdline):
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                continue

    time.sleep(2.0)

    proc = subprocess.run(
        ["ps", "-eo", "pid=,args="],
        capture_output=True,
        text=True,
        check=False,
    )
    for line in proc.stdout.splitlines():
        line = line.strip()
        if not line:
            continue
        pid_text, _, cmdline = line.partition(" ")
        try:
            pid = int(pid_text)
        except ValueError:
            continue
        if pid == os.getpid():
            continue
        if pattern.match(cmdline):
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                continue


def _cleanup_stale_sitl_processes() -> None:
    _kill_matching_processes(PX4_BIN_PATTERN)
    _kill_matching_processes(GZ_SIM_PATTERN)


def _prepare_model_override(px4_root: Path, model_name: str, override_models_root: Path) -> None:
    source_dir = px4_root / "Tools" / "simulation" / "gz" / "models" / model_name
    if not source_dir.exists():
        raise FileNotFoundError(f"Gazebo model not found: {source_dir}")
    target_dir = override_models_root / model_name
    if target_dir.exists():
        shutil.rmtree(target_dir)
    override_models_root.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source_dir, target_dir, symlinks=True)
    _inject_truth_logger_plugin(target_dir / "model.sdf")


def _close_window_by_title(title: str) -> None:
    if not shutil.which("wmctrl"):
        return
    subprocess.run(
        ["wmctrl", "-c", title],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    time.sleep(0.5)


def _open_console_window(console_log: Path, *, title: str = SITL_CONSOLE_WINDOW_TITLE) -> None:
    terminal = shutil.which("gnome-terminal")
    if terminal is None:
        return

    console_log.parent.mkdir(parents=True, exist_ok=True)
    console_log.touch(exist_ok=True)
    _close_window_by_title(title)

    subprocess.Popen(
        [
            terminal,
            f"--title={title}",
            "--maximize",
            "--",
            "bash",
            "-ic",
            f"tail -n +1 -f {shlex.quote(str(console_log))}",
        ],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    time.sleep(1.0)

    if shutil.which("wmctrl"):
        subprocess.run(
            ["wmctrl", "-a", title],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )


def _build_px4_env(px4_root: Path, build_dir: Path, run_rootfs: Path, override_models_root: Path, model_name: str, headless: bool) -> dict[str, str]:
    env = os.environ.copy()
    models = px4_root / "Tools" / "simulation" / "gz" / "models"
    worlds = px4_root / "Tools" / "simulation" / "gz" / "worlds"
    plugins = build_dir / "src" / "modules" / "simulation" / "gz_plugins"
    server_config = px4_root / "src" / "modules" / "simulation" / "gz_bridge" / "server.config"

    env["PX4_SYS_AUTOSTART"] = "4001"
    env["PX4_SIM_MODEL"] = f"gz_{model_name}"
    env["PX4_SIM_SPEED_FACTOR"] = "1.0"
    env["HEADLESS"] = "1" if headless else ""
    env["PX4_GZ_MODELS"] = str(models.resolve())
    env["PX4_GZ_WORLDS"] = str(worlds.resolve())
    env["PX4_GZ_PLUGINS"] = str(plugins.resolve())
    env["PX4_GZ_SERVER_CONFIG"] = str(server_config.resolve())
    env["GZ_SIM_RESOURCE_PATH"] = _append_path_list(env.get("GZ_SIM_RESOURCE_PATH", ""), override_models_root, models, worlds)
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = _append_path_list(env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""), plugins)
    env["GZ_SIM_SERVER_CONFIG_PATH"] = str(server_config.resolve())
    env["PX4_SYSID_LOG_DIR"] = str((run_rootfs / "sysid_truth_logs").resolve())
    env["PX4_SYSID_LOG_SLOT"] = run_rootfs.name
    if not headless:
        # Keep the visual workflow on a fixed near-top-down follow view unless the caller overrides it.
        env.setdefault("PX4_GZ_FOLLOW_OFFSET_X", str(VISUAL_FOLLOW_OFFSET_X))
        env.setdefault("PX4_GZ_FOLLOW_OFFSET_Y", str(VISUAL_FOLLOW_OFFSET_Y))
        env.setdefault("PX4_GZ_FOLLOW_OFFSET_Z", str(VISUAL_FOLLOW_OFFSET_Z))
    return env


class Px4SitlSession:
    def __init__(self, px4_root: Path, run_rootfs: Path, env: dict[str, str], *, timeout_s: float = 120.0) -> None:
        self._px4_root = px4_root
        self._run_rootfs = run_rootfs
        self._env = env
        self._timeout_s = timeout_s
        self._child: pexpect.spawn | None = None
        self._mav: mavutil.mavfile | None = None
        self._gcs: mavutil.mavfile | None = None
        self._gcs_stop = threading.Event()
        self._gcs_thread: threading.Thread | None = None
        self._console_log = run_rootfs / "px4_console.log"

    @staticmethod
    def _child_pids(root_pid: int) -> list[int]:
        pending = [root_pid]
        discovered: list[int] = []
        while pending:
            parent = pending.pop()
            proc = subprocess.run(
                ["ps", "-o", "pid=", "--ppid", str(parent)],
                capture_output=True,
                text=True,
                check=False,
            )
            for line in proc.stdout.splitlines():
                line = line.strip()
                if not line:
                    continue
                pid = int(line)
                if pid not in discovered:
                    discovered.append(pid)
                    pending.append(pid)
        return discovered

    def start(self) -> None:
        cmd = str(self._px4_root / "build" / "px4_sitl_default" / "bin" / "px4")
        romfs = str(self._px4_root / "ROMFS" / "px4fmu_common")
        log_fp = self._console_log.open("w", encoding="utf-8")
        self._child = pexpect.spawn(
            cmd,
            [romfs],
            cwd=str(self._run_rootfs),
            env=self._env,
            encoding="utf-8",
            timeout=self._timeout_s,
            echo=False,
        )
        self._child.logfile_read = log_fp
        self._expect_prompt()
        self._gcs = mavutil.mavlink_connection("udpout:127.0.0.1:18570", source_system=245, source_component=190)
        self._start_gcs_heartbeats()
        self._mav = mavutil.mavlink_connection("udpin:127.0.0.1:14540", source_system=255)
        self._mav.wait_heartbeat(timeout=15)

    def _start_gcs_heartbeats(self) -> None:
        if self._gcs is None or self._gcs_thread is not None:
            return

        def _loop() -> None:
            assert self._gcs is not None
            while not self._gcs_stop.is_set():
                try:
                    self._gcs.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0,
                        0,
                        mavutil.mavlink.MAV_STATE_ACTIVE,
                    )
                except Exception:
                    return
                self._gcs_stop.wait(0.5)

        self._gcs_stop.clear()
        self._gcs_thread = threading.Thread(target=_loop, daemon=True)
        self._gcs_thread.start()

    def _expect_prompt(self) -> None:
        assert self._child is not None
        self._child.expect_exact(PX4_PROMPT, timeout=self._timeout_s)

    def _drain_local_position(self) -> dict[str, float] | None:
        assert self._mav is not None
        latest = None
        while True:
            msg = self._mav.recv_match(type="LOCAL_POSITION_NED", blocking=False)
            if msg is None:
                break
            speed = math.sqrt(float(msg.vx) ** 2 + float(msg.vy) ** 2 + float(msg.vz) ** 2)
            latest = {"x": float(msg.x), "y": float(msg.y), "z": float(msg.z), "speed_mps": speed}
        return latest

    def send(self, command: str, *, timeout_s: float | None = None) -> str:
        assert self._child is not None
        self._child.sendline(command)
        self._child.expect_exact(PX4_PROMPT, timeout=timeout_s or self._timeout_s)
        return self._child.before

    def send_no_wait(self, command: str) -> None:
        assert self._child is not None
        self._child.sendline(command)

    def sync_prompt(self, *, timeout_s: float = 5.0) -> None:
        assert self._child is not None
        self._child.expect_exact(PX4_PROMPT, timeout=timeout_s)

    def arm(self, *, timeout_s: float = 15.0) -> None:
        assert self._mav is not None
        self._mav.arducopter_arm()
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self._mav.motors_armed():
                return
            self._mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        raise RuntimeError("vehicle did not arm in time")

    def set_mode(self, mode_name: str, *, timeout_s: float = 10.0) -> None:
        assert self._mav is not None
        self._mav.set_mode_px4(mode_name, 0, 0)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            heartbeat = self._mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if heartbeat is None:
                continue
            current_mode = self._mav.flightmode
            if isinstance(current_mode, str) and current_mode.upper() == mode_name.upper():
                return
        raise RuntimeError(f"vehicle did not enter {mode_name} mode in time")

    def expect(self, pattern: str | re.Pattern[str], *, timeout_s: float) -> re.Match[str] | None:
        assert self._child is not None
        if isinstance(pattern, re.Pattern):
            self._child.expect(pattern, timeout=timeout_s)
            return self._child.match
        self._child.expect_exact(pattern, timeout=timeout_s)
        return None

    def wait_until_position(self, target_xyz: tuple[float, float, float], *, xy_tol_m: float = 0.22, z_tol_m: float = 0.18, timeout_s: float = 35.0) -> dict[str, float]:
        assert self._mav is not None
        deadline = time.time() + timeout_s
        best: dict[str, float] | None = self._drain_local_position()
        streak = 0
        while time.time() < deadline:
            msg = self._mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
            if msg is None:
                continue
            state = {"x": float(msg.x), "y": float(msg.y), "z": float(msg.z)}
            best = state
            err_xy = ((state["x"] - target_xyz[0]) ** 2 + (state["y"] - target_xyz[1]) ** 2) ** 0.5
            err_z = abs(state["z"] - target_xyz[2])
            if err_xy <= xy_tol_m and err_z <= z_tol_m:
                streak += 1
                if streak >= 8:
                    return state
            else:
                streak = 0
        raise RuntimeError(f"vehicle did not settle near {target_xyz} in {timeout_s:.1f}s (last={best})")

    def sample_local_position(self, *, timeout_s: float = 10.0) -> dict[str, float]:
        assert self._mav is not None
        latest = self._drain_local_position()
        if latest is not None:
            return latest
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = self._mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0)
            if msg is None:
                continue
            speed = math.sqrt(float(msg.vx) ** 2 + float(msg.vy) ** 2 + float(msg.vz) ** 2)
            return {"x": float(msg.x), "y": float(msg.y), "z": float(msg.z), "speed_mps": speed}
        raise RuntimeError("vehicle local position sample was not received in time")

    def wait_until_hover_stable(self, *, min_altitude_m: float = 2.7, speed_tol_mps: float = 0.35, timeout_s: float = 35.0) -> dict[str, float]:
        assert self._mav is not None
        deadline = time.time() + timeout_s
        best: dict[str, float] | None = self._drain_local_position()
        streak = 0
        while time.time() < deadline:
            msg = self._mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
            if msg is None:
                continue
            speed = math.sqrt(float(msg.vx) ** 2 + float(msg.vy) ** 2 + float(msg.vz) ** 2)
            best = {"x": float(msg.x), "y": float(msg.y), "z": float(msg.z), "speed_mps": speed}
            if (-float(msg.z)) >= min_altitude_m and speed <= speed_tol_mps:
                streak += 1
                if streak >= 8:
                    return best
            else:
                streak = 0
        raise RuntimeError(f"vehicle did not reach a stable takeoff hover in {timeout_s:.1f}s (last={best})")

    def wait_until_landed(
        self,
        *,
        ground_z_m: float,
        z_tol_m: float = 0.18,
        speed_tol_mps: float = 0.3,
        timeout_s: float = 45.0,
    ) -> dict[str, float]:
        assert self._mav is not None
        deadline = time.time() + timeout_s
        best: dict[str, float] | None = self._drain_local_position()
        streak = 0
        while time.time() < deadline:
            msg = self._mav.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2.0)
            if msg is None:
                continue
            speed = math.sqrt(float(msg.vx) ** 2 + float(msg.vy) ** 2 + float(msg.vz) ** 2)
            best = {"x": float(msg.x), "y": float(msg.y), "z": float(msg.z), "speed_mps": speed}
            if abs(float(msg.z) - ground_z_m) <= z_tol_m and speed <= speed_tol_mps:
                streak += 1
                if streak >= 10:
                    return best
            else:
                streak = 0
        raise RuntimeError(f"vehicle did not land in {timeout_s:.1f}s (last={best})")

    def shutdown(self) -> None:
        self._gcs_stop.set()
        if self._gcs_thread is not None:
            self._gcs_thread.join(timeout=2.0)
        self._gcs_thread = None
        if self._gcs is not None:
            try:
                self._gcs.close()
            except Exception:
                pass
        self._gcs = None
        if self._child is None:
            return
        child_pid = self._child.pid
        try:
            self.send("shutdown", timeout_s=10)
        except Exception:
            pass
        descendants = []
        if child_pid:
            try:
                descendants = self._child_pids(child_pid)
            except Exception:
                descendants = []
        for sig in (signal.SIGTERM, signal.SIGKILL):
            for pid in reversed(descendants):
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
            try:
                if child_pid:
                    os.kill(child_pid, sig)
            except ProcessLookupError:
                pass
            time.sleep(1.0)
            if not self._child.isalive():
                break
        try:
            self._child.close(force=True)
        except Exception:
            pass
        self._child = None


def _copy_log(src: Path, dst: Path) -> str:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return str(dst.resolve())


def _sample_attitude(mav, *, timeout_s: float = 5.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = mav.recv_match(type=["ATTITUDE", "STATUSTEXT"], blocking=True, timeout=0.5)
        if msg is None or msg.get_type() == "STATUSTEXT":
            continue
        return msg
    raise RuntimeError("vehicle attitude sample was not received in time")


def _find_tracking_log(run_rootfs: Path, relative_path: str) -> Path:
    rel = relative_path.removeprefix("./")
    path = run_rootfs / rel
    if not path.exists():
        raise FileNotFoundError(f"tracking log announced by PX4 not found: {path}")
    return path


def run_validation_model(
    *,
    px4_root: str | Path,
    out_root: str | Path,
    model_spec: ValidationModelSpec,
    candidate_dir: str | Path,
    trajectories: Iterable[ValidationTrajectory] = DEFAULT_VALIDATION_TRAJECTORIES,
    sitl_esc_min: int = SITL_ESC_MIN_DEFAULT,
    sitl_esc_max: int | None = None,
    headless: bool = True,
    show_console: bool = False,
) -> dict:
    px4_root = Path(px4_root).expanduser().resolve()
    out_root = Path(out_root).expanduser().resolve()
    candidate_dir = Path(candidate_dir).expanduser().resolve()

    build_dir = px4_root / "build" / "px4_sitl_default"
    base_rootfs = build_dir / "rootfs"
    runtime_root = out_root / "runtime" / model_spec.label
    run_rootfs = runtime_root / "rootfs"
    override_models_root = runtime_root / "override_models"
    result_root = out_root / model_spec.label

    _prepare_run_rootfs(base_rootfs, run_rootfs)
    if show_console and not headless:
        _open_console_window(run_rootfs / "px4_console.log")
    export_manifest = export_validation_trajectories(run_rootfs / "trajectories")
    _prepare_model_override(px4_root, model_spec.gz_model, override_models_root)
    env = _build_px4_env(px4_root, build_dir, run_rootfs, override_models_root, model_spec.gz_model, headless)

    _cleanup_stale_sitl_processes()
    session = _start_session_with_retries(px4_root=px4_root, run_rootfs=run_rootfs, env=env)
    common_anchor = COMMON_LOGGED_START
    results: list[dict] = []
    try:
        if sitl_esc_max is not None:
            _apply_x500_esc_scaling(session, min_value=sitl_esc_min, max_value=sitl_esc_max)
        assert session._mav is not None
        mav = session._mav
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, interval_us=500_000)
        set_param(mav, "COM_RC_IN_MODE", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "COM_CPU_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "COM_RAM_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "MIS_TAKEOFF_ALT", abs(SITL_HOVER_Z), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_POS_ABS", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_POS_X", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Y", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Z", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_YAW", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        wait_for_sim_ready(mav, timeout=20.0, require_local_position=True, min_local_samples=3)
        wait_for_ground_quiet(mav, duration_s=2.0, timeout=10.0)
        baseline_x, baseline_y, baseline_z = robust_ground_baseline(mav)
        ground_xy = math.hypot(baseline_x, baseline_y)
        if ground_xy > SITL_GROUND_XY_WINDOW or abs(baseline_z) > SITL_GROUND_Z_WINDOW:
            raise RuntimeError(
                f"Dirty ground baseline: x={baseline_x:.3f} y={baseline_y:.3f} z={baseline_z:.3f}. "
                "Restart SITL clean before running validation."
            )

        hover_yaw = 0.0
        for _ in range(10):
            msg = mav.recv_match(type=["ATTITUDE", "STATUSTEXT"], blocking=True, timeout=0.5)
            if not msg:
                continue
            if msg.get_type() == "STATUSTEXT":
                continue
            hover_yaw = float(msg.yaw)
            break

        target_x = baseline_x
        target_y = baseline_y
        target_z = baseline_z + SITL_HOVER_Z

        session.set_mode("POSCTL")
        arm_with_retries(mav, attempts=SITL_ARM_ATTEMPTS)
        observe_armed_ground_hold(
            mav,
            duration_s=max(0.5, SITL_PRE_OFFBOARD_SECONDS),
            report_period=SITL_REPORT_PERIOD,
            ref_x=target_x,
            ref_y=target_y,
            ref_z=baseline_z,
            yaw=hover_yaw,
            setpoint_period=SITL_SETPOINT_PERIOD,
        )

        pre_offboard_end = time.time() + max(0.5, SITL_PRE_OFFBOARD_SECONDS)
        while time.time() < pre_offboard_end:
            send_position_target_local_ned(mav, target_x, target_y, baseline_z, hover_yaw)
            mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=SITL_SETPOINT_PERIOD)

        set_offboard(mav)

        ramp_start = time.time()
        ramp_end = ramp_start + max(0.5, SITL_RAMP_SECONDS)
        while time.time() < ramp_end:
            alpha = min(1.0, max(0.0, (time.time() - ramp_start) / max(0.5, SITL_RAMP_SECONDS)))
            z_sp = baseline_z + alpha * (target_z - baseline_z)
            send_position_target_local_ned(mav, target_x, target_y, z_sp, hover_yaw)
            mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=SITL_SETPOINT_PERIOD)

        latest_lpos, latest_att, direct_max_xy, direct_max_z_err, direct_max_tilt, direct_failsafe = observe_hold_phase(
            mav,
            label="Direct hover",
            duration_s=SITL_DIRECT_HOLD_SECONDS,
            report_period=SITL_REPORT_PERIOD,
            ref_x=target_x,
            ref_y=target_y,
            ref_z=target_z,
            send_direct_setpoint=True,
            yaw=hover_yaw,
        )
        if (
            direct_failsafe
            or direct_max_xy > SITL_DIRECT_XY_LIMIT
            or direct_max_z_err > SITL_DIRECT_Z_TOLERANCE
            or direct_max_tilt > SITL_DIRECT_TILT_LIMIT_DEG
        ):
            raise RuntimeError(
                f"Direct offboard hold unstable: max_xy={direct_max_xy:.3f}m "
                f"max_z_err={direct_max_z_err:.3f}m max_tilt={direct_max_tilt:.2f}deg "
                f"failsafe={direct_failsafe}"
            )

        session.send("custom_pos_control start")
        session.send("trajectory_reader start")
        set_position_target_absolute(
            mav,
            float(latest_lpos.x),
            float(latest_lpos.y),
            float(latest_lpos.z),
            float(latest_att.yaw),
        )
        set_anchor_from_position(mav, float(latest_lpos.x), float(latest_lpos.y), float(latest_lpos.z))
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CST_POS_CTRL_EN", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

        latest_lpos, latest_att, custom_max_xy, custom_max_z_err, custom_max_tilt, custom_failsafe = observe_hold_phase(
            mav,
            label="Custom hold",
            duration_s=SITL_CUSTOM_HOLD_SECONDS,
            report_period=SITL_REPORT_PERIOD,
            ref_x=float(latest_lpos.x),
            ref_y=float(latest_lpos.y),
            ref_z=float(latest_lpos.z),
        )
        if (
            custom_failsafe
            or custom_max_xy > SITL_CUSTOM_XY_LIMIT
            or custom_max_z_err > SITL_CUSTOM_Z_TOLERANCE
            or custom_max_tilt > SITL_CUSTOM_TILT_LIMIT_DEG
        ):
            raise RuntimeError(
                f"Custom hold unstable: max_xy={custom_max_xy:.3f}m "
                f"max_z_err={custom_max_z_err:.3f}m max_tilt={custom_max_tilt:.2f}deg "
                f"failsafe={custom_failsafe}"
            )

        common_anchor = (
            float(latest_lpos.x),
            float(latest_lpos.y),
            float(latest_lpos.z),
        )
        common_yaw = float(latest_att.yaw) if latest_att is not None else 0.0

        for entry in trajectories:
            set_position_target_absolute(mav, common_anchor[0], common_anchor[1], common_anchor[2], common_yaw)
            set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            set_anchor_from_position(mav, common_anchor[0], common_anchor[1], common_anchor[2])
            set_param(mav, "TRJ_ACTIVE_ID", entry.traj_id, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            set_param(mav, "TRJ_MODE_CMD", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            match = session.expect(TRACKING_LOG_START_RE, timeout_s=20)
            trajectory_duration_s = float(TRAJ_DURATION_S.get(entry.traj_id, entry.duration_s))
            latest_lpos, latest_att, traj_max_xy, traj_max_z_err, traj_max_tilt, traj_failsafe = observe_builtin_trajectory(
                mav,
                duration_s=trajectory_duration_s + SITL_TRAJECTORY_TAIL_SECONDS,
                ref_z=common_anchor[2],
                report_period=SITL_REPORT_PERIOD,
            )
            session.expect(TRAJECTORY_DONE_TEXT, timeout_s=max(20.0, trajectory_duration_s))
            assert match is not None
            if (
                traj_failsafe
                or traj_max_xy > SITL_TRAJECTORY_XY_ENVELOPE_LIMIT
                or traj_max_z_err > SITL_TRAJECTORY_Z_TOLERANCE
                or traj_max_tilt > SITL_TRAJECTORY_TILT_LIMIT_DEG
            ):
                raise RuntimeError(
                    f"Built-in trajectory unstable for {entry.name}: max_xy={traj_max_xy:.3f}m "
                    f"max_z_err={traj_max_z_err:.3f}m max_tilt={traj_max_tilt:.2f}deg "
                    f"failsafe={traj_failsafe}"
                )
            tracking_log = _find_tracking_log(run_rootfs, match.group(1))
            copied_log = _copy_log(tracking_log, result_root / "tracking_logs" / f"{entry.name}.csv")
            results.append(
                {
                    "traj_id": entry.traj_id,
                    "name": entry.name,
                    "duration_s": trajectory_duration_s,
                    "tracking_log": copied_log,
                }
            )
            set_position_target_absolute(mav, common_anchor[0], common_anchor[1], common_anchor[2], common_yaw)
            set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            session.wait_until_position(common_anchor, xy_tol_m=0.28, z_tol_m=0.28, timeout_s=20.0)

        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        session.set_mode("POSCTL")
        session.send_no_wait("commander mode auto:land")
        session.wait_until_landed(ground_z_m=0.0, timeout_s=45.0)
        session.sync_prompt(timeout_s=10.0)
    finally:
        session.shutdown()
        _cleanup_stale_sitl_processes()
        if (run_rootfs / "sysid_truth_logs").exists():
            shutil.copytree(run_rootfs / "sysid_truth_logs", result_root / "sysid_truth_logs", dirs_exist_ok=True)

    payload = {
        "label": model_spec.label,
        "display_name": model_spec.display_name,
        "gz_model": model_spec.gz_model,
        "common_logged_start": {
            "x_m": common_anchor[0],
            "y_m": common_anchor[1],
            "z_m": common_anchor[2],
        },
        "export_manifest": export_manifest,
        "results": results,
        "console_log": str((run_rootfs / "px4_console.log").resolve()),
    }
    (result_root / "run_manifest.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return payload


def run_validation_suite(
    *,
    px4_root: str | Path,
    out_root: str | Path,
    candidate_dir: str | Path,
    trajectories: Iterable[ValidationTrajectory] = DEFAULT_VALIDATION_TRAJECTORIES,
    model_specs: Iterable[ValidationModelSpec] = DEFAULT_MODEL_SPECS,
    sitl_esc_min: int = SITL_ESC_MIN_DEFAULT,
    sitl_esc_max: int | None = None,
    headless: bool = True,
    show_console: bool = False,
) -> dict:
    px4_root = Path(px4_root).expanduser().resolve()
    out_root = Path(out_root).expanduser().resolve()
    candidate_dir = Path(candidate_dir).expanduser().resolve()
    out_root.mkdir(parents=True, exist_ok=True)

    prepare_identified_model(px4_root, candidate_dir, model_name="x500_identified")

    manifests = []
    for model_spec in model_specs:
        manifests.append(
            run_validation_model(
                px4_root=px4_root,
                out_root=out_root,
                model_spec=model_spec,
                candidate_dir=candidate_dir,
                trajectories=trajectories,
                sitl_esc_min=sitl_esc_min,
                sitl_esc_max=sitl_esc_max,
                headless=headless,
                show_console=show_console,
            )
        )

    summary = {
        "px4_root": str(px4_root),
        "candidate_dir": str(candidate_dir),
        "out_root": str(out_root),
        "models": manifests,
    }
    (out_root / "sitl_validation_suite.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main() -> int:
    ap = argparse.ArgumentParser(description="Run the five validation trajectories in stock and identified PX4 SITL.")
    ap.add_argument("--px4-root", default="~/PX4-Autopilot-Identification")
    ap.add_argument("--out-root", default="~/px4-system-identification/examples/paper_assets/stage1_sitl")
    ap.add_argument("--candidate-dir", default="~/px4-system-identification/examples/paper_assets/candidates/x500_truth_assisted_sitl_v1")
    ap.add_argument(
        "--trajectory-names",
        default="",
        help="Comma-separated subset of trajectory names (e.g. circle or circle,hairpin). Default: all five.",
    )
    ap.add_argument(
        "--model-labels",
        default="",
        help="Comma-separated subset of model labels (stock_sitl_placeholder,digital_twin). Default: both.",
    )
    ap.add_argument(
        "--sitl-esc-max",
        type=int,
        default=None,
        help="Override stock SIM_GZ_EC_MAX[1-4] for SITL experiments. Default: keep the airframe defaults.",
    )
    ap.add_argument(
        "--sitl-esc-min",
        type=int,
        default=SITL_ESC_MIN_DEFAULT,
        help="Override SIM_GZ_EC_MIN[1-4] when --sitl-esc-max is provided. Default: 0.",
    )
    ap.add_argument("--visual", action="store_true", help="Launch Gazebo with GUI instead of headless mode.")
    ap.add_argument(
        "--show-console",
        action="store_true",
        help="Open a visible read-only PX4 log window alongside Gazebo in visual mode.",
    )
    args = ap.parse_args()

    trajectory_names = [item.strip() for item in args.trajectory_names.split(",") if item.strip()] or None
    model_labels = [item.strip() for item in args.model_labels.split(",") if item.strip()] or None

    summary = run_validation_suite(
        px4_root=args.px4_root,
        out_root=args.out_root,
        candidate_dir=args.candidate_dir,
        trajectories=_resolve_trajectories(trajectory_names),
        model_specs=_resolve_model_specs(model_labels),
        sitl_esc_min=args.sitl_esc_min,
        sitl_esc_max=args.sitl_esc_max,
        headless=not args.visual,
        show_console=args.show_console,
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
