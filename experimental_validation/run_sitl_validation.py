#!/usr/bin/env python3
"""Run the five validation trajectories in PX4 SITL for stock and identified Gazebo models."""

from __future__ import annotations

import argparse
import csv
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
    observe_hold_phase,
    send_position_target_local_ned,
)
from run_hitl_udp_sequence import (
    decode_statustext,
    request_message_interval,
    robust_ground_baseline,
    set_anchor_from_position,
    set_param,
    set_position_target_absolute,
    set_offboard,
    wait_for_ground_quiet,
    wait_for_local_position,
    wait_for_sim_ready,
)

PX4_PROMPT = "pxh> "
PX4_NEXT_PROMPT_RE = re.compile(r"(?:\r\n|\n|\r)(?:\x1b\[[0-9;]*m)*pxh> (?=$)")
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
XEPHYR_PATTERN = re.compile(r".*\bXephyr\b.*\bPX4 Gazebo Nested Display\b.*")
CLEANUP_SCRIPT = EXAMPLES_ROOT / "cleanup_px4_background_services.sh"
SITL_HOVER_Z = -5.0
SITL_HOVER_TIMEOUT_SECONDS = 60.0
SITL_TAKEOFF_SETTLE_SECONDS = 2.0
SITL_TAKEOFF_STABLE_WINDOW_SECONDS = 2.0
SITL_TAKEOFF_MAX_HORIZ_SPEED_MPS = 0.15
SITL_TAKEOFF_MAX_VERT_SPEED_MPS = 0.10
SITL_TAKEOFF_Z_TOLERANCE_M = 0.35
SITL_TAKEOFF_TILT_LIMIT_DEG = 5.0
SITL_MANUAL_CONTROL_PERIOD = 0.05
SITL_MANUAL_CLIMB_TIMEOUT_SECONDS = 30.0
SITL_MANUAL_CLIMB_ENTRY_Z_MARGIN = 0.25
SITL_MANUAL_CAPTURE_THROTTLE = 450
SITL_MANUAL_MAX_THROTTLE = 650
SITL_PRE_OFFBOARD_SECONDS = 2.0
SITL_RAMP_SECONDS = 16.0
SITL_SETPOINT_PERIOD = 0.05
SITL_DIRECT_HOLD_SECONDS = 5.0
SITL_DIRECT_SETTLE_TIMEOUT_SECONDS = 6.0
SITL_DIRECT_SETTLE_VZ_TOLERANCE = 0.20
SITL_DIRECT_SETTLE_Z_TOLERANCE = 0.80
SITL_STAGE_MAX_STEP_M = 1.0
SITL_STAGE_HOLD_SECONDS = 0.75
SITL_LIFT_HOLD_SECONDS = 0.0
SITL_ALLOW_UNSTABLE_FINAL_HOVER = True
SITL_ALLOW_UNSTABLE_CUSTOM_HOLD = True
SITL_CUSTOM_HOLD_SECONDS = 5.0
SITL_REPORT_PERIOD = 0.5
SITL_ARM_ATTEMPTS = 3
SITL_GROUND_QUIET_TIMEOUT_SECONDS = 45.0
SITL_GROUND_XY_WINDOW = 0.25
SITL_GROUND_Z_WINDOW = 0.25
SITL_DIRECT_XY_LIMIT = 0.25
SITL_DIRECT_Z_TOLERANCE = 0.55
SITL_DIRECT_TILT_LIMIT_DEG = 8.0
SITL_CUSTOM_XY_LIMIT = 0.20
SITL_CUSTOM_Z_TOLERANCE = 0.20
SITL_CUSTOM_TILT_LIMIT_DEG = 6.0
SITL_TRAJECTORY_XY_ENVELOPE_LIMIT = 3.50
SITL_TRAJECTORY_XY_TRACKING_ERROR_LIMIT = 1.0
SITL_TRAJECTORY_Z_TOLERANCE = 0.50
SITL_TRAJECTORY_TILT_LIMIT_DEG = 45.0
SITL_TRAJECTORY_TAIL_SECONDS = 10.0
SITL_ESC_MIN_DEFAULT = 0
SITL_START_ATTEMPTS = 3
SITL_START_RETRY_DELAY_SECONDS = 3.0
SITL_FREEZE_TRAJECTORY_YAW = True
SITL_LOCKED_YAW_RAD = 0.0
SITL_REFERENCE_WORLD_NAME = "validation_reference_world"
SITL_REFERENCE_MARKER_NAME = "takeoff_reference_marker"
SITL_REFERENCE_MARKER_X = 1.5
SITL_REFERENCE_MARKER_Y = 0.8
SITL_REFERENCE_MARKER_Z = 0.45
SITL_REFERENCE_MARKER_RADIUS = 0.18
SITL_REFERENCE_MARKER_LENGTH = 0.90
VISUAL_FOLLOW_OFFSET_X = 0.0
VISUAL_FOLLOW_OFFSET_Y = 0.0
VISUAL_FOLLOW_OFFSET_Z = 7.0
VISUAL_FOLLOW_PGAIN = 1.0
VISUAL_TRACK_PGAIN = 0.0
SITL_VISUAL_DISABLE_FOLLOW = True
SITL_CONSOLE_WINDOW_TITLE = "PX4 SITL Log (read-only)"
SITL_NESTED_DISPLAY_TITLE = "PX4 Gazebo Nested Display"
SITL_NESTED_DISPLAY_WIDTH = 1400
SITL_NESTED_DISPLAY_HEIGHT = 1000
SITL_NESTED_DISPLAY_MIN = 90
SITL_NESTED_DISPLAY_MAX = 110
SITL_READY_FOR_TAKEOFF_TIMEOUT_SECONDS = 30.0
SITL_PARAM_PROPAGATION_SECONDS = 0.5
SITL_CONSOLE_COLUMNS = 110
SITL_CONSOLE_ROWS = 26
SITL_LAND_TIMEOUT_SECONDS = 45.0
SITL_QGC_DISCOVERY_GRACE_SECONDS = 2.0
SITL_DISARM_AFTER_LAND_TIMEOUT_SECONDS = 12.0


@dataclass
class _NestedDisplaySession:
    display: str
    window_title: str
    process: subprocess.Popen
    log_fp: object
    log_path: Path

    def shutdown(self) -> None:
        for sig in (signal.SIGTERM, signal.SIGKILL):
            if self.process.poll() is None:
                try:
                    self.process.send_signal(sig)
                except ProcessLookupError:
                    break
                time.sleep(1.0)
            else:
                break
        try:
            self.process.wait(timeout=2.0)
        except Exception:
            pass
        try:
            self.log_fp.close()
        except Exception:
            pass


def _load_candidate_params(candidate_dir: Path) -> dict:
    return json.loads((candidate_dir / "identified_parameters.json").read_text(encoding="utf-8"))


def _estimate_candidate_hover_thrust(
    candidate_dir: Path,
    *,
    esc_min: int,
    esc_max: int,
) -> float | None:
    if esc_max <= esc_min:
        return None

    params = _load_candidate_params(candidate_dir)
    mass_kg = float(params["mass"]["mass_kg"])
    motor_constant = float(params["motor_model"]["motor_constant"]["value"])
    max_rot_velocity = float(params["motor_model"]["max_rot_velocity_radps"]["value"])
    if mass_kg <= 0.0 or motor_constant <= 0.0 or max_rot_velocity <= 0.0:
        return None

    per_rotor_hover_thrust = (mass_kg * 9.80665) / 4.0
    hover_rot_velocity = math.sqrt(per_rotor_hover_thrust / motor_constant)
    hover_thrust = (hover_rot_velocity - float(esc_min)) / float(esc_max - esc_min)
    if not math.isfinite(hover_thrust) or hover_thrust <= 0.20 or hover_thrust >= 0.85:
        return None
    return hover_thrust


def _apply_x500_esc_scaling(mav, *, min_value: int, max_value: int) -> None:
    for motor_idx in range(1, 5):
        set_param(mav, f"SIM_GZ_EC_MIN{motor_idx}", min_value, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, f"SIM_GZ_EC_MAX{motor_idx}", max_value, mavutil.mavlink.MAV_PARAM_TYPE_INT32)


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


def _prepare_reference_world(px4_root: Path, runtime_root: Path) -> tuple[str, Path, Path]:
    source = px4_root / "Tools" / "simulation" / "gz" / "worlds" / "default.sdf"
    override_worlds_root = runtime_root / "override_worlds"
    override_worlds_root.mkdir(parents=True, exist_ok=True)
    target = override_worlds_root / f"{SITL_REFERENCE_WORLD_NAME}.sdf"

    tree = ET.parse(source)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError(f"missing <world> in {source}")

    world.set("name", SITL_REFERENCE_WORLD_NAME)
    for model in list(world.findall("model")):
        if model.attrib.get("name") == SITL_REFERENCE_MARKER_NAME:
            world.remove(model)

    model = ET.SubElement(world, "model", {"name": SITL_REFERENCE_MARKER_NAME})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(
        model,
        "pose",
    ).text = (
        f"{SITL_REFERENCE_MARKER_X:.3f} "
        f"{SITL_REFERENCE_MARKER_Y:.3f} "
        f"{SITL_REFERENCE_MARKER_Z:.3f} 0 0 0"
    )

    link = ET.SubElement(model, "link", {"name": "link"})
    collision = ET.SubElement(link, "collision", {"name": "collision"})
    collision_geometry = ET.SubElement(collision, "geometry")
    collision_cylinder = ET.SubElement(collision_geometry, "cylinder")
    ET.SubElement(collision_cylinder, "radius").text = f"{SITL_REFERENCE_MARKER_RADIUS:.3f}"
    ET.SubElement(collision_cylinder, "length").text = f"{SITL_REFERENCE_MARKER_LENGTH:.3f}"

    visual = ET.SubElement(link, "visual", {"name": "visual"})
    visual_geometry = ET.SubElement(visual, "geometry")
    visual_cylinder = ET.SubElement(visual_geometry, "cylinder")
    ET.SubElement(visual_cylinder, "radius").text = f"{SITL_REFERENCE_MARKER_RADIUS:.3f}"
    ET.SubElement(visual_cylinder, "length").text = f"{SITL_REFERENCE_MARKER_LENGTH:.3f}"
    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = "0.90 0.10 0.10 1"
    ET.SubElement(material, "diffuse").text = "0.90 0.10 0.10 1"
    ET.SubElement(material, "specular").text = "0.25 0.05 0.05 1"
    ET.SubElement(material, "emissive").text = "0.08 0.00 0.00 1"

    _indent(root)
    tree.write(target, encoding="utf-8", xml_declaration=True)
    return SITL_REFERENCE_WORLD_NAME, override_worlds_root, target


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
    _patch_run_rootfs_gcs_link(run_rootfs)


def _patch_run_rootfs_gcs_link(run_rootfs: Path) -> None:
    rc_mavlink = run_rootfs / "etc" / "init.d-posix" / "px4-rc.mavlink"
    if not rc_mavlink.exists():
        return

    source_path = rc_mavlink.resolve() if rc_mavlink.is_symlink() else rc_mavlink
    text = source_path.read_text(encoding="utf-8")
    patched = re.sub(
        r"mavlink start -x -u \$udp_gcs_port_local -r 4000000 -f(?:\s+-p)*",
        "mavlink start -x -u $udp_gcs_port_local -r 4000000 -f -p",
        text,
        count=1,
    )
    if patched != text:
        if rc_mavlink.is_symlink():
            rc_mavlink.unlink()
            rc_mavlink.write_text(patched, encoding="utf-8")
        else:
            rc_mavlink.write_text(patched, encoding="utf-8")


def _patch_gz_env_world_override(run_rootfs: Path, override_worlds_root: Path) -> None:
    gz_env = run_rootfs / "gz_env.sh"
    if not gz_env.exists():
        return

    text = gz_env.read_text(encoding="utf-8")
    worlds_line = f"export PX4_GZ_WORLDS={override_worlds_root.resolve()}"
    text, count = re.subn(
        r"^export PX4_GZ_WORLDS=.*$",
        worlds_line,
        text,
        count=1,
        flags=re.MULTILINE,
    )
    if count == 0:
        text = f"{text.rstrip()}\n{worlds_line}\n"

    gz_env.write_text(text, encoding="utf-8")


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


def _cleanup_background_services() -> None:
    if CLEANUP_SCRIPT.exists():
        try:
            subprocess.run(
                [str(CLEANUP_SCRIPT)],
                cwd=str(REPO_ROOT),
                check=True,
            )
        except subprocess.CalledProcessError:
            pass

    _kill_matching_processes(XEPHYR_PATTERN)
    _cleanup_stale_sitl_processes()


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


def _find_free_x_display_number() -> int:
    used: set[int] = set()
    for path in Path("/tmp/.X11-unix").glob("X*"):
        suffix = path.name[1:]
        if suffix.isdigit():
            used.add(int(suffix))
    for path in Path("/tmp").glob(".X*-lock"):
        match = re.fullmatch(r"\.X(\d+)-lock", path.name)
        if match is not None:
            used.add(int(match.group(1)))
    for display_num in range(SITL_NESTED_DISPLAY_MIN, SITL_NESTED_DISPLAY_MAX + 1):
        if display_num not in used:
            return display_num
    raise RuntimeError("no free nested X display numbers available")


def _wait_for_x_display(display: str, *, timeout_s: float = 10.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if shutil.which("xdpyinfo"):
            probe = subprocess.run(
                ["xdpyinfo", "-display", display],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
            if probe.returncode == 0:
                return True
        elif Path(f"/tmp/.X11-unix/X{display.removeprefix(':')}").exists():
            return True
        time.sleep(0.25)
    return False


def _window_exists_by_title(title: str) -> bool:
    if shutil.which("wmctrl"):
        proc = subprocess.run(
            ["wmctrl", "-lx"],
            capture_output=True,
            text=True,
            check=False,
        )
        if title.lower() in proc.stdout.lower():
            return True

    if shutil.which("xdotool"):
        proc = subprocess.run(
            ["xdotool", "search", "--name", title],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if proc.returncode == 0:
            return True

    return False


def _wait_for_window_by_title(title: str, *, timeout_s: float = 5.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if _window_exists_by_title(title):
            return True
        time.sleep(0.25)
    return False


def _start_nested_visual_display(runtime_root: Path) -> _NestedDisplaySession | None:
    xephyr = shutil.which("Xephyr")
    if xephyr is None:
        return None

    display_num = _find_free_x_display_number()
    display = f":{display_num}"
    log_path = runtime_root / "xephyr.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_fp = log_path.open("w", encoding="utf-8", buffering=1)
    process = subprocess.Popen(
        [
            xephyr,
            display,
            "-screen",
            f"{SITL_NESTED_DISPLAY_WIDTH}x{SITL_NESTED_DISPLAY_HEIGHT}x24",
            "-title",
            SITL_NESTED_DISPLAY_TITLE,
            "-resizeable",
            "-br",
            "-ac",
            "-nolisten",
            "tcp",
        ],
        stdin=subprocess.DEVNULL,
        stdout=log_fp,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    if not _wait_for_x_display(display):
        try:
            process.terminate()
        except Exception:
            pass
        try:
            process.wait(timeout=2.0)
        except Exception:
            pass
        try:
            log_fp.close()
        except Exception:
            pass
        log_tail = ""
        try:
            log_tail = log_path.read_text(encoding="utf-8", errors="ignore")[-4000:]
        except OSError:
            pass
        raise RuntimeError(f"nested Gazebo display did not become ready on {display}\n{log_tail}")

    # If the host session never gets a visible Xephyr window, fall back to the
    # normal Gazebo GUI path instead of silently running without a visible view.
    if not _wait_for_window_by_title(SITL_NESTED_DISPLAY_TITLE, timeout_s=5.0):
        try:
            process.terminate()
        except Exception:
            pass
        try:
            process.wait(timeout=2.0)
        except Exception:
            pass
        try:
            log_fp.close()
        except Exception:
            pass
        return None

    return _NestedDisplaySession(
        display=display,
        window_title=SITL_NESTED_DISPLAY_TITLE,
        process=process,
        log_fp=log_fp,
        log_path=log_path,
    )


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


def _focus_window_by_title(title: str) -> None:
    if not shutil.which("wmctrl"):
        return
    subprocess.run(
        ["wmctrl", "-a", title],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    subprocess.run(
        ["wmctrl", "-R", title],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def _configure_visual_camera_follow(
    *,
    model_name: str,
    env: dict[str, str],
    follow_x: float = VISUAL_FOLLOW_OFFSET_X,
    follow_y: float = VISUAL_FOLLOW_OFFSET_Y,
    follow_z: float = VISUAL_FOLLOW_OFFSET_Z,
    follow_pgain: float = VISUAL_FOLLOW_PGAIN,
    track_pgain: float = VISUAL_TRACK_PGAIN,
    px4_instance: int = 0,
    attempts: int = 8,
    retry_delay_s: float = 0.5,
) -> bool:
    if env.get("PX4_GZ_NO_FOLLOW"):
        return False

    gz = shutil.which("gz")
    if gz is None:
        return False

    model_instance = f"{model_name}_{px4_instance}"
    payload = (
        f"track_mode: FOLLOW, follow_target: {{name: '{model_instance}'}}, "
        f"follow_offset: {{x: {follow_x}, y: {follow_y}, z: {follow_z}}}, "
        f"follow_pgain: {follow_pgain}, track_pgain: {track_pgain}"
    )

    for _ in range(max(1, attempts)):
        proc = subprocess.run(
            [gz, "topic", "-t", "/gui/track", "-m", "gz.msgs.CameraTrack", "-p", payload],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=env,
            check=False,
        )
        if proc.returncode == 0:
            return True
        time.sleep(retry_delay_s)

    return False


def _set_window_above(title: str, *, enabled: bool = True) -> None:
    if not shutil.which("wmctrl"):
        return
    action = "add,above" if enabled else "remove,above"
    subprocess.run(
        ["wmctrl", "-r", title, "-b", action],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def _set_window_sticky(title: str, *, enabled: bool = True) -> None:
    if not shutil.which("wmctrl"):
        return
    action = "add,sticky" if enabled else "remove,sticky"
    subprocess.run(
        ["wmctrl", "-r", title, "-b", action],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def _current_desktop_index() -> int | None:
    if not shutil.which("wmctrl"):
        return None

    proc = subprocess.run(
        ["wmctrl", "-d"],
        capture_output=True,
        text=True,
        check=False,
    )
    for line in proc.stdout.splitlines():
        if "*" not in line:
            continue
        fields = line.split()
        if not fields:
            continue
        try:
            return int(fields[0])
        except ValueError:
            continue
    return None


def _restore_window_by_title(title: str) -> None:
    desktop = _current_desktop_index()
    if shutil.which("wmctrl") and desktop is not None:
        subprocess.run(
            ["wmctrl", "-r", title, "-t", str(desktop)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        subprocess.run(
            ["wmctrl", "-r", title, "-b", "remove,hidden,shaded,skip_taskbar,skip_pager"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )

    if not shutil.which("xdotool"):
        return

    search = subprocess.run(
        ["xdotool", "search", "--name", title],
        capture_output=True,
        text=True,
        check=False,
    )
    window_ids = [line.strip() for line in search.stdout.splitlines() if line.strip()]
    if not window_ids:
        return

    window_id = window_ids[-1]
    for cmd in (
        ["xdotool", "windowmap", window_id],
        ["xdotool", "windowraise", window_id],
        ["xdotool", "windowactivate", "--sync", window_id],
    ):
        subprocess.run(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )


def _promote_window(title: str, *, attempts: int = 30, delay_s: float = 1.0) -> threading.Thread | None:
    if not shutil.which("wmctrl") and not shutil.which("xdotool"):
        return None

    def _worker() -> None:
        for _ in range(max(1, attempts)):
            _restore_window_by_title(title)
            _set_window_sticky(title, enabled=True)
            _set_window_above(title, enabled=True)
            _focus_window_by_title(title)
            time.sleep(max(0.1, delay_s))

    thread = threading.Thread(target=_worker, name=f"promote-window-{title}", daemon=True)
    thread.start()
    return thread


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
            f"--geometry={SITL_CONSOLE_COLUMNS}x{SITL_CONSOLE_ROWS}+0+0",
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
    env["GZ_IP"] = "127.0.0.1"
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
        if SITL_VISUAL_DISABLE_FOLLOW:
            env.setdefault("PX4_GZ_NO_FOLLOW", "1")
        else:
            env.setdefault("PX4_GZ_FOLLOW_OFFSET_X", str(VISUAL_FOLLOW_OFFSET_X))
            env.setdefault("PX4_GZ_FOLLOW_OFFSET_Y", str(VISUAL_FOLLOW_OFFSET_Y))
            env.setdefault("PX4_GZ_FOLLOW_OFFSET_Z", str(VISUAL_FOLLOW_OFFSET_Z))
    return env


def _wait_for_ready_for_takeoff(session: "Px4SitlSession", *, timeout_s: float = SITL_READY_FOR_TAKEOFF_TIMEOUT_SECONDS) -> None:
    try:
        if session._console_log.exists():
            text = session._console_log.read_text(encoding="utf-8", errors="ignore")
            if "Ready for takeoff!" in text:
                return
    except OSError:
        pass
    try:
        session.expect("Ready for takeoff!", timeout_s=timeout_s)
    except pexpect.TIMEOUT:
        return


def _wait_for_preflight_ok(session: "Px4SitlSession", *, timeout_s: float = SITL_READY_FOR_TAKEOFF_TIMEOUT_SECONDS) -> None:
    deadline = time.time() + timeout_s
    last_response = ""
    while time.time() < deadline:
        response = session.send("commander check", timeout_s=10.0)
        last_response = response
        if "Preflight check: OK" in response:
            return
        time.sleep(1.0)
    health = session.send("listener health_report 1", timeout_s=10.0)
    failsafe = session.send("listener failsafe_flags 1", timeout_s=10.0)
    raise RuntimeError(
        "vehicle did not report a clean preflight state in time\n"
        f"last commander check:\n{last_response}\n"
        f"health_report:\n{health}\n"
        f"failsafe_flags:\n{failsafe}"
    )


def _arm_via_internal_command(
    session: "Px4SitlSession",
    mav,
    *,
    attempts: int = SITL_ARM_ATTEMPTS,
    timeout_s: float = 8.0,
) -> None:
    last_error: RuntimeError = RuntimeError("vehicle did not arm")
    for attempt in range(1, attempts + 1):
        response = ""
        session.send_no_wait("commander arm")
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if mav.motors_armed():
                try:
                    session.sync_prompt(timeout_s=1.0)
                except Exception:
                    pass
                return
            mav.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=1.0)

        detail = " ".join(line.strip() for line in response.splitlines() if line.strip())
        if not detail:
            detail = "commander arm did not arm vehicle"
        try:
            check = session.send("commander check", timeout_s=10.0)
            health = session.send("listener health_report 1", timeout_s=10.0)
            failsafe = session.send("listener failsafe_flags 1", timeout_s=10.0)
            detail = (
                f"{detail}\n"
                f"commander check:\n{check}\n"
                f"health_report:\n{health}\n"
                f"failsafe_flags:\n{failsafe}"
            )
        except Exception:
            pass
        print(f"Arm attempt {attempt}/{attempts} failed: {detail}", flush=True)
        last_error = RuntimeError(detail)

    raise last_error


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
        log_fp = self._console_log.open("w", encoding="utf-8", buffering=1)
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
        self._wait_for_vehicle_heartbeat(timeout_s=15.0)

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

    def _wait_for_vehicle_heartbeat(self, *, timeout_s: float = 15.0) -> None:
        assert self._mav is not None
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = self._mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if msg is not None:
                return

        console_tail = ""
        try:
            if self._console_log.exists():
                console_tail = self._console_log.read_text(encoding="utf-8", errors="ignore")[-4000:]
        except OSError:
            pass
        raise RuntimeError(f"vehicle heartbeat was not received in time\n{console_tail}")

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
        self._child.expect(PX4_NEXT_PROMPT_RE, timeout=timeout_s or self._timeout_s)
        return self._child.before

    def send_no_wait(self, command: str) -> None:
        assert self._child is not None
        self._child.sendline(command)

    def sync_prompt(self, *, timeout_s: float = 5.0) -> None:
        assert self._child is not None
        self._child.expect(PX4_NEXT_PROMPT_RE, timeout=timeout_s)

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
            self.send_no_wait("shutdown")
            self._child.expect(pexpect.EOF, timeout=3.0)
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


def _capture_locked_yaw(mav, *, timeout_s: float = 5.0) -> float:
    return _wrap_pi(float(_sample_attitude(mav, timeout_s=timeout_s).yaw))


class _ManualControlThread:
    def __init__(self, mav, *, period_s: float = SITL_MANUAL_CONTROL_PERIOD) -> None:
        self._mav = mav
        self._period_s = period_s
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._x = 0
        self._y = 0
        self._z = 0
        self._r = 0
        self._thread = threading.Thread(target=self._run, name="sitl-manual-control", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def update(
        self,
        *,
        x: int | None = None,
        y: int | None = None,
        z: int | None = None,
        r: int | None = None,
    ) -> None:
        with self._lock:
            if x is not None:
                self._x = int(max(-1000, min(1000, x)))
            if y is not None:
                self._y = int(max(-1000, min(1000, y)))
            if z is not None:
                self._z = int(max(0, min(1000, z)))
            if r is not None:
                self._r = int(max(-1000, min(1000, r)))

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)

    def _run(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                x = self._x
                y = self._y
                z = self._z
                r = self._r
            self._mav.mav.manual_control_send(
                self._mav.target_system,
                x,
                y,
                z,
                r,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            self._stop.wait(self._period_s)


def _wrap_pi(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _manual_rudder_for_yaw(current_yaw: float, target_yaw: float) -> int:
    yaw_error = _wrap_pi(target_yaw - current_yaw)
    if abs(yaw_error) < math.radians(1.5):
        return 0
    command = yaw_error * 380.0
    return int(max(-250.0, min(250.0, command)))


def _throttle_for_target(current_z: float, target_z: float) -> int:
    remaining = current_z - target_z
    if remaining >= 0.0:
        command = 500.0 + remaining * 120.0
        if remaining > 0.8:
            command = max(command, 660.0)
        elif remaining > 0.3:
            command = max(command, 560.0)
    else:
        command = 500.0 + remaining * 220.0
        if remaining < -0.5:
            command = min(command, 380.0)
        if remaining < -1.5:
            command = min(command, 180.0)
        if remaining < -3.0:
            command = min(command, 0.0)

    return int(max(0.0, min(float(SITL_MANUAL_MAX_THROTTLE), command)))


def _wait_for_takeoff_hover(
    mav,
    *,
    manual_control: _ManualControlThread | None = None,
    target_z: float,
    target_yaw: float | None = None,
    timeout_s: float = SITL_HOVER_TIMEOUT_SECONDS,
    z_tolerance_m: float = SITL_TAKEOFF_Z_TOLERANCE_M,
    max_horiz_speed_mps: float = SITL_TAKEOFF_MAX_HORIZ_SPEED_MPS,
    max_vert_speed_mps: float = SITL_TAKEOFF_MAX_VERT_SPEED_MPS,
    max_tilt_deg: float = SITL_TAKEOFF_TILT_LIMIT_DEG,
    stable_window_s: float = SITL_TAKEOFF_STABLE_WINDOW_SECONDS,
    report_period: float = SITL_REPORT_PERIOD,
):
    deadline = time.time() + timeout_s
    latest_lpos = None
    latest_att = None
    stable_since = None
    next_report = 0.0
    last_throttle_cmd = 500
    last_rudder_cmd = 0
    hover_capture_active = False

    while time.time() < deadline:
        msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=0.5)
        if not msg:
            continue

        msg_type = msg.get_type()
        if msg_type == "STATUSTEXT":
            print(decode_statustext(msg), flush=True)
            continue
        if msg_type == "LOCAL_POSITION_NED":
            latest_lpos = msg
            if manual_control is not None:
                if hover_capture_active or float(msg.z) <= (target_z + SITL_MANUAL_CLIMB_ENTRY_Z_MARGIN):
                    hover_capture_active = True
                    last_throttle_cmd = SITL_MANUAL_CAPTURE_THROTTLE
                else:
                    last_throttle_cmd = _throttle_for_target(float(msg.z), target_z)
        elif msg_type == "ATTITUDE":
            latest_att = msg
            if manual_control is not None and target_yaw is not None:
                last_rudder_cmd = _manual_rudder_for_yaw(float(msg.yaw), target_yaw)

        if manual_control is not None:
            manual_control.update(z=last_throttle_cmd, r=last_rudder_cmd)

        now = time.time()
        if latest_lpos is not None and latest_att is not None and now >= next_report:
            print(
                f"Takeoff hover x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                f"vx={float(latest_lpos.vx):.3f} vy={float(latest_lpos.vy):.3f} vz={float(latest_lpos.vz):.3f} "
                f"thr={last_throttle_cmd} rud={last_rudder_cmd} "
                f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f} "
                f"yaw={math.degrees(float(latest_att.yaw)):.2f}",
                flush=True,
            )
            next_report = now + report_period

        if latest_lpos is None or latest_att is None:
            continue

        z_err = abs(float(latest_lpos.z) - target_z)
        horiz_speed = math.hypot(float(latest_lpos.vx), float(latest_lpos.vy))
        vert_speed = abs(float(latest_lpos.vz))
        tilt_deg = math.degrees(max(abs(float(latest_att.roll)), abs(float(latest_att.pitch))))

        stable = (
            z_err <= z_tolerance_m
            and horiz_speed <= max_horiz_speed_mps
            and vert_speed <= max_vert_speed_mps
            and tilt_deg <= max_tilt_deg
        )

        if stable:
            stable_since = stable_since or now
            if (now - stable_since) >= stable_window_s:
                return latest_lpos, latest_att
        else:
            stable_since = None

    raise RuntimeError(
        f"Vehicle did not settle into hover around z={target_z:.3f} "
        f"(last_lpos={latest_lpos}, last_att={latest_att})"
    )


def _landing_throttle_for_altitude(current_z: float, ground_z_m: float) -> int:
    altitude_agl = max(0.0, ground_z_m - current_z)
    if altitude_agl > 3.0:
        return 320
    if altitude_agl > 1.5:
        return 350
    if altitude_agl > 0.7:
        return 390
    if altitude_agl > 0.25:
        return 435
    return 465


def _land_in_posctl_with_manual_control(
    mav,
    *,
    manual_control: _ManualControlThread,
    ground_z_m: float,
    target_yaw: float,
    timeout_s: float = SITL_LAND_TIMEOUT_SECONDS,
    z_tolerance_m: float = 0.18,
    speed_tol_mps: float = 0.30,
    report_period: float = SITL_REPORT_PERIOD,
) -> dict[str, float]:
    deadline = time.time() + timeout_s
    next_report = 0.0
    latest_lpos = None
    latest_att = None
    landed_streak = 0
    throttle_cmd = 480
    rudder_cmd = 0
    touchdown_reported = False

    while time.time() < deadline:
        msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=0.3)
        if not msg:
            manual_control.update(z=throttle_cmd, r=rudder_cmd)
            continue

        msg_type = msg.get_type()
        if msg_type == "STATUSTEXT":
            text = decode_statustext(msg)
            print(text, flush=True)
            continue
        if msg_type == "LOCAL_POSITION_NED":
            latest_lpos = msg
            throttle_cmd = _landing_throttle_for_altitude(float(msg.z), ground_z_m)
        elif msg_type == "ATTITUDE":
            latest_att = msg
            rudder_cmd = 0

        manual_control.update(z=throttle_cmd, r=rudder_cmd)

        now = time.time()
        if latest_lpos is not None and latest_att is not None and now >= next_report:
            print(
                f"Landing x={float(latest_lpos.x):.3f} y={float(latest_lpos.y):.3f} z={float(latest_lpos.z):.3f} "
                f"vx={float(latest_lpos.vx):.3f} vy={float(latest_lpos.vy):.3f} vz={float(latest_lpos.vz):.3f} "
                f"thr={throttle_cmd} rud={rudder_cmd} "
                f"roll={math.degrees(float(latest_att.roll)):.2f} pitch={math.degrees(float(latest_att.pitch)):.2f} "
                f"yaw={math.degrees(float(latest_att.yaw)):.2f}",
                flush=True,
            )
            next_report = now + report_period

        if latest_lpos is None:
            continue

        speed = math.sqrt(float(latest_lpos.vx) ** 2 + float(latest_lpos.vy) ** 2 + float(latest_lpos.vz) ** 2)
        if abs(float(latest_lpos.z) - ground_z_m) <= z_tolerance_m and speed <= speed_tol_mps:
            landed_streak += 1
            if landed_streak >= 10:
                if not touchdown_reported:
                    print("Touchdown detected; waiting for auto-disarm", flush=True)
                    touchdown_reported = True
                manual_control.update(z=0, r=0)
                disarm_deadline = time.time() + SITL_DISARM_AFTER_LAND_TIMEOUT_SECONDS
                while time.time() < disarm_deadline:
                    if not mav.motors_armed():
                        return {
                            "x": float(latest_lpos.x),
                            "y": float(latest_lpos.y),
                            "z": float(latest_lpos.z),
                            "speed_mps": speed,
                            "disarmed": True,
                        }
                    heartbeat = mav.recv_match(type=["HEARTBEAT", "LOCAL_POSITION_NED", "STATUSTEXT"], blocking=True, timeout=0.5)
                    if heartbeat is not None and heartbeat.get_type() == "LOCAL_POSITION_NED":
                        latest_lpos = heartbeat
                        speed = math.sqrt(float(latest_lpos.vx) ** 2 + float(latest_lpos.vy) ** 2 + float(latest_lpos.vz) ** 2)
                        manual_control.update(z=0, r=0)
                return {
                    "x": float(latest_lpos.x),
                    "y": float(latest_lpos.y),
                    "z": float(latest_lpos.z),
                    "speed_mps": speed,
                    "disarmed": False,
                }
        else:
            landed_streak = 0

    raise RuntimeError(f"vehicle did not land in {timeout_s:.1f}s (last={latest_lpos})")


def _stabilize_direct_hover(
    mav,
    *,
    ref_x: float,
    ref_y: float,
    ref_z: float,
    yaw: float,
    timeout_s: float = SITL_DIRECT_SETTLE_TIMEOUT_SECONDS,
) -> tuple[object, object]:
    deadline = time.time() + timeout_s
    latest_lpos = None
    latest_att = None
    stable_streak = 0

    while time.time() < deadline:
        send_position_target_local_ned(mav, ref_x, ref_y, ref_z, yaw)
        msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=SITL_SETPOINT_PERIOD)
        if msg is None:
            stable_streak = 0
            continue

        mtype = msg.get_type()
        if mtype == "STATUSTEXT":
            continue
        if mtype == "LOCAL_POSITION_NED":
            latest_lpos = msg
        elif mtype == "ATTITUDE":
            latest_att = msg

        if latest_lpos is None or latest_att is None:
            stable_streak = 0
            continue

        z_err = abs(float(latest_lpos.z) - ref_z)
        vz = abs(float(latest_lpos.vz))
        tilt = math.degrees(max(abs(float(latest_att.roll)), abs(float(latest_att.pitch))))
        if z_err <= SITL_DIRECT_SETTLE_Z_TOLERANCE and vz <= SITL_DIRECT_SETTLE_VZ_TOLERANCE and tilt <= SITL_DIRECT_TILT_LIMIT_DEG:
            stable_streak += 1
            if stable_streak >= 10:
                return latest_lpos, latest_att
        else:
            stable_streak = 0

    if latest_lpos is None or latest_att is None:
        raise RuntimeError("direct hover settle phase did not produce telemetry")
    return latest_lpos, latest_att


def _start_direct_setpoint_stream(
    mav,
    *,
    ref_x: float,
    ref_y: float,
    ref_z: float,
    yaw: float,
) -> tuple[threading.Event, threading.Thread]:
    stop_event = threading.Event()

    def _loop() -> None:
        while not stop_event.is_set():
            send_position_target_local_ned(mav, ref_x, ref_y, ref_z, yaw)
            stop_event.wait(SITL_SETPOINT_PERIOD)

    thread = threading.Thread(target=_loop, daemon=True)
    thread.start()
    return stop_event, thread


def _find_tracking_log(run_rootfs: Path, relative_path: str) -> Path:
    rel = relative_path.removeprefix("./")
    path = run_rootfs / rel
    if not path.exists():
        raise FileNotFoundError(f"tracking log announced by PX4 not found: {path}")
    return path


def _tracking_log_error_metrics(path: Path) -> dict[str, float]:
    max_xy_err = 0.0
    max_z_err = 0.0
    sum_sq = 0.0
    count = 0

    with path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            dx = float(row["pos_x"]) - float(row["ref_x"])
            dy = float(row["pos_y"]) - float(row["ref_y"])
            dz = float(row["pos_z"]) - float(row["ref_z"])
            max_xy_err = max(max_xy_err, math.hypot(dx, dy))
            max_z_err = max(max_z_err, abs(dz))
            sum_sq += dx * dx + dy * dy + dz * dz
            count += 1

    if count == 0:
        raise RuntimeError(f"tracking log is empty: {path}")

    return {
        "max_xy_err_m": max_xy_err,
        "max_z_err_m": max_z_err,
        "rmse_m": math.sqrt(sum_sq / count),
    }


def run_validation_model(
    *,
    px4_root: str | Path,
    out_root: str | Path,
    model_spec: ValidationModelSpec,
    candidate_dir: str | Path,
    trajectories: Iterable[ValidationTrajectory] = DEFAULT_VALIDATION_TRAJECTORIES,
    sitl_esc_min: int = SITL_ESC_MIN_DEFAULT,
    sitl_esc_max: int | None = None,
    sitl_hover_thrust: float | None = None,
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

    _cleanup_background_services()
    _prepare_run_rootfs(base_rootfs, run_rootfs)
    if show_console and not headless:
        _open_console_window(run_rootfs / "px4_console.log")
    export_manifest = export_validation_trajectories(
        run_rootfs / "trajectories",
        freeze_yaw=SITL_FREEZE_TRAJECTORY_YAW,
        yaw_value=0.0,
    )
    if model_spec.label != "stock_sitl_placeholder":
        _prepare_model_override(px4_root, model_spec.gz_model, override_models_root)
    env = _build_px4_env(px4_root, build_dir, run_rootfs, override_models_root, model_spec.gz_model, headless)
    world_name, override_worlds_root, world_sdf = _prepare_reference_world(px4_root, runtime_root)
    _patch_gz_env_world_override(run_rootfs, override_worlds_root)
    env["PX4_GZ_WORLDS"] = str(override_worlds_root.resolve())
    env["GZ_SIM_RESOURCE_PATH"] = _append_path_list(env.get("GZ_SIM_RESOURCE_PATH", ""), override_worlds_root)
    env["PX4_GZ_WORLD"] = world_name

    display_session: _NestedDisplaySession | None = None
    if not headless:
        display_session = _start_nested_visual_display(runtime_root)
        if display_session is not None:
            env["DISPLAY"] = display_session.display
            print(f"Using nested Gazebo display {display_session.display}", flush=True)

    _cleanup_stale_sitl_processes()
    session: Px4SitlSession | None = None
    common_anchor = COMMON_LOGGED_START
    results: list[dict] = []
    manual_control: _ManualControlThread | None = None
    try:
        session = _start_session_with_retries(px4_root=px4_root, run_rootfs=run_rootfs, env=env)
        if not headless:
            time.sleep(1.0)
            visual_window_title = display_session.window_title if display_session is not None else "Gazebo Sim"
            _configure_visual_camera_follow(model_name=model_spec.gz_model, env=env)
            _promote_window(visual_window_title)
        assert session._mav is not None
        mav = session._mav
        if sitl_esc_max is not None:
            _apply_x500_esc_scaling(mav, min_value=sitl_esc_min, max_value=sitl_esc_max)
        if sitl_hover_thrust is not None:
            set_param(mav, "MPC_THR_HOVER", sitl_hover_thrust, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            print(f"Using explicit MPC_THR_HOVER={sitl_hover_thrust:.3f}", flush=True)
        if model_spec.label == "digital_twin" and sitl_esc_max is not None:
            hover_thrust = _estimate_candidate_hover_thrust(candidate_dir, esc_min=sitl_esc_min, esc_max=sitl_esc_max)
            if hover_thrust is not None:
                set_param(mav, "MPC_THR_HOVER", hover_thrust, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                print(
                    f"Derived MPC_THR_HOVER={hover_thrust:.3f} from candidate mass/motor model",
                    flush=True,
                )
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE)
        request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, interval_us=500_000)
        set_param(mav, "COM_CPU_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "COM_RAM_MAX", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_POS_ABS", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_POS_X", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Y", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_Z", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "TRJ_POS_YAW", SITL_LOCKED_YAW_RAD, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        wait_for_sim_ready(mav, timeout=20.0, require_local_position=True, min_local_samples=3)
        wait_for_ground_quiet(mav, duration_s=2.0, timeout=SITL_GROUND_QUIET_TIMEOUT_SECONDS)
        baseline_x, baseline_y, baseline_z = robust_ground_baseline(mav)
        ground_xy = math.hypot(baseline_x, baseline_y)
        if ground_xy > SITL_GROUND_XY_WINDOW or abs(baseline_z) > SITL_GROUND_Z_WINDOW:
            raise RuntimeError(
                f"Dirty ground baseline: x={baseline_x:.3f} y={baseline_y:.3f} z={baseline_z:.3f}. "
                "Restart SITL clean before running validation."
            )

        set_param(mav, "COM_RC_IN_MODE", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "NAV_DLL_ACT", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CBRK_SUPPLY_CHK", 894281, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "COM_DISARM_PRFLT", -1.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        set_param(mav, "MAV_0_BROADCAST", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "MIS_TAKEOFF_ALT", abs(SITL_HOVER_Z), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        if SITL_PARAM_PROPAGATION_SECONDS > 0.0:
            time.sleep(SITL_PARAM_PROPAGATION_SECONDS)

        hover_yaw = _capture_locked_yaw(mav)
        set_param(mav, "TRJ_POS_YAW", hover_yaw, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

        _wait_for_ready_for_takeoff(session)
        _wait_for_preflight_ok(session)
        if not headless and SITL_QGC_DISCOVERY_GRACE_SECONDS > 0.0:
            time.sleep(SITL_QGC_DISCOVERY_GRACE_SECONDS)
        _arm_via_internal_command(session, mav, attempts=SITL_ARM_ATTEMPTS)
        manual_control = _ManualControlThread(mav)
        manual_control.update(x=0, y=0, z=500, r=0)
        manual_control.start()
        set_param(mav, "COM_RC_IN_MODE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        if SITL_PARAM_PROPAGATION_SECONDS > 0.0:
            time.sleep(SITL_PARAM_PROPAGATION_SECONDS)
        session.set_mode("POSCTL")
        time.sleep(1.0)
        latest_lpos, latest_att = _wait_for_takeoff_hover(
            mav,
            manual_control=manual_control,
            target_z=SITL_HOVER_Z,
            target_yaw=hover_yaw,
            timeout_s=SITL_HOVER_TIMEOUT_SECONDS,
        )
        manual_control.update(z=500, r=0)
        if SITL_TAKEOFF_SETTLE_SECONDS > 0.0:
            time.sleep(SITL_TAKEOFF_SETTLE_SECONDS)

        hold_x = float(latest_lpos.x)
        hold_y = float(latest_lpos.y)
        hold_z = float(latest_lpos.z)

        set_position_target_absolute(
            mav,
            hold_x,
            hold_y,
            hold_z,
            hover_yaw,
        )
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "TRJ_POS_YAW", hover_yaw, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        stream_stop, stream_thread = _start_direct_setpoint_stream(
            mav,
            ref_x=hold_x,
            ref_y=hold_y,
            ref_z=hold_z,
            yaw=hover_yaw,
        )
        try:
            pre_offboard_end = time.time() + max(0.5, SITL_PRE_OFFBOARD_SECONDS)
            while time.time() < pre_offboard_end:
                mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=SITL_SETPOINT_PERIOD)

            set_offboard(mav)
            latest_lpos, latest_att = _stabilize_direct_hover(
                mav,
                ref_x=hold_x,
                ref_y=hold_y,
                ref_z=hold_z,
                yaw=hover_yaw,
            )
            set_param(mav, "CST_POS_CTRL_TYP", 4, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            set_param(mav, "CST_POS_CTRL_EN", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            session.send("custom_pos_control start")
            session.send("trajectory_reader start")
            session.send(
                "trajectory_reader abs_ref "
                f"{float(latest_lpos.x):.3f} "
                f"{float(latest_lpos.y):.3f} "
                f"{float(latest_lpos.z):.3f} "
                f"{hover_yaw:.3f}"
            )
        finally:
            stream_stop.set()
            stream_thread.join(timeout=1.0)

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
            if custom_failsafe or not SITL_ALLOW_UNSTABLE_CUSTOM_HOLD:
                raise RuntimeError(
                    f"Custom hold unstable: max_xy={custom_max_xy:.3f}m "
                    f"max_z_err={custom_max_z_err:.3f}m max_tilt={custom_max_tilt:.2f}deg "
                    f"failsafe={custom_failsafe}"
                )
            print(
                f"Custom hold gate bypassed: max_xy={custom_max_xy:.3f}m "
                f"max_z_err={custom_max_z_err:.3f}m max_tilt={custom_max_tilt:.2f}deg",
                flush=True,
            )

        common_anchor = (
            float(latest_lpos.x),
            float(latest_lpos.y),
            float(latest_lpos.z),
        )
        common_yaw = hover_yaw

        for entry in trajectories:
            set_position_target_absolute(mav, common_anchor[0], common_anchor[1], common_anchor[2], common_yaw)
            set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            set_anchor_from_position(mav, common_anchor[0], common_anchor[1], common_anchor[2])
            set_param(mav, "TRJ_ACTIVE_ID", entry.traj_id, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            set_param(mav, "TRJ_MODE_CMD", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            match = session.expect(TRACKING_LOG_START_RE, timeout_s=20)
            trajectory_duration_s = float(TRAJ_DURATION_S.get(entry.traj_id, entry.nominal_duration_s))
            latest_lpos, latest_att, traj_max_xy, traj_max_z_err, traj_max_tilt, traj_failsafe = observe_builtin_trajectory(
                mav,
                duration_s=trajectory_duration_s + SITL_TRAJECTORY_TAIL_SECONDS,
                ref_z=common_anchor[2],
                report_period=SITL_REPORT_PERIOD,
            )
            session.expect(TRAJECTORY_DONE_TEXT, timeout_s=max(20.0, trajectory_duration_s))
            assert match is not None
            tracking_log = _find_tracking_log(run_rootfs, match.group(1))
            copied_log = _copy_log(tracking_log, result_root / "tracking_logs" / f"{entry.name}.csv")
            tracking_metrics = _tracking_log_error_metrics(tracking_log)
            if (
                traj_failsafe
                or tracking_metrics["max_xy_err_m"] > SITL_TRAJECTORY_XY_TRACKING_ERROR_LIMIT
                or tracking_metrics["max_z_err_m"] > SITL_TRAJECTORY_Z_TOLERANCE
                or traj_max_tilt > SITL_TRAJECTORY_TILT_LIMIT_DEG
            ):
                raise RuntimeError(
                    f"Built-in trajectory unstable for {entry.name}: max_xy_err={tracking_metrics['max_xy_err_m']:.3f}m "
                    f"max_z_err={tracking_metrics['max_z_err_m']:.3f}m rmse={tracking_metrics['rmse_m']:.3f}m "
                    f"max_tilt={traj_max_tilt:.2f}deg "
                    f"failsafe={traj_failsafe}"
                )
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

        manual_control.update(x=0, y=0, z=500, r=0)
        time.sleep(1.0)
        session.set_mode("POSCTL")
        set_param(mav, "TRJ_MODE_CMD", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_param(mav, "CST_POS_CTRL_EN", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        land_result = _land_in_posctl_with_manual_control(
            mav,
            manual_control=manual_control,
            ground_z_m=float(baseline_z),
            target_yaw=common_yaw,
        )
        print(
            f"Landing complete x={land_result['x']:.3f} y={land_result['y']:.3f} "
            f"z={land_result['z']:.3f} speed={land_result['speed_mps']:.3f} "
            f"disarmed={land_result['disarmed']}",
            flush=True,
        )
    finally:
        if manual_control is not None:
            manual_control.stop()
        if session is not None:
            session.shutdown()
        if display_session is not None:
            display_session.shutdown()
        _cleanup_stale_sitl_processes()
        if (run_rootfs / "sysid_truth_logs").exists():
            shutil.copytree(run_rootfs / "sysid_truth_logs", result_root / "sysid_truth_logs", dirs_exist_ok=True)

    payload = {
        "label": model_spec.label,
        "display_name": model_spec.display_name,
        "gz_model": model_spec.gz_model,
        "world_name": world_name,
        "world_sdf": str(world_sdf.resolve()),
        "common_logged_start": {
            "x_m": common_anchor[0],
            "y_m": common_anchor[1],
            "z_m": common_anchor[2],
        },
        "export_manifest": export_manifest,
        "results": results,
        "console_log": str((run_rootfs / "px4_console.log").resolve()),
        "nested_display": display_session.display if display_session is not None else None,
        "nested_display_log": str(display_session.log_path.resolve()) if display_session is not None else None,
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
    sitl_hover_thrust: float | None = None,
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
                sitl_hover_thrust=sitl_hover_thrust,
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
    ap.add_argument(
        "--sitl-hover-thrust",
        type=float,
        default=None,
        help="Optional MPC_THR_HOVER override for SITL tuning experiments.",
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

    try:
        summary = run_validation_suite(
            px4_root=args.px4_root,
            out_root=args.out_root,
            candidate_dir=args.candidate_dir,
            trajectories=_resolve_trajectories(trajectory_names),
            model_specs=_resolve_model_specs(model_labels),
            sitl_esc_min=args.sitl_esc_min,
            sitl_esc_max=args.sitl_esc_max,
            sitl_hover_thrust=args.sitl_hover_thrust,
            headless=not args.visual,
            show_console=args.show_console,
        )
    except KeyboardInterrupt:
        print("Interrupted; cleaning PX4/Gazebo background services.", file=sys.stderr, flush=True)
        _cleanup_background_services()
        return 130

    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
