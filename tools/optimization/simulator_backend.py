#!/usr/bin/env python3
"""Simulator launch helpers shared by worker execution and replay."""

from __future__ import annotations

import os
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree as ET


DEFAULT_SIMULATOR = "gz"
DEFAULT_GZ_VEHICLE = "x500"
DEFAULT_GZ_WORLD = "default"


@dataclass(frozen=True)
class SimulatorLaunchSpec:
    simulator: str
    display_name: str
    process_label: str
    command: list[str]
    environment: dict[str, str]
    px4_environment: dict[str, str]
    cleanup_hints: list[str]


def normalize_simulator_name(raw: object) -> str:
    text = str(raw or "").strip().lower()
    return text or DEFAULT_SIMULATOR


def available_simulators() -> list[dict]:
    return [
        {
            "name": "gz",
            "display_name": "Gazebo",
            "vehicle_default": DEFAULT_GZ_VEHICLE,
            "world_default": DEFAULT_GZ_WORLD,
            "description": "Modern Gazebo (gz sim) with SDF models and worlds.",
        },
        {
            "name": "jmavsim",
            "display_name": "jMAVSim",
            "vehicle_default": "iris",
            "world_default": "",
            "description": "Legacy Java simulator kept for compatibility only.",
        },
    ]


def discover_gz_worlds(repo_root: Path) -> list[dict]:
    out: list[dict] = []
    worlds_dir = repo_root / "Tools" / "simulation" / "gz" / "worlds"
    if not worlds_dir.exists():
        return out
    for path in sorted(worlds_dir.glob("*.sdf")):
        out.append({
            "name": path.stem,
            "display_name": path.stem.replace("_", " ").title(),
            "path": str(path.resolve()),
        })
    return out


def discover_gz_models(repo_root: Path) -> list[dict]:
    out: list[dict] = []
    models_dir = repo_root / "Tools" / "simulation" / "gz" / "models"
    if not models_dir.exists():
        return out
    for path in sorted(models_dir.iterdir()):
        if not path.is_dir():
            continue
        if not (path / "model.sdf").exists():
            continue
        out.append({
            "name": path.name,
            "display_name": path.name.replace("_", " ").title(),
            "path": str(path.resolve()),
        })
    return out


def prepare_rootfs_simulator_assets(repo_root: Path, build_dir: Path, base_rootfs: Path, rootfs: Path, simulator: str) -> None:
    simulator = normalize_simulator_name(simulator)
    if simulator == "jmavsim":
        jmavsim_link = rootfs / "jmavsim_run.sh"
        if jmavsim_link.exists() or jmavsim_link.is_symlink():
            jmavsim_link.unlink()
        jmavsim_link.symlink_to((repo_root / "Tools" / "simulation" / "jmavsim" / "jmavsim_run.sh").resolve())
        return

    gz_env_src = base_rootfs / "gz_env.sh"
    if not gz_env_src.exists():
        gz_env_src = build_dir / "rootfs" / "gz_env.sh"
    if gz_env_src.exists():
        target = rootfs / "gz_env.sh"
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(gz_env_src.read_text(encoding="utf-8"), encoding="utf-8")


def simulator_state_fields(simulator: str, pid: int) -> dict:
    simulator = normalize_simulator_name(simulator)
    fields = {
        "simulator": simulator,
        "sim_pid": pid,
    }
    if simulator == "jmavsim":
        fields["jmavsim_pid"] = pid
    return fields


def _append_resource_path(existing: str, extra_paths: Iterable[Path]) -> str:
    parts = [part for part in str(existing or "").split(":") if part]
    for path in extra_paths:
        text = str(path.resolve())
        if text not in parts:
            parts.append(text)
    return ":".join(parts)


def _inject_sysid_logger_plugin(model_sdf_path: Path) -> None:
    tree = ET.parse(model_sdf_path)
    root = tree.getroot()
    model = root.find(".//model")
    if model is None:
        raise RuntimeError(f"Gazebo model file missing <model>: {model_sdf_path}")

    for plugin in list(model.findall("plugin")):
        if (plugin.attrib.get("name") == "SystemIdentificationLoggerPlugin"
                or plugin.attrib.get("name") == "px4::simulation::SystemIdentificationLogger"
                or plugin.attrib.get("filename") == "SystemIdentificationLoggerPlugin"):
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

    def copy_child(dst_parent: ET.Element, dst_name: str, src_plugin: ET.Element, src_name: str) -> None:
        src = src_plugin.find(src_name)
        if src is not None and src.text is not None:
            ET.SubElement(dst_parent, dst_name).text = src.text.strip()

    for plugin in rotor_plugins:
        rotor = ET.SubElement(logger, "rotor")
        copy_child(rotor, "motor_number", plugin, "motorNumber")
        copy_child(rotor, "joint_name", plugin, "jointName")
        copy_child(rotor, "link_name", plugin, "linkName")
        copy_child(rotor, "turning_direction", plugin, "turningDirection")
        copy_child(rotor, "time_constant_up_s", plugin, "timeConstantUp")
        copy_child(rotor, "time_constant_down_s", plugin, "timeConstantDown")
        copy_child(rotor, "max_rot_velocity_radps", plugin, "maxRotVelocity")
        copy_child(rotor, "motor_constant", plugin, "motorConstant")
        copy_child(rotor, "moment_constant", plugin, "momentConstant")
        copy_child(rotor, "rotor_drag_coefficient", plugin, "rotorDragCoefficient")
        copy_child(rotor, "rolling_moment_coefficient", plugin, "rollingMomentCoefficient")
        copy_child(rotor, "rotor_velocity_slowdown_sim", plugin, "rotorVelocitySlowdownSim")

    tree.write(model_sdf_path, encoding="utf-8", xml_declaration=True)


def _prepare_gz_model_override(repo_root: Path, rootfs: Path, vehicle: str) -> Path | None:
    src_dir = repo_root / "Tools" / "simulation" / "gz" / "models" / vehicle
    model_sdf = src_dir / "model.sdf"
    if not model_sdf.exists():
        return None

    dst_models_root = rootfs / "gz_models"
    dst_vehicle_dir = dst_models_root / vehicle
    if dst_vehicle_dir.exists():
        import shutil

        shutil.rmtree(dst_vehicle_dir)
    import shutil

    shutil.copytree(src_dir, dst_vehicle_dir)
    _inject_sysid_logger_plugin(dst_vehicle_dir / "model.sdf")
    return dst_models_root


def _gz_environment(repo_root: Path, partition: str, headless: bool, *, rootfs: Path, vehicle: str) -> dict[str, str]:
    env = os.environ.copy()
    models = repo_root / "Tools" / "simulation" / "gz" / "models"
    worlds = repo_root / "Tools" / "simulation" / "gz" / "worlds"
    plugins = repo_root / "build" / "px4_sitl_default" / "src" / "modules" / "simulation" / "gz_plugins"
    server_config = repo_root / "src" / "modules" / "simulation" / "gz_bridge" / "server.config"
    override_models = _prepare_gz_model_override(repo_root, rootfs, vehicle)
    env["GZ_PARTITION"] = partition
    env["PX4_GZ_MODELS"] = str(models.resolve())
    env["PX4_GZ_WORLDS"] = str(worlds.resolve())
    env["PX4_GZ_PLUGINS"] = str(plugins.resolve())
    env["PX4_GZ_SERVER_CONFIG"] = str(server_config.resolve())
    resource_paths: list[Path] = []
    if override_models is not None:
        resource_paths.append(override_models)
    resource_paths.extend([models, worlds])
    env["GZ_SIM_RESOURCE_PATH"] = _append_resource_path(env.get("GZ_SIM_RESOURCE_PATH", ""), resource_paths)
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = _append_resource_path(env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""), [plugins])
    env["GZ_SIM_SERVER_CONFIG_PATH"] = str(server_config.resolve())
    env["PX4_SYSID_LOG_DIR"] = str((rootfs / "sysid_truth_logs").resolve())
    env["PX4_SYSID_LOG_SLOT"] = rootfs.name
    env.setdefault("GZ_VERBOSE", "1")
    if headless:
        env["HEADLESS"] = "1"
    return env


def _wait_for_gz_world(world_name: str, env: dict[str, str], timeout_s: float = 30.0) -> None:
    deadline = time.time() + timeout_s
    last_error = ""
    clock_topic = f"/world/{world_name}/clock"
    stats_topic = f"/world/{world_name}/stats"
    control_service = f"/world/{world_name}/control"
    scene_service = f"/world/{world_name}/scene/info"
    while time.time() < deadline:
        try:
            topics_proc = subprocess.run(
                ["gz", "topic", "-l"],
                capture_output=True,
                text=True,
                timeout=3.0,
                env=env,
            )
            topics_out = (topics_proc.stdout or "") + (topics_proc.stderr or "")
            services_proc = subprocess.run(
                ["gz", "service", "-l"],
                capture_output=True,
                text=True,
                timeout=3.0,
                env=env,
            )
            services_out = (services_proc.stdout or "") + (services_proc.stderr or "")

            topic_ready = topics_proc.returncode == 0 and (clock_topic in topics_out or stats_topic in topics_out)
            service_ready = services_proc.returncode == 0 and (control_service in services_out or scene_service in services_out)
            if topic_ready and service_ready:
                return

            probe_proc = subprocess.run(
                ["gz", "service", "-i", "--service", scene_service],
                capture_output=True,
                text=True,
                timeout=3.0,
                env=env,
            )
            probe_out = (probe_proc.stdout or "") + (probe_proc.stderr or "")
            if probe_proc.returncode == 0 and "Service providers" in probe_out:
                return
            last_error = "\n".join(filter(None, [topics_out.strip(), services_out.strip(), probe_out.strip()]))
        except Exception as exc:  # pragma: no cover - exercised in integration flow
            last_error = str(exc)
        time.sleep(0.4)
    raise RuntimeError(f"Gazebo world '{world_name}' not ready in {timeout_s:.1f}s: {last_error}")


def _prepare_gz_world_copy(repo_root: Path, rootfs: Path, world_name: str, slot_name: str) -> Path:
    src = repo_root / "Tools" / "simulation" / "gz" / "worlds" / f"{world_name}.sdf"
    if not src.exists():
        raise RuntimeError(f"Gazebo world not found: {src}")
    dst_dir = rootfs / "gz_worlds"
    dst_dir.mkdir(parents=True, exist_ok=True)
    dst = dst_dir / f"{world_name}_{slot_name}.sdf"
    dst.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")
    return dst


def build_simulator_launch_spec(
    *,
    repo_root: Path,
    build_dir: Path,
    base_rootfs: Path,
    rootfs: Path,
    simulator: str,
    instance_id: int,
    tcp_port: int,
    sim_speed_factor: float,
    headless: bool,
    slot_name: str,
    simulator_vehicle: str = DEFAULT_GZ_VEHICLE,
    simulator_world: str = DEFAULT_GZ_WORLD,
) -> SimulatorLaunchSpec:
    simulator = normalize_simulator_name(simulator)
    prepare_rootfs_simulator_assets(repo_root, build_dir, base_rootfs, rootfs, simulator)

    if simulator == "jmavsim":
        env = os.environ.copy()
        env["JMAVSIM_DETERMINISTIC"] = "1"
        env["JMAVSIM_ZERO_SENSOR_NOISE"] = "1"
        env["JMAVSIM_ZERO_WIND"] = "1"
        env["JMAVSIM_SENSOR_SEED"] = "1337"
        env["JMAVSIM_ENV_SEED"] = "7331"
        env["PX4_SIM_SPEED_FACTOR"] = f"{max(0.1, float(sim_speed_factor)):.3f}"
        if headless:
            env["HEADLESS"] = "1"
        return SimulatorLaunchSpec(
            simulator="jmavsim",
            display_name="jMAVSim",
            process_label="jMAVSim",
            command=[
                str(repo_root / "Tools" / "simulation" / "jmavsim" / "jmavsim_run.sh"),
                "-p", str(tcp_port),
                "-l",
                "-r", "250",
            ],
            environment=env,
            px4_environment={
                "PX4_SYS_AUTOSTART": "10016",
                "PX4_SIM_MODEL": "none_iris",
                "PX4_SIM_TCP_PORT": str(tcp_port),
            },
            cleanup_hints=[str(rootfs)],
        )

    if simulator != "gz":
        raise RuntimeError(f"Unsupported simulator: {simulator}")

    partition = f"px4opt_{slot_name}_{instance_id}"
    world_name = str(simulator_world or DEFAULT_GZ_WORLD).strip() or DEFAULT_GZ_WORLD
    vehicle = str(simulator_vehicle or DEFAULT_GZ_VEHICLE).strip() or DEFAULT_GZ_VEHICLE
    world_copy = _prepare_gz_world_copy(repo_root, rootfs, world_name, slot_name)
    env = _gz_environment(repo_root, partition, headless=headless, rootfs=rootfs, vehicle=vehicle)
    command = ["gz", "sim", "-r"]
    if headless:
        command.append("-s")
    command.append(str(world_copy))
    return SimulatorLaunchSpec(
        simulator="gz",
        display_name="Gazebo",
        process_label="Gazebo",
        command=command,
        environment=env,
        px4_environment={
            "PX4_SYS_AUTOSTART": "4001",
            "PX4_SIM_MODEL": f"gz_{vehicle}",
            "PX4_SIMULATOR": "gz",
            "PX4_GZ_STANDALONE": "1",
            "PX4_GZ_WORLD": world_name,
            "GZ_PARTITION": partition,
            "PX4_SIM_SPEED_FACTOR": f"{max(0.1, float(sim_speed_factor)):.3f}",
            "HEADLESS": "1" if headless else "",
        },
        cleanup_hints=[str(world_copy), str(rootfs)],
    )


def wait_for_simulator_ready(spec: SimulatorLaunchSpec, timeout_s: float = 30.0) -> None:
    if spec.simulator != "gz":
        return
    world_name = str(spec.px4_environment.get("PX4_GZ_WORLD") or DEFAULT_GZ_WORLD)
    _wait_for_gz_world(world_name, spec.environment, timeout_s=timeout_s)
