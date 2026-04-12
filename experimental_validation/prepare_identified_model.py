from __future__ import annotations

import argparse
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experimental_validation.twin_metrics import flatten_identified_metrics
from experimental_validation.sdf_export import apply_inertial_snippet_to_sdf, build_inertial_snippet


def _regularize_diagonal_inertia(
    ixx: float,
    iyy: float,
    izz: float,
    *,
    eps: float = 1e-6,
) -> tuple[dict[str, float], list[str]]:
    values = {"ixx_kgm2": max(float(ixx), eps), "iyy_kgm2": max(float(iyy), eps), "izz_kgm2": max(float(izz), eps)}
    notes: list[str] = []

    if (
        values["ixx_kgm2"] + values["iyy_kgm2"] > values["izz_kgm2"]
        and values["ixx_kgm2"] + values["izz_kgm2"] > values["iyy_kgm2"]
        and values["iyy_kgm2"] + values["izz_kgm2"] > values["ixx_kgm2"]
    ):
        return values, notes

    original = dict(values)
    for _ in range(12):
        changed = False
        if values["ixx_kgm2"] >= values["iyy_kgm2"] + values["izz_kgm2"] - eps:
            values["ixx_kgm2"] = max(eps, values["iyy_kgm2"] + values["izz_kgm2"] - eps)
            changed = True
        if values["iyy_kgm2"] >= values["ixx_kgm2"] + values["izz_kgm2"] - eps:
            values["iyy_kgm2"] = max(eps, values["ixx_kgm2"] + values["izz_kgm2"] - eps)
            changed = True
        if values["izz_kgm2"] >= values["ixx_kgm2"] + values["iyy_kgm2"] - eps:
            values["izz_kgm2"] = max(eps, values["ixx_kgm2"] + values["iyy_kgm2"] - eps)
            changed = True
        if not changed:
            break

    notes.append(
        "Diagonal inertia regularized to satisfy rigid-body triangle inequalities before SDF export: "
        f"ixx {original['ixx_kgm2']:.9f}->{values['ixx_kgm2']:.9f}, "
        f"iyy {original['iyy_kgm2']:.9f}->{values['iyy_kgm2']:.9f}, "
        f"izz {original['izz_kgm2']:.9f}->{values['izz_kgm2']:.9f}."
    )
    return values, notes


def _indent(elem: ET.Element, level: int = 0) -> None:
    indent_text = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = indent_text + "  "
        for child in elem:
            _indent(child, level + 1)
        if not elem[-1].tail or not elem[-1].tail.strip():
            elem[-1].tail = indent_text
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = indent_text


def _set_text(root: ET.Element, path: str, value: float | str) -> None:
    node = root.find(path)
    if node is None:
        raise KeyError(f"missing XML path: {path}")
    node.text = str(value)


def _load_params(candidate_json: Path) -> dict:
    return json.loads(candidate_json.read_text(encoding="utf-8"))


def _extract_geometry(candidate_payload: dict) -> dict | None:
    geometry = candidate_payload.get("geometry")
    if not isinstance(geometry, dict):
        return None
    rotor_positions = geometry.get("rotor_positions_m")
    turning_directions = geometry.get("turning_directions")
    if not rotor_positions and not turning_directions:
        return None
    return {
        "rotor_positions_m": rotor_positions if isinstance(rotor_positions, dict) else None,
        "turning_directions": turning_directions if isinstance(turning_directions, dict) else None,
        "label": geometry.get("label"),
        "notes": geometry.get("notes"),
    }


def _extract_simulator_residuals(candidate_payload: dict) -> dict[str, float]:
    residuals = candidate_payload.get("simulator_residuals")
    if not isinstance(residuals, dict):
        return {}
    out: dict[str, float] = {}
    for key in (
        "body_velocity_decay_linear",
        "body_velocity_decay_angular",
        "body_drag_scale_x",
        "body_drag_scale_y",
        "body_drag_scale_z",
        "body_angular_damping_x",
        "body_angular_damping_y",
        "body_angular_damping_z",
        "motor_balance_x",
        "motor_balance_y",
        "moment_balance_x",
        "moment_balance_y",
    ):
        value = residuals.get(key)
        if value is None:
            continue
        try:
            out[key] = float(value)
        except (TypeError, ValueError):
            continue
    return out


def _rotor_axis_sign_lookup(geometry: dict | None) -> dict[str, tuple[float, float]]:
    rotor_positions = (geometry or {}).get("rotor_positions_m")
    if not isinstance(rotor_positions, dict):
        return {}

    lookup: dict[str, tuple[float, float]] = {}
    for rotor_name, position in rotor_positions.items():
        if not isinstance(position, (list, tuple)) or len(position) != 3:
            continue
        x_sign = 1.0 if float(position[0]) >= 0.0 else -1.0
        y_sign = 1.0 if float(position[1]) >= 0.0 else -1.0
        lookup[rotor_name] = (x_sign, y_sign)
        if rotor_name.startswith("rotor_"):
            suffix = rotor_name.split("_", 1)[1]
            lookup[f"motor_{suffix}"] = (x_sign, y_sign)
    return lookup


def _residual_axis_scale(
    base_value: float,
    *,
    axis_signs: tuple[float, float] | None,
    scale_x: float,
    scale_y: float,
    floor: float = 0.2,
) -> float:
    if axis_signs is None:
        return float(base_value)
    x_sign, y_sign = axis_signs
    scale = 1.0 + float(scale_x) * float(x_sign) + float(scale_y) * float(y_sign)
    return float(base_value) * max(floor, scale)


def _apply_rotor_positions(base_root: ET.Element, rotor_positions: dict[str, object] | None) -> list[str]:
    if not rotor_positions:
        return []
    notes: list[str] = []
    for rotor_name, position in rotor_positions.items():
        if not isinstance(position, (list, tuple)) or len(position) != 3:
            continue
        link = base_root.find(f".//link[@name='{rotor_name}']")
        if link is None:
            notes.append(f"Skipped unknown rotor link pose override: {rotor_name}")
            continue
        pose_node = link.find("pose")
        if pose_node is None:
            pose_node = ET.SubElement(link, "pose")
            current = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        else:
            raw = (pose_node.text or "").split()
            current = [float(item) for item in raw[:6]] if raw else [0.0] * 6
            if len(current) < 6:
                current.extend([0.0] * (6 - len(current)))
        current[0] = float(position[0])
        current[1] = float(position[1])
        current[2] = float(position[2])
        pose_node.text = " ".join(f"{value:.9f}" for value in current)
        notes.append(
            f"Rotor pose override applied: {rotor_name} -> "
            f"({current[0]:.6f}, {current[1]:.6f}, {current[2]:.6f})"
        )
    return notes


def _apply_turning_directions(wrapper_root: ET.Element, turning_directions: dict[str, object] | None) -> list[str]:
    if not turning_directions:
        return []
    notes: list[str] = []
    plugin_updates = 0
    logger_updates = 0
    for plugin in wrapper_root.findall("./model/plugin"):
        motor_number = plugin.findtext("motorNumber")
        if motor_number is None:
            continue
        key = f"motor_{motor_number}"
        value = turning_directions.get(key)
        if isinstance(value, str) and value.strip():
            direction = value.strip().lower()
            if direction in {"cw", "ccw"}:
                _set_text(plugin, "turningDirection", direction)
                plugin_updates += 1
    for rotor in wrapper_root.findall("./model/plugin[@name='SystemIdentificationLoggerPlugin']/rotor"):
        motor_number = rotor.findtext("motor_number")
        if motor_number is None:
            continue
        key = f"motor_{motor_number}"
        value = turning_directions.get(key)
        if isinstance(value, str) and value.strip():
            direction = value.strip().lower()
            if direction in {"cw", "ccw"}:
                _set_text(rotor, "turning_direction", direction)
                logger_updates += 1
    if plugin_updates or logger_updates:
        notes.append(
            f"Turning-direction override applied to {plugin_updates} motor plugins and {logger_updates} logger rotor entries."
        )
    return notes


def _apply_velocity_decay(base_root: ET.Element, *, linear: float | None, angular: float | None) -> list[str]:
    if linear is None and angular is None:
        return []
    link = base_root.find("./model/link[@name='base_link']")
    if link is None:
        return ["Skipped body velocity-decay override because base_link was not found."]

    decay = link.find("velocity_decay")
    if decay is None:
        decay = ET.SubElement(link, "velocity_decay")

    notes: list[str] = []
    if linear is not None:
        linear_node = decay.find("linear")
        if linear_node is None:
            linear_node = ET.SubElement(decay, "linear")
        linear_node.text = f"{max(0.0, float(linear)):.9f}"
        notes.append(f"Applied base_link velocity_decay/linear={float(linear):.6f}")
    if angular is not None:
        angular_node = decay.find("angular")
        if angular_node is None:
            angular_node = ET.SubElement(decay, "angular")
        angular_node.text = f"{max(0.0, float(angular)):.9f}"
        notes.append(f"Applied base_link velocity_decay/angular={float(angular):.6f}")
    return notes


def _extract_body_drag(candidate_payload: dict) -> tuple[dict[str, float], list[str]]:
    drag = candidate_payload.get("drag")
    notes: list[str] = []
    coeffs = {"x": 0.0, "y": 0.0, "z": 0.0}
    if not isinstance(drag, dict):
        return coeffs, notes

    for axis in ("x", "y", "z"):
        axis_payload = drag.get(axis)
        if not isinstance(axis_payload, dict):
            continue
        value = axis_payload.get("coefficient")
        if value is None:
            continue
        try:
            coeff = float(value)
        except (TypeError, ValueError):
            continue
        if coeff < 0.0:
            notes.append(
                f"Negative identified drag coefficient on axis {axis} ({coeff:.6f}) projected to abs value for SDF drag export."
            )
        coeffs[axis] = abs(coeff)
    return coeffs, notes


def _apply_body_drag_plugin(
    wrapper_root: ET.Element,
    *,
    quadratic_drag_xyz: tuple[float, float, float],
    angular_damping_xyz: tuple[float, float, float],
    link_name: str = "base_link",
) -> list[str]:
    if all(abs(value) <= 1e-12 for value in quadratic_drag_xyz + angular_damping_xyz):
        return []

    model = wrapper_root.find("./model")
    if model is None:
        return ["Skipped body drag plugin insertion because model node was not found."]

    plugin = None
    for existing in model.findall("plugin"):
        if existing.attrib.get("filename") == "BodyFrameDragPlugin" or existing.attrib.get("name") == "BodyFrameDragPlugin":
            plugin = existing
            break
    if plugin is None:
        plugin = ET.SubElement(model, "plugin", {"filename": "BodyFrameDragPlugin", "name": "BodyFrameDragPlugin"})

    def _set_plugin_text(tag: str, text: str) -> None:
        node = plugin.find(tag)
        if node is None:
            node = ET.SubElement(plugin, tag)
        node.text = text

    _set_plugin_text("link_name", link_name)
    _set_plugin_text("linear_drag_coefficients", "0 0 0")
    _set_plugin_text(
        "quadratic_drag_coefficients",
        f"{quadratic_drag_xyz[0]:.9f} {quadratic_drag_xyz[1]:.9f} {quadratic_drag_xyz[2]:.9f}",
    )
    _set_plugin_text(
        "angular_damping_coefficients",
        f"{angular_damping_xyz[0]:.9f} {angular_damping_xyz[1]:.9f} {angular_damping_xyz[2]:.9f}",
    )

    return [
        "Applied BodyFrameDragPlugin with quadratic drag "
        f"({quadratic_drag_xyz[0]:.6f}, {quadratic_drag_xyz[1]:.6f}, {quadratic_drag_xyz[2]:.6f}) "
        "and angular damping "
        f"({angular_damping_xyz[0]:.6f}, {angular_damping_xyz[1]:.6f}, {angular_damping_xyz[2]:.6f})."
    ]


def _build_candidate_base_tree(stock_base_sdf: Path, params: dict[str, float], *, model_name: str) -> ET.ElementTree:
    inertial_snippet = build_inertial_snippet(
        mass_kg=params["mass_kg"],
        ixx=params["ixx_kgm2"],
        iyy=params["iyy_kgm2"],
        izz=params["izz_kgm2"],
    )
    base_model_text = stock_base_sdf.read_text(encoding="utf-8")
    patched = apply_inertial_snippet_to_sdf(base_model_text, inertial_snippet)
    root = ET.fromstring(patched)
    model_node = root.find("model")
    if model_node is not None:
        model_node.set("name", model_name)
    return ET.ElementTree(root)


def prepare_identified_model(px4_root: str | Path, candidate_dir: str | Path, *, model_name: str = "x500_identified") -> dict:
    px4_root = Path(px4_root).expanduser().resolve()
    candidate_dir = Path(candidate_dir).expanduser().resolve()
    models_root = px4_root / "Tools" / "simulation" / "gz" / "models"
    stock_x500_dir = models_root / "x500"
    stock_base_dir = models_root / "x500_base"
    identified_dir = models_root / model_name
    identified_base_dir = models_root / f"{model_name}_base"

    raw_payload = _load_params(candidate_dir / "identified_parameters.json")
    params = flatten_identified_metrics(raw_payload)
    geometry = _extract_geometry(raw_payload)
    simulator_residuals = _extract_simulator_residuals(raw_payload)
    body_drag_coeffs, drag_notes = _extract_body_drag(raw_payload)
    regularized_inertia, inertia_notes = _regularize_diagonal_inertia(
        params["ixx_kgm2"],
        params["iyy_kgm2"],
        params["izz_kgm2"],
    )
    params.update(regularized_inertia)
    rotor_axis_signs = _rotor_axis_sign_lookup(geometry)

    if identified_dir.exists():
        shutil.rmtree(identified_dir)
    if identified_base_dir.exists():
        shutil.rmtree(identified_base_dir)

    shutil.copytree(stock_x500_dir, identified_dir)
    shutil.copytree(stock_base_dir, identified_base_dir)

    # Replace the base SDF with the identified inertial model and rename references.
    base_target_sdf = identified_base_dir / "model.sdf"
    candidate_base_sdf = candidate_dir / "candidate_x500_base.sdf"
    if candidate_base_sdf.exists():
        base_tree = ET.parse(candidate_base_sdf)
        base_root = base_tree.getroot()
        model_node = base_root.find("model")
        if model_node is not None:
            model_node.set("name", f"{model_name}_base")
    else:
        base_tree = _build_candidate_base_tree(stock_base_dir / "model.sdf", params, model_name=f"{model_name}_base")
        base_root = base_tree.getroot()
    geometry_notes = _apply_rotor_positions(base_root, geometry.get("rotor_positions_m") if geometry else None)
    geometry_notes.extend(
        _apply_velocity_decay(
            base_root,
            linear=simulator_residuals.get("body_velocity_decay_linear"),
            angular=simulator_residuals.get("body_velocity_decay_angular"),
        )
    )
    _indent(base_root)
    base_tree.write(base_target_sdf, encoding="utf-8", xml_declaration=True)

    base_config = ET.parse(identified_base_dir / "model.config")
    config_root = base_config.getroot()
    name_node = config_root.find("name")
    if name_node is not None:
        name_node.text = f"{model_name}_base"
    _indent(config_root)
    base_config.write(identified_base_dir / "model.config", encoding="utf-8", xml_declaration=True)

    wrapper_tree = ET.parse(identified_dir / "model.sdf")
    wrapper_root = wrapper_tree.getroot()
    wrapper_model = wrapper_root.find("model")
    if wrapper_model is not None:
        wrapper_model.set("name", model_name)
    include_uri = wrapper_root.find("./model/include/uri")
    if include_uri is not None:
        include_uri.text = f"model://{model_name}_base"

    plugin_paths = [
        plugin
        for plugin in wrapper_root.findall("./model/plugin")
        if plugin.attrib.get("name") == "gz::sim::systems::MulticopterMotorModel"
        or plugin.attrib.get("filename") == "gz-sim-multicopter-motor-model-system"
    ]
    for plugin in plugin_paths:
        motor_number = plugin.findtext("motorNumber")
        link_name = plugin.findtext("linkName")
        axis_signs = rotor_axis_signs.get(f"motor_{motor_number}") if motor_number is not None else None
        if axis_signs is None and link_name:
            axis_signs = rotor_axis_signs.get(link_name)
        _set_text(plugin, "timeConstantUp", params["time_constant_up_s"])
        _set_text(plugin, "timeConstantDown", params["time_constant_down_s"])
        _set_text(plugin, "maxRotVelocity", params["max_rot_velocity_radps"])
        _set_text(
            plugin,
            "motorConstant",
            _residual_axis_scale(
                params["motor_constant"],
                axis_signs=axis_signs,
                scale_x=float(simulator_residuals.get("motor_balance_x", 0.0)),
                scale_y=float(simulator_residuals.get("motor_balance_y", 0.0)),
            ),
        )
        _set_text(
            plugin,
            "momentConstant",
            _residual_axis_scale(
                params["moment_constant"],
                axis_signs=axis_signs,
                scale_x=float(simulator_residuals.get("moment_balance_x", 0.0)),
                scale_y=float(simulator_residuals.get("moment_balance_y", 0.0)),
            ),
        )
        _set_text(plugin, "rotorDragCoefficient", params["rotor_drag_coefficient"])
        _set_text(plugin, "rollingMomentCoefficient", params["rolling_moment_coefficient"])
        _set_text(plugin, "rotorVelocitySlowdownSim", params["rotor_velocity_slowdown_sim"])

    logger_plugins = [
        plugin
        for plugin in wrapper_root.findall("./model/plugin")
        if plugin.attrib.get("name") == "SystemIdentificationLoggerPlugin"
    ]
    for plugin in logger_plugins:
        for rotor in plugin.findall("rotor"):
            motor_number = rotor.findtext("motor_number")
            axis_signs = rotor_axis_signs.get(f"motor_{motor_number}") if motor_number is not None else None
            _set_text(rotor, "time_constant_up_s", params["time_constant_up_s"])
            _set_text(rotor, "time_constant_down_s", params["time_constant_down_s"])
            _set_text(rotor, "max_rot_velocity_radps", params["max_rot_velocity_radps"])
            _set_text(
                rotor,
                "motor_constant",
                _residual_axis_scale(
                    params["motor_constant"],
                    axis_signs=axis_signs,
                    scale_x=float(simulator_residuals.get("motor_balance_x", 0.0)),
                    scale_y=float(simulator_residuals.get("motor_balance_y", 0.0)),
                ),
            )
            _set_text(
                rotor,
                "moment_constant",
                _residual_axis_scale(
                    params["moment_constant"],
                    axis_signs=axis_signs,
                    scale_x=float(simulator_residuals.get("moment_balance_x", 0.0)),
                    scale_y=float(simulator_residuals.get("moment_balance_y", 0.0)),
                ),
            )
            _set_text(rotor, "rotor_drag_coefficient", params["rotor_drag_coefficient"])
            _set_text(rotor, "rolling_moment_coefficient", params["rolling_moment_coefficient"])
            _set_text(rotor, "rotor_velocity_slowdown_sim", params["rotor_velocity_slowdown_sim"])
    geometry_notes.extend(_apply_turning_directions(wrapper_root, geometry.get("turning_directions") if geometry else None))
    geometry_notes.extend(drag_notes)
    if abs(float(simulator_residuals.get("motor_balance_x", 0.0))) > 1e-9 or abs(float(simulator_residuals.get("motor_balance_y", 0.0))) > 1e-9:
        geometry_notes.append(
            "Applied directional motorConstant residuals with "
            f"motor_balance_x={float(simulator_residuals.get('motor_balance_x', 0.0)):.6f}, "
            f"motor_balance_y={float(simulator_residuals.get('motor_balance_y', 0.0)):.6f}."
        )
    if abs(float(simulator_residuals.get("moment_balance_x", 0.0))) > 1e-9 or abs(float(simulator_residuals.get("moment_balance_y", 0.0))) > 1e-9:
        geometry_notes.append(
            "Applied directional momentConstant residuals with "
            f"moment_balance_x={float(simulator_residuals.get('moment_balance_x', 0.0)):.6f}, "
            f"moment_balance_y={float(simulator_residuals.get('moment_balance_y', 0.0)):.6f}."
        )
    quadratic_drag_xyz = (
        body_drag_coeffs["x"] * float(simulator_residuals.get("body_drag_scale_x", 1.0)),
        body_drag_coeffs["y"] * float(simulator_residuals.get("body_drag_scale_y", 1.0)),
        body_drag_coeffs["z"] * float(simulator_residuals.get("body_drag_scale_z", 1.0)),
    )
    angular_damping_xyz = (
        float(simulator_residuals.get("body_angular_damping_x", 0.0)),
        float(simulator_residuals.get("body_angular_damping_y", 0.0)),
        float(simulator_residuals.get("body_angular_damping_z", 0.0)),
    )
    geometry_notes.extend(
        _apply_body_drag_plugin(
            wrapper_root,
            quadratic_drag_xyz=quadratic_drag_xyz,
            angular_damping_xyz=angular_damping_xyz,
        )
    )

    _indent(wrapper_root)
    wrapper_tree.write(identified_dir / "model.sdf", encoding="utf-8", xml_declaration=True)

    wrapper_config = ET.parse(identified_dir / "model.config")
    wrapper_config_root = wrapper_config.getroot()
    name_node = wrapper_config_root.find("name")
    if name_node is not None:
        name_node.text = model_name
    _indent(wrapper_config_root)
    wrapper_config.write(identified_dir / "model.config", encoding="utf-8", xml_declaration=True)

    payload = {
        "model_dir": str(identified_dir),
        "base_model_dir": str(identified_base_dir),
        "candidate_dir": str(candidate_dir),
        "model_name": model_name,
        "notes": inertia_notes + geometry_notes,
    }
    (identified_dir / "prepared_from_candidate.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return payload


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Create x500_identified Gazebo models from an identified candidate.")
    ap.add_argument("--px4-root", default="~/PX4-Autopilot-Identification")
    ap.add_argument("--candidate-dir", default="~/px4-system-identification/examples/paper_assets/candidates/x500_truth_assisted_sitl_v1")
    ap.add_argument("--model-name", default="x500_identified")
    args = ap.parse_args()
    result = prepare_identified_model(args.px4_root, args.candidate_dir, model_name=args.model_name)
    print(json.dumps(result, indent=2))
