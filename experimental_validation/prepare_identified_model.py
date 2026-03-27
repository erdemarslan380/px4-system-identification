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


def prepare_identified_model(px4_root: str | Path, candidate_dir: str | Path, *, model_name: str = "x500_identified") -> dict:
    px4_root = Path(px4_root).expanduser().resolve()
    candidate_dir = Path(candidate_dir).expanduser().resolve()
    models_root = px4_root / "Tools" / "simulation" / "gz" / "models"
    stock_x500_dir = models_root / "x500"
    stock_base_dir = models_root / "x500_base"
    identified_dir = models_root / model_name
    identified_base_dir = models_root / f"{model_name}_base"

    params = flatten_identified_metrics(_load_params(candidate_dir / "identified_parameters.json"))

    if identified_dir.exists():
        shutil.rmtree(identified_dir)
    if identified_base_dir.exists():
        shutil.rmtree(identified_base_dir)

    shutil.copytree(stock_x500_dir, identified_dir)
    shutil.copytree(stock_base_dir, identified_base_dir)

    # Replace the base SDF with the identified inertial model and rename references.
    candidate_base_sdf = candidate_dir / "candidate_x500_base.sdf"
    base_target_sdf = identified_base_dir / "model.sdf"
    base_tree = ET.parse(candidate_base_sdf)
    base_root = base_tree.getroot()
    model_node = base_root.find("model")
    if model_node is not None:
        model_node.set("name", f"{model_name}_base")
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

    plugin_paths = wrapper_root.findall("./model/plugin")
    for plugin in plugin_paths:
        _set_text(plugin, "timeConstantUp", params["time_constant_up_s"])
        _set_text(plugin, "timeConstantDown", params["time_constant_down_s"])
        _set_text(plugin, "maxRotVelocity", params["max_rot_velocity_radps"])
        _set_text(plugin, "motorConstant", params["motor_constant"])
        _set_text(plugin, "momentConstant", params["moment_constant"])
        _set_text(plugin, "rotorDragCoefficient", params["rotor_drag_coefficient"])
        _set_text(plugin, "rollingMomentCoefficient", params["rolling_moment_coefficient"])
        _set_text(plugin, "rotorVelocitySlowdownSim", params["rotor_velocity_slowdown_sim"])

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
