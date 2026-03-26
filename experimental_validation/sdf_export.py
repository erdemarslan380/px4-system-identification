"""Helpers to export identified parameters into Gazebo-friendly text blocks."""

from __future__ import annotations

import re
from xml.sax.saxutils import escape


def _fmt(value: float) -> str:
    return f"{float(value):.6f}"


def build_inertial_snippet(
    *,
    mass_kg: float,
    ixx: float,
    iyy: float,
    izz: float,
    ixy: float = 0.0,
    ixz: float = 0.0,
    iyz: float = 0.0,
) -> str:
    return "\n".join(
        [
            "<inertial>",
            f"  <mass>{_fmt(mass_kg)}</mass>",
            "  <inertia>",
            f"    <ixx>{_fmt(ixx)}</ixx>",
            f"    <ixy>{_fmt(ixy)}</ixy>",
            f"    <ixz>{_fmt(ixz)}</ixz>",
            f"    <iyy>{_fmt(iyy)}</iyy>",
            f"    <iyz>{_fmt(iyz)}</iyz>",
            f"    <izz>{_fmt(izz)}</izz>",
            "  </inertia>",
            "</inertial>",
        ]
    )


def build_parameter_summary(
    *,
    mass_kg: float,
    thrust_scale_n_per_cmd: float | None = None,
    yaw_moment_scale_nm_per_cmd: float | None = None,
    drag_coeff_x: float | None = None,
    drag_coeff_y: float | None = None,
    drag_coeff_z: float | None = None,
) -> str:
    lines = [
        "# Gazebo candidate parameters",
        f"mass_kg: {_fmt(mass_kg)}",
    ]
    if thrust_scale_n_per_cmd is not None:
        lines.append(f"thrust_scale_n_per_cmd: {_fmt(thrust_scale_n_per_cmd)}")
    if yaw_moment_scale_nm_per_cmd is not None:
        lines.append(f"yaw_moment_scale_nm_per_cmd: {_fmt(yaw_moment_scale_nm_per_cmd)}")
    if drag_coeff_x is not None:
        lines.append(f"drag_coeff_x: {_fmt(drag_coeff_x)}")
    if drag_coeff_y is not None:
        lines.append(f"drag_coeff_y: {_fmt(drag_coeff_y)}")
    if drag_coeff_z is not None:
        lines.append(f"drag_coeff_z: {_fmt(drag_coeff_z)}")
    return "\n".join(lines)


def apply_inertial_snippet_to_sdf(template_text: str, inertial_snippet: str) -> str:
    pattern = re.compile(r"<inertial>.*?</inertial>", flags=re.DOTALL)
    if pattern.search(template_text):
        return pattern.sub(inertial_snippet, template_text, count=1)
    return template_text + "\n" + inertial_snippet + "\n"
