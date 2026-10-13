from __future__ import annotations

import ast
import math
import py_compile
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "designgrounded_ros2"
sys.path.insert(0, str(PACKAGE))

from designgrounded_ros2.component_catalog import (  # noqa: E402
    ROBOT_BASE_XYZ,
    build_components,
)


def validate_xml(path: Path) -> None:
    ET.parse(path)


def main() -> int:
    errors: list[str] = []
    xml_files = [
        PACKAGE / "package.xml",
        PACKAGE / "urdf" / "kr6_d405_workcell.urdf.xacro",
        PACKAGE / "worlds" / "workcell.world",
    ]
    for path in xml_files:
        try:
            validate_xml(path)
        except ET.ParseError as exc:
            errors.append(f"{path}: XML error: {exc}")

    python_files = [
        PACKAGE / "setup.py",
        PACKAGE / "launch" / "sim_gazebo.launch.py",
        PACKAGE / "designgrounded_ros2" / "assembly_node.py",
        PACKAGE / "designgrounded_ros2" / "component_catalog.py",
    ]
    for path in python_files:
        try:
            py_compile.compile(str(path), doraise=True)
        except py_compile.PyCompileError as exc:
            errors.append(f"{path}: Python syntax error: {exc}")

    try:
        controllers = yaml.safe_load(
            (PACKAGE / "config" / "controllers.yaml").read_text(encoding="utf-8")
        )
        if "controller_manager" not in controllers:
            errors.append("controllers.yaml missing controller_manager")
    except (OSError, yaml.YAMLError) as exc:
        errors.append(f"controllers.yaml: {exc}")

    try:
        components = build_components()
    except Exception as exc:
        errors.append(f"component catalog error: {exc}")
        components = []

    ids = [component["id"] for component in components]
    if len(components) != 52:
        errors.append(f"component count is {len(components)}, expected 52")
    if len(ids) != len(set(ids)):
        errors.append("component ids are not unique")

    max_reach = 0.0
    max_mass = 0.0
    for component in components:
        position = component["position"]
        distance = math.dist(position, ROBOT_BASE_XYZ)
        max_reach = max(max_reach, distance)
        volume = component["size"][0] * component["size"][1] * component["size"][2]
        max_mass = max(max_mass, volume * 550.0)
    if max_reach > 0.901:
        errors.append(f"component reach {max_reach:.3f} m exceeds 0.901 m")
    if max_mass > 5.0:
        errors.append(f"component mass {max_mass:.3f} kg exceeds 5.0 kg")

    urdf_text = (PACKAGE / "urdf" / "kr6_d405_workcell.urdf.xacro").read_text(
        encoding="utf-8"
    )
    if "Intel RealSense" not in urdf_text and "d405" not in urdf_text.lower():
        errors.append("D405 not found in URDF")
    if "d405_depth_camera" not in urdf_text:
        errors.append("D405 Gazebo camera sensor missing")

    if errors:
        print("ROS2_WORKSPACE=FAIL")
        for error in errors:
            print(f"- {error}")
        return 1
    print("ROS2_WORKSPACE=PASS")
    print("components=52")
    print(f"max_reach_m={max_reach:.3f}")
    print(f"max_estimated_mass_kg={max_mass:.3f}")
    print("robot=KUKA_KR6_R900_2")
    print("camera=Intel_RealSense_D405")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

