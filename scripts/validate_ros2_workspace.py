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
KUKA_ROOT = ROOT / "src"
sys.path.insert(0, str(PACKAGE))

from designgrounded_ros2.component_catalog import (  # noqa: E402
    ROBOT_BASE_XYZ,
    build_components,
    build_place_targets,
    build_source_blocks,
)


def validate_xml(path: Path) -> None:
    ET.parse(path)


def main() -> int:
    errors: list[str] = []
    xml_files = [
        PACKAGE / "package.xml",
        PACKAGE / "urdf" / "kr6_d405_workcell.urdf.xacro",
        PACKAGE / "worlds" / "workcell.world",
        KUKA_ROOT / "kuka_resources" / "package.xml",
        KUKA_ROOT / "kuka_kr6_support" / "package.xml",
        KUKA_ROOT / "kuka_kr10_support" / "package.xml",
        KUKA_ROOT / "kuka_kr6_support" / "urdf" / "kr6r900_2_macro.xacro",
    ]
    for path in xml_files:
        try:
            validate_xml(path)
        except ET.ParseError as exc:
            errors.append(f"{path}: XML error: {exc}")

    python_files = [
        PACKAGE / "setup.py",
        PACKAGE / "launch" / "sim_gazebo.launch.py",
        PACKAGE / "launch" / "pick_place.launch.py",
        PACKAGE / "launch" / "vision_experiment.launch.py",
        PACKAGE / "launch" / "real_d405_capture.launch.py",
        PACKAGE / "designgrounded_ros2" / "assembly_node.py",
        PACKAGE / "designgrounded_ros2" / "component_catalog.py",
        PACKAGE / "designgrounded_ros2" / "perception_node.py",
        PACKAGE / "designgrounded_ros2" / "pick_place_node.py",
        PACKAGE / "designgrounded_ros2" / "scene_manager_node.py",
        PACKAGE / "designgrounded_ros2" / "vision_benchmark_node.py",
        PACKAGE / "designgrounded_ros2" / "dataset_capture_node.py",
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

    source_blocks = build_source_blocks(8)
    placement_targets = build_place_targets(8)
    if len(source_blocks) != 8 or len(placement_targets) != 8:
        errors.append("source blocks or placement targets must contain 8 entries")

    urdf_text = (PACKAGE / "urdf" / "kr6_d405_workcell.urdf.xacro").read_text(
        encoding="utf-8"
    )
    if "Intel RealSense" not in urdf_text and "d405" not in urdf_text.lower():
        errors.append("D405 not found in URDF")
    if "d405_depth_camera" not in urdf_text:
        errors.append("D405 Gazebo camera sensor missing")
    if "kuka_kr6_support" not in urdf_text or "kuka_kr6r900_2" not in urdf_text:
        errors.append("top-level URDF does not include the official KUKA KR6 macro")
    for joint_name in ["joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"]:
        if joint_name not in urdf_text:
            errors.append(f"official KUKA joint missing from control config: {joint_name}")

    required_mesh = (
        KUKA_ROOT
        / "kuka_kr10_support"
        / "meshes"
        / "kr10r900_2"
        / "visual"
        / "link_1.stl"
    )
    if not required_mesh.exists():
        errors.append(f"official KR10 R900-2 mesh missing: {required_mesh}")

    package_text = (PACKAGE / "package.xml").read_text(encoding="utf-8")
    for dependency in [
        "cv_bridge",
        "message_filters",
        "tf2_ros",
        "tf2_geometry_msgs",
        "realsense2_camera",
    ]:
        if dependency not in package_text:
            errors.append(f"package.xml missing dependency: {dependency}")

    if errors:
        print("ROS2_WORKSPACE=FAIL")
        for error in errors:
            print(f"- {error}")
        return 1
    print("ROS2_WORKSPACE=PASS")
    print("components=52")
    print(f"max_reach_m={max_reach:.3f}")
    print(f"max_estimated_mass_kg={max_mass:.3f}")
    print("robot=KUKA_KR6_R900_2_official_ros_industrial")
    print("camera=Intel_RealSense_D405")
    print("pick_place_nodes=perception+pick_place")
    print("vision_experiment=scene_manager+perception+benchmark")
    print("real_d405_capture=realsense+capture+perception")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

