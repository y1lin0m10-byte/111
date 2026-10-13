from __future__ import annotations

import math
from typing import Any


CROSS_M = 0.04
DEPTH_M = 0.04
BASE_Z_M = 0.12
ROBOT_BASE_XYZ = (-0.40, 0.22, 0.0)


def build_components() -> list[dict[str, Any]]:
    components: list[dict[str, Any]] = []

    def add(
        group: str,
        label: str,
        size: tuple[float, float, float],
        position: tuple[float, float, float],
        rotation: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        components.append(
            {
                "id": f"{group}_{len(components) + 1:02d}",
                "group": group,
                "label": label,
                "size": list(size),
                "position": list(position),
                "rotation": list(rotation),
            }
        )

    add(
        "bottom",
        "bottom plate L",
        (0.35, DEPTH_M, CROSS_M),
        (-0.175, 0.0, BASE_Z_M + 0.02),
    )
    add(
        "bottom",
        "bottom plate R",
        (0.35, DEPTH_M, CROSS_M),
        (0.175, 0.0, BASE_Z_M + 0.02),
    )

    for y in (-0.018, 0.018):
        for x in (-0.30, -0.225, -0.15, 0.15, 0.225, 0.30):
            add(
                "stud",
                "main stud",
                (CROSS_M, DEPTH_M, 0.41),
                (x, y, BASE_Z_M + 0.23),
            )

    for y in (-0.018, 0.018):
        add(
            "king",
            "king stud L",
            (CROSS_M, DEPTH_M, 0.41),
            (-0.105, y, BASE_Z_M + 0.23),
        )
        add(
            "king",
            "king stud R",
            (CROSS_M, DEPTH_M, 0.41),
            (0.105, y, BASE_Z_M + 0.23),
        )

    for y in (-0.018, 0.018):
        add(
            "opening",
            "sill plate",
            (0.20, DEPTH_M, CROSS_M),
            (0.0, y, BASE_Z_M + 0.135),
        )
        add(
            "opening",
            "header",
            (0.20, DEPTH_M, CROSS_M),
            (0.0, y, BASE_Z_M + 0.325),
        )

    for y in (-0.018, 0.018):
        add(
            "opening",
            "upper cripple",
            (CROSS_M, DEPTH_M, 0.125),
            (-0.055, y, BASE_Z_M + 0.388),
        )
        add(
            "opening",
            "upper cripple",
            (CROSS_M, DEPTH_M, 0.125),
            (0.055, y, BASE_Z_M + 0.388),
        )

    for y in (-0.018, 0.018):
        add(
            "opening",
            "lower cripple",
            (CROSS_M, DEPTH_M, 0.065),
            (-0.055, y, BASE_Z_M + 0.085),
        )
        add(
            "opening",
            "lower cripple",
            (CROSS_M, DEPTH_M, 0.065),
            (0.055, y, BASE_Z_M + 0.085),
        )

    for y in (-0.018, 0.018):
        add(
            "brace",
            "brace L up",
            (0.165, DEPTH_M, CROSS_M),
            (-0.225, y, BASE_Z_M + 0.345),
            (0.0, math.radians(45.0), 0.0),
        )
        add(
            "brace",
            "brace R up",
            (0.165, DEPTH_M, CROSS_M),
            (0.225, y, BASE_Z_M + 0.345),
            (0.0, math.radians(-45.0), 0.0),
        )
        add(
            "brace",
            "brace L low",
            (0.135, DEPTH_M, CROSS_M),
            (-0.245, y, BASE_Z_M + 0.115),
            (0.0, math.radians(-45.0), 0.0),
        )
        add(
            "brace",
            "brace R low",
            (0.135, DEPTH_M, CROSS_M),
            (0.245, y, BASE_Z_M + 0.115),
            (0.0, math.radians(45.0), 0.0),
        )

    for z in (BASE_Z_M + 0.435, BASE_Z_M + 0.472):
        add(
            "top",
            "top plate L",
            (0.35, DEPTH_M, CROSS_M),
            (-0.175, 0.0, z),
        )
        add(
            "top",
            "top plate R",
            (0.35, DEPTH_M, CROSS_M),
            (0.175, 0.0, z),
        )

    for y in (-0.018, 0.018):
        for x in (-0.28, -0.19, -0.095, 0.095, 0.19):
            add(
                "connector",
                "connector block",
                (0.065, DEPTH_M, CROSS_M),
                (x, y, BASE_Z_M + 0.058 + abs(x) * 0.22),
            )

    if len(components) != 52:
        raise AssertionError(f"expected 52 components, got {len(components)}")
    return components


def component_sdf(component: dict[str, Any]) -> str:
    sx, sy, sz = component["size"]
    x, y, z = component["position"]
    roll, pitch, yaw = component["rotation"]
    return f"""
<sdf version="1.6">
  <model name="{component['id']}">
    <static>true</static>
    <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material><ambient>0.7 0.45 0.2 1</ambient><diffuse>0.8 0.55 0.25 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
""".strip()


def build_source_blocks(count: int = 8) -> list[dict[str, Any]]:
    blocks: list[dict[str, Any]] = []
    for index in range(count):
        row = index // 4
        column = index % 4
        blocks.append(
            {
                "id": f"source_block_{index + 1:02d}",
                "label": f"wood block {index + 1}",
                "size": [0.10, 0.08, 0.04],
                "position": [
                    0.08 + 0.14 * column,
                    0.24 - 0.12 * row,
                    0.145,
                ],
                "rotation": [0.0, 0.0, 0.0],
            }
        )
    return blocks


def source_block_sdf(block: dict[str, Any]) -> str:
    sx, sy, sz = block["size"]
    mass = max(sx * sy * sz * 550.0, 0.02)
    return f"""
<sdf version="1.6">
  <model name="{block['id']}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>{mass:.6f}</mass>
        <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
      </inertial>
      <visual name="visual">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material><ambient>0.62 0.32 0.12 1</ambient><diffuse>0.80 0.55 0.25 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
""".strip()


def build_place_targets(count: int = 8) -> list[list[float]]:
    targets: list[list[float]] = []
    for index in range(count):
        row = index // 4
        column = index % 4
        targets.append(
            [
                -0.27 + 0.18 * column,
                0.0,
                0.145,
            ]
        )
    return targets

