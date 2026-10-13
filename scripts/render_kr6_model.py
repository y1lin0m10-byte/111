from __future__ import annotations

import math
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image
import trimesh


ROOT = Path(__file__).resolve().parents[1]
MESH_ROOT = ROOT / "src" / "kuka_kr10_support" / "meshes" / "kr10r900_2" / "visual"
OUTPUT = ROOT / "docs" / "kr6_r900_2_official_model.png"


def translation(x: float, y: float, z: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = [x, y, z]
    return transform


def rotation(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    vector = np.asarray(axis, dtype=float)
    return trimesh.transformations.rotation_matrix(angle, vector)


def mesh_vertices(name: str) -> tuple[np.ndarray, np.ndarray]:
    mesh = trimesh.load(MESH_ROOT / f"{name}.stl", force="mesh")
    return np.asarray(mesh.vertices), np.asarray(mesh.faces)


def transformed_mesh(
    transform: np.ndarray,
    name: str,
) -> tuple[np.ndarray, np.ndarray]:
    vertices, faces = mesh_vertices(name)
    homogeneous = np.c_[vertices, np.ones(len(vertices))]
    transformed = (transform @ homogeneous.T).T[:, :3]
    return transformed, faces


def add_mesh(
    axis: matplotlib.axes.Axes,
    transform: np.ndarray,
    name: str,
    color: tuple[float, float, float],
    *,
    alpha: float = 1.0,
) -> None:
    vertices, faces = transformed_mesh(transform, name)
    polygons = vertices[faces]
    collection = Poly3DCollection(
        polygons,
        facecolors=[(*color, alpha)] * len(faces),
        edgecolors="none",
        linewidths=0.0,
    )
    axis.add_collection3d(collection)


def main() -> int:
    q = [0.0, math.radians(-35.0), math.radians(55.0), 0.0, math.radians(-45.0), 0.0]

    base = np.eye(4)
    joint_1 = base @ translation(0, 0, 0.400) @ rotation((0, 0, -1), q[0])
    joint_2 = joint_1 @ translation(0.025, 0, 0) @ rotation((0, 1, 0), q[1])
    joint_3 = joint_2 @ translation(0.455, 0, 0) @ rotation((0, 1, 0), q[2])
    joint_4 = joint_3 @ translation(0, 0, 0.025) @ rotation((-1, 0, 0), q[3])
    joint_5 = joint_4 @ translation(0.420, 0, 0) @ rotation((0, 1, 0), q[4])
    joint_6 = joint_5 @ translation(0.090, 0, 0) @ rotation((-1, 0, 0), q[5])
    tool0 = joint_6 @ np.array(
        [
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    d405 = tool0 @ translation(0, 0, 0.06)

    figure = plt.figure(figsize=(12, 9), dpi=160)
    axis = figure.add_subplot(111, projection="3d")
    axis.set_facecolor("white")
    figure.patch.set_facecolor("white")

    white = (0.72, 0.74, 0.76)
    gray = (0.62, 0.65, 0.68)
    dark = (0.08, 0.09, 0.10)
    blue = (0.12, 0.48, 0.78)

    add_mesh(axis, base, "base_link", dark)
    add_mesh(axis, joint_1, "link_1", white)
    add_mesh(axis, joint_2, "link_2", white)
    add_mesh(axis, joint_3, "link_3", white)
    add_mesh(axis, joint_4, "link_4", white)
    add_mesh(axis, joint_5, "link_5", white)
    add_mesh(axis, joint_6, "link_6", dark)

    camera_box = trimesh.creation.box(extents=(0.042, 0.042, 0.023))
    vertices = np.asarray(camera_box.vertices)
    homogeneous = np.c_[vertices, np.ones(len(vertices))]
    transformed = (d405 @ homogeneous.T).T[:, :3]
    camera_polygons = transformed[np.asarray(camera_box.faces)]
    axis.add_collection3d(
        Poly3DCollection(
            camera_polygons,
            facecolors=[(*blue, 1.0)] * len(camera_box.faces),
            edgecolors="none",
        )
    )

    axis.set_xlim(-0.15, 1.05)
    axis.set_ylim(-0.55, 0.55)
    axis.set_zlim(-0.05, 1.05)
    axis.set_box_aspect((1.2, 1.1, 1.1))
    axis.view_init(elev=20, azim=-55)
    axis.set_axis_off()
    figure.tight_layout()
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(OUTPUT, bbox_inches="tight")
    plt.close(figure)
    image = Image.open(OUTPUT).convert("RGB")
    pixels = np.asarray(image)
    nonwhite = np.any(pixels < 245, axis=2)
    rows, columns = np.where(nonwhite)
    margin = 35
    if rows.size and columns.size:
        image.crop(
            (
                max(0, int(columns.min()) - margin),
                max(0, int(rows.min()) - margin),
                min(image.width, int(columns.max()) + margin),
                min(image.height, int(rows.max()) + margin),
            )
        ).save(OUTPUT)
    print(OUTPUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

