from __future__ import annotations

import math
import unittest

import numpy as np

from designgrounded_ros2.vision_core import (
    RGBDConfig,
    RGBDWoodVision,
    TargetSlot,
    TemperatureCalibrator,
    rotation_matrix_to_quaternion,
)


def box_surface_points(
    center: np.ndarray,
    dimensions: np.ndarray,
    yaw: float,
    *,
    points_per_face: int = 300,
    seed: int = 4,
) -> np.ndarray:
    rng = np.random.default_rng(seed)
    points = []
    half = dimensions / 2.0
    for axis in range(3):
        for sign in (-1.0, 1.0):
            face = rng.uniform(-half, half, size=(points_per_face, 3))
            face[:, axis] = sign * half[axis]
            points.append(face)
    local = np.vstack(points)
    rotation = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    return local @ rotation.T + center


class VisionCoreTests(unittest.TestCase):
    def setUp(self) -> None:
        self.vision = RGBDWoodVision(
            RGBDConfig(
                min_confidence=0.0,
                target_match_distance_m=0.10,
            )
        )

    def test_box_pose_estimation(self) -> None:
        center = np.array([0.25, 0.05, 0.42])
        dimensions = np.array([0.12, 0.08, 0.04])
        yaw = math.radians(25.0)
        points = box_surface_points(center, dimensions, yaw)
        rgb = np.full((40, 40, 3), (40, 120, 180), dtype=np.uint8)
        mask = np.ones((40, 40), dtype=np.uint8)
        estimate = self.vision.estimate_pose(
            points,
            400.0,
            rgb,
            mask.astype(bool),
            len(points),
            detection_id="box",
        )
        self.assertLess(math.dist(estimate.position, center), 0.01)
        self.assertEqual(len(estimate.quaternion_xyzw), 4)
        self.assertAlmostEqual(
            float(np.linalg.norm(estimate.quaternion_xyzw)),
            1.0,
            places=6,
        )
        self.assertEqual(estimate.covariance.shape, (6, 6))
        self.assertGreater(estimate.confidence, 0.2)

    def test_rotation_quaternion_is_normalized(self) -> None:
        quaternion = rotation_matrix_to_quaternion(np.eye(3))
        self.assertAlmostEqual(float(np.linalg.norm(quaternion)), 1.0, places=8)

    def test_target_matching(self) -> None:
        vision = RGBDWoodVision(
            RGBDConfig(target_match_distance_m=0.10),
            targets=[
                TargetSlot("target_a", (0.10, 0.0, 0.20)),
                TargetSlot("target_b", (0.30, 0.0, 0.20)),
            ],
        )
        rgb = np.full((40, 40, 3), (40, 120, 180), dtype=np.uint8)
        mask = np.ones((40, 40), dtype=bool)
        first = vision.estimate_pose(
            box_surface_points(
                np.array([0.105, 0.0, 0.20]),
                np.array([0.1, 0.08, 0.04]),
                0.0,
            ),
            400.0,
            rgb,
            mask,
            1800,
            detection_id="first",
        )
        second = vision.estimate_pose(
            box_surface_points(
                np.array([0.295, 0.0, 0.20]),
                np.array([0.1, 0.08, 0.04]),
                0.0,
            ),
            400.0,
            rgb,
            mask,
            1800,
            detection_id="second",
        )
        vision.match_targets([first, second])
        self.assertEqual(first.target_id, "target_a")
        self.assertEqual(second.target_id, "target_b")
        self.assertEqual(first.visual_state, "matched")

    def test_temperature_calibrator(self) -> None:
        calibrator = TemperatureCalibrator(temperature=2.0, bias=0.0)
        self.assertLess(calibrator.calibrate(0.95), 0.95)
        self.assertGreater(calibrator.calibrate(0.05), 0.05)

    def test_depth_projection_shape(self) -> None:
        depth = np.full((4, 5), 0.4, dtype=np.float32)
        camera = np.array(
            [[100.0, 0.0, 2.0], [0.0, 100.0, 2.0], [0.0, 0.0, 1.0]]
        )
        points = RGBDWoodVision.depth_to_points(depth, camera)
        self.assertEqual(points.shape, (4, 5, 3))
        array = points[2, 2]
        self.assertLess(abs(float(array[2]) - 0.4), 1e-6)


if __name__ == "__main__":
    unittest.main()

