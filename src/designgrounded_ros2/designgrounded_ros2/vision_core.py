from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable

import cv2
import numpy as np


@dataclass
class TargetSlot:
    target_id: str
    position: tuple[float, float, float]
    yaw_rad: float = 0.0
    role: str = "wood_block"

    def to_dict(self) -> dict[str, Any]:
        return {
            "target_id": self.target_id,
            "position": list(self.position),
            "yaw_rad": self.yaw_rad,
            "role": self.role,
        }


@dataclass
class PoseEstimate:
    detection_id: str
    position: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    dimensions: tuple[float, float, float]
    covariance: np.ndarray
    confidence: float
    mask_area_px: float
    point_count: int
    target_id: str | None = None
    target_distance_m: float | None = None
    visual_state: str = "detected"

    @property
    def position_uncertainty_m2(self) -> float:
        return float(np.trace(self.covariance[:3, :3]))

    @property
    def orientation_uncertainty_rad2(self) -> float:
        return float(np.trace(self.covariance[3:, 3:]))

    @property
    def uncertainty(self) -> float:
        pose_term = min(
            self.position_uncertainty_m2 * 25.0
            + self.orientation_uncertainty_rad2 * 2.0,
            1.0,
        )
        return min(max(1.0 - self.confidence + 0.35 * pose_term, 0.0), 1.0)

    def to_dict(self) -> dict[str, Any]:
        return {
            "detection_id": self.detection_id,
            "position": [round(float(value), 6) for value in self.position],
            "quaternion_xyzw": [
                round(float(value), 6) for value in self.quaternion_xyzw
            ],
            "dimensions": [round(float(value), 6) for value in self.dimensions],
            "covariance": [
                [round(float(value), 9) for value in row]
                for row in self.covariance
            ],
            "confidence": round(float(self.confidence), 6),
            "uncertainty": round(float(self.uncertainty), 6),
            "position_uncertainty_m2": round(
                float(self.position_uncertainty_m2),
                9,
            ),
            "orientation_uncertainty_rad2": round(
                float(self.orientation_uncertainty_rad2),
                9,
            ),
            "mask_area_px": round(float(self.mask_area_px), 3),
            "point_count": int(self.point_count),
            "target_id": self.target_id,
            "target_distance_m": (
                round(float(self.target_distance_m), 6)
                if self.target_distance_m is not None
                else None
            ),
            "visual_state": self.visual_state,
        }


@dataclass
class RGBDConfig:
    hsv_lower: tuple[int, int, int] = (5, 40, 30)
    hsv_upper: tuple[int, int, int] = (38, 255, 255)
    min_mask_area_px: float = 80.0
    max_points_per_instance: int = 6000
    depth_min_m: float = 0.05
    depth_max_m: float = 1.20
    depth_outlier_mad_scale: float = 3.0
    target_match_distance_m: float = 0.15
    min_confidence: float = 0.15


@dataclass
class VisionFrameResult:
    instances: list[PoseEstimate] = field(default_factory=list)
    active_observations: list[dict[str, Any]] = field(default_factory=list)
    segmentation_mask: np.ndarray | None = None

    @property
    def matched_count(self) -> int:
        return sum(instance.target_id is not None for instance in self.instances)

    @property
    def uncertain_count(self) -> int:
        return sum(instance.uncertainty >= 0.45 for instance in self.instances)


class RGBDWoodVision:
    def __init__(
        self,
        config: RGBDConfig | None = None,
        targets: Iterable[TargetSlot] = (),
    ) -> None:
        self.config = config or RGBDConfig()
        self.targets = list(targets)

    def process(
        self,
        rgb_bgr: np.ndarray,
        depth_m: np.ndarray,
        camera_matrix: np.ndarray,
        *,
        camera_to_world: np.ndarray | None = None,
    ) -> VisionFrameResult:
        if rgb_bgr.shape[:2] != depth_m.shape[:2]:
            raise ValueError("RGB and depth dimensions must match")
        mask = self.segment(rgb_bgr, depth_m)
        instances = self.instances_from_mask(
            rgb_bgr,
            depth_m,
            mask,
            camera_matrix,
            camera_to_world=camera_to_world,
        )
        self.match_targets(instances)
        result = VisionFrameResult(
            instances=instances,
            active_observations=self.rank_active_observations(instances),
            segmentation_mask=mask,
        )
        return result

    def segment(self, rgb_bgr: np.ndarray, depth_m: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array(self.config.hsv_lower, dtype=np.uint8),
            np.array(self.config.hsv_upper, dtype=np.uint8),
        )
        depth_valid = (
            np.isfinite(depth_m)
            & (depth_m >= self.config.depth_min_m)
            & (depth_m <= self.config.depth_max_m)
        )
        mask[~depth_valid] = 0
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask

    def instances_from_mask(
        self,
        rgb_bgr: np.ndarray,
        depth_m: np.ndarray,
        mask: np.ndarray,
        camera_matrix: np.ndarray,
        *,
        camera_to_world: np.ndarray | None = None,
    ) -> list[PoseEstimate]:
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
        points_all = self.depth_to_points(depth_m, camera_matrix)
        instances: list[PoseEstimate] = []
        for label in range(1, count):
            area = float(stats[label, cv2.CC_STAT_AREA])
            if area < self.config.min_mask_area_px:
                continue
            instance_mask = labels == label
            points = points_all[instance_mask]
            points = points[np.all(np.isfinite(points), axis=1)]
            points = self.remove_depth_outliers(points)
            if len(points) < 20:
                continue
            if len(points) > self.config.max_points_per_instance:
                indices = np.linspace(
                    0,
                    len(points) - 1,
                    self.config.max_points_per_instance,
                    dtype=int,
                )
                points = points[indices]
            if camera_to_world is not None:
                homogeneous = np.c_[points, np.ones(len(points))]
                points = (camera_to_world @ homogeneous.T).T[:, :3]
            instance = self.estimate_pose(
                points,
                area,
                rgb_bgr,
                instance_mask,
                len(points),
                detection_id=f"wood_{label:02d}",
            )
            if instance.confidence >= self.config.min_confidence:
                instances.append(instance)
        instances.sort(key=lambda item: (item.position[0], item.position[1]))
        return instances

    def estimate_pose(
        self,
        points: np.ndarray,
        mask_area_px: float,
        rgb_bgr: np.ndarray,
        instance_mask: np.ndarray,
        original_point_count: int,
        *,
        detection_id: str,
    ) -> PoseEstimate:
        centroid = np.mean(points, axis=0)
        centered = points - centroid
        covariance = np.cov(centered, rowvar=False)
        covariance = np.nan_to_num(covariance)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        order = np.argsort(eigenvalues)[::-1]
        eigenvalues = np.maximum(eigenvalues[order], 1e-10)
        axes = eigenvectors[:, order]
        axes = self.canonicalize_axes(axes)
        if np.linalg.det(axes) < 0:
            axes[:, 2] *= -1

        projected = centered @ axes
        lower = projected.min(axis=0)
        upper = projected.max(axis=0)
        dimensions = np.maximum(upper - lower, 1e-5)
        center_local = (lower + upper) / 2.0
        center = centroid + axes @ center_local

        validity = min(original_point_count / max(mask_area_px * 0.40, 1.0), 1.0)
        point_score = min(original_point_count / 300.0, 1.0)
        color_saturation = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2HSV)[..., 1][
            instance_mask
        ]
        color_score = float(np.mean(color_saturation) / 255.0) if len(color_saturation) else 0.0
        planarity = 1.0 - min(float(eigenvalues[0] / max(eigenvalues[1], 1e-9)), 1.0)
        confidence = float(
            np.clip(
                0.30 * min(mask_area_px / 500.0, 1.0)
                + 0.25 * validity
                + 0.20 * point_score
                + 0.15 * color_score
                + 0.10 * planarity,
                0.0,
                1.0,
            )
        )
        covariance_6 = self.build_pose_covariance(
            covariance,
            eigenvalues,
            len(points),
            confidence,
        )
        quaternion = rotation_matrix_to_quaternion(axes)
        return PoseEstimate(
            detection_id=detection_id,
            position=tuple(float(value) for value in center),
            quaternion_xyzw=tuple(float(value) for value in quaternion),
            dimensions=tuple(float(value) for value in dimensions),
            covariance=covariance_6,
            confidence=confidence,
            mask_area_px=mask_area_px,
            point_count=original_point_count,
        )

    def match_targets(self, instances: list[PoseEstimate]) -> None:
        if not self.targets:
            return
        available = set(range(len(self.targets)))
        matches: list[tuple[float, int, int]] = []
        for instance_index, instance in enumerate(instances):
            for target_index in available:
                distance = math.dist(
                    instance.position,
                    self.targets[target_index].position,
                )
                if distance <= self.config.target_match_distance_m:
                    matches.append((distance, instance_index, target_index))
        matches.sort()
        used_instances: set[int] = set()
        used_targets: set[int] = set()
        for distance, instance_index, target_index in matches:
            if instance_index in used_instances or target_index in used_targets:
                continue
            instance = instances[instance_index]
            instance.target_id = self.targets[target_index].target_id
            instance.target_distance_m = distance
            instance.visual_state = (
                "uncertain" if instance.uncertainty >= 0.45 else "matched"
            )
            used_instances.add(instance_index)
            used_targets.add(target_index)
        for instance in instances:
            if instance.target_id is None:
                instance.visual_state = (
                    "uncertain" if instance.uncertainty >= 0.45 else "detected"
                )

    def rank_active_observations(
        self,
        instances: list[PoseEstimate],
    ) -> list[dict[str, Any]]:
        ranked = sorted(
            instances,
            key=lambda instance: (
                instance.uncertainty,
                instance.position_uncertainty_m2,
            ),
            reverse=True,
        )
        return [
            {
                "detection_id": instance.detection_id,
                "target_id": instance.target_id,
                "utility": instance.uncertainty,
                "reason": (
                    "matched target uncertain"
                    if instance.target_id is not None
                    else "unmatched or occluded wood block"
                ),
            }
            for instance in ranked
        ]

    @staticmethod
    def depth_to_points(
        depth_m: np.ndarray,
        camera_matrix: np.ndarray,
    ) -> np.ndarray:
        height, width = depth_m.shape
        yy, xx = np.mgrid[0:height, 0:width]
        fx = float(camera_matrix[0, 0])
        fy = float(camera_matrix[1, 1])
        cx = float(camera_matrix[0, 2])
        cy = float(camera_matrix[1, 2])
        x = (xx - cx) * depth_m / fx
        y = (yy - cy) * depth_m / fy
        return np.dstack([x, y, depth_m])

    def remove_depth_outliers(self, points: np.ndarray) -> np.ndarray:
        if len(points) < 10:
            return points
        z = points[:, 2]
        median = float(np.median(z))
        mad = float(np.median(np.abs(z - median)))
        if mad < 1e-6:
            return points
        threshold = self.config.depth_outlier_mad_scale * 1.4826 * mad
        return points[np.abs(z - median) <= threshold]

    @staticmethod
    def canonicalize_axes(axes: np.ndarray) -> np.ndarray:
        canonical = axes.copy()
        for index in range(3):
            vector = canonical[:, index]
            pivot = int(np.argmax(np.abs(vector)))
            if vector[pivot] < 0:
                canonical[:, index] *= -1
        return canonical

    @staticmethod
    def build_pose_covariance(
        point_covariance: np.ndarray,
        eigenvalues: np.ndarray,
        point_count: int,
        confidence: float,
    ) -> np.ndarray:
        translation = point_covariance / max(point_count, 1)
        translation += np.eye(3) * (0.002 ** 2)
        smallest_ratio = float(eigenvalues[2] / max(eigenvalues[1], 1e-9))
        rotation_variance = 0.004 + 0.08 * smallest_ratio + 0.03 * (1.0 - confidence)
        covariance = np.zeros((6, 6), dtype=float)
        covariance[:3, :3] = translation
        covariance[3:, 3:] = np.eye(3) * rotation_variance
        return covariance


def rotation_matrix_to_quaternion(
    matrix: np.ndarray,
) -> tuple[float, float, float, float]:
    rotation = np.asarray(matrix, dtype=float)
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            w = (rotation[2, 1] - rotation[1, 2]) / scale
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            w = (rotation[0, 2] - rotation[2, 0]) / scale
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            w = (rotation[1, 0] - rotation[0, 1]) / scale
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale
    quaternion = np.array([x, y, z, w], dtype=float)
    quaternion /= max(float(np.linalg.norm(quaternion)), 1e-9)
    return tuple(float(value) for value in quaternion)


@dataclass
class TemperatureCalibrator:
    temperature: float = 1.0
    bias: float = 0.0

    def calibrate(self, probability: float) -> float:
        clipped = min(max(probability, 1e-6), 1.0 - 1e-6)
        logit = math.log(clipped / (1.0 - clipped))
        value = logit / max(self.temperature, 1e-3) + self.bias
        return 1.0 / (1.0 + math.exp(-max(min(value, 30.0), -30.0)))

