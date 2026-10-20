from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import Buffer, TransformListener

from .component_catalog import build_place_targets
from .vision_core import RGBDConfig, RGBDWoodVision, TargetSlot


class WoodBlockPerception(Node):
    def __init__(self) -> None:
        super().__init__("wood_block_perception")
        self.declare_parameter("color_topic", "/d405/color/image_raw")
        self.declare_parameter("depth_topic", "/d405/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/d405/color/camera_info")
        self.declare_parameter("camera_frame", "d405_color_optical_frame")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("target_slots_json", "")
        self.declare_parameter("min_mask_area_px", 80.0)
        self.declare_parameter("min_confidence", 0.15)
        self.declare_parameter("target_match_distance_m", 0.15)
        self.declare_parameter("depth_min_m", 0.05)
        self.declare_parameter("depth_max_m", 1.20)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_publisher = self.create_publisher(
            PoseArray,
            "/detected_wood_blocks",
            10,
        )
        self.info_publisher = self.create_publisher(
            String,
            "/detected_wood_blocks_info",
            10,
        )
        self.state_publisher = self.create_publisher(
            String,
            "/vision/visual_state",
            10,
        )
        self.uncertainty_publisher = self.create_publisher(
            String,
            "/vision/uncertainty",
            10,
        )
        self.active_publisher = self.create_publisher(
            String,
            "/vision/active_observation",
            10,
        )
        self.mask_publisher = self.create_publisher(
            Image,
            "/vision/instance_masks",
            10,
        )

        camera_frame = str(self.get_parameter("camera_frame").value)
        target_frame = str(self.get_parameter("target_frame").value)
        targets = self.load_targets()
        if target_frame != "world":
            targets = []
        self.vision = RGBDWoodVision(
            RGBDConfig(
                min_mask_area_px=float(
                    self.get_parameter("min_mask_area_px").value
                ),
                min_confidence=float(
                    self.get_parameter("min_confidence").value
                ),
                target_match_distance_m=float(
                    self.get_parameter("target_match_distance_m").value
                ),
                depth_min_m=float(self.get_parameter("depth_min_m").value),
                depth_max_m=float(self.get_parameter("depth_max_m").value),
            ),
            targets=targets,
        )

        color_sub = message_filters.Subscriber(
            self,
            Image,
            str(self.get_parameter("color_topic").value),
        )
        depth_sub = message_filters.Subscriber(
            self,
            Image,
            str(self.get_parameter("depth_topic").value),
        )
        camera_info_sub = message_filters.Subscriber(
            self,
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub, camera_info_sub],
            queue_size=10,
            slop=0.08,
        )
        self.sync.registerCallback(self.on_frame)
        self.get_logger().info(
            f"RGB-D vision ready: camera={camera_frame}, target={target_frame}, "
            f"targets={len(targets)}"
        )

    def load_targets(self) -> list[TargetSlot]:
        target_json = str(self.get_parameter("target_slots_json").value)
        if target_json:
            payload = json.loads(Path(target_json).read_text(encoding="utf-8"))
            return [
                TargetSlot(
                    target_id=str(item["target_id"]),
                    position=tuple(float(value) for value in item["position"]),
                    yaw_rad=float(item.get("yaw_rad", 0.0)),
                    role=str(item.get("role", "wood_block")),
                )
                for item in payload
            ]
        return [
            TargetSlot(
                target_id=f"target_{index + 1:02d}",
                position=tuple(float(value) for value in position),
            )
            for index, position in enumerate(build_place_targets(8))
        ]

    def on_frame(
        self,
        color_message: Image,
        depth_message: Image,
        camera_info: CameraInfo,
    ) -> None:
        color = self.bridge.imgmsg_to_cv2(color_message, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
        depth_m = self.depth_to_meters(depth, depth_message.encoding)
        camera_matrix = np.asarray(camera_info.k, dtype=float).reshape(3, 3)
        camera_frame = str(self.get_parameter("camera_frame").value)
        target_frame = str(self.get_parameter("target_frame").value)
        camera_to_target = self.lookup_camera_to_target(
            camera_frame,
            target_frame,
            color_message,
        )
        if camera_to_target is None:
            return
        result = self.vision.process(
            color,
            depth_m,
            camera_matrix,
            camera_to_world=camera_to_target,
        )
        if result.segmentation_mask is not None:
            mask_message = self.bridge.cv2_to_imgmsg(
                result.segmentation_mask,
                encoding="mono8",
            )
            mask_message.header = color_message.header
            self.mask_publisher.publish(mask_message)

        pose_array = PoseArray()
        pose_array.header.stamp = color_message.header.stamp
        pose_array.header.frame_id = target_frame
        for instance in result.instances:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = instance.position
            pose.orientation.x = instance.quaternion_xyzw[0]
            pose.orientation.y = instance.quaternion_xyzw[1]
            pose.orientation.z = instance.quaternion_xyzw[2]
            pose.orientation.w = instance.quaternion_xyzw[3]
            pose_array.poses.append(pose)
        self.pose_publisher.publish(pose_array)

        instances_payload = [
            {
                "detection_id": instance.detection_id,
                "world_xyz_m": list(instance.position),
                "position": list(instance.position),
                "quaternion_xyzw": list(instance.quaternion_xyzw),
                "dimensions": list(instance.dimensions),
                "confidence": instance.confidence,
                "uncertainty": instance.uncertainty,
                "target_id": instance.target_id,
                "visual_state": instance.visual_state,
            }
            for instance in result.instances
        ]
        self.info_publisher.publish(
            self.json_message(
                {
                    "frame_id": target_frame,
                    "count": len(result.instances),
                    "matched_count": result.matched_count,
                    "uncertain_count": result.uncertain_count,
                    "detections": instances_payload,
                }
            )
        )
        self.state_publisher.publish(
            self.json_message(
                {
                    "frame_id": target_frame,
                    "instances": [instance.to_dict() for instance in result.instances],
                }
            )
        )
        self.uncertainty_publisher.publish(
            self.json_message(
                {
                    "mean_uncertainty": (
                        float(
                            np.mean(
                                [instance.uncertainty for instance in result.instances]
                            )
                        )
                        if result.instances
                        else 0.0
                    ),
                    "uncertain_count": result.uncertain_count,
                    "instances": [
                        {
                            "detection_id": instance.detection_id,
                            "uncertainty": instance.uncertainty,
                            "position_uncertainty_m2": instance.position_uncertainty_m2,
                            "orientation_uncertainty_rad2": instance.orientation_uncertainty_rad2,
                        }
                        for instance in result.instances
                    ],
                }
            )
        )
        self.active_publisher.publish(
            self.json_message(
                {
                    "observations": result.active_observations,
                }
            )
        )

    def lookup_camera_to_target(
        self,
        camera_frame: str,
        target_frame: str,
        message: Image,
    ) -> np.ndarray | None:
        if camera_frame == target_frame:
            return np.eye(4)
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception:
            return None
        matrix = np.eye(4)
        rotation = transform.transform.rotation
        translation = transform.transform.translation
        matrix[:3, :3] = quaternion_to_matrix(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        matrix[:3, 3] = [translation.x, translation.y, translation.z]
        return matrix

    @staticmethod
    def depth_to_meters(depth: np.ndarray, encoding: str) -> np.ndarray:
        depth_float = depth.astype(np.float32)
        if encoding == "16UC1":
            return depth_float / 1000.0
        return depth_float

    @staticmethod
    def json_message(payload: dict[str, Any]) -> String:
        message = String()
        message.data = json.dumps(payload)
        return message


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WoodBlockPerception()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

