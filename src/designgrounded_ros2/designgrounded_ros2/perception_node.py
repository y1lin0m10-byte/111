from __future__ import annotations

import json
from typing import Any

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Pose, PoseArray
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import Buffer, TransformListener


class WoodBlockPerception(Node):
    def __init__(self) -> None:
        super().__init__("wood_block_perception")
        self.declare_parameter("color_topic", "/d405/color/image_raw")
        self.declare_parameter("depth_topic", "/d405/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/d405/color/camera_info")
        self.declare_parameter("camera_frame", "d405_color_optical_frame")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("min_contour_area_px", 80.0)
        self.declare_parameter("max_blocks", 8)

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
            slop=0.1,
        )
        self.sync.registerCallback(self.on_frame)
        self.get_logger().info("D405 wood-block perception ready")

    def on_frame(
        self,
        color_message: Image,
        depth_message: Image,
        camera_info: CameraInfo,
    ) -> None:
        color = self.bridge.imgmsg_to_cv2(color_message, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
        depth_m = self.depth_to_meters(depth, depth_message.encoding)
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([5, 45, 35], dtype=np.uint8),
            np.array([35, 255, 255], dtype=np.uint8),
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections: list[dict[str, Any]] = []
        max_blocks = int(self.get_parameter("max_blocks").value)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < float(self.get_parameter("min_contour_area_px").value):
                continue
            x, y, width, height = cv2.boundingRect(contour)
            component_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(component_mask, [contour], -1, 255, -1)
            valid_depth = depth_m[component_mask > 0]
            valid_depth = valid_depth[np.isfinite(valid_depth)]
            if valid_depth.size < 10:
                continue
            z = float(np.median(valid_depth))
            if not 0.05 <= z <= 1.0:
                continue
            u = x + width / 2.0
            v = y + height / 2.0
            camera_point = self.pixel_to_camera_point(u, v, z, camera_info)
            world_point = self.transform_to_world(camera_point, color_message)
            if world_point is None:
                continue
            detections.append(
                {
                    "center_px": [float(u), float(v)],
                    "camera_xyz_m": [float(camera_point[0]), float(camera_point[1]), float(camera_point[2])],
                    "world_xyz_m": [
                        float(world_point.point.x),
                        float(world_point.point.y),
                        float(world_point.point.z),
                    ],
                    "area_px": area,
                    "confidence": min(area / 400.0, 1.0),
                }
            )

        detections.sort(key=lambda item: (item["world_xyz_m"][0], item["world_xyz_m"][1]))
        detections = detections[:max_blocks]
        pose_array = PoseArray()
        pose_array.header.stamp = color_message.header.stamp
        pose_array.header.frame_id = str(self.get_parameter("target_frame").value)
        for detection in detections:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = detection["world_xyz_m"]
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        self.pose_publisher.publish(pose_array)
        info = String()
        info.data = json.dumps(
            {
                "frame_id": pose_array.header.frame_id,
                "count": len(detections),
                "detections": detections,
            }
        )
        self.info_publisher.publish(info)

    @staticmethod
    def depth_to_meters(depth: np.ndarray, encoding: str) -> np.ndarray:
        depth_float = depth.astype(np.float32)
        if encoding == "16UC1":
            return depth_float / 1000.0
        return depth_float

    @staticmethod
    def pixel_to_camera_point(
        u: float,
        v: float,
        z: float,
        camera_info: CameraInfo,
    ) -> tuple[float, float, float]:
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])
        return ((u - cx) * z / fx, (v - cy) * z / fy, z)

    def transform_to_world(
        self,
        camera_point: tuple[float, float, float],
        source_header: Any,
    ) -> PointStamped | None:
        point = PointStamped()
        point.header = source_header
        point.header.frame_id = str(self.get_parameter("camera_frame").value)
        point.point.x, point.point.y, point.point.z = camera_point
        target_frame = str(self.get_parameter("target_frame").value)
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                point.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception:
            return None
        return tf2_geometry_msgs.do_transform_point(point, transform)


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

