from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from cv_bridge import CvBridge
import cv2
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class DatasetCapture(Node):
    def __init__(self) -> None:
        super().__init__("real_d405_dataset_capture")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("output_dir", "/home/chirss/data/d405_session")
        self.declare_parameter("session_id", "session_001")
        self.declare_parameter("save_every", 5)
        self.declare_parameter("max_frames", 300)
        self.declare_parameter("min_depth_m", 0.05)
        self.declare_parameter("max_depth_m", 1.0)

        output_dir = Path(str(self.get_parameter("output_dir").value))
        self.session_id = str(self.get_parameter("session_id").value)
        self.color_dir = output_dir / "color"
        self.depth_dir = output_dir / "depth"
        self.meta_path = output_dir / "metadata.jsonl"
        self.color_dir.mkdir(parents=True, exist_ok=True)
        self.depth_dir.mkdir(parents=True, exist_ok=True)
        self.meta_path.parent.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.frame_index = 0
        self.saved_count = 0

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
        info_sub = message_filters.Subscriber(
            self,
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub, info_sub],
            queue_size=10,
            slop=0.08,
        )
        self.sync.registerCallback(self.on_frame)
        self.status_publisher = self.create_publisher(String, "/dataset_capture/status", 10)
        self.get_logger().info(f"saving real D405 data to {output_dir}")

    def on_frame(
        self,
        color_message: Image,
        depth_message: Image,
        camera_info: CameraInfo,
    ) -> None:
        self.frame_index += 1
        if self.frame_index % max(int(self.get_parameter("save_every").value), 1):
            return
        max_frames = int(self.get_parameter("max_frames").value)
        if max_frames > 0 and self.saved_count >= max_frames:
            return

        color = self.bridge.imgmsg_to_cv2(color_message, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
        depth_m = self.depth_to_meters(depth, depth_message.encoding)
        minimum = float(self.get_parameter("min_depth_m").value)
        maximum = float(self.get_parameter("max_depth_m").value)
        valid = np.isfinite(depth_m) & (depth_m >= minimum) & (depth_m <= maximum)
        if valid.sum() < 100:
            self.get_logger().warn("skipping frame: too few valid depth pixels")
            return

        stem = f"{self.session_id}_{self.saved_count:06d}"
        color_path = self.color_dir / f"{stem}.png"
        depth_path = self.depth_dir / f"{stem}.png"
        cv2.imwrite(str(color_path), color)
        depth_mm = np.clip(depth_m * 1000.0, 0, 65535).astype(np.uint16)
        cv2.imwrite(str(depth_path), depth_mm)

        metadata = {
            "session_id": self.session_id,
            "frame_id": stem,
            "color_path": str(color_path),
            "depth_path": str(depth_path),
            "color_timestamp_ns": color_message.header.stamp.sec * 1_000_000_000
            + color_message.header.stamp.nanosec,
            "depth_timestamp_ns": depth_message.header.stamp.sec * 1_000_000_000
            + depth_message.header.stamp.nanosec,
            "color_frame_id": color_message.header.frame_id,
            "depth_frame_id": depth_message.header.frame_id,
            "camera_matrix": list(camera_info.k),
            "distortion_model": camera_info.distortion_model,
            "distortion_coefficients": list(camera_info.d),
            "valid_depth_pixels": int(valid.sum()),
            "min_depth_m": float(depth_m[valid].min()),
            "max_depth_m": float(depth_m[valid].max()),
        }
        with self.meta_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(metadata) + "\n")
        self.saved_count += 1
        status = String()
        status.data = json.dumps(
            {
                "session_id": self.session_id,
                "saved_frames": self.saved_count,
                "max_frames": max_frames,
                "last_frame": stem,
            }
        )
        self.status_publisher.publish(status)
        self.get_logger().info(f"saved {stem}")

    @staticmethod
    def depth_to_meters(depth: np.ndarray, encoding: str) -> np.ndarray:
        depth_float = depth.astype(np.float32)
        if encoding == "16UC1":
            return depth_float / 1000.0
        return depth_float


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DatasetCapture()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

