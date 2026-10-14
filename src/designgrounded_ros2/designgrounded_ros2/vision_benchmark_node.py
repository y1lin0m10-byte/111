from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any

from gazebo_msgs.msg import ModelStates
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VisionBenchmark(Node):
    def __init__(self) -> None:
        super().__init__("vision_benchmark")
        self.declare_parameter("model_states_topic", "/gazebo/model_states")
        self.declare_parameter("detections_topic", "/detected_wood_blocks_info")
        self.declare_parameter("match_distance_m", 0.15)
        self.declare_parameter("output_csv", "")

        self.true_positions: dict[str, tuple[float, float, float]] = {}
        self.detections: list[list[float]] = []
        self.frames = 0
        self.true_positive = 0
        self.false_positive = 0
        self.false_negative = 0
        self.error_sum = 0.0
        self.error_count = 0

        self.model_subscription = self.create_subscription(
            ModelStates,
            str(self.get_parameter("model_states_topic").value),
            self.on_model_states,
            10,
        )
        self.detection_subscription = self.create_subscription(
            String,
            str(self.get_parameter("detections_topic").value),
            self.on_detections,
            10,
        )
        self.metrics_publisher = self.create_publisher(String, "/vision/metrics", 10)
        self.timer = self.create_timer(1.0, self.publish_metrics)
        self.csv_path = str(self.get_parameter("output_csv").value)
        if self.csv_path:
            path = Path(self.csv_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            if not path.exists():
                with path.open("w", newline="", encoding="utf-8") as handle:
                    writer = csv.writer(handle)
                    writer.writerow(
                        [
                            "frame",
                            "true_count",
                            "detected_count",
                            "true_positive",
                            "false_positive",
                            "false_negative",
                            "position_rmse_m",
                        ]
                    )
        self.get_logger().info("vision benchmark ready")

    def on_model_states(self, message: ModelStates) -> None:
        self.true_positions = {
            name: (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            )
            for name, pose in zip(message.name, message.pose)
            if name.startswith("source_block_")
        }

    def on_detections(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.detections = [
            list(item.get("world_xyz_m", []))
            for item in payload.get("detections", [])
            if len(item.get("world_xyz_m", [])) == 3
        ]
        self.evaluate_once()

    def evaluate_once(self) -> None:
        if not self.true_positions:
            return
        self.frames += 1
        unmatched_true = set(self.true_positions)
        unmatched_detections = set(range(len(self.detections)))
        match_distance = float(self.get_parameter("match_distance_m").value)
        matches: list[tuple[str, int, float]] = []
        for name in list(unmatched_true):
            true_position = self.true_positions[name]
            candidates = [
                (
                    math.dist(true_position, self.detections[index]),
                    index,
                )
                for index in unmatched_detections
            ]
            if not candidates:
                continue
            distance, index = min(candidates)
            if distance <= match_distance:
                matches.append((name, index, distance))
                unmatched_true.remove(name)
                unmatched_detections.remove(index)

        self.true_positive += len(matches)
        self.false_positive += len(unmatched_detections)
        self.false_negative += len(unmatched_true)
        self.error_sum += sum(distance for _, _, distance in matches)
        self.error_count += len(matches)

        if self.csv_path:
            with Path(self.csv_path).open("a", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(
                    [
                        self.frames,
                        len(self.true_positions),
                        len(self.detections),
                        len(matches),
                        len(unmatched_detections),
                        len(unmatched_true),
                        (
                            math.sqrt(
                                sum(distance * distance for _, _, distance in matches)
                                / len(matches)
                            )
                            if matches
                            else 0.0
                        ),
                    ]
                )

    def publish_metrics(self) -> None:
        precision = self.true_positive / max(
            self.true_positive + self.false_positive,
            1,
        )
        recall = self.true_positive / max(
            self.true_positive + self.false_negative,
            1,
        )
        mean_error = self.error_sum / max(self.error_count, 1)
        message = String()
        message.data = json.dumps(
            {
                "frames": self.frames,
                "true_positions": len(self.true_positions),
                "detections": len(self.detections),
                "precision": precision,
                "recall": recall,
                "mean_match_error_m": mean_error,
                "true_positive": self.true_positive,
                "false_positive": self.false_positive,
                "false_negative": self.false_negative,
            }
        )
        self.metrics_publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VisionBenchmark()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

