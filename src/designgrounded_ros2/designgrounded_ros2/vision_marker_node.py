from __future__ import annotations

import json

from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class VisionMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_marker_node")
        self.declare_parameter("visual_state_topic", "/vision/visual_state")
        self.publisher = self.create_publisher(MarkerArray, "/vision/visualization", 10)
        self.subscription = self.create_subscription(
            String,
            str(self.get_parameter("visual_state_topic").value),
            self.on_visual_state,
            10,
        )

    def on_visual_state(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        marker_array = MarkerArray()
        frame_id = str(payload.get("frame_id", "world"))
        for index, instance in enumerate(payload.get("instances", [])):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "wood_blocks"
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = self.pose_from_instance(instance)
            dimensions = instance.get("dimensions", [0.1, 0.08, 0.04])
            marker.scale.x = float(dimensions[0])
            marker.scale.y = float(dimensions[1])
            marker.scale.z = float(dimensions[2])
            uncertainty = float(instance.get("uncertainty", 1.0))
            matched = instance.get("target_id") is not None
            if uncertainty >= 0.45:
                marker.color.r = 1.0
                marker.color.g = 0.25
                marker.color.b = 0.10
            elif matched:
                marker.color.r = 0.15
                marker.color.g = 0.75
                marker.color.b = 0.30
            else:
                marker.color.r = 0.15
                marker.color.g = 0.55
                marker.color.b = 0.95
            marker.color.a = 0.38
            marker.lifetime.sec = 1
            marker_array.markers.append(marker)
        self.publisher.publish(marker_array)

    @staticmethod
    def pose_from_instance(instance: dict) -> Pose:
        pose = Pose()
        position = instance.get("position", [0.0, 0.0, 0.0])
        quaternion = instance.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
        pose.position.x, pose.position.y, pose.position.z = [
            float(value) for value in position
        ]
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = [
            float(value) for value in quaternion
        ]
        return pose


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VisionMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

