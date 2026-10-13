from __future__ import annotations

import json
import time
from typing import Any

from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import SpawnEntity
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from .component_catalog import build_components, component_sdf


JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


class AssemblyDemo(Node):
    def __init__(self) -> None:
        super().__init__("assembly_demo")
        self.declare_parameter("spawn_entity_service", "/spawn_entity")
        self.declare_parameter("trajectory_action", "/joint_trajectory_controller/follow_joint_trajectory")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("cycle_time_s", 2.0)
        self.declare_parameter("spawn_components", True)

        self.components = build_components()
        self.index = 0
        self.busy = False
        self.spawn_client = self.create_client(
            SpawnEntity,
            str(self.get_parameter("spawn_entity_service").value),
        )
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("trajectory_action").value),
        )
        self.progress_publisher = self.create_publisher(String, "/assembly_progress", 10)
        self.timer = self.create_timer(0.5, self.on_timer)
        self.get_logger().info(f"assembly demo loaded {len(self.components)} components")

    def on_timer(self) -> None:
        if self.busy or not bool(self.get_parameter("auto_start").value):
            return
        if self.index >= len(self.components):
            return
        component = self.components[self.index]
        self.busy = True
        self.publish_progress(component, "moving")
        if not self.trajectory_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn("trajectory action server not ready")
            self.busy = False
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = self.demo_joint_positions(self.index)
        point.time_from_start.sec = int(float(self.get_parameter("cycle_time_s").value))
        goal.trajectory.points = [point]
        future = self.trajectory_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future: Any) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("trajectory goal rejected")
            self.busy = False
            return
        goal_handle.get_result_async().add_done_callback(self.on_trajectory_done)

    def on_trajectory_done(self, future: Any) -> None:
        component = self.components[self.index]
        if bool(self.get_parameter("spawn_components").value):
            self.spawn_component(component)
        else:
            self.finish_component(component)

    def spawn_component(self, component: dict[str, Any]) -> None:
        if not self.spawn_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("spawn_entity service unavailable")
            self.finish_component(component)
            return
        request = SpawnEntity.Request()
        request.name = component["id"]
        request.xml = component_sdf(component)
        request.robot_namespace = ""
        request.reference_frame = "world"
        future = self.spawn_client.call_async(request)
        future.add_done_callback(lambda _future: self.finish_component(component))

    def finish_component(self, component: dict[str, Any]) -> None:
        self.publish_progress(component, "installed")
        self.index += 1
        self.busy = False

    def publish_progress(self, component: dict[str, Any], state: str) -> None:
        message = String()
        message.data = json.dumps(
            {
                "component_id": component["id"],
                "label": component["label"],
                "group": component["group"],
                "state": state,
                "installed_count": self.index if state == "installed" else self.index,
                "total_count": len(self.components),
                "timestamp": time.time(),
            }
        )
        self.progress_publisher.publish(message)

    @staticmethod
    def demo_joint_positions(index: int) -> list[float]:
        x = index % 5
        return [
            -0.85 + 0.40 * x,
            -0.55 + 0.18 * x,
            0.95 - 0.12 * x,
            -0.35 + 0.16 * (index % 4),
            -0.45 + 0.20 * (index % 3),
            0.65 - 0.18 * (index % 4),
        ]


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AssemblyDemo()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

