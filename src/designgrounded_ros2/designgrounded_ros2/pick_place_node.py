from __future__ import annotations

import json
import math
import time
from typing import Any

from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState, SpawnEntity
from geometry_msgs.msg import PoseArray
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from .component_catalog import (
    build_place_targets,
    build_source_blocks,
    source_block_sdf,
)


JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


class PickPlaceNode(Node):
    def __init__(self) -> None:
        super().__init__("wood_pick_place")
        self.declare_parameter("source_block_count", 8)
        self.declare_parameter("cycle_time_s", 2.0)
        self.declare_parameter("spawn_entity_service", "/spawn_entity")
        self.declare_parameter("set_entity_state_service", "/set_entity_state")
        self.declare_parameter("trajectory_action", "/joint_trajectory_controller/follow_joint_trajectory")
        self.declare_parameter("spawn_delay_s", 8.0)

        count = int(self.get_parameter("source_block_count").value)
        self.blocks = build_source_blocks(count)
        self.targets = build_place_targets(count)
        self.completed: set[str] = set()
        self.spawned = False
        self.busy = False
        self.state = "starting"
        self.pending_detection: list[float] | None = None
        self.pending_block: dict[str, Any] | None = None
        self.start_time = time.monotonic()

        self.spawn_client = self.create_client(
            SpawnEntity,
            str(self.get_parameter("spawn_entity_service").value),
        )
        self.set_state_client = self.create_client(
            SetEntityState,
            str(self.get_parameter("set_entity_state_service").value),
        )
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("trajectory_action").value),
        )
        self.detection_subscription = self.create_subscription(
            PoseArray,
            "/detected_wood_blocks",
            self.on_detections,
            10,
        )
        self.progress_publisher = self.create_publisher(
            String,
            "/pick_place_progress",
            10,
        )
        self.timer = self.create_timer(0.5, self.on_timer)
        self.get_logger().info(f"pick-and-place node loaded {len(self.blocks)} source blocks")

    def on_timer(self) -> None:
        elapsed = time.monotonic() - self.start_time
        if elapsed < float(self.get_parameter("spawn_delay_s").value):
            return
        if not self.spawned:
            self.spawn_source_blocks()
            return
        if self.state == "waiting_detection" and not self.busy and self.pending_detection:
            self.begin_pick_place()

    def spawn_source_blocks(self) -> None:
        if not self.spawn_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn("waiting for spawn_entity service")
            return
        for block in self.blocks:
            request = SpawnEntity.Request()
            request.name = block["id"]
            request.xml = source_block_sdf(block)
            request.reference_frame = "world"
            self.spawn_client.call_async(request)
        self.spawned = True
        self.state = "survey"
        self.get_logger().info("source blocks spawned; moving camera to survey pose")
        self.send_trajectory(self.survey_joint_positions(), self.on_survey_done)
        self.publish_progress("survey", None, "moving")

    def on_survey_done(self) -> None:
        self.state = "waiting_detection"
        self.publish_progress("waiting_detection", None, "ready")

    def on_detections(self, message: PoseArray) -> None:
        if self.busy or self.state != "waiting_detection":
            return
        if not message.poses:
            return
        for pose in message.poses:
            point = [pose.position.x, pose.position.y, pose.position.z]
            block = self.closest_available_block(point)
            if block is None:
                continue
            source = block["position"]
            distance = math.dist(point, source)
            if distance <= 0.18:
                self.pending_detection = point
                self.pending_block = block
                self.get_logger().info(
                    f"matched detected block {block['id']} at distance {distance:.3f} m"
                )
                return

    def closest_available_block(self, point: list[float]) -> dict[str, Any] | None:
        candidates = [
            block for block in self.blocks if block["id"] not in self.completed
        ]
        if not candidates:
            return None
        return min(candidates, key=lambda block: math.dist(block["position"], point))

    def begin_pick_place(self) -> None:
        if self.pending_block is None:
            return
        self.busy = True
        self.state = "moving_pick"
        self.publish_progress("moving_pick", self.pending_block, "moving")
        self.send_trajectory(self.pick_joint_positions(), self.on_pick_done)

    def on_pick_done(self) -> None:
        if self.pending_block is None:
            self.busy = False
            self.state = "waiting_detection"
            return
        self.state = "moving_place"
        self.publish_progress("moving_place", self.pending_block, "moving")
        self.send_trajectory(self.place_joint_positions(), self.on_place_done)

    def on_place_done(self) -> None:
        if self.pending_block is None:
            self.busy = False
            self.state = "waiting_detection"
            return
        index = self.blocks.index(self.pending_block)
        self.set_block_pose(self.pending_block, self.targets[index])

    def set_block_pose(self, block: dict[str, Any], target: list[float]) -> None:
        if not self.set_state_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("set_entity_state service unavailable")
            self.finish_place(block)
            return
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = block["id"]
        request.state.reference_frame = "world"
        request.state.pose.position.x = float(target[0])
        request.state.pose.position.y = float(target[1])
        request.state.pose.position.z = float(target[2])
        request.state.pose.orientation.w = 1.0
        future = self.set_state_client.call_async(request)
        future.add_done_callback(lambda _future: self.finish_place(block))

    def finish_place(self, block: dict[str, Any]) -> None:
        self.completed.add(block["id"])
        self.pending_block = None
        self.pending_detection = None
        self.busy = False
        self.state = "waiting_detection"
        self.publish_progress("placed", block, "installed")
        self.get_logger().info(
            f"placed {block['id']}; completed {len(self.completed)}/{len(self.blocks)}"
        )

    def send_trajectory(self, positions: list[float], done_callback: Any) -> None:
        if not self.trajectory_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("trajectory action server unavailable")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(float(self.get_parameter("cycle_time_s").value))
        goal.trajectory.points = [point]
        future = self.trajectory_client.send_goal_async(goal)

        def goal_response_callback(goal_future: Any) -> None:
            goal_handle = goal_future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().warn("trajectory goal rejected")
                return

            def result_callback(_result_future: Any) -> None:
                done_callback()

            goal_handle.get_result_async().add_done_callback(result_callback)

        future.add_done_callback(goal_response_callback)

    def publish_progress(
        self,
        stage: str,
        block: dict[str, Any] | None,
        state: str,
    ) -> None:
        message = String()
        message.data = json.dumps(
            {
                "stage": stage,
                "state": state,
                "block_id": block["id"] if block else None,
                "completed_count": len(self.completed),
                "total_count": len(self.blocks),
                "timestamp": time.time(),
            }
        )
        self.progress_publisher.publish(message)

    @staticmethod
    def pick_joint_positions() -> list[float]:
        return [-0.65, -0.35, 1.05, -0.15, -0.55, 0.55]

    @staticmethod
    def place_joint_positions() -> list[float]:
        return [0.55, -0.20, 0.90, 0.10, -0.35, 0.25]

    @staticmethod
    def survey_joint_positions() -> list[float]:
        return [-0.25, -0.75, 1.25, 0.0, -0.70, 0.0]


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

