from __future__ import annotations

import random
from typing import Any

from gazebo_msgs.srv import SpawnEntity
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

from .component_catalog import build_source_blocks, source_block_sdf


class SceneManager(Node):
    def __init__(self) -> None:
        super().__init__("vision_scene_manager")
        self.declare_parameter("source_block_count", 8)
        self.declare_parameter("scenario", "clear")
        self.declare_parameter("random_seed", 7)
        self.declare_parameter("spawn_delay_s", 7.0)
        self.declare_parameter("spawn_entity_service", "/spawn_entity")

        self.spawn_client = self.create_client(
            SpawnEntity,
            str(self.get_parameter("spawn_entity_service").value),
        )
        self.ready_publisher = self.create_publisher(String, "/scene_ready", 10)
        self.timer = self.create_timer(0.5, self.on_timer)
        self.start_time = time.monotonic()
        self.spawned = False

    def on_timer(self) -> None:
        if self.spawned:
            return
        if time.monotonic() - self.start_time < float(
            self.get_parameter("spawn_delay_s").value
        ):
            return
        if not self.spawn_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn("waiting for /spawn_entity")
            return
        count = int(self.get_parameter("source_block_count").value)
        scenario = str(self.get_parameter("scenario").value)
        seed = int(self.get_parameter("random_seed").value)
        rng = random.Random(seed)
        blocks = build_source_blocks(count)
        for block in blocks:
            if scenario in {"random", "occluded", "clutter"}:
                block["position"][0] += rng.uniform(-0.035, 0.035)
                block["position"][1] += rng.uniform(-0.025, 0.025)
                block["rotation"][2] += rng.uniform(-0.20, 0.20)
            self.spawn_model(block["id"], source_block_sdf(block))

        clutter_count = {
            "clear": 0,
            "random": 1,
            "occluded": 3,
            "clutter": 5,
        }.get(scenario, 0)
        for index in range(clutter_count):
            self.spawn_model(
                f"visual_clutter_{index + 1:02d}",
                self.clutter_sdf(index),
            )
        self.spawned = True
        message = String()
        message.data = (
            f'{{"scenario":"{scenario}",'
            f'"source_blocks":{count},'
            f'"clutter_objects":{clutter_count}}}'
        )
        self.ready_publisher.publish(message)
        self.get_logger().info(message.data)

    def spawn_model(self, name: str, xml: str) -> None:
        request = SpawnEntity.Request()
        request.name = name
        request.xml = xml
        request.reference_frame = "world"
        self.spawn_client.call_async(request)

    @staticmethod
    def clutter_sdf(index: int) -> str:
        x = 0.28 + 0.06 * (index % 2)
        y = 0.10 + 0.07 * (index // 2)
        z = 0.22 + 0.05 * index
        return f"""
<sdf version="1.6">
  <model name="visual_clutter_{index + 1:02d}">
    <static>true</static>
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>0.08 0.04 0.18</size></box></geometry>
        <material>
          <ambient>0.18 0.20 0.22 1</ambient>
          <diffuse>0.30 0.33 0.36 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry><box><size>0.08 0.04 0.18</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
""".strip()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SceneManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

