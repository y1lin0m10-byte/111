from __future__ import annotations

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("designgrounded_ros2")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "sim_gazebo.launch.py"])
        ),
        launch_arguments={"start_assembly_demo": "false"}.items(),
    )

    perception = Node(
        package="designgrounded_ros2",
        executable="wood_block_perception",
        parameters=[
            {
                "use_sim_time": True,
                "color_topic": "/d405/color/image_raw",
                "depth_topic": "/d405/depth/image_raw",
                "camera_info_topic": "/d405/color/camera_info",
                "camera_frame": "d405_color_optical_frame",
                "target_frame": "world",
            }
        ],
        output="screen",
    )

    pick_place = Node(
        package="designgrounded_ros2",
        executable="wood_pick_place",
        parameters=[
            {
                "use_sim_time": True,
                "source_block_count": 8,
                "cycle_time_s": 2.0,
                "spawn_delay_s": 10.0,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            simulation,
            TimerAction(period=8.0, actions=[perception]),
            TimerAction(period=10.0, actions=[pick_place]),
        ]
    )

