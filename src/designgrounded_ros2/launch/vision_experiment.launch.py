from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("designgrounded_ros2")
    scenario = LaunchConfiguration("scenario")
    random_seed = LaunchConfiguration("random_seed")
    output_csv = LaunchConfiguration("output_csv")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "sim_gazebo.launch.py"])
        ),
        launch_arguments={"start_assembly_demo": "false"}.items(),
    )

    scene_manager = Node(
        package="designgrounded_ros2",
        executable="vision_scene_manager",
        parameters=[
            {
                "use_sim_time": True,
                "source_block_count": 8,
                "scenario": scenario,
                "random_seed": random_seed,
                "spawn_delay_s": 7.0,
            }
        ],
        output="screen",
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
                "max_blocks": 8,
            }
        ],
        output="screen",
    )

    benchmark = Node(
        package="designgrounded_ros2",
        executable="vision_benchmark",
        parameters=[
            {
                "use_sim_time": True,
                "model_states_topic": "/gazebo/model_states",
                "detections_topic": "/detected_wood_blocks_info",
                "output_csv": output_csv,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario",
                default_value="clear",
                description="clear, random, occluded or clutter",
            ),
            DeclareLaunchArgument("random_seed", default_value="7"),
            DeclareLaunchArgument(
                "output_csv",
                default_value="/tmp/vision_metrics.csv",
            ),
            simulation,
            TimerAction(period=7.0, actions=[scene_manager]),
            TimerAction(period=8.0, actions=[perception, benchmark]),
        ]
    )

