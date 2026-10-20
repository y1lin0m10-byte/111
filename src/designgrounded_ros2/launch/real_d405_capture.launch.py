from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("designgrounded_ros2")
    camera_namespace = LaunchConfiguration("camera_namespace")
    camera_name = LaunchConfiguration("camera_name")
    serial_no = LaunchConfiguration("serial_no")
    output_dir = LaunchConfiguration("output_dir")
    session_id = LaunchConfiguration("session_id")
    start_camera = LaunchConfiguration("start_camera")
    camera_frame = LaunchConfiguration("camera_frame")
    target_frame = LaunchConfiguration("target_frame")

    color_topic = PathJoinSubstitution(
        ["/", camera_namespace, "/", camera_name, "/color/image_raw"]
    )
    depth_topic = PathJoinSubstitution(
        ["/", camera_namespace, "/", camera_name, "/depth/image_raw"]
    )
    camera_info_topic = PathJoinSubstitution(
        ["/", camera_namespace, "/", camera_name, "/color/camera_info"]
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        condition=IfCondition(start_camera),
        launch_arguments={
            "camera_namespace": camera_namespace,
            "camera_name": camera_name,
            "serial_no": serial_no,
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
        }.items(),
    )

    capture = Node(
        package="designgrounded_ros2",
        executable="real_d405_dataset_capture",
        parameters=[
            {
                "use_sim_time": False,
                "color_topic": color_topic,
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
                "output_dir": output_dir,
                "session_id": session_id,
                "save_every": 5,
                "max_frames": 300,
            }
        ],
        output="screen",
    )

    perception = Node(
        package="designgrounded_ros2",
        executable="wood_block_perception",
        parameters=[
            {
                "use_sim_time": False,
                "color_topic": color_topic,
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
                "camera_frame": camera_frame,
                "target_frame": target_frame,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_namespace", default_value="camera"),
            DeclareLaunchArgument("camera_name", default_value="camera"),
            DeclareLaunchArgument("serial_no", default_value=""),
            DeclareLaunchArgument(
                "output_dir",
                default_value="/home/chirss/data/d405_session",
            ),
            DeclareLaunchArgument("session_id", default_value="session_001"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument(
                "camera_frame",
                default_value="camera_color_optical_frame",
            ),
            DeclareLaunchArgument(
                "target_frame",
                default_value="camera_color_optical_frame",
                description="Use world after hand-eye TF calibration is installed",
            ),
            camera,
            capture,
            perception,
        ]
    )

