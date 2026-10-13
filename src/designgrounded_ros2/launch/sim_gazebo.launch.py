from __future__ import annotations

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("designgrounded_ros2")
    world_path = PathJoinSubstitution([package_share, "worlds", "workcell.world"])
    xacro_path = PathJoinSubstitution(
        [package_share, "urdf", "kr6_d405_workcell.urdf.xacro"]
    )
    controller_config = PathJoinSubstitution(
        [package_share, "config", "controllers.yaml"]
    )

    robot_description = Command(
        [FindExecutable(name="xacro"), " ", xacro_path]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={"world": world_path}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "kr6_r900_2",
            "-x",
            "-0.40",
            "-y",
            "0.22",
            "-z",
            "0.10",
        ],
        output="screen",
    )

    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    assembly_demo = Node(
        package="designgrounded_ros2",
        executable="assembly_demo",
        parameters=[
            {
                "use_sim_time": True,
                "auto_start": True,
                "cycle_time_s": 2.0,
                "spawn_components": True,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            spawn_robot,
            TimerAction(
                period=5.0,
                actions=[spawn_joint_state_broadcaster],
            ),
            TimerAction(
                period=7.0,
                actions=[spawn_joint_trajectory_controller],
            ),
            TimerAction(
                period=9.0,
                actions=[assembly_demo],
            ),
        ]
    )

