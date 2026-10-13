from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("designgrounded_ros2")
    world_path = PathJoinSubstitution([package_share, "worlds", "workcell.world"])
    xacro_path = PathJoinSubstitution(
        [package_share, "urdf", "kr6_d405_workcell.urdf.xacro"]
    )
    start_assembly_demo = LaunchConfiguration("start_assembly_demo")

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
                "robot_description": ParameterValue(
                    robot_description,
                    value_type=str,
                ),
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

    assembly_demo_node = Node(
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
        condition=IfCondition(start_assembly_demo),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_assembly_demo",
                default_value="true",
                description="Start the sequential component-spawning demo",
            ),
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
                actions=[assembly_demo_node],
            ),
        ]
    )

