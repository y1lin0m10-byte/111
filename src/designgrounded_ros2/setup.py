from setuptools import find_packages, setup


package_name = "designgrounded_ros2"


setup(
    name=package_name,
    version="0.5.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/sim_gazebo.launch.py",
                "launch/pick_place.launch.py",
                "launch/vision_experiment.launch.py",
                "launch/real_d405_capture.launch.py",
            ],
        ),
        (f"share/{package_name}/urdf", ["urdf/kr6_d405_workcell.urdf.xacro"]),
        (f"share/{package_name}/config", ["config/controllers.yaml"]),
        (f"share/{package_name}/worlds", ["worlds/workcell.world"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Design-Grounded Research",
    maintainer_email="research@example.com",
    description="KUKA KR 6 and D405 modular wood assembly simulation.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "assembly_demo = designgrounded_ros2.assembly_node:main",
            "wood_block_perception = designgrounded_ros2.perception_node:main",
            "wood_pick_place = designgrounded_ros2.pick_place_node:main",
            "vision_scene_manager = designgrounded_ros2.scene_manager_node:main",
            "vision_benchmark = designgrounded_ros2.vision_benchmark_node:main",
            "real_d405_dataset_capture = designgrounded_ros2.dataset_capture_node:main",
        ],
    },
)

