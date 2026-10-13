#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble/setup.bash" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
cd "${SCRIPT_DIR}"

python3 scripts/validate_ros2_workspace.py
colcon build --symlink-install
source install/setup.bash

ros2 launch designgrounded_ros2 sim_gazebo.launch.py

