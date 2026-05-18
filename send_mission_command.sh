#!/usr/bin/env bash

# Publish a target class command to the persistent mission node.
# This avoids nested JSON quoting for shells and small-model tool harnesses.

set -eo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 '<target class name>'" >&2
  exit 2
fi

target_class_name="$*"

if [[ -f "/workspaces/isaac_ros-dev/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source /workspaces/isaac_ros-dev/install/setup.bash
elif [[ -f "install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source install/setup.bash
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"

ros2 topic pub --once /multi_camera_object_mission/command std_msgs/msg/String \
  "{data: '${target_class_name}'}"
