#!/usr/bin/env bash

# Stop the AMR demo stack started by start_demo.sh.

set -eo pipefail

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
WORKSPACE_DIR="${WORKSPACE_DIR:-/workspaces/isaac_ros-dev}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source_ros() {
  if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${WORKSPACE_DIR}/install/setup.bash"
  elif [[ -f "install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source install/setup.bash
  fi
  export ROS_DOMAIN_ID
}

main() {
  cd "${WORKSPACE_DIR}"
  source_ros
  cd "${SCRIPT_DIR}"

  ros2 topic pub --once /multi_camera_object_mission/cancel std_msgs/msg/String \
    "{data: 'stop demo'}" >/dev/null 2>&1 || true
  ros2 topic pub --rate 20 --times 60 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true

  pkill -INT -f "demo_loop_runner.py|multi_camera_object_mission.py|launch_three_yolo_world.sh|yolo-world.launch.py|yolo_ros" || true
  sleep 3
  pkill -TERM -f "demo_loop_runner.py|multi_camera_object_mission.py|launch_three_yolo_world.sh|yolo-world.launch.py|yolo_ros" || true

  rm -f launch_three_yolo_world.pid multi_camera_object_mission.pid demo_loop_runner.pid

  echo "Stopped AMR demo stack."
  pgrep -af "demo_loop_runner.py|multi_camera_object_mission.py|launch_three_yolo_world.sh|yolo-world.launch.py|yolo_ros" || true
}

main "$@"
