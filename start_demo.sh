#!/usr/bin/env bash

# Start the 0514 AMR demo stack in order:
# YOLO-World -> class setup -> mission node -> autonomous demo loop.

set -eo pipefail

YOLO_WAIT_SEC="${YOLO_WAIT_SEC:-25}"
MISSION_WAIT_SEC="${MISSION_WAIT_SEC:-3}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
PAUSE_FLAG_FILE="${AMR_PAUSE_FLAG_FILE:-/tmp/demo_loop_pause}"

CLASSES=(
  "forklift"
  "fire extinguisher"
  "grey barrel"
  "traffic cone"
  "person"
  "red cylinder"
  "box"
  "package"
  "Cart"
  "Ladder"
)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-/workspaces/isaac_ros-dev}"

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

ensure_not_running() {
  local pattern="launch_three_yolo_world.sh|multi_camera_object_mission.py|demo_loop_runner.py"
  if pgrep -af "${pattern}" >/dev/null; then
    echo "Demo process already running. Stop it first with ./stop_demo.sh" >&2
    pgrep -af "${pattern}" >&2
    exit 1
  fi
}

wait_for_set_class_services() {
  local deadline=$((SECONDS + YOLO_WAIT_SEC))
  local services=(
    "/yolo_front/set_classes"
    "/yolo_left/set_classes"
    "/yolo_right/set_classes"
  )

  while (( SECONDS < deadline )); do
    local ready=1
    local service
    for service in "${services[@]}"; do
      if ! ros2 service list | grep -Fx "${service}" >/dev/null; then
        ready=0
        break
      fi
    done
    if [[ "${ready}" -eq 1 ]]; then
      return 0
    fi
    sleep 1
  done

  echo "Timed out waiting for YOLO set_classes services." >&2
  ros2 service list | grep set_classes >&2 || true
  return 1
}

main() {
  cd "${WORKSPACE_DIR}"
  source_ros
  cd "${SCRIPT_DIR}"

  ensure_not_running
  rm -f "${PAUSE_FLAG_FILE}"

  echo "Starting YOLO-World..."
  nohup ./launch_three_yolo_world.sh > launch_three_yolo_world.log 2>&1 &
  echo "$!" > launch_three_yolo_world.pid

  wait_for_set_class_services

  echo "Setting YOLO classes: ${CLASSES[*]}"
  ./set_multi_yolo_classes.sh "${CLASSES[@]}" > set_multi_yolo_classes.log 2>&1

  echo "Starting mission node..."
  nohup python3 multi_camera_object_mission.py > multi_camera_object_mission.log 2>&1 &
  echo "$!" > multi_camera_object_mission.pid

  sleep "${MISSION_WAIT_SEC}"

  echo "Starting demo loop..."
  nohup python3 demo_loop_runner.py > demo_loop_runner.stdout.log 2>&1 &
  echo "$!" > demo_loop_runner.pid

  echo "Started AMR demo stack."
  pgrep -af "launch_three_yolo_world.sh|multi_camera_object_mission.py|demo_loop_runner.py|yolo-world.launch.py|yolo_ros" || true
}

main "$@"
