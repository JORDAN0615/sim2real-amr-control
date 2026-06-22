#!/usr/bin/env bash

# Small, stable command surface for agents that should not hand-write ROS2
# topic commands or shell quoting.

set -eo pipefail

PAUSE_FLAG_FILE="${AMR_PAUSE_FLAG_FILE:-/tmp/demo_loop_pause}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
MISSION_COMMAND_TOPIC="${AMR_MISSION_COMMAND_TOPIC:-/multi_camera_object_mission/command}"
MISSION_CANCEL_TOPIC="${AMR_MISSION_CANCEL_TOPIC:-/multi_camera_object_mission/cancel}"
MISSION_STATUS_TOPIC="${AMR_MISSION_STATUS_TOPIC:-/multi_camera_object_mission/status}"
CMD_VEL_TOPIC="${AMR_CMD_VEL_TOPIC:-/cmd_vel}"
ECHO_TIMEOUT_SEC="${AMR_ECHO_TIMEOUT_SEC:-5}"
GO_CONFIRM_TIMEOUT_SEC="${AMR_GO_CONFIRM_TIMEOUT_SEC:-8}"

DETECTION_TOPICS=(
  "/yolo_front/detections_3d"
  "/yolo_left/detections_3d"
  "/yolo_right/detections_3d"
)

usage() {
  cat >&2 <<'EOF'
Usage:
  ./amr_control.sh pause
  ./amr_control.sh stop
  ./amr_control.sh resume
  ./amr_control.sh status
  ./amr_control.sh visible
  ./amr_control.sh go <person|traffic_cone|fire_extinguisher|grey_barrel|blue_barrel|cardboard_box|box|cart|ladder>
  ./amr_control.sh processes

Notes:
  Use underscore target names for this script. It will publish the correct
  ROS2 class name internally.
EOF
}

setup_ros() {
  if [[ -f "/workspaces/isaac_ros-dev/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source /workspaces/isaac_ros-dev/install/setup.bash
  elif [[ -f "install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source install/setup.bash
  fi
  export ROS_DOMAIN_ID
}

normalize_target() {
  local target="$*"
  target="${target// /_}"
  case "$target" in
    person) echo "person" ;;
    traffic_cone) echo "traffic cone" ;;
    fire_extinguisher) echo "fire extinguisher" ;;
    grey_barrel|gray_barrel) echo "grey barrel" ;;
    blue_barrel) echo "blue barrel" ;;
    cardboard_box) echo "cardboard box" ;;
    box) echo "box" ;;
    cart|Cart) echo "Cart" ;;
    ladder|Ladder) echo "Ladder" ;;
    *)
      echo "Unknown target: $*" >&2
      echo "Allowed: person, traffic_cone, fire_extinguisher, grey_barrel, blue_barrel, cardboard_box, box, cart, ladder" >&2
      exit 2
      ;;
  esac
}

publish_stop() {
  ros2 topic pub --rate 20 --times 60 "$CMD_VEL_TOPIC" geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}"
}

pause_loop() {
  touch "$PAUSE_FLAG_FILE"
  ros2 topic pub --once "$MISSION_CANCEL_TOPIC" std_msgs/msg/String \
    "{data: 'external control'}" >/dev/null
  publish_stop >/dev/null
  echo "paused"
}

resume_loop() {
  rm -f "$PAUSE_FLAG_FILE"
  echo "resumed"
}

show_status() {
  timeout "$ECHO_TIMEOUT_SEC" ros2 topic echo "$MISSION_STATUS_TOPIC" --once
}

read_status_once() {
  timeout "$ECHO_TIMEOUT_SEC" ros2 topic echo "$MISSION_STATUS_TOPIC" --once 2>/dev/null || true
}

wait_for_target_status() {
  local target_class_name="$1"
  local deadline=$((SECONDS + GO_CONFIRM_TIMEOUT_SEC))
  local status

  while (( SECONDS < deadline )); do
    status="$(read_status_once)"
    if printf '%s\n' "$status" | grep -F "\"target_class_name\": \"${target_class_name}\"" >/dev/null; then
      printf '%s\n' "$status"
      return 0
    fi
    sleep 1
  done

  return 1
}

show_visible() {
  local topic
  for topic in "${DETECTION_TOPICS[@]}"; do
    echo "topic=$topic"
    if ! timeout "$ECHO_TIMEOUT_SEC" ros2 topic echo "$topic" --once \
      | awk -F: '
          /class_name:/ {
            value=$2
            sub(/^[ \t]+/, "", value)
            sub(/[ \t]+$/, "", value)
            gsub(/^"|"$/, "", value)
            if (value != "" && !seen[value]++) {
              print "  " value
              found=1
            }
          }
          END { if (!found) print "  none" }
        '; then
      echo "  timeout_or_error"
    fi
  done
  return 0
}

send_target() {
  local target_class_name
  target_class_name="$(normalize_target "$@")"
  touch "$PAUSE_FLAG_FILE"
  ros2 topic pub --once "$MISSION_CANCEL_TOPIC" std_msgs/msg/String \
    "{data: 'external control'}" >/dev/null
  publish_stop >/dev/null
  ros2 topic pub --once "$MISSION_COMMAND_TOPIC" std_msgs/msg/String \
    "{data: '${target_class_name}'}"
  echo "sent target=${target_class_name}"
  echo "status:"
  if ! wait_for_target_status "$target_class_name"; then
    echo "target_status_not_confirmed target=${target_class_name}"
    show_status || true
  fi
}

show_processes() {
  pgrep -af "demo_loop_runner.py|multi_camera_object_mission.py|ros2_move_to_object.py|ros2_move_to_tag.py" || true
}

main() {
  if [[ $# -lt 1 ]]; then
    usage
    exit 2
  fi

  setup_ros

  local command="$1"
  shift
  case "$command" in
    pause) pause_loop ;;
    stop) publish_stop ;;
    resume) resume_loop ;;
    status) show_status ;;
    visible) show_visible ;;
    go)
      if [[ $# -lt 1 ]]; then
        usage
        exit 2
      fi
      send_target "$@"
      ;;
    processes) show_processes ;;
    help|-h|--help) usage ;;
    *)
      echo "Unknown command: $command" >&2
      usage
      exit 2
      ;;
  esac
}

main "$@"
