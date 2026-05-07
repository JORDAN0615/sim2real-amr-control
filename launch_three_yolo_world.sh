#!/usr/bin/env bash

# Launch three yolo_ros YOLO-World instances for front/left/right RGB-D cameras.
# Run from inside the Isaac ROS dev workspace/container.

if [[ -f "install/setup.bash" ]]; then
  source install/setup.bash
fi

set -euo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"

pids=()

cleanup() {
  echo
  echo "Stopping YOLO-World launch processes..."
  for pid in "${pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
}

trap cleanup INT TERM EXIT

launch_yolo() {
  local namespace="$1"
  local image_topic="$2"
  local depth_topic="$3"
  local camera_info_topic="$4"

  echo "Starting ${namespace}:"
  echo "  image=${image_topic}"
  echo "  depth=${depth_topic}"
  echo "  camera_info=${camera_info_topic}"

  ros2 launch yolo_bringup yolo-world.launch.py \
    namespace:="${namespace}" \
    input_image_topic:="${image_topic}" \
    input_depth_topic:="${depth_topic}" \
    input_depth_info_topic:="${camera_info_topic}" \
    target_frame:=base_link \
    depth_image_units_divisor:=1 \
    use_3d:=True &

  pids+=("$!")
}

launch_yolo \
  yolo_front \
  /front_stereo_camera/left/image_rect_color \
  /front_stereo_camera/depth \
  /front_stereo_camera/left/camera_info

launch_yolo \
  yolo_left \
  /left_stereo_camera/left/image_rect_color \
  /left_stereo_camera/depth \
  /left_stereo_camera/left/camera_info

launch_yolo \
  yolo_right \
  /right_stereo_camera/left/image_rect_color \
  /right_stereo_camera/depth \
  /right_stereo_camera/left/camera_info

echo
echo "All YOLO-World launch processes started."
echo "Use another terminal to set classes and run multi_camera_object_mission.py."
echo "Press Ctrl+C here to stop all three yolo_ros instances."

wait
