#!/usr/bin/env bash

# Set the same YOLO-World class list on all multi-camera yolo_ros instances.
# Run after launch_three_yolo_world.sh has started the yolo_front/left/right/back nodes.

set -euo pipefail

if [[ -f "install/setup.bash" ]]; then
  source install/setup.bash
fi

classes=("$@")
if [[ "${#classes[@]}" -eq 0 ]]; then
  classes=("person" "traffic cone" "grey barrel" "blue barrel")
fi

json_classes="["
for class_name in "${classes[@]}"; do
  escaped=${class_name//\\/\\\\}
  escaped=${escaped//\"/\\\"}
  if [[ "${json_classes}" != "[" ]]; then
    json_classes+=", "
  fi
  json_classes+="'${escaped}'"
done
json_classes+="]"

for namespace in yolo_front yolo_left yolo_right yolo_back; do
  echo "Setting ${namespace} classes: ${classes[*]}"
  ros2 service call "/${namespace}/set_classes" yolo_msgs/srv/SetClasses \
    "{classes: ${json_classes}}"
done
