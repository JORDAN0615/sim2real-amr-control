---
name: yolo-world-amr-control
description: Use this skill when an agent needs to run the YOLO-World 3D object detection pipeline and control the Isaac Sim AMR through ros2_move_to_object.py, including launching yolo_bringup, setting YOLO-World classes, reading /yolo/detections_3d, and passing target object parameters.
---

# YOLO-World AMR Control

Use this skill for semantic object navigation with YOLO-World and depth.

Pipeline:

```text
Isaac Sim RGB/depth camera
  -> yolo_ros YOLO-World
  -> /yolo/detections_3d
  -> ros2_move_to_object.py
  -> /cmd_vel
  -> AMR moves toward selected object class
```

## Required Session

Run commands from a persistent interactive TTY session inside the Isaac ROS container.

Prepare the shell:

```bash
cd /workspaces/isaac_ros-dev && source install/setup.bash && export ROS_DOMAIN_ID=23
```

Required Isaac Sim topics:

```text
/front_stereo_camera/left/image_rect_color
/front_stereo_camera/left/depth
/front_stereo_camera/left/camera_info
/cmd_vel
```

Check once if needed:

```bash
ros2 topic list | grep -E "front_stereo_camera|cmd_vel"
```

## Step 1: Launch YOLO-World

Preferred launch command:

```bash
ros2 launch yolo_bringup yolo-world.launch.py input_image_topic:=/front_stereo_camera/left/image_rect_color input_depth_topic:=/front_stereo_camera/left/depth input_depth_info_topic:=/front_stereo_camera/left/camera_info target_frame:=base_link depth_image_units_divisor:=1 use_3d:=True use_tracking:=False use_debug:=True
```

If `yolo-world.launch.py` is unavailable, use the generic launch:

```bash
ros2 launch yolo_bringup yolo.launch.py model_type:=World model:=yolov8s-world.pt input_image_topic:=/front_stereo_camera/left/image_rect_color input_depth_topic:=/front_stereo_camera/left/depth input_depth_info_topic:=/front_stereo_camera/left/camera_info target_frame:=base_link depth_image_units_divisor:=1 use_3d:=True use_tracking:=False use_debug:=True
```

Keep this launch running in its own terminal/session.

## Step 2: Set YOLO-World Classes

YOLO-World needs the target class prompts before detection is useful.

Default demo classes:

```bash
ros2 service call /yolo/set_classes yolo_msgs/srv/SetClasses "{classes: ['yellow forklift', 'cardboard box', 'orange barrel', 'blue barrel']}"
```

Only use class names that are also passed later to `ros2_move_to_object.py`.

Verify detections:

```bash
ros2 topic echo /yolo/detections_3d --once
```

Expected fields:

```text
detections[]
  class_name
  score
  bbox3d.center.position.x
  bbox3d.center.position.y
  bbox3d.center.position.z
  bbox3d.frame_id
```

`bbox3d.frame_id` should be:

```text
base_link
```

## Step 3: Move To Object

Run `ros2_move_to_object.py` after YOLO-World is publishing `/yolo/detections_3d`.

Example: move to yellow forklift and stop 3 meters away:

```bash
python3 ros2_move_to_object.py --ros-args -p target_class_name:="yellow forklift" -p detections_topic:=/yolo/detections_3d -p target_frame:=base_link -p cmd_vel_topic:=/cmd_vel -p speed:=0.50 -p stop_distance:=3.0 -p detection_timeout_sec:=2.0 -p debug:=true
```

Example: move to cardboard box:

```bash
python3 ros2_move_to_object.py --ros-args -p target_class_name:="cardboard box" -p detections_topic:=/yolo/detections_3d -p target_frame:=base_link -p cmd_vel_topic:=/cmd_vel -p speed:=0.50 -p stop_distance:=3.0 -p detection_timeout_sec:=2.0 -p debug:=true
```

Example: move to orange barrel:

```bash
python3 ros2_move_to_object.py --ros-args -p target_class_name:="orange barrel" -p detections_topic:=/yolo/detections_3d -p target_frame:=base_link -p cmd_vel_topic:=/cmd_vel -p speed:=0.50 -p stop_distance:=3.0 -p detection_timeout_sec:=2.0 -p debug:=true
```

Example: move to blue barrel:

```bash
python3 ros2_move_to_object.py --ros-args -p target_class_name:="blue barrel" -p detections_topic:=/yolo/detections_3d -p target_frame:=base_link -p cmd_vel_topic:=/cmd_vel -p speed:=0.50 -p stop_distance:=3.0 -p detection_timeout_sec:=2.0 -p debug:=true
```

## ros2_move_to_object.py Parameters

`target_class_name`
: Required target object class. Must exactly match `class_name` from `/yolo/detections_3d`, for example `"yellow forklift"`.

`detections_topic`
: YOLO 3D detections topic. Default for this pipeline: `/yolo/detections_3d`.

`target_frame`
: Expected frame for `bbox3d`. Use `base_link` so `x` means robot-forward and `y` means robot-left.

`cmd_vel_topic`
: AMR command velocity topic. Default: `/cmd_vel`.

`speed`
: Forward speed in meters per second. Demo default: `0.50`.

`stop_distance`
: Desired stopping distance from the object in meters. Demo default: `3.0`.

`distance_tolerance`
: Acceptable distance error. Default: `0.04`.

`angle_tolerance_rad`
: Small heading errors below this value are ignored to avoid oscillation.

`rotate_in_place_angle_rad`
: If object heading error is larger than this, rotate in place before driving.

`angular_gain`
: Multiplier from heading error to `cmd.angular.z`.

`max_angular_z`
: Maximum yaw speed command.

`detection_timeout_sec`
: Stop if the target class is not detected for this long. Demo default: `2.0`.

`mission_timeout_sec`
: Optional maximum mission duration. Defaults to a long duration if not set.

`debug`
: If `true`, prints live `forward`, `left`, `score`, `cmd.linear.x`, `cmd.angular.z`, and state.

## Data Conversion

`ros2_move_to_object.py` expects `yolo_ros` to publish 3D detections in `base_link`.

Mapping:

```text
bbox3d.center.position.x -> forward
bbox3d.center.position.y -> left
```

Controller:

```text
forward_error = forward - stop_distance
heading_error = atan2(left, forward)
cmd.linear.x = speed
cmd.angular.z = angular_gain * heading_error
```

Stop conditions:

```text
forward <= stop_distance + distance_tolerance
target class lost for more than detection_timeout_sec
mission timeout
manual Ctrl+C
```

## Debug and Visualization

View class boxes in image:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select:

```text
/yolo/dbg_image
```

RViz marker topics:

```text
/yolo/dgb_bb_markers
/yolo/dgb_kp_markers
```

Read one 3D detection message:

```bash
ros2 topic echo /yolo/detections_3d --once
```

## Safety Stop

If anything looks wrong:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Agent Policy

- Start YOLO-World before running `ros2_move_to_object.py`.
- Always call `/yolo/set_classes` after starting YOLO-World.
- Use exact class names from the configured classes.
- If the user asks to move to an object class, run `ros2_move_to_object.py` directly after YOLO is active.
- Use `--once` for diagnostic topic echoes unless the user asks to monitor continuously.
- Report only the final success/failure line and final `forward/left/score` when motion completes.
