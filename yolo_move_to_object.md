# YOLO 3D Move-To-Object

## Purpose

This feature lets the AMR move toward a semantic object detected by `yolo_ros`.

It is separate from the AprilTag flow:

```text
AprilTag flow:
isaac_ros_apriltag -> /tag_detections -> ros2_move_to_tag.py -> /cmd_vel

YOLO object flow:
yolo_ros -> /yolo/detections_3d -> ros2_move_to_object.py -> /cmd_vel
```

## Required Inputs

Isaac Sim must publish RGB and depth:

```text
/front_stereo_camera/left/image_rect_color
/front_stereo_camera/left/depth
/front_stereo_camera/left/camera_info
```

`yolo_ros` must publish 3D detections:

```text
/yolo/detections_3d
```

Expected detection fields:

```text
detections[]
  class_name
  score
  bbox3d.center.position.x
  bbox3d.center.position.y
  bbox3d.center.position.z
  bbox3d.frame_id
```

For AMR control, `bbox3d.frame_id` should be:

```text
base_link
```

## Start YOLO-World

Run this in the Isaac ROS container after `source install/setup.bash` and `export ROS_DOMAIN_ID=23`:

```bash
ros2 launch yolo_bringup yolo.launch.py model_type:=World model:=yolov8s-world.pt input_image_topic:=/front_stereo_camera/left/image_rect_color input_depth_topic:=/front_stereo_camera/left/depth input_depth_info_topic:=/front_stereo_camera/left/camera_info target_frame:=base_link depth_image_units_divisor:=1 use_3d:=True use_tracking:=False use_debug:=True
```

Set semantic classes:

```bash
ros2 service call /yolo/set_classes yolo_msgs/srv/SetClasses "{classes: ['yellow forklift', 'cardboard box', 'orange barrel', 'blue barrel']}"
```

Verify output:

```bash
ros2 topic echo /yolo/detections_3d --once
```

## Move To Object

Script:

```text
ros2_move_to_object.py
```

Example: move toward yellow forklift and stop about 3 meters away:

```bash
python3 ros2_move_to_object.py --ros-args \
  -p target_class_name:="yellow forklift" \
  -p detections_topic:=/yolo/detections_3d \
  -p target_frame:=base_link \
  -p cmd_vel_topic:=/cmd_vel \
  -p speed:=0.50 \
  -p stop_distance:=3.0 \
  -p detection_timeout_sec:=2.0 \
  -p debug:=true
```

Single-line version:

```bash
python3 ros2_move_to_object.py --ros-args -p target_class_name:="yellow forklift" -p detections_topic:=/yolo/detections_3d -p target_frame:=base_link -p cmd_vel_topic:=/cmd_vel -p speed:=0.50 -p stop_distance:=3.0 -p detection_timeout_sec:=2.0 -p debug:=true
```

## Parameters

`target_class_name`
: Object class to approach. Must match `class_name` in `/yolo/detections_3d`.

`detections_topic`
: YOLO 3D detection topic. Default: `/yolo/detections_3d`.

`target_frame`
: Expected 3D detection frame. Use `base_link` for AMR control.

`cmd_vel_topic`
: AMR velocity command topic. Default: `/cmd_vel`.

`speed`
: Forward speed in meters per second.

`stop_distance`
: Desired stopping distance from the object in meters.

`distance_tolerance`
: Allowed stop-distance error. Default: `0.04`.

`angle_tolerance_rad`
: Ignore small heading errors below this angle.

`rotate_in_place_angle_rad`
: If object heading error exceeds this value, rotate in place before driving.

`angular_gain`
: Proportional gain from heading error to `cmd.angular.z`.

`max_angular_z`
: Max yaw speed command.

`detection_timeout_sec`
: Stop if the selected object has not been detected for this long.

`mission_timeout_sec`
: Max mission duration.

`debug`
: Print live control state.

## Control Logic

`yolo_ros` already transforms 3D boxes into `target_frame`.

With `target_frame=base_link`:

```text
bbox3d.center.position.x -> forward distance
bbox3d.center.position.y -> left/right offset
```

The controller computes:

```text
forward_error = forward - stop_distance
heading_error = atan2(left, forward)
```

Then publishes:

```text
/cmd_vel.linear.x = speed
/cmd_vel.angular.z = angular_gain * heading_error
```

Protection rules:

```text
if forward <= stop_distance + distance_tolerance:
  stop and return success

if abs(heading_error) > rotate_in_place_angle_rad:
  rotate in place, do not drive forward

if object is lost for more than detection_timeout_sec:
  stop and return failed
```

## Debug Topics

View YOLO debug image:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select:

```text
/yolo/dbg_image
```

View RViz 3D boxes:

```text
/yolo/dgb_bb_markers
```

View raw 3D detections:

```bash
ros2 topic echo /yolo/detections_3d --once
```

## Safety Stop

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Notes

- This first version assumes one object per requested class in the demo scene.
- It does not choose between multiple objects of the same class.
- It does not modify the existing AprilTag controller.
- YOLO object distance is based on depth inside the detection box, so it may be less stable than AprilTag pose.
