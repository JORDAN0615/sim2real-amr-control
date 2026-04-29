# Sim2Real AMR Control

This repository contains small ROS 2 controllers for a visual-target AMR demo in Isaac Sim. The main idea is to let an agent issue a high-level command such as "go to the forklift" or "go to tag 0", while a deterministic ROS script handles the short control loop and publishes `/cmd_vel`.

The project currently supports two target sources:

```text
AprilTag
  Isaac Sim camera -> isaac_ros_apriltag -> /tag_detections or /tf
  -> ros2_move_to_tag.py -> /cmd_vel

YOLO-World 3D object detection
  Isaac Sim RGB/depth camera -> yolo_ros -> /yolo/detections_3d
  -> ros2_move_to_object.py -> /cmd_vel
```

## Demo

The demo shows Isaac Sim publishing perception topics, the ROS 2 controller selecting a visual target, and the AMR moving through `/cmd_vel`.

<video src="assets/demo.mp4" controls width="100%"></video>

If the embedded player does not load, open [assets/demo.mp4](assets/demo.mp4).

## What This Shows

- Visual servo style AMR control in Isaac Sim through normal ROS 2 topics.
- AprilTag-based navigation using NVIDIA Isaac ROS AprilTag.
- Semantic object navigation using YOLO-World, depth, and `yolo_ros`.
- A simple agent-friendly interface: choose a tag ID or object class, then run one command.
- A reusable sim-to-real direction: the controller only depends on ROS topics, so the perception source can later move from Isaac Sim cameras to real sensors.

## Repository Contents

```text
ros2_move_to_tag.py
  Move the AMR toward a selected AprilTag ID.

ros2_move_to_object.py
  Move the AMR toward a selected YOLO-World class name from /yolo/detections_3d.

yolo_move_to_object.md
  Development notes for the YOLO 3D object-control pipeline.

project.md
  Current progress, design notes, and next-step TODOs.

skills/
  Agent-facing operation guides for AprilTag and YOLO-World AMR control.
```

## Requirements

- ROS 2 Humble.
- Isaac Sim publishing camera, TF, odometry, and `/cmd_vel` topics.
- NVIDIA Isaac ROS AprilTag for AprilTag navigation.
- `yolo_ros` with YOLO-World support for semantic object navigation.
- A mobile base or simulated AMR that accepts `geometry_msgs/msg/Twist` on `/cmd_vel`.

The scripts were developed against an Isaac ROS dev container workflow, but the code itself is plain ROS 2 Python.

## AprilTag Control

Start the Isaac ROS AprilTag pipeline:

```bash
ros2 launch isaac_ros_apriltag isaac_ros_apriltag_isaac_sim_pipeline.launch.py
```

Run the controller with `/tag_detections`:

```bash
python3 ros2_move_to_tag.py --ros-args \
  -p pose_source:=detections \
  -p detections_topic:=/tag_detections \
  -p tag_id:=0 \
  -p cmd_vel_topic:=/cmd_vel \
  -p speed:=0.50 \
  -p stop_distance:=3.0 \
  -p tag_timeout_sec:=2.0 \
  -p debug:=true
```

Demo tag mapping:

```text
tag_id 0 = forklift
tag_id 5 = brown cardboard box
tag_id 2 = orange barrel
```

## YOLO-World Object Control

The YOLO path expects RGB, depth, and camera info from Isaac Sim:

```text
/front_stereo_camera/left/image_rect_color
/front_stereo_camera/left/depth
/front_stereo_camera/left/camera_info
```

Launch YOLO-World with 3D detection enabled:

```bash
ros2 launch yolo_bringup yolo-world.launch.py \
  input_image_topic:=/front_stereo_camera/left/image_rect_color \
  input_depth_topic:=/front_stereo_camera/left/depth \
  input_depth_info_topic:=/front_stereo_camera/left/camera_info \
  target_frame:=base_link \
  depth_image_units_divisor:=1 \
  use_3d:=True \
  use_tracking:=False \
  use_debug:=True
```

Set the YOLO-World prompt classes:

```bash
ros2 service call /yolo/set_classes yolo_msgs/srv/SetClasses \
  "{classes: ['yellow forklift', 'cardboard box', 'orange barrel', 'blue barrel']}"
```

Move toward a selected object class:

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

## Control Logic

Both controllers convert perception output into the same two variables:

```text
forward = target distance in front of the AMR
left    = lateral offset from the AMR centerline
```

The command calculation is intentionally simple:

```text
forward_error = forward - stop_distance
heading_error = atan2(left, forward)
```

The scripts publish:

```text
cmd.linear.x  -> forward velocity
cmd.angular.z -> yaw velocity
```

The AMR stops when it reaches `stop_distance`, loses the target longer than the configured timeout, receives Ctrl+C, or hits `mission_timeout_sec`.

## Safety Notes

These scripts are prototypes for controlled demos. They do not replace a full navigation stack, obstacle avoidance, or safety controller. For real-world AMR deployment, keep a separate safety layer and use conservative speed, timeout, and stop-distance values.

## Roadmap

- Convert the one-shot scripts into a persistent ROS service/action server.
- Add explicit commands such as `/move_to_tag`, `/move_to_object`, `/get_tag_pose`, and `/stop_motion`.
- Improve target selection when multiple detections of the same class are present.
- Validate the same interface with a real camera and physical AMR.
