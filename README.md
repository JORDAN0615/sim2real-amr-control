# Sim2Real AMR Control

ROS 2 prototype for semantic AMR object approach in Isaac Sim. The demo runs
YOLO-World on three stereo cameras, selects visible object classes, and drives
the AMR by publishing `geometry_msgs/msg/Twist` to `/cmd_vel`.

## Demo

![AMR demo](./assets/amr.gif)

YOLO-World debug image:

![YOLO-World detection result](./assets/yolo.png)

## Runtime Flow

```text
Isaac Sim AMR scene
  -> stereo RGB/depth/camera_info topics
  -> yolo_ros YOLO-World front/left/right detections
  -> /yolo_{front,left,right}/detections_3d
  -> demo_loop_runner selects a visible target class
  -> multi_camera_object_mission executes one mission
  -> /cmd_vel drives the AMR
```

Side cameras are used only to decide which way to rotate until the target is in
the front camera. The final approach uses front-camera 3D detections in
`base_link`.

## Requirements

- ROS 2 with Python `rclpy`.
- Isaac Sim publishing camera, depth, TF, odometry, and `/cmd_vel` topics.
- [`yolo_ros`](https://github.com/mgonzs13/yolo_ros) with YOLO-World support.
- A simulated or real AMR that accepts `geometry_msgs/msg/Twist` on `/cmd_vel`.

The scripts were developed in an Isaac ROS dev container. They assume the
workspace path `/workspaces/isaac_ros-dev` by default.

## Main Commands

Start the full demo stack:

```bash
./start_demo.sh
```

Stop the stack and publish zero velocity:

```bash
./stop_demo.sh
```

Manual operator helper:

```bash
./amr_control.sh pause
./amr_control.sh visible
./amr_control.sh go person
./amr_control.sh status
./amr_control.sh resume
./amr_control.sh stop
```

`amr_control.sh` maps safe target aliases such as `traffic_cone`,
`fire_extinguisher`, `grey_barrel`, `cardboard_box`, `box`, `cart`, and
`ladder` to the class names sent to the mission node.

## YOLO-World Startup

`start_demo.sh` starts `launch_three_yolo_world.sh`, waits for the three
`set_classes` services, configures the object classes, starts
`multi_camera_object_mission.py`, and then starts `demo_loop_runner.py`.

Useful environment overrides:

```bash
YOLO_WAIT_SEC=120 ./start_demo.sh
ROS_DOMAIN_ID=23 ./start_demo.sh
WORKSPACE_DIR=/workspaces/isaac_ros-dev ./start_demo.sh
```

If the container needs the YOLO runtime environment prepared first:

```bash
source ./setup_yolo_env.sh
```

## Topics

Expected camera topics:

```text
/front_stereo_camera/left/image_rect_color
/front_stereo_camera/depth
/front_stereo_camera/left/camera_info
/left_stereo_camera/left/image_rect_color
/left_stereo_camera/depth
/left_stereo_camera/left/camera_info
/right_stereo_camera/left/image_rect_color
/right_stereo_camera/depth
/right_stereo_camera/left/camera_info
```

YOLO 3D detection topics:

```text
/yolo_front/detections_3d
/yolo_left/detections_3d
/yolo_right/detections_3d
```

Mission command and status topics:

```text
/multi_camera_object_mission/command
/multi_camera_object_mission/cancel
/multi_camera_object_mission/status
```

Motion command topic:

```text
/cmd_vel
```

## Configuration

Mission and loop parameters live in:

```text
config/mission_demo.yaml
```

Important parameters:

- `speed`: forward command used during front approach.
- `turn_speed`: angular command used while acquiring the target with the front camera.
- `stop_distance`: target stopping distance in meters.
- `priority_targets`: ordered class list used by `demo_loop_runner`.

For launch-file operation:

```bash
ros2 launch apriltag_amr mission_demo.launch.py
```

For direct script operation:

```bash
python3 multi_camera_object_mission.py
python3 demo_loop_runner.py
```

## Testing

```bash
python3 -m unittest
```

## Safety Notes

This is a controlled demo prototype. It does not replace a navigation stack,
obstacle avoidance, or a safety controller. Keep a separate stop mechanism and
use conservative speeds when running on physical hardware.
