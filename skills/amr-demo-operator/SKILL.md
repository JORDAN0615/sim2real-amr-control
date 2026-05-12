---
name: amr-demo-operator
description: Operate the Isaac ROS AMR multi-camera YOLO-World demo loop safely. Use when the user asks what the AMR can currently see, asks the AMR to go to a visible object, or asks to resume/pause the autonomous demo loop.
---

# AMR Demo Operator

Use this skill for live operation of the Isaac ROS AMR demo. Assume YOLO-World, class setup, and `demo_loop_runner.py` are already running unless the user explicitly asks for startup help.

Control ownership rule:

- `demo_loop_runner.py` schedules automatic missions.
- `multi_camera_object_mission.py` is the only mission controller that should publish `/cmd_vel`.
- Agent intervention must pause the loop before manual control.
- Do not launch extra `multi_camera_object_mission.py` processes during normal operation; send command/cancel topics to the persistent mission node.

## Fixed Classes And Aliases

When reporting visible objects, preserve exact ROS `class_name` strings.

Chinese alias map for user requests:

```text
人, 行人 -> person
三角錐, 交通錐 -> traffic cone
灰桶, 灰色桶子, 灰色圓桶 -> grey barrel
藍桶, 藍色桶子, 藍色圓桶 -> blue barrel
```

Only use an alias if the mapped exact class is currently visible. Do not invent a class name. If the visible class spelling differs, use the exact visible spelling.

## Mode A: User Asks What AMR Can See

Trigger examples:

```text
AMR 現在看到什麼？
現在能看到什麼物件？
你看得到什麼？
```

Use one TTY session for the full intervention. SSH to the server, then enter the container as the container user `admin`:

```bash
ssh -tt sky@172.17.5.205 "docker exec -it -u admin isaac_ros_dev-x86_64-container bash"
```

Prepare ROS:

```bash
cd /workspaces/isaac_ros-dev
source install/setup.bash
export ROS_DOMAIN_ID=23
```

Confirm the loop runner and persistent mission node are alive:

```bash
ros2 node list | grep -E "demo_loop_runner|multi_camera_object_mission"
ros2 topic info /multi_camera_object_mission/command
ros2 topic info /multi_camera_object_mission/cancel
ros2 topic info /multi_camera_object_mission/status
```

If `multi_camera_object_mission` is missing, do not continue with manual target commands; tell the user the mission controller is not running.

Pause the loop:

```bash
touch /tmp/demo_loop_pause
```

Stop the currently running mission, if any:

```bash
ros2 topic pub --once /multi_camera_object_mission/cancel std_msgs/msg/String "{data: 'agent intervention'}"
```

Do not kill `demo_loop_runner.py` or `multi_camera_object_mission.py` during normal operation. They should remain alive while the loop is paused.

Inspect mission status:

```bash
ros2 topic echo /multi_camera_object_mission/status --once
```

If a mission was active, wait until status reports:

```text
"active": false
```

Immediately publish zero velocity continuously for 3 seconds:

```bash
ros2 topic pub --rate 20 --times 60 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Confirm the mission controller is idle or terminal:

```bash
ros2 topic echo /multi_camera_object_mission/status --once
```

Inspect live detections. Do not use `demo_loop_runner.log` visible targets as proof for manual target selection.

```bash
ros2 topic echo /yolo_front/detections_3d --once
ros2 topic echo /yolo_left/detections_3d --once
ros2 topic echo /yolo_right/detections_3d --once
```

If one camera `--once` command does not print a message after about 5 seconds, press `Ctrl+C` and continue to the next camera. Do not wrap `ros2 topic echo` with `timeout`.

Valid visible object rule:

```text
score >= 0.1
bbox3d.frame_id == base_link
0.2 <= sqrt(position.x * position.x + position.y * position.y) <= 20.0
```

For reporting to the user:

- Report only unique class names.
- Do not report score, camera, or distance.
- Say that patrol/demo loop is paused and the robot is stopped.
- Keep `/tmp/demo_loop_pause` in place after replying.

Example response:

```text
已暫停巡檢並停車。目前看到：person、traffic cone、grey barrel。
```

If nothing valid is visible:

```text
已暫停巡檢並停車。目前沒有看到可用物件。
```

## Mode B: User Asks AMR To Go To An Object

Trigger examples:

```text
走到 person
去三角錐
移動到 grey barrel
```

Precondition:

- `/tmp/demo_loop_pause` should still exist from Mode A.
- If it does not exist, create it before continuing:

```bash
touch /tmp/demo_loop_pause
```

Before launching a mission, always re-check live detections:

```bash
ros2 topic echo /yolo_front/detections_3d --once
ros2 topic echo /yolo_left/detections_3d --once
ros2 topic echo /yolo_right/detections_3d --once
```

If a camera `--once` hangs for about 5 seconds, press `Ctrl+C` and continue.

Use the same valid visible object rule as Mode A:

```text
score >= 0.1
bbox3d.frame_id == base_link
0.2 <= sqrt(position.x * position.x + position.y * position.y) <= 20.0
```

Resolve the requested object:

- If the user used an exact visible `class_name`, use it exactly.
- If the user used Chinese, map it through the alias table.
- The resolved class must be present in the current live detections.
- If it is not currently visible, do not launch a mission. Keep pause enabled and ask the user to choose from the currently visible classes.

Do not normalize, translate, singularize, pluralize, or rewrite class names before passing them to ROS.

Run the mission by sending a command to the persistent mission node:

```bash
ros2 topic pub --once /multi_camera_object_mission/command std_msgs/msg/String \
  "{data: '{\"target_class_name\": \"<EXACT_CLASS_NAME>\"}'}"
```

Watch mission status for the first 10 seconds:

```bash
ros2 topic echo /multi_camera_object_mission/status
```

Expected healthy progression is one of:

```text
"state": "SEARCH", "active": true
"state": "TURN_LEFT_TO_ACQUIRE_FRONT", "active": true
"state": "TURN_RIGHT_TO_ACQUIRE_FRONT", "active": true
"state": "FRONT_APPROACH", "active": true
```

Stop watching with `Ctrl+C` after confirming the mission is active and progressing.

If status remains in `SEARCH`, or reports no usable tracking for the requested target, stop safely and keep pause enabled:

```bash
ros2 topic pub --once /multi_camera_object_mission/cancel std_msgs/msg/String "{data: 'target not stable for manual mission'}"
ros2 topic pub --rate 20 --times 60 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Tell the user the target was visible in the snapshot but not stable for mission tracking. Ask for another target. Do not resume the loop.

If mission status reports success:

```text
"result_code": "succeeded"
```

confirm the mission controller returned to idle or terminal-ready state:

```bash
ros2 topic echo /multi_camera_object_mission/status --once
```

If the mission remains active unexpectedly, publish cancel and zero velocity.

Then resume automatic loop:

```bash
sudo rm -f /tmp/demo_loop_pause
```

Tell the user:

```text
已到達 <EXACT_CLASS_NAME>，並恢復巡檢。
```

If mission status reports failure:

```text
"result_code": "<failure reason>"
```

publish zero velocity if needed, keep `/tmp/demo_loop_pause`, and tell the user the mission failed. Do not resume the loop unless the user explicitly asks.

## Mode C: User Asks To Resume Loop

Trigger examples:

```text
恢復巡檢
繼續 loop
不去了
```

Stop any manual mission that is still running:

```bash
ros2 topic pub --once /multi_camera_object_mission/cancel std_msgs/msg/String "{data: 'resume loop requested'}"
ros2 topic pub --rate 20 --times 60 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

If no mission is active, the cancel command is harmless.

Resume the loop:

```bash
sudo rm -f /tmp/demo_loop_pause
```

Confirm pause flag is gone:

```bash
ls -l /tmp/demo_loop_pause
```

Expected result:

```text
No such file or directory
```

Tell the user:

```text
已恢復巡檢。
```

## Diagnostics

Loop is not launching missions:

```bash
tail -n 80 /workspaces/isaac_ros-dev/demo_loop_runner.log
ros2 topic echo /multi_camera_object_mission/status --once
ls -l /tmp/demo_loop_pause
```

YOLO topic publisher check:

```bash
ros2 topic info /yolo_front/detections_3d
ros2 topic info /yolo_left/detections_3d
ros2 topic info /yolo_right/detections_3d
```

Class check:

```bash
ros2 topic echo /yolo_front/detections_3d --once
ros2 topic echo /yolo_left/detections_3d --once
ros2 topic echo /yolo_right/detections_3d --once
```
