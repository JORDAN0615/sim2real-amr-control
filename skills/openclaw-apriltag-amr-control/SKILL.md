---
name: openclaw-amr-control
description: Use this skill when an agent needs to control the Isaac Sim AMR/OpenClaw demo through ROS 2 AprilTag pose data, including SSH login, entering the Isaac ROS dev container, setting ROS_DOMAIN_ID=23, running ros2_move_to_tag.py, selecting semantic tag targets, and tuning motion parameters.
---

# OpenClaw AMR Control

Use this skill to run the fixed AprilTag-to-AMR control pipeline for the Isaac Sim demo.

The agent should treat this as a deterministic ROS control workflow:

```text
Isaac Sim camera
  -> isaac_ros_apriltag_isaac_sim_pipeline
  -> /tag_detections or /tf
  -> ros2_move_to_tag.py
  -> /cmd_vel
  -> AMR moves toward selected tag
```

## Semantic Tag Map

Use these tag IDs unless the user explicitly changes the scene:

```text
tag_id 0 = forklift
tag_id 5 = brown cardboard box
tag_id 2 = orange barrel
```

## Required Runtime Assumptions

- Server host: `sky@172.17.5.205`
- Host workspace: `/home/sky/workspaces/isaac_ros-dev`
- Container workspace: `/workspaces/isaac_ros-dev`
- Container name: `isaac_ros_dev-x86_64-container`
- Control script: `/workspaces/isaac_ros-dev/ros2_move_to_tag.py`
- ROS domain: `ROS_DOMAIN_ID=23`
- Command velocity topic: `/cmd_vel`
- AprilTag detections topic: `/tag_detections`

## Fixed Connection Pipeline

Motion commands must run from a persistent interactive TTY session.

Do not use one-shot commands like this for motion:

```bash
ssh sky@172.17.5.205 "docker exec isaac_ros_dev-x86_64-container bash -lc 'python3 ros2_move_to_tag.py ...'"
```

Reason: one-shot SSH/docker exec creates a fresh non-interactive shell, does not preserve setup state, is hard to interrupt, and can leave stale motion processes. Use it only for short read-only checks when explicitly needed.

Required session pattern:

```text
persistent SSH TTY
  -> enter Isaac ROS container
  -> source install/setup.bash
  -> export ROS_DOMAIN_ID=23
  -> run ros2_move_to_tag.py in that same session
```

From the local terminal, open a persistent SSH TTY:

```bash
ssh sky@172.17.5.205
```

On the server host, enter the Isaac ROS dev container:

```bash
export ISAAC_ROS_WS=$HOME/workspaces/isaac_ros-dev && cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh
```

Inside the container, prepare ROS:

```bash
cd /workspaces/isaac_ros-dev && source install/setup.bash && export ROS_DOMAIN_ID=23
```

Verify the AprilTag pipeline and robot control topics exist:

```bash
ros2 topic list | grep -E "tag_detections|cmd_vel|tf"
```

Only run this verification when the user asks for checks or when the environment is unknown. If the user gives a motion target and no extra diagnostic request, go straight to the motion command.

## AprilTag Pipeline Requirement

`ros2_move_to_tag.py` does not detect AprilTags by itself. Before running motion control, make sure these are already running:

```text
Isaac Sim is playing
isaac_ros_apriltag_isaac_sim_pipeline.launch.py is running
/tag_detections has detections
/cmd_vel controls the AMR
```

If AprilTag detection is not running, start it in a separate container terminal:

```bash
cd /workspaces/isaac_ros-dev && source install/setup.bash && export ROS_DOMAIN_ID=23 && ros2 launch isaac_ros_apriltag isaac_ros_apriltag_isaac_sim_pipeline.launch.py
```

## Motion Command

Default behavior: if the user asks the agent to move to a known target and gives no special diagnostic instruction, directly run `ros2_move_to_tag.py`. Do not first echo `/tag_detections`.

Default command pattern:

```bash
python3 ros2_move_to_tag.py --ros-args -p pose_source:=detections -p detections_topic:=/tag_detections -p tag_id:=0 -p cmd_vel_topic:=/cmd_vel -p speed:=0.50 -p stop_distance:=3.0 -p mission_timeout_sec:=999.0 -p tag_timeout_sec:=2.0 -p angle_tolerance_rad:=0.02 -p debug:=true
```

Use `pose_source:=detections` for the simple camera-frame demo. Use `pose_source:=tf` only when `base_link -> tag36h11:<id>` is confirmed with `tf2_echo`.

When running in an already prepared persistent container session, do not repeat `cd`, `source`, or `export` before every command unless needed. Confirm the prompt is:

```text
admin@sky-SKY-602E3:/workspaces/isaac_ros-dev$
```

If operating through a tool API, keep and reuse the same session ID. Send subsequent commands by writing to the existing session stdin rather than opening a new SSH command.

Tool API implementation rule:

```text
1. Use exec_command with tty=true to run: ssh sky@172.17.5.205
2. Keep the returned session_id.
3. Use write_stdin(session_id, "...") for all later commands.
4. Enter the container, source setup, export ROS_DOMAIN_ID, and run motion commands in that same session.
5. Do not start motion with exec_command("ssh ... docker exec ... python3 ...").
```

Example tool sequence:

```text
exec_command(cmd="ssh sky@172.17.5.205", tty=true)
write_stdin(session_id, "export ISAAC_ROS_WS=$HOME/workspaces/isaac_ros-dev\n")
write_stdin(session_id, "cd ${ISAAC_ROS_WS}/src/isaac_ros_common\n")
write_stdin(session_id, "./scripts/run_dev.sh\n")
write_stdin(session_id, "source install/setup.bash\n")
write_stdin(session_id, "export ROS_DOMAIN_ID=23\n")
write_stdin(session_id, "python3 ros2_move_to_tag.py --ros-args ...\n")
```

## Parameter Meanings

`pose_source`
: `detections` reads `/tag_detections` directly. `tf` reads `base_link -> tag36h11:<id>` from TF.

`detections_topic`
: Topic published by Isaac ROS AprilTag. Default: `/tag_detections`.

`tag_id`
: Which target object to approach. `0` forklift, `5` brown cardboard box, `2` orange barrel.

`cmd_vel_topic`
: AMR velocity command topic. Default: `/cmd_vel`.

`speed`
: Forward speed in meters per second. Start with `0.10` to `0.20`. Use high values like `0.80` only after verifying behavior.

`stop_distance`
: Desired stopping distance from the tag in meters. Example: `3.0` stops about 3 meters from the selected tag.

`mission_timeout_sec`
: Max allowed mission duration. Use `999.0` for long demos, but keep emergency stop available.

`tag_timeout_sec`
: How long the controller tolerates missing tag detections before stopping. Use `2.0` if `/tag_detections` updates slowly.

`angle_tolerance_rad`
: If heading error is below this, angular correction is suppressed. Smaller values make the robot correct heading more often.

`debug`
: `true` prints live `forward`, `left`, `cmd.linear.x`, `cmd.angular.z`, and state.

## Target-Specific Commands

Approach forklift, tag 0:

```bash
python3 ros2_move_to_tag.py --ros-args -p pose_source:=detections -p detections_topic:=/tag_detections -p tag_id:=0 -p cmd_vel_topic:=/cmd_vel -p speed:=0.50 -p stop_distance:=3.0 -p mission_timeout_sec:=999.0 -p tag_timeout_sec:=2.0 -p angle_tolerance_rad:=0.02 -p debug:=true
```

Approach brown cardboard box, tag 5:

```bash
python3 ros2_move_to_tag.py --ros-args -p pose_source:=detections -p detections_topic:=/tag_detections -p tag_id:=5 -p cmd_vel_topic:=/cmd_vel -p speed:=0.50 -p stop_distance:=3.0 -p mission_timeout_sec:=999.0 -p tag_timeout_sec:=2.0 -p angle_tolerance_rad:=0.02 -p debug:=true
```

Approach orange barrel, tag 2:

```bash
python3 ros2_move_to_tag.py --ros-args -p pose_source:=detections -p detections_topic:=/tag_detections -p tag_id:=2 -p cmd_vel_topic:=/cmd_vel -p speed:=0.50 -p stop_distance:=3.0 -p mission_timeout_sec:=999.0 -p tag_timeout_sec:=2.0 -p angle_tolerance_rad:=0.02 -p debug:=true
```

## Safety Stop

If behavior is wrong, stop immediately:

```bash
cd /workspaces/isaac_ros-dev && source install/setup.bash && export ROS_DOMAIN_ID=23 && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Validation Commands

Use validation commands only when asked or when motion fails. Keep them one-shot whenever possible.

Check current detections:

```bash
ros2 topic echo /tag_detections --once
```

Check detection rate:

```bash
cd /workspaces/isaac_ros-dev && source install/setup.bash && export ROS_DOMAIN_ID=23 && ros2 topic hz /tag_detections
```

Check AMR command stream:

```bash
ros2 topic echo /cmd_vel --once
```

Check TF mode availability for a tag:

```bash
cd /workspaces/isaac_ros-dev && source install/setup.bash && export ROS_DOMAIN_ID=23 && ros2 run tf2_ros tf2_echo base_link tag36h11:0
```

## Agent Policy

- Motion commands must use an existing persistent TTY/container session whenever possible.
- Do not run long motion commands through one-shot `ssh docker exec ...` unless explicitly instructed.
- If the user gives a known target and no diagnostic instruction, directly run `ros2_move_to_tag.py`; do not first search for tags.
- Use topic checks only on request, on unknown environment, or after a failure.
- Topic echo checks must use `--once` unless the user explicitly asks to monitor continuously.
- After a motion command finishes, report only the target, final success/failure line, and the final distance/offset if available.
- Avoid returning long debug streams. Summarize the key lines.
- Prefer `speed:=0.50` for the current Isaac Sim demo unless the user asks for slower or faster.
- Use `tag_timeout_sec:=2.0` when detections are intermittent.
- Keep safety stop ready when using speeds above `0.20`.
- If the user gives a semantic target, map it to the tag ID before running the command.
- Do not modify NVIDIA Isaac ROS packages for this demo path; use `ros2_move_to_tag.py` as the motion executor.
