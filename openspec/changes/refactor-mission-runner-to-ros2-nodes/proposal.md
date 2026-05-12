## Why

`multi_camera_object_mission.py` and `demo_loop_runner.py` already use `rclpy.Node`, but the demo loop still orchestrates missions by spawning Python subprocesses, scanning processes with `pgrep`, parsing stdout for terminal text, and forcing process exit with `os._exit`. This makes the demo harder to compose with ROS 2 launch files, harder to test, and brittle when moving toward a proper ROS 2 package.

## What Changes

- Refactor the multi-camera mission controller into a persistent ROS 2 node that accepts mission requests through a ROS-native command interface instead of relying on process startup arguments.
- Refactor the demo loop runner into a ROS 2 node that starts, tracks, and cooldowns missions through that command interface instead of `subprocess.Popen`, stdout parsing, and `pgrep`.
- Preserve the current four-camera target-selection behavior: front detections drive approach, side/back detections choose turn direction until the target is visible in front, and priority targets are selected by configured order.
- Add explicit mission terminal reporting for success, failure, timeout, target lost, and preemption/stop conditions.
- Move launch/runtime concerns toward ROS 2 conventions: parameters, node entry points, package layout, and launch-compatible execution.
- Remove hard process termination from mission completion paths and replace it with normal ROS node state transitions and responses.

## Capabilities

### New Capabilities
- `ros2-mission-orchestration`: Defines how the demo loop requests, monitors, and sequences multi-camera object missions using ROS 2 node interfaces.

### Modified Capabilities

## Impact

- Affected code: `multi_camera_object_mission.py`, `demo_loop_runner.py`, and related README/run documentation.
- Likely new/changed ROS interfaces: mission request/response or action goal/result/status messages, plus parameters for target classes, camera topics, timeouts, cooldowns, and safety limits.
- Packaging impact: add or update ROS 2 Python package metadata, console entry points, launch files, and tests so the nodes can be launched with `ros2 run` or `ros2 launch`.
- Runtime behavior impact: mission completion should no longer depend on child process exit codes, stdout strings, or operating-system process scans.
