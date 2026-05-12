## 1. Baseline Review and Safety

- [x] 1.1 Record current behavior for `multi_camera_object_mission.py`: target selection, state transitions, success/failure cases, and `/cmd_vel` stop behavior.
- [x] 1.2 Record current behavior for `demo_loop_runner.py`: priority selection, stale target pruning, cooldown, pause flag, and mission logging.
- [x] 1.3 Add or isolate unit-testable pure functions for target parsing, candidate selection, stale pruning, and command calculation before lifecycle refactoring.

## 2. Mission Controller Lifecycle

- [x] 2.1 Refactor `MultiCameraObjectMissionNode` so it can reset all per-mission state without restarting the process.
- [x] 2.2 Replace `finish()` hard-exit behavior with terminal state recording, zero Twist publishing, and normal idle/ready transition.
- [x] 2.3 Ensure stale detections from before a mission request cannot satisfy a new mission goal.
- [x] 2.4 Add explicit result codes/messages for success, mission timeout, front acquire timeout, target lost, invalid frame, rejected goal, and canceled mission.

## 3. ROS 2 Mission Interface

- [x] 3.1 Define the ROS 2 mission command interface, preferably an action with goal, feedback, and result fields.
- [ ] 3.2 Implement an action server in the mission controller node with one active mission at a time.
- [x] 3.3 Publish feedback when mission state changes or when useful target/control telemetry changes.
- [x] 3.4 Implement cancel/preempt handling that stops the robot and returns a structured canceled result.

## 4. Runner Refactor

- [x] 4.1 Replace `launch_mission()` subprocess execution with an action client request to the mission controller.
- [x] 4.2 Remove stdout parsing for `success:` and `failed:` mission terminal detection.
- [x] 4.3 Remove `external_mission_running()` and replace it with ROS 2 active-goal tracking.
- [x] 4.4 Preserve cooldown, pause flag, priority target selection, and status logging using structured mission results.
- [x] 4.5 Ensure runner shutdown cancels or waits for any active mission according to the chosen safety policy.

## 5. ROS 2 Package and Launch Integration

- [x] 5.1 Add or update ROS 2 Python package metadata for console entry points.
- [x] 5.2 Add launch configuration for running the mission controller and demo runner together.
- [x] 5.3 Move configuration that belongs in launch files or parameter YAML out of ad hoc CLI-only arguments.
- [x] 5.4 Keep a temporary one-shot manual command path if needed for demo compatibility.

## 6. Verification and Documentation

- [ ] 6.1 Add tests for mission lifecycle reset, result mapping, timeout handling, and cancel behavior.
- [ ] 6.2 Add tests for runner action-client sequencing, cooldown behavior, and no-target waiting behavior.
- [ ] 6.3 Run a simulation smoke test: visible front target succeeds, side-only target rotates then approaches, missing target times out, and pause flag prevents new goals.
- [x] 6.4 Update README/demo docs with the new `ros2 run` or `ros2 launch` workflow and remove subprocess-specific instructions.
