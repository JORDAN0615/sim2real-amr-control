## Context

`multi_camera_object_mission.py` is already structured around `MultiCameraObjectMissionNode`, with camera subscriptions, control timers, parameters, state transitions, and `/cmd_vel` publishing. However, its lifecycle is still one-shot: target class is fixed at process startup, completion schedules `force_exit()`, and the process exits through `os._exit`.

`demo_loop_runner.py` is also a ROS 2 node, but it orchestrates missions outside ROS 2 by launching `multi_camera_object_mission.py` as a child process, reading stdout until it sees `success:` or `failed:`, using `pgrep` to detect external mission processes, and mapping mission outcome to process exit codes. This works for a prototype, but it is brittle for launch files, composition, testing, and future real AMR integration.

## Goals / Non-Goals

**Goals:**

- Convert mission execution into a ROS-native lifecycle: idle, active, succeeded, failed, canceled/preempted.
- Let the demo runner request missions without spawning Python scripts.
- Keep current target selection and four-camera control behavior intact.
- Make terminal mission results machine-readable instead of log-string-readable.
- Prepare both nodes for ROS 2 package entry points and launch-file execution.
- Keep safety behavior explicit: publish zero `/cmd_vel` on idle, target lost, mission failure, cancellation, and shutdown.

**Non-Goals:**

- Replacing the simple servo controller with Nav2, obstacle avoidance, path planning, or full autonomy.
- Changing YOLO detection topics or message types beyond what is needed for orchestration.
- Adding a custom GUI or operator dashboard.
- Optimizing detection quality or target ranking beyond the current priority/camera/distance rules.

## Decisions

### Use a ROS 2 action for mission execution

Represent "move to this target class using multi-camera detections" as an action goal with feedback and a terminal result.

Rationale: the mission is long-running, has progress/state, can fail, and should support cancel/preempt. That maps better to an action than to a simple service. A service can be kept as a thin compatibility layer later, but the primary internal contract should be action-oriented.

Alternative considered: keep subprocess launch and wrap it in a cleaner helper. This preserves current behavior but does not solve launch composition, process scanning, stdout parsing, or testability.

Alternative considered: use only parameters and a `/start` service. This is simpler, but result reporting and cancel semantics become ad hoc.

### Make `MultiCameraObjectMissionNode` persistent

The mission node should start in an idle state, subscribe to configured detection topics, and accept one active mission goal at a time. Each goal provides the target class and optional per-goal overrides where needed.

Rationale: persistent subscriptions avoid process startup latency, avoid repeated ROS graph churn, and let launch files run the controller as a normal node. One active goal keeps behavior close to the current one-shot controller and avoids unsafe command arbitration.

### Make `DemoLoopRunner` a pure ROS 2 scheduler

The runner should keep its current visibility cache and priority/cooldown logic, but mission launch should become an action client call. It should wait on action result/status instead of child process output.

Rationale: the runner's job is policy and sequencing. Process management is not part of the AMR domain model and creates failure modes unrelated to robot behavior.

### Replace hard exits with state transitions

Mission completion should set a terminal state, publish a zero Twist, complete the action result, and return to idle. Node shutdown should happen only through normal ROS shutdown or launch control.

Rationale: `os._exit` bypasses cleanup and makes integration tests and composed execution unsafe.

### Introduce package/launch structure after behavior is isolated

Move from top-level scripts toward a ROS 2 Python package with console entry points only after the mission interface is stable. Keep wrappers or compatibility commands during migration if useful for demos.

Rationale: packaging before lifecycle cleanup would preserve the current process-based coupling under a cleaner file layout.

## Risks / Trade-offs

- Action interface requires a custom action definition or package dependency -> Mitigation: keep the action minimal: target class, timeout/stop-distance overrides if needed, state feedback, result code/message.
- Persistent node can retain stale detections between missions -> Mitigation: clear per-goal counters and candidate caches, or ignore detections older than goal start time.
- Runner and mission node may both observe detections and apply slightly different visibility gates -> Mitigation: keep broad visibility gating in runner and authoritative motion gating in mission node; document the distinction.
- Single active mission limits concurrency -> Mitigation: this matches one mobile base publishing one `/cmd_vel`; reject or preempt additional goals explicitly.
- Migration can disrupt the existing quick demo command -> Mitigation: keep a temporary CLI/launch path that sends one action goal for manual testing.

## Migration Plan

1. Extract mission state and result reporting so completion no longer depends on `os._exit`.
2. Add the ROS 2 action interface and implement an action server inside the mission node.
3. Refactor `demo_loop_runner.py` to use an action client instead of `subprocess.Popen`, stdout parsing, and `external_mission_running()`.
4. Add tests for selection, stale detection pruning, state transitions, action result mapping, and cancel/timeout behavior.
5. Add ROS 2 package metadata, console scripts, and a launch file for mission node plus runner.
6. Update README/demo docs with `ros2 run` or `ros2 launch` commands.

Rollback strategy: keep the current scripts on the original branch until the action path passes simulation smoke tests; the refactor should land on a new branch as requested.

## Open Questions

- Should the action live in this repository as a custom interface package, or should it reuse an existing action type with string fields?
- Should a new target preempt the active mission automatically, or should it be rejected while the AMR is moving?
- Which parameters should be per-goal overrides versus node-level configuration?
