## Baseline Behavior Review

### `multi_camera_object_mission.py`

- Subscribes to front, left, right, and back `yolo_msgs/DetectionArray` topics.
- Filters detections by `target_class_name`, `target_frame`, score, planar distance, and front-camera positive x.
- Keeps only the nearest valid detection per camera stream.
- State machine behavior:
  - `SEARCH`: prefer fresh front detection; otherwise choose a side/back acquisition source.
  - `TURN_LEFT_TO_ACQUIRE_FRONT` / `TURN_RIGHT_TO_ACQUIRE_FRONT`: rotate in place until the front camera has enough fresh detections.
  - `FRONT_APPROACH`: publish `/cmd_vel` from front detection position until arrival or target loss.
  - `ARRIVED` / `FAILED`: publish zero Twist and report terminal result.
- Current refactor removes `os._exit` and keeps the node alive after terminal results.
- Safety behavior: publish zero Twist when no target is available during search/approach, when a mission fails, when a mission is canceled, and during shutdown.

### `demo_loop_runner.py`

- Subscribes to the same camera detection topics and caches visible priority targets.
- Treats `priority_targets` as both priority order and allowlist.
- Applies broad visibility gates: score, `target_frame`, min/max planar distance, and stale timeout.
- Waits while the pause flag exists, while a mission is active, while no priority target is visible, or while visible targets are cooling down.
- Selects the first visible target in priority order whose cooldown has expired.
- Current refactor replaces subprocess mission launch with a ROS-native command/status topic protocol.
- Mission cooldown and logging are now based on structured mission status messages instead of process exit codes.

## Temporary ROS Mission Protocol

Until a generated ROS 2 action interface is added, the nodes use standard `std_msgs/String` topics with JSON payloads:

- Command topic: `/multi_camera_object_mission/command`
- Cancel topic: `/multi_camera_object_mission/cancel`
- Status topic: `/multi_camera_object_mission/status`

Command payload:

```json
{"target_class_name": "person"}
```

Terminal status payload includes:

```json
{
  "active": false,
  "terminal": true,
  "state": "ARRIVED",
  "result_code": "succeeded",
  "target_class_name": "person",
  "message": "success: arrived ..."
}
```

This keeps the implementation ROS-native and package-light while preserving a clean migration path to a custom action.
