## ADDED Requirements

### Requirement: ROS-native mission request interface
The system SHALL expose a ROS 2 interface for requesting a multi-camera object mission by target class without launching a new Python process for each mission.

#### Scenario: Runner requests a mission
- **WHEN** the demo runner selects a visible priority target
- **THEN** it SHALL send a ROS 2 mission request containing the selected target class to the mission controller node

#### Scenario: No subprocess launch is required
- **WHEN** a mission starts
- **THEN** the runner SHALL NOT use `subprocess.Popen` to execute `multi_camera_object_mission.py`

### Requirement: Persistent mission controller node
The mission controller SHALL run as a persistent ROS 2 node that can move between idle, active, and terminal mission states.

#### Scenario: Node starts idle
- **WHEN** the mission controller node starts
- **THEN** it SHALL subscribe to configured camera detection topics and remain idle until a mission request is accepted

#### Scenario: Mission completes
- **WHEN** an active mission succeeds, fails, times out, or is canceled
- **THEN** the mission controller SHALL publish a zero Twist and return to an idle or terminal-ready state without calling `os._exit`

### Requirement: Machine-readable mission result
The mission controller SHALL report mission completion with structured status instead of requiring callers to parse log text.

#### Scenario: Successful arrival
- **WHEN** the AMR reaches the configured stop distance for the requested target
- **THEN** the mission result SHALL identify the mission as succeeded and include a human-readable message

#### Scenario: Failed mission
- **WHEN** the mission times out, loses the front target, receives invalid detection frames, or violates configured safety gates
- **THEN** the mission result SHALL identify the mission as failed and include the failure reason

### Requirement: Runner uses ROS 2 mission status
The demo runner SHALL use ROS 2 mission status and result data to sequence missions, apply cooldowns, and log outcomes.

#### Scenario: Cooldown after mission result
- **WHEN** the runner receives a terminal mission result
- **THEN** it SHALL apply the configured cooldown for the completed target class based on that result

#### Scenario: Existing mission in progress
- **WHEN** a mission is active
- **THEN** the runner SHALL wait for the active ROS 2 mission result instead of checking operating-system process lists

### Requirement: Current selection behavior is preserved
The refactor SHALL preserve the current target priority, visibility gating, and four-camera acquisition behavior unless explicitly changed by configuration.

#### Scenario: Priority target selection
- **WHEN** multiple configured target classes are visible
- **THEN** the runner SHALL select the first visible class in configured priority order that is not cooling down

#### Scenario: Side-camera acquisition
- **WHEN** the target is visible only from a non-front camera
- **THEN** the mission controller SHALL rotate toward acquisition until the front camera sees the target or acquisition timeout is reached

#### Scenario: Front-camera approach
- **WHEN** the front camera has a fresh valid target detection
- **THEN** the mission controller SHALL compute `/cmd_vel` from the front target position and configured control gains

### Requirement: ROS 2 launch compatibility
The system SHALL support running the mission controller and demo runner as ROS 2 package entry points suitable for `ros2 run` and `ros2 launch`.

#### Scenario: Launch starts both nodes
- **WHEN** the demo launch configuration is executed
- **THEN** both the mission controller and demo runner SHALL start with configurable ROS parameters
