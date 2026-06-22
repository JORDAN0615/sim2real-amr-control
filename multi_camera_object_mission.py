#!/usr/bin/env python3
"""Four-camera YOLO 3D object mission controller.

This one-shot ROS 2 node subscribes to front, left, right, and back yolo_ros
DetectionArray topics. Non-front cameras are used only to decide which way to turn
until the front camera acquires the requested class. Approach is controlled
only from front-camera detections in base_link.
"""

from dataclasses import dataclass
import json
import math
from enum import Enum
import sys

import rclpy
from geometry_msgs.msg import Twist
from mission_logic import clamp, planar_distance, select_acquisition_candidate
from rclpy.node import Node
from std_msgs.msg import String
from yolo_msgs.msg import DetectionArray


class MissionState(str, Enum):
    """Controller states for the one-shot mission."""

    IDLE = "IDLE"
    SEARCH = "SEARCH"
    TURN_LEFT_TO_ACQUIRE_FRONT = "TURN_LEFT_TO_ACQUIRE_FRONT"
    TURN_RIGHT_TO_ACQUIRE_FRONT = "TURN_RIGHT_TO_ACQUIRE_FRONT"
    FRONT_APPROACH = "FRONT_APPROACH"
    ARRIVED = "ARRIVED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


class MissionResultCode(str, Enum):
    """Machine-readable terminal mission outcomes."""

    IDLE = "idle"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    MISSION_TIMEOUT = "mission_timeout"
    NO_TARGET = "no_target"
    FRONT_ACQUIRE_TIMEOUT = "front_acquire_timeout"
    TARGET_LOST = "target_lost"
    INVALID_FRAME = "invalid_frame"
    REJECTED = "rejected"
    CANCELED = "canceled"


@dataclass
class Candidate:
    """Selected target candidate for one camera stream."""

    camera: str
    position: object
    score: float
    frame_id: str
    distance: float
    received_at: object
    received_ns: int


class MultiCameraObjectMissionNode(Node):
    """ROS node that handles side-camera acquisition and front-camera approach."""

    RIGHT_TIE_EPSILON_M = 0.05

    def __init__(self):
        super().__init__("multi_camera_object_mission")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("target_class_name", "yellow forklift"),
                ("front_detections_topic", "/yolo_front/detections_3d"),
                ("left_detections_topic", "/yolo_left/detections_3d"),
                ("right_detections_topic", "/yolo_right/detections_3d"),
                ("back_detections_topic", "/yolo_back/detections_3d"),
                ("target_frame", "base_link"),
                ("cmd_vel_topic", "/cmd_vel"),
                ("speed", 15.0),
                ("turn_speed", 5.0),
                ("angular_gain", 1.5),
                ("max_angular_z", 0.4),
                ("rotate_in_place_angle_rad", 0.35),
                ("angle_tolerance_rad", 0.02),
                ("stop_distance", 2.0),
                ("distance_tolerance", 0.04),
                ("mission_timeout_sec", 300.0),
                ("detection_timeout_sec", 8.0),
                ("front_acquire_timeout_sec", 30.0),
                ("front_acquire_required_ticks", 1),
                ("min_detection_score", 0.1),
                ("min_target_distance", 0.2),
                ("max_target_distance", 20.0),
                ("control_rate_hz", 20.0),
                ("debug", False),
                ("auto_start", False),
                ("mission_command_topic", "/multi_camera_object_mission/command"),
                ("mission_cancel_topic", "/multi_camera_object_mission/cancel"),
                ("mission_status_topic", "/multi_camera_object_mission/status"),
                ("status_publish_rate_hz", 1.0),
            ],
        )

        self.target_class_name = self.get_parameter("target_class_name").value
        self.target_frame = self.get_parameter("target_frame").value
        self.validate_parameters()

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.latest_candidates = {
            "front": None,
            "left": None,
            "right": None,
            "back": None,
        }

        self.detection_subscriptions = [
            self.create_subscription(
                DetectionArray,
                self.get_parameter("front_detections_topic").value,
                lambda msg: self.on_detections("front", msg),
                10,
            ),
            self.create_subscription(
                DetectionArray,
                self.get_parameter("left_detections_topic").value,
                lambda msg: self.on_detections("left", msg),
                10,
            ),
            self.create_subscription(
                DetectionArray,
                self.get_parameter("right_detections_topic").value,
                lambda msg: self.on_detections("right", msg),
                10,
            ),
            self.create_subscription(
                DetectionArray,
                self.get_parameter("back_detections_topic").value,
                lambda msg: self.on_detections("back", msg),
                10,
            ),
        ]

        self.status_pub = self.create_publisher(
            String,
            self.get_parameter("mission_status_topic").value,
            10,
        )
        self.command_sub = self.create_subscription(
            String,
            self.get_parameter("mission_command_topic").value,
            self.on_mission_command,
            10,
        )
        self.cancel_sub = self.create_subscription(
            String,
            self.get_parameter("mission_cancel_topic").value,
            self.on_mission_cancel,
            10,
        )

        self.state = MissionState.IDLE
        self.started_at = self.get_clock().now()
        self.state_entered_at = self.started_at
        self.goal_started_at = None
        self.front_seen_count = 0
        self.last_front_acquire_received_ns = None
        self.last_front_seen_at = None
        self.done = True
        self.mission_active = False
        self.exit_code = 0
        self.step_count = 0
        self.selected_camera = None
        self.saw_any_valid_target = False
        self.last_result = None

        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(1.0 / control_rate_hz, self.control_step)
        status_publish_rate_hz = float(self.get_parameter("status_publish_rate_hz").value)
        self.status_timer = self.create_timer(
            1.0 / status_publish_rate_hz,
            self.publish_current_status,
        )

        self.get_logger().info(
            "multi_camera_object_mission started: "
            f"target_class_name='{self.target_class_name}', "
            f"front={self.get_parameter('front_detections_topic').value}, "
            f"left={self.get_parameter('left_detections_topic').value}, "
            f"right={self.get_parameter('right_detections_topic').value}, "
            f"back={self.get_parameter('back_detections_topic').value}, "
            f"target_frame={self.target_frame}, cmd_vel={cmd_vel_topic}"
        )
        if bool(self.get_parameter("auto_start").value):
            self.start_mission(self.target_class_name)
        else:
            self.publish_status("idle", MissionResultCode.SUCCEEDED, "waiting for mission command")

    def on_mission_command(self, msg):
        """Start a mission from a ROS-native command message.

        The temporary command protocol accepts either a raw target-class string
        or JSON like {"target_class_name": "person"}. It can be replaced by a
        generated ROS action without changing the controller state machine.
        """
        target_class_name = msg.data.strip()
        if not target_class_name:
            self.publish_status(
                "rejected",
                MissionResultCode.REJECTED,
                "empty mission command",
                terminal=True,
            )
            return
        try:
            payload = json.loads(target_class_name)
        except json.JSONDecodeError:
            payload = {"target_class_name": target_class_name}

        target_class_name = str(payload.get("target_class_name", "")).strip()
        if not target_class_name:
            self.publish_status(
                "rejected",
                MissionResultCode.REJECTED,
                "mission command missing target_class_name",
                terminal=True,
            )
            return
        self.start_mission(target_class_name)

    def on_mission_cancel(self, msg):
        """Cancel the active mission when a runner/operator requests stop."""
        if not self.mission_active:
            self.publish_status("idle", MissionResultCode.CANCELED, "no active mission to cancel")
            return
        reason = msg.data.strip() or "mission canceled"
        self.transition_to(MissionState.CANCELED)
        self.finish(reason, MissionResultCode.CANCELED)

    def start_mission(self, target_class_name):
        """Reset per-mission state and begin searching for the requested class."""
        if self.mission_active:
            self.publish_status(
                "rejected",
                MissionResultCode.REJECTED,
                f"mission already active for {self.target_class_name!r}",
                terminal=True,
            )
            return False

        self.target_class_name = target_class_name
        self.reset_mission_state()
        self.mission_active = True
        self.done = False
        self.transition_to(MissionState.SEARCH)
        self.publish_status("accepted", MissionResultCode.SUCCEEDED, "mission accepted")
        self.get_logger().info(f"mission accepted: target_class_name={target_class_name!r}")
        return True

    def reset_mission_state(self):
        """Clear all state that must not leak between mission goals."""
        now = self.get_clock().now()
        self.latest_candidates = {
            "front": None,
            "left": None,
            "right": None,
            "back": None,
        }
        self.started_at = now
        self.state_entered_at = now
        self.goal_started_at = now
        self.front_seen_count = 0
        self.last_front_acquire_received_ns = None
        self.last_front_seen_at = None
        self.step_count = 0
        self.selected_camera = None
        self.saw_any_valid_target = False
        self.last_result = None

    def validate_parameters(self):
        """Fail early on parameters that would make the mission unsafe."""
        numeric_positive = [
            "speed",
            "turn_speed",
            "max_angular_z",
            "stop_distance",
            "mission_timeout_sec",
            "detection_timeout_sec",
            "front_acquire_timeout_sec",
            "control_rate_hz",
            "status_publish_rate_hz",
        ]
        for name in numeric_positive:
            if float(self.get_parameter(name).value) <= 0.0:
                raise ValueError(f"invalid parameter: {name} must be positive")

        required_ticks = int(self.get_parameter("front_acquire_required_ticks").value)
        if required_ticks <= 0:
            raise ValueError("invalid parameter: front_acquire_required_ticks must be positive")

        min_distance = float(self.get_parameter("min_target_distance").value)
        max_distance = float(self.get_parameter("max_target_distance").value)
        if min_distance < 0.0 or max_distance <= min_distance:
            raise ValueError(
                "invalid parameter: max_target_distance must be greater than min_target_distance"
            )

    def on_detections(self, camera, msg):
        """Cache the nearest valid target-class detection from one camera."""
        if not self.mission_active:
            return

        now = self.get_clock().now()
        selected = None

        for detection in msg.detections:
            if detection.class_name != self.target_class_name:
                continue

            frame_id = detection.bbox3d.frame_id
            if frame_id != self.target_frame:
                self.finish(
                    "failed: unexpected detection frame "
                    f"camera={camera} frame={frame_id!r}; expected {self.target_frame!r}",
                    MissionResultCode.INVALID_FRAME,
                )
                return

            candidate = self.build_candidate(camera, detection, now)
            if candidate is None:
                continue
            if selected is None or candidate.distance < selected.distance:
                selected = candidate

        self.latest_candidates[camera] = selected
        if selected is not None:
            self.saw_any_valid_target = True

    def build_candidate(self, camera, detection, received_at):
        """Return a gated target candidate, or None when the detection is invalid."""
        position = detection.bbox3d.center.position
        score = float(detection.score)
        x = float(position.x)
        y = float(position.y)
        distance = planar_distance(position)

        if score < float(self.get_parameter("min_detection_score").value):
            return None
        # Front detections drive approach, so they must be in front of base_link.
        # Non-front detections only trigger heading acquisition and may appear with
        # x <= 0 depending on side-camera placement and base_link origin.
        if camera == "front" and x <= 0.0:
            return None

        min_distance = float(self.get_parameter("min_target_distance").value)
        max_distance = float(self.get_parameter("max_target_distance").value)
        if distance < min_distance or distance > max_distance:
            return None

        return Candidate(
            camera=camera,
            position=position,
            score=score,
            frame_id=detection.bbox3d.frame_id,
            distance=distance,
            received_at=received_at,
            received_ns=received_at.nanoseconds,
        )

    def control_step(self):
        """Run one deterministic control tick."""
        if not self.mission_active or self.done:
            return

        if self.mission_timed_out():
            if self.saw_any_valid_target:
                self.finish("failed: mission timeout", MissionResultCode.MISSION_TIMEOUT)
            else:
                self.finish(
                    "failed: no target detected before mission timeout",
                    MissionResultCode.NO_TARGET,
                )
            return

        if self.state == MissionState.SEARCH:
            self.control_search()
        elif self.state == MissionState.TURN_RIGHT_TO_ACQUIRE_FRONT:
            self.control_turn("right")
        elif self.state == MissionState.TURN_LEFT_TO_ACQUIRE_FRONT:
            self.control_turn("left")
        elif self.state == MissionState.FRONT_APPROACH:
            self.control_front_approach()

    def control_search(self):
        """Select front approach or side acquisition from fresh candidates."""
        front = self.fresh_candidate("front")
        left = self.fresh_candidate("left")
        right = self.fresh_candidate("right")
        back = self.fresh_candidate("back")

        if front is not None:
            self.front_seen_count = 1
            self.last_front_seen_at = front.received_at
            self.selected_camera = "front"
            self.transition_to(MissionState.FRONT_APPROACH)
            self.control_front_approach()
            return

        acquisition_source = self.select_acquisition_candidate(left, right, back)
        if acquisition_source is None:
            self.selected_camera = None
            self.front_seen_count = 0
            self.stop_robot()
            self.log_debug_state(Twist(), front, left, right, back)
            return

        self.selected_camera = acquisition_source.camera
        if acquisition_source.camera in ("right", "back"):
            self.transition_to(MissionState.TURN_RIGHT_TO_ACQUIRE_FRONT)
            self.control_turn("right")
        else:
            self.transition_to(MissionState.TURN_LEFT_TO_ACQUIRE_FRONT)
            self.control_turn("left")

    def control_turn(self, direction):
        """Rotate in place until the front camera acquires the target."""
        front = self.fresh_candidate("front")
        left = self.fresh_candidate("left")
        right = self.fresh_candidate("right")
        back = self.fresh_candidate("back")

        self.update_front_acquire_count(front)

        required_ticks = int(self.get_parameter("front_acquire_required_ticks").value)
        if self.front_seen_count >= required_ticks:
            self.selected_camera = "front"
            self.transition_to(MissionState.FRONT_APPROACH)
            self.control_front_approach()
            return

        elapsed = (self.get_clock().now() - self.state_entered_at).nanoseconds / 1e9
        acquire_timeout = float(self.get_parameter("front_acquire_timeout_sec").value)
        if elapsed > acquire_timeout:
            self.finish(
                "failed: front acquire timeout",
                MissionResultCode.FRONT_ACQUIRE_TIMEOUT,
            )
            return

        turn_speed = float(self.get_parameter("turn_speed").value)
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -turn_speed if direction == "right" else turn_speed
        self.cmd_pub.publish(cmd)
        self.log_debug_state(cmd, front, left, right, back)

    def control_front_approach(self):
        """Approach the target using only front-camera detections."""
        front = self.fresh_candidate("front")
        left = self.fresh_candidate("left")
        right = self.fresh_candidate("right")
        back = self.fresh_candidate("back")

        if front is None:
            self.front_seen_count = 0
            self.stop_robot()
            if self.front_object_lost():
                self.finish("failed: front object lost", MissionResultCode.TARGET_LOST)
                return
            self.log_debug_state(Twist(), front, left, right, back)
            return

        self.last_front_acquire_received_ns = front.received_ns
        self.front_seen_count += 1
        self.last_front_seen_at = front.received_at
        self.selected_camera = "front"

        object_forward = float(front.position.x)
        object_left = float(front.position.y)
        cmd, control_state = self.compute_approach_cmd(object_forward, object_left)
        self.cmd_pub.publish(cmd)
        self.log_debug_state(cmd, front, left, right, back)

        if control_state == "arrived":
            self.transition_to(MissionState.ARRIVED)
            self.finish(
                "success: arrived "
                f"class='{self.target_class_name}' "
                f"forward={object_forward:.3f} left={object_left:.3f} "
                f"score={front.score:.3f}",
                MissionResultCode.SUCCEEDED,
            )

    def compute_approach_cmd(self, object_forward, object_left):
        """Convert front-camera object position into a Twist command."""
        speed = float(self.get_parameter("speed").value)
        stop_distance = float(self.get_parameter("stop_distance").value)
        distance_tolerance = float(self.get_parameter("distance_tolerance").value)
        angle_tolerance = float(self.get_parameter("angle_tolerance_rad").value)
        rotate_in_place_angle = float(self.get_parameter("rotate_in_place_angle_rad").value)
        angular_gain = float(self.get_parameter("angular_gain").value)
        max_angular_z = float(self.get_parameter("max_angular_z").value)

        forward_error = object_forward - stop_distance
        heading_error = math.atan2(object_left, max(object_forward, 1e-6))

        cmd = Twist()
        cmd.angular.z = clamp(angular_gain * heading_error, -max_angular_z, max_angular_z)

        if forward_error <= distance_tolerance:
            return Twist(), "arrived"

        if abs(heading_error) > rotate_in_place_angle:
            cmd.linear.x = 0.0
            return cmd, "turning"

        cmd.linear.x = speed
        if abs(heading_error) <= angle_tolerance:
            cmd.angular.z = 0.0

        return cmd, "driving"

    def update_front_acquire_count(self, front):
        """Count distinct front-camera detection updates during side acquisition."""
        if front is None:
            self.front_seen_count = 0
            self.last_front_acquire_received_ns = None
            return

        if front.received_ns == self.last_front_acquire_received_ns:
            return

        self.front_seen_count += 1
        self.last_front_acquire_received_ns = front.received_ns
        self.last_front_seen_at = front.received_at

    def fresh_candidate(self, camera):
        """Return the camera candidate only if it has not expired."""
        candidate = self.latest_candidates[camera]
        if candidate is None:
            return None

        if self.goal_started_at is not None and candidate.received_at < self.goal_started_at:
            return None

        age = (self.get_clock().now() - candidate.received_at).nanoseconds / 1e9
        if age > float(self.get_parameter("detection_timeout_sec").value):
            return None
        return candidate

    def select_acquisition_candidate(self, left, right, back):
        """Pick a non-front target source, preferring right turns on near ties."""
        return select_acquisition_candidate(
            (left, right, back),
            ("right", "back", "left"),
            self.RIGHT_TIE_EPSILON_M,
        )

    def mission_timed_out(self):
        elapsed = (self.get_clock().now() - self.started_at).nanoseconds / 1e9
        return elapsed > float(self.get_parameter("mission_timeout_sec").value)

    def front_object_lost(self):
        if self.last_front_seen_at is None:
            return False
        missing_for = (self.get_clock().now() - self.last_front_seen_at).nanoseconds / 1e9
        return missing_for > float(self.get_parameter("detection_timeout_sec").value)

    def transition_to(self, next_state):
        """Record and log state transitions."""
        if self.state == next_state:
            return
        previous = self.state
        self.state = next_state
        self.state_entered_at = self.get_clock().now()
        if next_state in (
            MissionState.TURN_LEFT_TO_ACQUIRE_FRONT,
            MissionState.TURN_RIGHT_TO_ACQUIRE_FRONT,
        ):
            self.front_seen_count = 0
            self.last_front_acquire_received_ns = None
        self.get_logger().info(f"state transition: {previous.value} -> {next_state.value}")
        if self.mission_active:
            self.publish_status(next_state.value, MissionResultCode.SUCCEEDED, "state transition")

    def stop_robot(self):
        """Publish a zero Twist to stop the AMR."""
        if not rclpy.ok():
            return
        self.cmd_pub.publish(Twist())

    def finish(self, message, result_code=None):
        """End the active mission, stop the robot, publish result, and return idle."""
        if self.done:
            return
        if result_code is None:
            result_code = (
                MissionResultCode.SUCCEEDED
                if message.startswith("success")
                else MissionResultCode.MISSION_TIMEOUT
            )

        if result_code == MissionResultCode.SUCCEEDED:
            terminal_state = MissionState.ARRIVED
            self.exit_code = 0
        elif result_code == MissionResultCode.CANCELED:
            terminal_state = MissionState.CANCELED
            self.exit_code = 130
        else:
            terminal_state = MissionState.FAILED
            self.exit_code = 1

        if self.state != terminal_state:
            self.transition_to(terminal_state)
        self.done = True
        self.mission_active = False
        self.stop_robot()
        self.last_result = {
            "state": terminal_state.value,
            "result_code": result_code.value,
            "message": message,
            "target_class_name": self.target_class_name,
        }
        self.publish_status(terminal_state.value, result_code, message, terminal=True)
        self.get_logger().info(message)
        self.transition_to(MissionState.IDLE)

    def publish_current_status(self):
        """Publish periodic state so operators can reliably use topic echo --once."""
        if self.mission_active:
            self.publish_status(
                self.state.value,
                MissionResultCode.RUNNING,
                "mission active",
            )
            return

        self.publish_status(
            self.state.value,
            MissionResultCode.IDLE,
            "mission idle",
        )

    def publish_status(self, state, result_code, message, terminal=False):
        """Publish machine-readable mission state for runner/orchestration nodes."""
        payload = {
            "state": str(state),
            "result_code": result_code.value if isinstance(result_code, MissionResultCode) else str(result_code),
            "message": message,
            "target_class_name": self.target_class_name,
            "terminal": bool(terminal),
            "active": bool(self.mission_active),
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)

    def log_debug_state(self, cmd, front, left, right, back):
        """Print compact telemetry every 10 control ticks when debug is enabled."""
        if not bool(self.get_parameter("debug").value):
            return

        self.step_count += 1
        if self.step_count % 10 != 0:
            return

        self.get_logger().info(
            f"state={self.state.value} "
            f"front_seen_count={self.front_seen_count} "
            f"selected_camera={self.selected_camera} "
            f"front_distance={self.format_distance(front)} "
            f"left_distance={self.format_distance(left)} "
            f"right_distance={self.format_distance(right)} "
            f"back_distance={self.format_distance(back)} "
            f"cmd.linear.x={cmd.linear.x:.3f} "
            f"cmd.angular.z={cmd.angular.z:.3f}"
        )

    @staticmethod
    def format_distance(candidate):
        if candidate is None:
            return "none"
        return f"{candidate.distance:.3f}"


def main():
    """ROS 2 entry point."""
    rclpy.init()
    node = None
    exit_code = 0
    try:
        node = MultiCameraObjectMissionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        exit_code = 130
        if node is not None and rclpy.ok():
            node.stop_robot()
    finally:
        if node is not None:
            exit_code = node.exit_code if exit_code == 0 else exit_code
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
