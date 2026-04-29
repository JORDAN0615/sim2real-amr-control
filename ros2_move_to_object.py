#!/usr/bin/env python3
"""Minimal ROS 2 YOLO 3D object servo controller.

Data pipeline:
    yolo_ros publishes /yolo/detections_3d as yolo_msgs/DetectionArray.
    Each detection contains:
        class_name
        score
        bbox3d.center.position.x/y/z
        bbox3d.frame_id

    This node selects the requested class_name, converts the 3D center into:
        forward: object distance in front of the robot, in meters
        left: object lateral offset to the left, in meters

    compute_cmd() then converts forward/left into a Twist:
        Twist.linear.x: forward speed command
        Twist.angular.z: yaw/turn command

    The Twist is published on /cmd_vel until the selected object reaches the
    requested stop distance.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray


def clamp(value, lower, upper):
    """Limit a numeric command to a safe min/max range."""
    return max(lower, min(value, upper))


class MoveToObjectNode(Node):
    """ROS node that turns YOLO 3D object detections into /cmd_vel commands."""

    def __init__(self):
        """Create subscriber, command publisher, and control timer."""
        super().__init__("move_to_object")

        # Target selection. YOLO-World lets the user define class names, so the
        # agent can pass a semantic name such as "yellow forklift".
        self.declare_parameter("target_class_name", "yellow forklift")
        self.declare_parameter("detections_topic", "/yolo/detections_3d")
        self.declare_parameter("target_frame", "base_link")

        # Motion output and controller tuning.
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("speed", 0.50)
        self.declare_parameter("stop_distance", 3.0)
        self.declare_parameter("distance_tolerance", 0.04)
        self.declare_parameter("angle_tolerance_rad", 0.02)
        self.declare_parameter("rotate_in_place_angle_rad", 0.35)
        self.declare_parameter("angular_gain", 1.5)
        self.declare_parameter("max_angular_z", 0.4)
        self.declare_parameter("detection_timeout_sec", 2.0)
        self.declare_parameter("mission_timeout_sec", 999.0)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("debug", False)

        self.target_class_name = self.get_parameter("target_class_name").value
        self.target_frame = self.get_parameter("target_frame").value

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Cache the newest selected object. The control loop consumes this
        # cached position at a deterministic rate.
        self.latest_object_position = None
        self.latest_object_score = None
        self.latest_object_frame = None
        self.latest_detection_received_at = None

        detections_topic = self.get_parameter("detections_topic").value
        self.detections_sub = self.create_subscription(
            DetectionArray,
            detections_topic,
            self.on_detections,
            10,
        )

        self.started_at = self.get_clock().now()
        self.last_seen_at = None
        self.done = False
        self.step_count = 0

        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(1.0 / control_rate_hz, self.control_step)

        self.get_logger().info(
            "move_to_object started: "
            f"target_class_name='{self.target_class_name}', "
            f"detections_topic={detections_topic}, "
            f"target_frame={self.target_frame}, "
            f"cmd_vel={cmd_vel_topic}, "
            f"speed={self.get_parameter('speed').value:.3f} m/s, "
            f"stop_distance={self.get_parameter('stop_distance').value:.3f} m"
        )

    def on_detections(self, msg):
        """Cache the selected object position from /yolo/detections_3d.

        In the demo scene each requested class is expected to appear at most
        once, so the first matching class_name is treated as the target.
        """
        for detection in msg.detections:
            if detection.class_name != self.target_class_name:
                continue

            self.latest_object_position = detection.bbox3d.center.position
            self.latest_object_score = float(detection.score)
            self.latest_object_frame = detection.bbox3d.frame_id
            self.latest_detection_received_at = self.get_clock().now()
            return

    def control_step(self):
        """Run one control tick: read cached object pose and publish /cmd_vel."""
        if self.done:
            return

        mission_timeout = float(self.get_parameter("mission_timeout_sec").value)
        elapsed = (self.get_clock().now() - self.started_at).nanoseconds / 1e9
        if elapsed > mission_timeout:
            self.finish("failed: mission timeout")
            return

        if self.latest_object_position is None:
            self.handle_missing_detection("no matching object received yet")
            return

        self.last_seen_at = self.latest_detection_received_at
        detection_timeout = float(self.get_parameter("detection_timeout_sec").value)
        detection_age = (self.get_clock().now() - self.latest_detection_received_at).nanoseconds / 1e9
        if detection_age > detection_timeout:
            self.handle_missing_detection(f"detection age {detection_age:.2f}s")
            return

        if self.latest_object_frame != self.target_frame:
            self.finish(
                "failed: unexpected detection frame "
                f"{self.latest_object_frame!r}; expected {self.target_frame!r}"
            )
            return

        # yolo_ros already transformed bbox3d into target_frame. With
        # target_frame=base_link, +x is forward and +y is left.
        object_forward = float(self.latest_object_position.x)
        object_left = float(self.latest_object_position.y)

        cmd, state = self.compute_cmd(object_forward, object_left)
        self.cmd_pub.publish(cmd)
        self.log_debug_state(object_forward, object_left, cmd, state)

        if state == "arrived":
            self.finish(
                "success: arrived "
                f"class='{self.target_class_name}' "
                f"forward={object_forward:.3f} left={object_left:.3f} "
                f"score={self.latest_object_score:.3f}"
            )

    def compute_cmd(self, object_forward, object_left):
        """Convert selected object position into a geometry_msgs/Twist.

        Args:
            object_forward: Object distance in front of the robot, in meters.
            object_left: Object lateral offset, in meters; positive means left.

        Output mapping:
            cmd.linear.x controls forward AMR speed.
            cmd.angular.z controls yaw/turning speed.
        """
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

    def log_debug_state(self, forward, left, cmd, state):
        """Print compact controller telemetry every 10 control ticks."""
        if not bool(self.get_parameter("debug").value):
            return

        self.step_count += 1
        if self.step_count % 10 != 0:
            return

        self.get_logger().info(
            f"object='{self.target_class_name}' "
            f"forward={forward:.3f} left={left:.3f} "
            f"score={self.latest_object_score:.3f} "
            f"cmd.linear.x={cmd.linear.x:.3f} cmd.angular.z={cmd.angular.z:.3f} "
            f"state={state}"
        )

    def handle_missing_detection(self, reason):
        """Stop or fail safely when the selected object cannot be read."""
        now = self.get_clock().now()
        detection_timeout = float(self.get_parameter("detection_timeout_sec").value)

        if self.last_seen_at is None:
            self.stop_robot()
            return

        missing_for = (now - self.last_seen_at).nanoseconds / 1e9
        if missing_for > detection_timeout:
            self.finish(f"failed: object lost for {missing_for:.2f}s ({reason})")
        else:
            self.stop_robot()

    def stop_robot(self):
        """Publish a zero Twist to stop the AMR."""
        self.cmd_pub.publish(Twist())

    def finish(self, message):
        """End the mission, stop the robot, log the result, and shut down."""
        self.done = True
        self.stop_robot()
        self.get_logger().info(message)
        rclpy.shutdown()


def main():
    """ROS 2 entry point."""
    rclpy.init()
    node = MoveToObjectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
