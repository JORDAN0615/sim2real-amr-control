#!/usr/bin/env python3
"""Minimal ROS 2 AprilTag servo controller.

Data pipeline:
    Isaac ROS AprilTag publishes either:
        1. /tag_detections: tag pose in the camera optical frame, or
        2. /tf: tag frame, which TF can transform into the robot base frame.

    This node converts either input into the same control-space variables:
        forward: tag distance in front of the robot/camera, in meters
        left: tag lateral offset to the left, in meters

    compute_cmd() then converts forward/left into a Twist:
        Twist.linear.x: forward speed command
        Twist.angular.z: yaw/turn command

    The Twist is published on /cmd_vel until the selected tag reaches the
    requested stop distance.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value, lower, upper):
    """Limit a numeric command to a safe min/max range."""
    return max(lower, min(value, upper))


class MoveToTagNode(Node):
    """ROS node that turns AprilTag pose data into /cmd_vel commands."""

    def __init__(self):
        """Create subscribers/listeners, command publisher, and control timer."""
        super().__init__("move_to_tag")

        # Target selection and pose source.
        self.declare_parameter("tag_id", 0)
        self.declare_parameter("tag_family", "tag36h11")
        self.declare_parameter("pose_source", "tf")
        self.declare_parameter("detections_topic", "/tag_detections")
        self.declare_parameter("target_frame", "base_link")

        # Motion output and controller tuning.
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("speed", 0.15)
        self.declare_parameter("stop_distance", 0.40)
        self.declare_parameter("distance_tolerance", 0.04)
        self.declare_parameter("angle_tolerance_rad", 0.08)
        self.declare_parameter("rotate_in_place_angle_rad", 0.35)
        self.declare_parameter("angular_gain", 1.5)
        self.declare_parameter("max_angular_z", 0.4)
        self.declare_parameter("tag_timeout_sec", 1.0)
        self.declare_parameter("mission_timeout_sec", 10.0)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("debug", False)

        # tag_frame is the TF child frame created by Isaac ROS AprilTag.
        # Example: tag_family=tag36h11 and tag_id=0 -> tag36h11:0
        self.tag_id = self.get_parameter("tag_id").value
        self.tag_family = self.get_parameter("tag_family").value
        self.pose_source = self.get_parameter("pose_source").value
        self.target_frame = self.get_parameter("target_frame").value
        self.tag_frame = f"{self.tag_family}:{self.tag_id}"

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # TF mode reads target_frame -> tag_frame directly. If target_frame is
        # base_link, TF combines base_link -> camera and camera -> tag for us.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Detections mode stores the latest selected tag pose from
        # /tag_detections. The control loop consumes this cached pose.
        self.latest_detection_pose = None
        self.latest_detection_received_at = None

        if self.pose_source == "detections":
            detections_topic = self.get_parameter("detections_topic").value
            self.detections_sub = self.create_subscription(
                AprilTagDetectionArray,
                detections_topic,
                self.on_detections,
                10,
            )
        elif self.pose_source != "tf":
            raise ValueError("pose_source must be 'tf' or 'detections'")

        self.started_at = self.get_clock().now()
        self.last_seen_at = None
        self.done = False
        self.step_count = 0

        # control_step() is the high-rate loop. At 20 Hz it reads pose,
        # computes a Twist, and publishes /cmd_vel every 0.05 seconds.
        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(1.0 / control_rate_hz, self.control_step)

        self.get_logger().info(
            "move_to_tag started: "
            f"pose_source={self.pose_source}, "
            f"{self.target_frame} -> {self.tag_frame}, "
            f"cmd_vel={cmd_vel_topic}, "
            f"speed={self.get_parameter('speed').value:.3f} m/s, "
            f"stop_distance={self.get_parameter('stop_distance').value:.3f} m"
        )

    def on_detections(self, msg):
        """Cache the selected tag pose from /tag_detections.

        msg is an AprilTagDetectionArray from isaac_ros_apriltag. It contains
        all visible tags in msg.detections. This callback filters by family and
        tag_id, then saves only the selected tag position for the control loop.
        """
        for detection in msg.detections:
            if detection.family != self.tag_family or detection.id != self.tag_id:
                continue

            # detection.pose.pose.pose.position is geometry_msgs/Point:
            #   x/y/z are in the camera optical frame for /tag_detections.
            position = detection.pose.pose.pose.position
            self.latest_detection_pose = position
            self.latest_detection_received_at = self.get_clock().now()
            return

    def control_step(self):
        """Run one control tick.

        The timer calls this repeatedly. It checks mission timeout, chooses the
        active pose source, and delegates to the corresponding control path.
        """
        if self.done:
            return

        mission_timeout = float(self.get_parameter("mission_timeout_sec").value)
        elapsed = (self.get_clock().now() - self.started_at).nanoseconds / 1e9
        if elapsed > mission_timeout:
            self.finish("failed: mission timeout")
            return

        if self.pose_source == "detections":
            self.control_from_detection()
            return

        self.control_from_tf()

    def control_from_tf(self):
        """Read tag pose from TF, convert it to forward/left, publish /cmd_vel.

        Expected TF query:
            target_frame -> tag_frame

        In the normal AMR control case:
            base_link -> tag36h11:<id>

        Translation in base_link convention:
            x = forward distance
            y = left/right offset, positive left
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.tag_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException as exc:
            self.handle_missing_tag(str(exc))
            return

        self.last_seen_at = self.get_clock().now()

        translation = transform.transform.translation
        tag_x = float(translation.x)  # ROS base_link convention: +x is forward.
        tag_y = float(translation.y)  # +y is left.

        # compute_cmd() is the only place that decides linear.x/angular.z.
        cmd, state = self.compute_cmd(tag_x, tag_y)
        self.cmd_pub.publish(cmd)
        self.log_debug_state("tf", tag_x, tag_y, cmd, state)

        if state == "arrived":
            self.finish(
                "success: arrived "
                f"x={tag_x:.3f} y={tag_y:.3f} "
                f"distance={math.hypot(tag_x, tag_y):.3f}"
            )

    def control_from_detection(self):
        """Read cached /tag_detections pose and publish /cmd_vel.

        /tag_detections pose is in the camera optical frame:
            +z = forward from camera
            +x = right in the image/camera frame

        For the simplified camera-frame demo we map it into control-space:
            forward = optical_z
            left = -optical_x

        This works best when the camera points roughly along the robot forward
        direction. TF mode is more accurate when camera mounting is offset.
        """
        if self.latest_detection_pose is None:
            self.handle_missing_tag("no detection received yet")
            return

        self.last_seen_at = self.latest_detection_received_at
        tag_timeout = float(self.get_parameter("tag_timeout_sec").value)
        # nanoseconds to seconds
        detection_age = (self.get_clock().now() - self.latest_detection_received_at).nanoseconds / 1e9 
        if detection_age > tag_timeout:
            self.handle_missing_tag(f"detection age {detection_age:.2f}s")
            return

        optical_x = float(self.latest_detection_pose.x)
        optical_z = float(self.latest_detection_pose.z)

        # ROS optical frame convention: +z is forward, +x is right.
        # The controller expects positive lateral offset to mean "tag is left",
        # so right-positive optical_x becomes left = -optical_x.
        tag_forward = optical_z
        tag_left = -optical_x

        cmd, state = self.compute_cmd(tag_forward, tag_left)
        self.cmd_pub.publish(cmd)
        self.log_debug_state("detections", tag_forward, tag_left, cmd, state)

        if state == "arrived":
            self.finish(
                "success: arrived "
                f"optical_x={optical_x:.3f} optical_z={optical_z:.3f} "
                f"forward={tag_forward:.3f} left={tag_left:.3f}"
            )

    def compute_cmd(self, tag_x, tag_y):
        """Convert selected tag position into a geometry_msgs/Twist.

        Args:
            tag_x: Forward distance to the tag in meters.
            tag_y: Lateral offset to the tag in meters; positive means left.

        Returns:
            (cmd, state), where cmd is the Twist to publish on /cmd_vel and
            state is one of:
                arrived: stop because tag_x is within stop distance
                turning: rotate in place because heading error is large
                driving: drive forward, optionally with angular correction

        Control math:
            forward_error = tag_x - stop_distance
            heading_error = atan2(tag_y, tag_x)

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

        
        forward_error = tag_x - stop_distance
        heading_error = math.atan2(tag_y, max(tag_x, 1e-6))

        cmd = Twist()

        # Turn toward the tag. Positive heading_error means the tag is left, so
        # positive angular.z rotates the robot left under the standard ROS base
        # frame convention.
        cmd.angular.z = clamp(angular_gain * heading_error, -max_angular_z, max_angular_z)

        # Stop once the forward distance is close enough to the desired standoff.
        if forward_error <= distance_tolerance:
            return Twist(), "arrived"

        # If the tag is far off-axis, rotate first instead of driving forward.
        if abs(heading_error) > rotate_in_place_angle:
            cmd.linear.x = 0.0
            return cmd, "turning"

        # Otherwise drive forward while applying angular correction.
        cmd.linear.x = speed

        # Ignore tiny heading errors to avoid unnecessary oscillation.
        if abs(heading_error) <= angle_tolerance:
            cmd.angular.z = 0.0

        return cmd, "driving"

    def log_debug_state(self, source, forward, left, cmd, state):
        """Print compact controller telemetry every 10 control ticks."""
        if not bool(self.get_parameter("debug").value):
            return

        self.step_count += 1
        if self.step_count % 10 != 0:
            return

        self.get_logger().info(
            f"{source}: forward={forward:.3f} left={left:.3f} "
            f"cmd.linear.x={cmd.linear.x:.3f} cmd.angular.z={cmd.angular.z:.3f} "
            f"state={state}"
        )

    def handle_missing_tag(self, reason):
        """Stop or fail safely when the selected tag cannot be read."""
        now = self.get_clock().now()
        tag_timeout = float(self.get_parameter("tag_timeout_sec").value)

        if self.last_seen_at is None:
            self.stop_robot()
            return

        missing_for = (now - self.last_seen_at).nanoseconds / 1e9
        if missing_for > tag_timeout:
            self.finish(f"failed: tag lost for {missing_for:.2f}s ({reason})")
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
    node = MoveToTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
