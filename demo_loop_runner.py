#!/usr/bin/env python3
"""Deterministic scheduler for the multi-camera AMR demo loop."""

from dataclasses import dataclass
from datetime import datetime
import argparse
import json
import os
import time

from mission_logic import (
    DEFAULT_PRIORITY_TARGETS,
    is_distance_visible,
    parse_targets,
    planar_distance,
    prune_stale_targets,
    select_priority_target,
)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from yolo_msgs.msg import DetectionArray


@dataclass
class VisibleTarget:
    """Latest visible target-class observation."""

    class_name: str
    camera: str
    score: float
    distance: float
    received_at_sec: float


class DemoLoopRunner(Node):
    """Pick visible targets and launch one mission at a time."""

    def __init__(self, args):
        super().__init__("demo_loop_runner")
        self.args = args
        self.apply_ros_parameters()
        self.priority_targets = parse_targets(args.priority_targets)
        self.cooldown_until = {}
        self.visible_targets = {}
        self.last_status_log_sec = 0.0
        self.mission_active = False
        self.active_target = None
        self.last_mission_result = None
        self.active_mission_started = False
        self.mission_command_sent_at = None

        self.log_file = open(args.log_file, "a", encoding="utf-8")
        self.mission_command_pub = self.create_publisher(
            String,
            args.mission_command_topic,
            10,
        )
        self.mission_cancel_pub = self.create_publisher(
            String,
            args.mission_cancel_topic,
            10,
        )
        self.mission_status_sub = self.create_subscription(
            String,
            args.mission_status_topic,
            self.on_mission_status,
            10,
        )

        self.detection_subscriptions = [
            self.create_subscription(
                DetectionArray,
                args.front_detections_topic,
                lambda msg: self.on_detections("front", msg),
                10,
            ),
            self.create_subscription(
                DetectionArray,
                args.left_detections_topic,
                lambda msg: self.on_detections("left", msg),
                10,
            ),
            self.create_subscription(
                DetectionArray,
                args.right_detections_topic,
                lambda msg: self.on_detections("right", msg),
                10,
            ),
            self.create_subscription(
                DetectionArray,
                args.back_detections_topic,
                lambda msg: self.on_detections("back", msg),
                10,
            ),
        ]

        self.write_log(
            "started "
            f"priority_targets={self.priority_targets} "
            f"pause_flag={args.pause_flag_file}"
        )

    def apply_ros_parameters(self):
        """Let launch/parameter YAML override CLI defaults for runner settings."""
        parameter_defaults = {
            "priority_targets": self.args.priority_targets,
            "front_detections_topic": self.args.front_detections_topic,
            "left_detections_topic": self.args.left_detections_topic,
            "right_detections_topic": self.args.right_detections_topic,
            "back_detections_topic": self.args.back_detections_topic,
            "target_frame": self.args.target_frame,
            "min_visible_score": self.args.min_visible_score,
            "min_visible_distance": self.args.min_visible_distance,
            "max_visible_distance": self.args.max_visible_distance,
            "visible_detection_timeout_sec": self.args.visible_detection_timeout_sec,
            "poll_interval_sec": self.args.poll_interval_sec,
            "pause_between_missions_sec": self.args.pause_between_missions_sec,
            "target_cooldown_sec": self.args.target_cooldown_sec,
            "pause_flag_file": self.args.pause_flag_file,
            "mission_command_topic": self.args.mission_command_topic,
            "mission_cancel_topic": self.args.mission_cancel_topic,
            "mission_status_topic": self.args.mission_status_topic,
            "log_file": self.args.log_file,
            "status_log_interval_sec": self.args.status_log_interval_sec,
        }
        for name, default in parameter_defaults.items():
            self.declare_parameter(name, default)
            setattr(self.args, name, self.get_parameter(name).value)

    def on_mission_status(self, msg):
        """Track structured mission controller status."""
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.write_log(f"ignored invalid mission status: {msg.data!r}")
            return

        status_active = bool(status.get("active", False))
        self.mission_active = status_active
        if self.active_target is not None and status_active:
            self.active_mission_started = True

        if status.get("terminal"):
            self.last_mission_result = status
            self.mission_active = False
            self.active_mission_started = False
        elif self.active_target is not None and self.is_completed_status(status, status_active):
            self.last_mission_result = {
                "target_class_name": self.active_target.class_name,
                "result_code": status.get("result_code", "inactive_without_terminal"),
                "message": status.get(
                    "message",
                    "mission became inactive without terminal status",
                ),
                "state": status.get("state", "IDLE"),
                "terminal": True,
                "active": False,
            }
            self.active_mission_started = False

    def is_completed_status(self, status, status_active):
        """Return true when a mission command appears to have completed."""
        if status_active:
            return False
        if self.active_mission_started:
            return True
        if self.mission_command_sent_at is None:
            return False
        if time.time() - self.mission_command_sent_at < 1.0:
            return False
        return status.get("target_class_name") == self.active_target.class_name

    def on_detections(self, camera, msg):
        """Cache latest visible detections from one camera topic."""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        for detection in msg.detections:
            class_name = detection.class_name
            # 如果 class_name 不在 priority_targets 中，則跳過
            if class_name not in self.priority_targets:
                continue
            # 如果 detection 分數小於 min_visible_score 並且距離不在 min_visible_distance 和 max_visible_distance 之間，則跳過
            if not self.is_detection_visible(detection):
                continue

            position = detection.bbox3d.center.position
            target = VisibleTarget(
                class_name=class_name,
                # 相機名稱
                camera=camera,
                score=float(detection.score),
                distance=planar_distance(position),
                received_at_sec=now_sec,
            )
            current = self.visible_targets.get(class_name)
            if current is None or target.received_at_sec >= current.received_at_sec:
                self.visible_targets[class_name] = target

    def is_detection_visible(self, detection):
        """Apply the same broad gating defaults used by the mission controller."""
        if float(detection.score) < self.args.min_visible_score:
            return False
        if detection.bbox3d.frame_id != self.args.target_frame:
            return False

        position = detection.bbox3d.center.position
        distance = planar_distance(position)
        return is_distance_visible(
            distance,
            self.args.min_visible_distance,
            self.args.max_visible_distance,
        )

    def run_forever(self):
        """Main scheduler loop."""
        try:
            while rclpy.ok():
                # 檢查是否有新的 detection，讓 ROS2 node 處理一次 callback
                rclpy.spin_once(self, timeout_sec=0.1)
                self.prune_stale_targets()
                # 檢查是否需要等待
                reason = self.next_wait_reason()
                # 如果 reason 不為 None，則記錄等待原因並等待 poll_interval_sec 秒
                if reason is not None:
                    self.log_wait_reason(reason)
                    time.sleep(self.args.poll_interval_sec)
                    continue

                target = self.select_target()
                # 如果 target 為 None，則記錄等待原因並等待 poll_interval_sec 秒
                if target is None:
                    self.log_wait_reason("no visible priority target")
                    time.sleep(self.args.poll_interval_sec)
                    continue
                
                result = self.launch_mission(target)
                result_code = result.get("result_code", "unknown")
                if self.args.target_cooldown_sec > 0.0:
                    self.cooldown_until[target.class_name] = (
                        time.time() + self.args.target_cooldown_sec
                    )
                    self.write_log(
                        f"cooldown target={target.class_name!r} "
                        f"until={format_epoch(self.cooldown_until[target.class_name])} "
                        f"mission_result={result_code}"
                    )
                else:
                    self.write_log(
                        f"cooldown disabled target={target.class_name!r} "
                        f"mission_result={result_code}"
                    )
                time.sleep(self.args.pause_between_missions_sec)
        finally:
            if self.mission_active:
                self.cancel_active_mission("demo loop shutting down")
            self.log_file.close()
    # 
    def prune_stale_targets(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        self.visible_targets = prune_stale_targets(
            self.visible_targets,
            now_sec,
            self.args.visible_detection_timeout_sec,
        )

    def next_wait_reason(self):
        # 如果 pause_flag_file 存在，則返回 pause flag present: /tmp/demo_loop_pause
        if os.path.exists(self.args.pause_flag_file):
            return f"pause flag present: {self.args.pause_flag_file}"
        if self.mission_active:
            return "mission already active"
        return None

    def select_target(self):
        now = time.time()
        visible_names = set(self.visible_targets)
        target, cooling = select_priority_target(
            self.priority_targets,
            self.visible_targets,
            self.cooldown_until,
            now,
        )
        if target is not None:
            return target

        if visible_names and cooling:
            cooling_text = ", ".join(
                f"{class_name} until {format_epoch(cooldown_until)}"
                for class_name, cooldown_until in cooling
            )
            self.log_wait_reason("visible targets cooling down: " + cooling_text)
        return None

    def launch_mission(self, target):
        command = {
            "target_class_name": target.class_name,
            "selected_camera": target.camera,
            "score": target.score,
            "distance": target.distance,
        }
        self.write_log(
            f"launching target={target.class_name!r} camera={target.camera} "
            f"score={target.score:.3f} distance={target.distance:.3f} "
            f"command_topic={self.args.mission_command_topic}"
        )
        self.active_target = target
        self.last_mission_result = None
        self.active_mission_started = False
        self.mission_active = True
        self.mission_command_sent_at = time.time()

        msg = String()
        msg.data = json.dumps(command, sort_keys=True)
        self.mission_command_pub.publish(msg)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_mission_result is not None:
                result = self.last_mission_result
                self.write_log(
                    f"mission_finished target={target.class_name!r} "
                    f"result={result.get('result_code')} "
                    f"message={result.get('message')!r}"
                )
                self.active_target = None
                self.mission_command_sent_at = None
                return result
            time.sleep(self.args.poll_interval_sec)

        return {
            "result_code": "interrupted",
            "message": "rclpy stopped before mission result",
            "target_class_name": target.class_name,
        }

    def cancel_active_mission(self, reason):
        """Ask the mission controller to stop the active goal."""
        if not self.mission_active:
            return
        msg = String()
        msg.data = reason
        self.mission_cancel_pub.publish(msg)
        self.write_log(f"cancel_requested reason={reason!r}")

    def log_wait_reason(self, reason):
        now = time.time()
        if now - self.last_status_log_sec < self.args.status_log_interval_sec:
            return
        self.last_status_log_sec = now
        visible = ", ".join(sorted(self.visible_targets)) or "none"
        self.write_log(f"waiting reason={reason}; visible={visible}")

    def write_log(self, message):
        line = f"{datetime.now().isoformat(timespec='seconds')} {message}"
        print(line, flush=True)
        self.log_file.write(line + "\n")
        self.log_file.flush()

def format_epoch(epoch_seconds):
    return datetime.fromtimestamp(epoch_seconds).isoformat(timespec="seconds")


def build_parser():
    parser = argparse.ArgumentParser(description="Run deterministic AMR demo missions.")
    parser.add_argument(
        "--priority-targets",
        default=",".join(DEFAULT_PRIORITY_TARGETS),
        help="Comma-separated priority target classes.",
    )
    parser.add_argument("--front-detections-topic", default="/yolo_front/detections_3d")
    parser.add_argument("--left-detections-topic", default="/yolo_left/detections_3d")
    parser.add_argument("--right-detections-topic", default="/yolo_right/detections_3d")
    parser.add_argument("--back-detections-topic", default="/yolo_back/detections_3d")
    parser.add_argument("--target-frame", default="base_link")
    parser.add_argument("--min-visible-score", type=float, default=0.1)
    parser.add_argument("--min-visible-distance", type=float, default=0.2)
    parser.add_argument("--max-visible-distance", type=float, default=20.0)
    parser.add_argument("--visible-detection-timeout-sec", type=float, default=8.0)
    parser.add_argument("--poll-interval-sec", type=float, default=2.0)
    parser.add_argument("--pause-between-missions-sec", type=float, default=30.0)
    parser.add_argument("--target-cooldown-sec", type=float, default=60.0)
    parser.add_argument("--pause-flag-file", default="/tmp/demo_loop_pause")
    parser.add_argument("--mission-command-topic", default="/multi_camera_object_mission/command")
    parser.add_argument("--mission-cancel-topic", default="/multi_camera_object_mission/cancel")
    parser.add_argument("--mission-status-topic", default="/multi_camera_object_mission/status")
    parser.add_argument("--log-file", default="demo_loop_runner.log")
    parser.add_argument("--status-log-interval-sec", type=float, default=10.0)
    return parser


def main():
    args, _ = build_parser().parse_known_args()
    rclpy.init()
    runner = DemoLoopRunner(args)
    try:
        runner.run_forever()
    except KeyboardInterrupt:
        runner.write_log("stopped by KeyboardInterrupt")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
