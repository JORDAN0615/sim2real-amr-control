#!/usr/bin/env python3
"""Deterministic scheduler for the multi-camera AMR demo loop."""

from dataclasses import dataclass
from datetime import datetime
import argparse
import math
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray


DEFAULT_PRIORITY_TARGETS = [
    "person",
    "traffic cone",
    "grey barrel",
    "blue barrel",
]


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
        self.priority_targets = parse_targets(args.priority_targets)
        self.cooldown_until = {}
        self.visible_targets = {}
        self.last_status_log_sec = 0.0

        self.log_file = open(args.log_file, "a", encoding="utf-8")

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
        ]

        self.write_log(
            "started "
            f"priority_targets={self.priority_targets} "
            f"pause_flag={args.pause_flag_file}"
        )

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
                distance=math.hypot(float(position.x), float(position.y)),
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
        distance = math.hypot(float(position.x), float(position.y))
        return self.args.min_visible_distance <= distance <= self.args.max_visible_distance

    def run_forever(self):
        """Main scheduler loop."""
        try:
            while rclpy.ok():
                rclpy.spin_once() (self, timeout_sec=0.1)
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
                
                exit_code = self.launch_mission(target)
                if self.args.target_cooldown_sec > 0.0:
                    self.cooldown_until[target.class_name] = (
                        time.time() + self.args.target_cooldown_sec
                    )
                    self.write_log(
                        f"cooldown target={target.class_name!r} "
                        f"until={format_epoch(self.cooldown_until[target.class_name])} "
                        f"mission_exit_code={exit_code}"
                    )
                else:
                    self.write_log(
                        f"cooldown disabled target={target.class_name!r} "
                        f"mission_exit_code={exit_code}"
                    )
                time.sleep(self.args.pause_between_missions_sec)
        finally:
            self.log_file.close()
    # 
    def prune_stale_targets(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        stale = [
            class_name
            for class_name, target in self.visible_targets.items()
            if now_sec - target.received_at_sec > self.args.visible_detection_timeout_sec
        ]
        for class_name in stale:
            del self.visible_targets[class_name]

    def next_wait_reason(self):
        # 如果 pause_flag_file 存在，則返回 pause flag present: /tmp/demo_loop_pause
        if os.path.exists(self.args.pause_flag_file):
            return f"pause flag present: {self.args.pause_flag_file}"
        # 如果 external_mission_running() 為 True，則返回 mission process already running
        if external_mission_running():
            return "mission process already running"
        return None

    def select_target(self):
        now = time.time()
        visible_names = set(self.visible_targets)
        cooling = []
        # 如果 class_name 不在 visible_names 中，則跳過
        for class_name in self.priority_targets:
            if class_name not in visible_names:
                continue
            cooldown_until = self.cooldown_until.get(class_name, 0.0)
            # 如果 cooldown_until 大於 now，則跳過
            if cooldown_until > now:
                cooling.append(f"{class_name} until {format_epoch(cooldown_until)}")
                continue
            # 回傳目前 class_name 的 VisibleTarget
            return self.visible_targets[class_name]

        if visible_names and cooling:
            self.log_wait_reason("visible targets cooling down: " + ", ".join(cooling))
        return None

    def launch_mission(self, target):
        command = [
            sys.executable,
            self.args.mission_script,
            "--ros-args",
            "-p",
            f"target_class_name:={target.class_name}",
        ]
        self.write_log(
            f"launching target={target.class_name!r} camera={target.camera} "
            f"score={target.score:.3f} distance={target.distance:.3f} "
            f"command={' '.join(command)}"
        )
        # 用目前 Python interpreter 執行 multi_camera_object_mission.py
        process = subprocess.Popen(
            command,
            # 把 child 的輸出接回來
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self.write_log(f"mission_pid={process.pid}")
        terminal_seen = False
        assert process.stdout is not None
        for line in process.stdout:
            print(line, end="", flush=True)
            # 看到 success: 或 failed: scheduler 就知道這個 mission 已經到 terminal stage
            if "success:" in line or "failed:" in line:
                terminal_seen = True
                break
        # 當terminal_seen 為 True 且 process.poll() is None 等 2 秒後強制關掉 process
        if terminal_seen and process.poll() is None:
            try:
                process.wait(timeout=self.args.mission_exit_grace_sec)
            except subprocess.TimeoutExpired:
                self.write_log(
                    f"mission_terminal_log_seen pid={process.pid}; terminating stuck process"
                )
                process.terminate()
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self.write_log(f"mission_pid={process.pid} did not terminate; killing")
                    process.kill()
                    process.wait()
        else:
            process.wait()
        # 取得 process 的 exit code
        exit_code = process.returncode
        self.write_log(f"mission_finished target={target.class_name!r} exit_code={exit_code}")
        return exit_code

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

# 轉成 Python list, ["person", "traffic cone", "grey barrel", "blue barrel"]
def parse_targets(raw_targets):
    if isinstance(raw_targets, list):
        return raw_targets
    targets = [target.strip() for target in raw_targets.split(",") if target.strip()]
    return targets or DEFAULT_PRIORITY_TARGETS


def external_mission_running():
    current_pid = os.getpid()
    try:
        result = subprocess.run(
            ["pgrep", "-af", "multi_camera_object_mission.py"],
            check=False,
            text=True,
            capture_output=True,
        )
    except FileNotFoundError:
        return False

    for line in result.stdout.splitlines():
        parts = line.split(maxsplit=1)
        if not parts:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        if pid != current_pid:
            return True
    return False


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
    parser.add_argument("--target-frame", default="base_link")
    parser.add_argument("--min-visible-score", type=float, default=0.1)
    parser.add_argument("--min-visible-distance", type=float, default=0.2)
    parser.add_argument("--max-visible-distance", type=float, default=20.0)
    parser.add_argument("--visible-detection-timeout-sec", type=float, default=8.0)
    parser.add_argument("--poll-interval-sec", type=float, default=2.0)
    parser.add_argument("--pause-between-missions-sec", type=float, default=30.0)
    parser.add_argument("--target-cooldown-sec", type=float, default=0.0)
    parser.add_argument("--pause-flag-file", default="/tmp/demo_loop_pause")
    parser.add_argument("--mission-script", default="multi_camera_object_mission.py")
    parser.add_argument("--mission-exit-grace-sec", type=float, default=2.0)
    parser.add_argument("--log-file", default="demo_loop_runner.log")
    parser.add_argument("--status-log-interval-sec", type=float, default=10.0)
    return parser


def main():
    args = build_parser().parse_args()
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
