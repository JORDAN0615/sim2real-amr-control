# Project TODO

## Current Status

- 已完成 RealSense + AprilTag prototype，能取得相機與 tag 的相對位置。
- 已能透過 `pose_t` 取得 tag 在 camera frame 內的 `x / y / z`。
- 已確認 Isaac ROS AprilTag 比較適合未來 Isaac Sim 與 Thor 實機部署路線。
- 在 Isaac Sim 中跑通 `isaac_ros_apriltag_isaac_sim_pipeline.launch.py`。
- 確認 Isaac Sim camera topic、`camera_info`、`/tag_detections`、`/tf` 都正常發布。
- 確認 openclaw / AMR 在 Isaac Sim 中接收 `/cmd_vel` 的 topic / namespace。
- 已完成 `ros2_move_to_tag.py` 一次性任務 node prototype，可用 `/tag_detections` 或 TF 控制 AMR 靠近指定 tag。
- 已用 detections mode 實測 AMR 可靠近 `tag_id=0` forklift，並在 `stop_distance=3.0m` 成功停車。
- 已建立專案內 agent skill：`skills/openclaw-amr-control/SKILL.md`，固定 persistent TTY 連線與 motion command pipeline。
- 已在 Isaac Sim 開啟 depth topic：`/front_stereo_camera/left/depth`。
- 已安裝並 build `yolo_ros`，確認 `/yolo/detections_3d` 可輸出物體 `bbox3d.center.position`。
- 已新增 `ros2_move_to_object.py` prototype，準備用 YOLO 3D detection 控制 AMR 靠近指定 `class_name`。
- 已建立 YOLO-World agent skill：`skills/yolo-world-amr-control/SKILL.md`，記錄 launch、set_classes、move_to_object 參數與安全流程。

## Goal

目標是建立一個 OpenClaw(NemoClaw) agent 控制 AMR 靠近 AprilTag 的 demo。OpenClaw agent 負責決定任務與參數，例如要靠近哪個 tag、速度多少、停在 tag 前多遠；底層 ROS tool / executor node 負責高頻率讀取 AprilTag pose 並穩定發布 `/cmd_vel`。

這個設計不是 Nav2，也不是完整 autonomous planner。它是 agent-driven motion demo：OpenClaw 是決策者，ROS executor 是動作執行器。

## Target Architecture

```text
Isaac Sim camera
  -> image_rect_color
  -> camera_info

isaac_ros_apriltag
  -> /tag_detections
  -> /tf: camera_frame -> tag36h11:<id>

OpenClaw agent
  -> get_tag_pose(tag_id)
  -> move_to_tag(tag_id, speed, stop_distance)
  -> move_forward_for_duration(speed, duration)

ROS motion executor
  -> read TF or /tag_detections
  -> calculate cmd_vel
  -> publish /cmd_vel at fixed rate
  -> enforce timeout / stop / safety rules

AMR in Isaac Sim
  -> subscribe /cmd_vel
  -> move in simulation
```

## Design Decision

- Agent 不直接承擔高頻率 `/cmd_vel` loop。
- Agent 透過明確 function / ROS service / tool 觸發移動行為。
- 底層 executor node 負責固定頻率 publish `/cmd_vel`。
- 第一版已用一次性 `ros2_move_to_tag.py` 驗證 `move_to_tag()` 控制 loop。
- 下一階段要把一次性 script 升級成常駐 ROS service/action server，讓 agent 可多次呼叫，不必每次重新啟動 Python process。

## Agent Tool Interface

### `get_tag_pose(tag_id)`

用途：讓 OpenClaw agent 查詢目前指定 AprilTag 的相對位置。

輸入：

```text
tag_id: int
reference_frame: base_link or camera_frame
```

輸出：

```text
found: bool
x: float
y: float
z: float
distance: float
timestamp: time
```

建議優先讀 TF：

```text
base_link -> tag36h11:<id>
```

如果 TF 尚未整理好，第一版可先讀：

```text
camera_frame -> tag36h11:<id>
```

### `move_to_tag(tag_id, speed, stop_distance, timeout)`

用途：OpenClaw agent 呼叫這個 tool 後，executor node 持續控制 AMR 靠近 tag。

輸入：

```text
tag_id: int
speed: float
stop_distance: float
timeout: float
```

行為：

```text
1. 持續讀 tag pose。
2. 用左右偏差計算 heading correction。
3. 用前方距離計算是否繼續前進。
4. 以固定 rate 發布 /cmd_vel。
5. 到達 stop_distance 後停車並回報 success。
6. 若 tag lost、timeout、topic error，立即停車並回報 failed。
```

### `move_forward_for_duration(speed, duration)`

用途：展示 agent 以公式直接計算移動時間的 baseline。

公式：

```text
duration = max(tag_distance - stop_distance, 0) / speed
```

限制：

```text
duration 必須有上限。
執行期間 executor 仍需負責固定頻率 publish /cmd_vel。
執行結束必須 publish zero cmd_vel。
建議每次只執行短時間段，完成後重新讀 tag pose。
```

## First-Pass Control Strategy

第一版不要直接一次算完整秒數盲走到 tag。建議使用閉迴路 `move_to_tag()`：

```text
1. agent 呼叫 get_tag_pose(tag_id)。
2. agent 確認 tag 存在且距離合理。
3. agent 呼叫 move_to_tag(tag_id, speed=0.15, stop_distance=0.4, timeout=10.0)。
4. executor node 以 20 Hz 左右讀 TF / tag pose。
5. 若 tag 在畫面左右偏移，先轉向或邊走邊小幅修正。
6. 若 tag 前方距離大於 stop_distance，發布 linear.x。
7. 若距離小於 stop threshold，停車。
8. agent 再次呼叫 get_tag_pose(tag_id) 驗證結果。
```

建議初始參數：

```text
speed: 0.10 - 0.15 m/s
stop_distance: 0.40 m
control_rate: 20 Hz
tag_timeout: 0.3 s
max_angular_z: 0.4 rad/s
distance_tolerance: 0.03 - 0.05 m
angle_tolerance: 3 - 5 deg
```

## Isaac Sim Setup TODO

- 下載並解壓 Isaac ROS AprilTag quickstart assets。
- 確認 `isaac_ros_apriltag` repo 為 `release-3.2`。
- build `isaac_ros_apriltag` 與相關 interface。
- 啟動 Isaac Sim 教學場景。
- 啟動 `isaac_ros_apriltag_isaac_sim_pipeline.launch.py`。
- 確認 Isaac Sim camera 發布的 topic 名稱。
- 若 topic 名稱不同，修改 launch remapping。
- 確認 tag size 與場景內 AprilTag 實際尺寸一致。

## ROS Verification TODO

- `ros2 topic list` 確認 camera topics。
- `ros2 topic echo /tag_detections` 確認 tag detection。
- `ros2 run tf2_ros tf2_echo <camera_frame> tag36h11:<tag_id>` 確認 tag pose。
- 確認 AMR 訂閱的 `/cmd_vel` topic。
- 手動 publish `/cmd_vel` 驗證 AMR 會移動。
- 確認 `/cmd_vel` frame convention 與 AMR driver 一致。

## Motion Executor TODO

- 建立最小 ROS node prototype：`ros2_move_to_tag.py`。
- 實作第一版 `move_to_tag(tag_id, speed, stop_distance, timeout)` 控制 loop。
- 加入 tag lost timeout。
- 加入最大速度、最大角速度、最大執行時間限制。
- 每次移動結束都 publish zero `/cmd_vel`。
- 在 code 中補上資料流與 function 註解，說明 `/tag_detections` / TF 如何轉成 `linear.x` / `angular.z`。
- TODO: 實作 `get_tag_pose(tag_id)` 查詢功能。
- TODO: 實作 `move_forward_for_duration(speed, duration)` 作為 direct mode。
- TODO: 記錄每次 agent command、tag pose、cmd_vel、結果狀態。

## YOLO 3D Object Control TODO

目標：使用 `yolo_ros` 的 `/yolo/detections_3d`，讓 AMR 可以靠近 YOLO-World 指定的物體類別。

目前資料格式：

```text
/yolo/detections_3d
  detections[]
    class_name
    score
    bbox3d.center.position.x/y/z
    bbox3d.frame_id
```

設計：

```text
target_class_name -> filter matching detection
bbox3d.center.position.x -> forward
bbox3d.center.position.y -> left
compute_cmd(forward, left) -> /cmd_vel
```

已完成：

- 建立 `ros2_move_to_object.py` 獨立檔案，不修改既有 AprilTag 控制器。
- 參數包含 `target_class_name`、`detections_topic`、`target_frame`、`speed`、`stop_distance`、`detection_timeout_sec`、`debug`。
- 第一版假設 demo 場景中每個 class 只有一個物件，因此不做 multi-object selection。

待測：

- 將 `ros2_move_to_object.py` scp 到 Isaac ROS workspace。
- 在已 source `yolo_ros` 的 container 中執行。
- 先測 `target_class_name:="yellow forklift"`。
- 確認 AMR 能以 `/yolo/detections_3d` 的 `bbox3d` 位置靠近物體。

範例命令：

```bash
python3 ros2_move_to_object.py --ros-args \
  -p target_class_name:="yellow forklift" \
  -p detections_topic:=/yolo/detections_3d \
  -p target_frame:=base_link \
  -p cmd_vel_topic:=/cmd_vel \
  -p speed:=0.50 \
  -p stop_distance:=3.0 \
  -p detection_timeout_sec:=2.0 \
  -p debug:=true
```

## Persistent Motion Server TODO

下一階段目標：將目前一次性 `ros2_move_to_tag.py` 改成常駐 ROS service 或 action server。

動機：

```text
現在：
agent 每次要移動都重新執行 python3 ros2_move_to_tag.py
任務成功後 node shutdown

下一版：
motion_server 常駐在 ROS container
agent 呼叫 /move_to_tag service/action
server 執行控制 loop
完成後回傳 success/failed
server 保持啟動，等待下一次呼叫
```

TODO:

- 建立 `openclaw_apriltag_control` ROS 2 package。
- 將 `compute_cmd()`、detections mode、TF mode 抽成可重用 controller class。
- 建立常駐 `motion_server` node。
- 定義 `/move_to_tag` service 或 action interface，輸入 `tag_id`、`speed`、`stop_distance`、`timeout`、`pose_source`。
- 回傳 `success`、`message`、final `forward`、final `left`、執行時間。
- 加入 `/stop_motion` service，讓 agent 或人可以立即停止 AMR。
- 加入 `/get_tag_pose` service，讓 agent 可先查目標位置再決策。
- 保持底層控制 loop 在 server 內固定頻率執行，不讓 agent 直接高頻發布 `/cmd_vel`。
- 更新 `skills/openclaw-amr-control/SKILL.md`，把預設 motion command 從直接跑 Python 改成呼叫 service/action。

## Prototype Command

前置檢查：

```bash
ros2 run tf2_ros tf2_echo base_link tag36h11:0
ros2 topic echo /cmd_vel
```

如果 TF 可以看到 tag，先用最小版控制器測試：

```bash
python3 ros2_move_to_tag.py --ros-args \
  -p pose_source:=tf \
  -p tag_id:=0 \
  -p target_frame:=base_link \
  -p cmd_vel_topic:=/cmd_vel \
  -p speed:=0.15 \
  -p stop_distance:=0.40 \
  -p mission_timeout_sec:=10.0
```

如果目前只有 camera frame 到 tag 的 TF，先把 `target_frame` 改成實際 camera frame：

```bash
python3 ros2_move_to_tag.py --ros-args \
  -p tag_id:=0 \
  -p target_frame:=<camera_frame> \
  -p cmd_vel_topic:=/cmd_vel
```

注意：正式控制 AMR 時仍建議使用 `base_link -> tag36h11:<id>`，避免 camera mounting angle 影響 `/cmd_vel` 方向。

如果 `/tag_detections` 已經有 pose，但 TF 尚未接到 `base_link`，可以先用 detections mode 測試：

```bash
python3 ros2_move_to_tag.py --ros-args \
  -p pose_source:=detections \
  -p detections_topic:=/tag_detections \
  -p tag_id:=0 \
  -p cmd_vel_topic:=/cmd_vel \
  -p speed:=0.15 \
  -p stop_distance:=0.40 \
  -p mission_timeout_sec:=10.0
```

detections mode 使用 camera optical frame：

```text
optical z -> robot forward distance
-optical x -> robot left/right correction
```

這個模式適合先驗證 Isaac Sim AprilTag 到 `/cmd_vel` 的資料流；正式版仍建議補齊 camera frame 到 `base_link` 的 TF 後改回 TF mode。

## OpenClaw Agent TODO

- 已建立專案內 skill：`skills/openclaw-amr-control/SKILL.md`。
- 已在 skill 中規定使用 persistent interactive TTY session，不使用 one-shot SSH/docker exec 執行長時間 motion。
- 已在 skill 中規定未被要求診斷時直接執行 `ros2_move_to_tag.py`，topic echo 類檢查使用 `--once`。
- 定義 agent 可以呼叫的 tool schema。
- 限制 agent 只能呼叫安全 tool，而不是任意 shell command 發 `/cmd_vel`。
- 讓 agent 先查詢 tag pose，再決定是否移動。
- 讓 agent 可以調整 `tag_id`、`speed`、`stop_distance`、`timeout`。
- 讓 agent 在移動後重新查詢 tag pose 並回報結果。
- 定義失敗時的行為，例如 tag lost、timeout、距離異常、AMR 未移動。

## Future Real-World Deployment

- 實機相機改用 RealSense ROS driver 或其他 ROS camera driver。
- 確認實機會發布 `sensor_msgs/Image` 與 `sensor_msgs/CameraInfo`。
- 將 Isaac Sim camera topic remapping 改成實機 camera topic remapping。
- 使用實機 camera calibration，避免 `camera_info` 錯誤造成 pose 偏差。
- 確認 camera frame 到 `base_link` 的 TF。
- 在 Thor / Jetson Thor 上驗證 Isaac ROS AprilTag performance。
- 加入更完整的安全停止條件與手動 emergency stop。

## Suggested Next Steps

- 將 `ros2_move_to_tag.py` 包成 ROS 2 package。
- 實作常駐 `motion_server` service/action。
- 補 `/get_tag_pose` 與 `/stop_motion` service。
- 將 OpenClaw agent 從「執行 Python 指令」改成「呼叫 ROS service/action」。
- 保留目前一次性 Python script 作為 debug / fallback tool。

## TF_Chain

base_link
-> camera_link
-> camera_color_optical_frame (realsense D435i)
-> apriltag
