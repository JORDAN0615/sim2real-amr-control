# Computex AMR Demo Setup SOP

This SOP is for rebuilding the demo environment when the old containers are gone.

## 0. Assumptions

- Server has NVIDIA GPU driver and Docker runtime ready.
- Isaac Sim is installed on the server.
- AMR ROS workspace is available at:

```bash
/workspaces/isaac_ros-dev
```

- AMR demo repo is available at:

```bash
/workspaces/isaac_ros-dev/apriltag-AMR-0514
```

- AMR skill is available on the base host at:

```bash
/home/sky/skills/amr-demo-operator
```

Replace `server-ip` with the current server IP when needed.

Check server IP:

```bash
hostname -I
```

## 1. Isaac Sim

Open Isaac Sim and load:

```text
yolo_detection_with_agent_control_AMR.usd
```

Example:

```bash
cd ~/isaacsim/5.1.0
./isaac-sim.sh
```

After the USD is loaded, click **Play**.

## 2. LLM Inference

Start any LLM inference engine that provides an OpenAI-compatible API endpoint.
The later Hermes setup only needs a reachable base URL, model name, and API key.

Required endpoint shape:

```text
http://<llm-server-ip>:<port>/v1
```

Verify the endpoint before continuing:

```bash
curl http://127.0.0.1:8000/v1/models
```

If the endpoint is on another machine, replace `127.0.0.1` with that machine's
IP address.

Example using vLLM:

```bash
docker run -d \
  --name llm-openai-server \
  --gpus all \
  --ipc=host \
  --network host \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  vllm/vllm-openai:latest \
  --model <model_name_or_path> \
  --served-model-name <served_model_name> \
  --host 0.0.0.0 \
  --port 8000
```

Check logs for the engine you started. For the vLLM example:

```bash
docker logs -f --tail 100 llm-openai-server
```

Wait until the server is ready, then confirm `/v1/models` responds.

## 3. Start Isaac ROS Dev Container

Use NVIDIA Isaac ROS Common to create and enter the ROS2 GPU container. Do not
use `nvcr.io/nvidia/l4t-base:r35.2.1` for this x86 demo; that image is for
Jetson L4T base workflows, not the Isaac ROS x86 development container.

First-time setup on the host:

```bash
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
```

If the AMR demo repository is not already in the workspace, clone or copy it to:

```text
~/workspaces/isaac_ros-dev/apriltag-AMR-0514
```

Start the Isaac ROS dev container:

```bash
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
./scripts/run_dev.sh
```

The script builds or pulls the correct Isaac ROS development image for the x86
host, mounts the workspace at `/workspaces/isaac_ros-dev`, enables GPU access,
and opens a shell inside the container.

If the container already exists later, enter it directly:

```bash
docker start isaac_ros_dev_container
docker exec -it -u admin isaac_ros_dev_container bash
```

Inside the container, verify the environment:

```bash
nvidia-smi
ros2 --help
```

## 4. Start AMR Demo Stack

Inside `isaac_ros_dev_container`:

```bash
source /workspaces/isaac_ros-dev/use_amr_env.sh
cd /workspaces/isaac_ros-dev/apriltag-AMR-0514
./start_demo.sh
```

If YOLO takes longer to load:

```bash
YOLO_WAIT_SEC=120 ./start_demo.sh
```

Check running processes:

```bash
./amr_control.sh processes
```

Check visible objects:

```bash
./amr_control.sh pause
./amr_control.sh visible
```

Resume autonomous loop:

```bash
./amr_control.sh resume
```

Stop demo:

```bash
./stop_demo.sh
```

## 5. Configure Hermes Provider

On the base host, not inside the sandbox:

```bash
NEMOCLAW_GATEWAY_PORT=8081 nemohermes hermes connect
```

Update OpenAI-compatible provider:

```bash
openshell provider update compatible-endpoint \
  --credential OPENAI_API_KEY=empty \
  --config OPENAI_BASE_URL=http://server-ip:8000/v1
```

Set inference provider:

```bash
nemoclaw inference set \
  --provider compatible-endpoint \
  --model gemma-4-26b-a4b-nvfp4 \
  --sandbox hermes \
  --no-verify
```

Allow sandbox outbound SSH to the server:

```bash
NEMOCLAW_GATEWAY_PORT=8081 openshell policy update hermes \
  --add-endpoint server-ip:22 \
  --binary /usr/bin/ssh \
  --binary /usr/bin/python3 \
  --binary /usr/bin/python3.11 \
  --wait
```

## 6. Install AMR Skill Before Entering Sandbox

Run this on the base host before entering the Hermes sandbox:

```bash
NEMOCLAW_GATEWAY_PORT=8081 nemohermes hermes skill install /home/sky/skills/amr-demo-operator
```

After this, entering the sandbox and starting the agent will expose the AMR skill.

## 7. Enter Hermes Sandbox

```bash
NEMOCLAW_GATEWAY_PORT=8081 nemohermes hermes connect
```

Inside the sandbox, configure SSH alias:

```bash
mkdir -p ~/.ssh
chmod 700 ~/.ssh
cat > ~/.ssh/config <<'EOF'
Host sky-base
HostName server-ip
User sky
ProxyCommand /sandbox/http-connect.py %h %p
StrictHostKeyChecking accept-new
EOF
chmod 600 ~/.ssh/config
```

Test:

```bash
ssh sky-base 'hostname && whoami'
```

If `/sandbox/http-connect.py` does not exist, create it:

```bash
cat > /sandbox/http-connect.py <<'PY'
#!/usr/bin/env python3
import select
import socket
import sys

proxy_host = '10.200.0.1'
proxy_port = 3128
target = f'{sys.argv[1]}:{sys.argv[2]}'

sock = socket.create_connection((proxy_host, proxy_port), timeout=10)
sock.sendall((f'CONNECT {target} HTTP/1.1\r\nHost: {target}\r\n\r\n').encode())

header = b''
while b'\r\n\r\n' not in header:
    chunk = sock.recv(1)
    if not chunk:
        raise SystemExit('proxy closed before CONNECT completed')
    header += chunk

if b' 200 ' not in header.split(b'\r\n', 1)[0]:
    raise SystemExit(header.decode(errors='replace'))

sock.setblocking(False)
stdin = sys.stdin.buffer
stdout = sys.stdout.buffer

while True:
    readable, _, _ = select.select([sock, stdin], [], [])
    if sock in readable:
        data = sock.recv(65536)
        if not data:
            break
        stdout.write(data)
        stdout.flush()
    if stdin in readable:
        data = stdin.read1(65536)
        if not data:
            try:
                sock.shutdown(socket.SHUT_WR)
            except OSError:
                pass
        else:
            sock.sendall(data)
PY
chmod +x /sandbox/http-connect.py
```

## 8. Start Agent

Inside the sandbox:

```bash
hermes
```

First message in a new conversation:

```text
/AMR
```

Useful prompts:

```text
what do you see now?
take me to box
take me to Ladder
turn right 90 degrees
turn left 90 degrees
turn right 180 degrees
resume loop
```

Use `take me to {object}` only when the object is visible.

If the agent context window is near full:

```text
/compress
```

Or start a new conversation:

```text
/new
```

## 9. Play Loop Demo Video

Open VLC on the desktop:

```text
Menu -> VLC media player -> Media -> Desktop -> Computex -> Computex_DEMO.mov
```

Enable repeat loop in VLC.

## 10. RViz2 Optional Setup

Inside `isaac_ros_dev_container`, if `rviz2` is missing:

```bash
sudo apt-get update
sudo apt-get install -y ros-jazzy-rviz2
source /opt/ros/jazzy/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export ROS_DOMAIN_ID=23
rviz2
```

If RViz2 fails with:

```text
qt.qpa.xcb: could not connect to display
```

Then the container does not have access to the graphical display. Use the server desktop session, AnyDesk, or restart the container with valid `DISPLAY` and `/tmp/.X11-unix` mounts.

## 11. Quick Recovery Commands

Stop AMR motion:

```bash
cd /workspaces/isaac_ros-dev/apriltag-AMR-0514
./amr_control.sh stop
```

Pause loop:

```bash
./amr_control.sh pause
```

Resume loop:

```bash
./amr_control.sh resume
```

Restart full demo stack:

```bash
./stop_demo.sh
YOLO_WAIT_SEC=120 ./start_demo.sh
```

Check YOLO services:

```bash
ros2 service list | grep set_classes
```

Check YOLO topics:

```bash
ros2 topic list | grep yolo
```

Check command velocity:

```bash
ros2 topic echo /cmd_vel
```
