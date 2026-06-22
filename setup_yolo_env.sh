#!/usr/bin/env bash

# Source this before starting the AMR demo stack.
# It prepares ROS, ROS_DOMAIN_ID, and uv for yolo_ros.

set -eo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-/workspaces/isaac_ros-dev}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
UV_BIN_DIR="${UV_BIN_DIR:-/home/admin/.local/bin}"

if [[ -d "${UV_BIN_DIR}" ]]; then
  export PATH="${UV_BIN_DIR}:${PATH}"
fi

if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${WORKSPACE_DIR}/install/setup.bash"
  set -u
elif [[ -f "install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source install/setup.bash
  set -u
fi

# Source setup.bash may rewrite PATH, so add uv again afterward.
if [[ -d "${UV_BIN_DIR}" ]]; then
  export PATH="${UV_BIN_DIR}:${PATH}"
fi

export ROS_DOMAIN_ID

if ! command -v uv >/dev/null 2>&1; then
  echo "warning: uv not found. Install it with:" >&2
  echo "  curl -LsSf https://astral.sh/uv/install.sh | sh" >&2
fi

echo "YOLO env ready:"
echo "  WORKSPACE_DIR=${WORKSPACE_DIR}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  uv=$(command -v uv 2>/dev/null || echo missing)"
