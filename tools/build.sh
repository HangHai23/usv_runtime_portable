#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${PROJECT_DIR}/ros2_ws"

source /opt/ros/humble/setup.bash
cd "${WORKSPACE}"
colcon build --symlink-install "$@"

echo
echo "Build complete. Run:"
echo "  source ${WORKSPACE}/install/setup.bash"
