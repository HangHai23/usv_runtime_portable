#!/usr/bin/env bash
# Formal RC-controlled modes; no calibration, simulation or capture helpers.
set -Eeo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-}"
SIDE="${2:-right}"
if [[ "$MODE" != tracking && "$MODE" != navigation ]] || [[ "$SIDE" != left && "$SIDE" != right ]]; then
    echo "用法：$0 tracking|navigation [left|right]" >&2; exit 2
fi
# Same collision guard as integrated startup, without killing existing nodes.
python3 - <<'PY'
from pathlib import Path
names={'goal_navigation_node','mission_manager','ball_follow_controller','pwm_actuator_node','control_mode_arbiter'}
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:args=(proc/'cmdline').read_bytes().decode(errors='replace').split('\0')
    except OSError:continue
    if any(Path(a).name in names for a in args if a):
        raise SystemExit(f'启动拒绝：已有控制进程PID {proc.name}，请先在原终端Ctrl+C。')
PY
source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/ros2_ws/install/setup.bash"
set -u
cd "$PROJECT_DIR/ros2_ws"
exec ros2 launch usv_mission_manager standalone.launch.py mode:="$MODE" goal_side:="$SIDE" \
    dry_run:="${DRY_RUN:-true}" web_port:="${WEB_PORT:-8080}"
