#!/usr/bin/env bash
# Integrated drone-to-camera acquisition and goal delivery with RC takeover.
set -Eeo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GOAL_SIDE="${1:-left}"
if [[ "$GOAL_SIDE" != left && "$GOAL_SIDE" != right ]]; then
    echo "用法：$0 [left|right]" >&2
    exit 2
fi
# Reject an existing control stack before opening serial ports or creating
# another automatic propulsion publisher. Never terminate it implicitly.
python3 - <<'PY'
from pathlib import Path
import sys
executables = {'goal_navigation_node', 'mission_manager', 'ball_follow_controller',
               'pwm_actuator_node', 'control_mode_arbiter'}
conflicts = []
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():
        continue
    try:
        args = (proc/'cmdline').read_bytes().decode(errors='replace').split('\0')
    except OSError:
        continue
    if any(Path(arg).name in executables for arg in args if arg):
        conflicts.append((proc.name, ' '.join(arg for arg in args if arg)))
if conflicts:
    print('启动已阻止：已有导航/追球/动力控制进程。请在原启动终端按Ctrl+C，确认停止后再启动联合任务。', file=sys.stderr)
    for pid, command in conflicts:
        print(f'PID {pid}: {command}', file=sys.stderr)
    sys.exit(2)
PY
source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/ros2_ws/install/setup.bash"
set -u
cd "$PROJECT_DIR/ros2_ws"
echo "拿球进门任务：CH8低位手动、高位自动；CH5高位急停。目标球门：$GOAL_SIDE"
echo "自动模式下本终端按B主动脱困，无需回车；默认DRY_RUN=true不输出真实动力。"
exec python3 "$PROJECT_DIR/tools/start_monitored_mission.py" \
    --project-dir "$PROJECT_DIR" --goal-side "$GOAL_SIDE" \
    --web-port "${WEB_PORT:-8080}" --dry-run "${DRY_RUN:-true}"
