#!/usr/bin/env bash
# Host launcher; no privileged container and no GPU libraries on the CPU route.
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ACTION="${1:-shell}"
SIDE="${2:-right}"
IMAGE="${USV_CONTAINER_IMAGE:-usv-runtime:nano-cpu}"
if [[ "$SIDE" != left && "$SIDE" != right ]]; then echo '球门必须left或right' >&2; exit 2; fi
if [[ "$ACTION" == image ]]; then
    exec docker build -f "$PROJECT_DIR/deployment/Dockerfile.nano" -t "$IMAGE" "$PROJECT_DIR"
fi
case "$ACTION" in
    build) COMMAND='bash tools/build.sh --executor sequential --parallel-workers 1' ; DRY_RUN=true;;
    shell) COMMAND='exec bash'; DRY_RUN=true;;
    bench) COMMAND="exec bash start_ball_mission.sh $SIDE"; DRY_RUN=true;;
    mission|tracking|navigation)
        if [[ "${USV_ALLOW_MOTION:-}" != YES ]]; then
            echo '真实PWM未授权。先bench；确认硬件安全后使用USV_ALLOW_MOTION=YES。' >&2; exit 2
        fi
        DRY_RUN=false
        case "$ACTION" in
            mission) COMMAND="exec bash start_ball_mission.sh $SIDE";;
            tracking) COMMAND='exec bash start_tracking.sh';;
            navigation) COMMAND="exec bash start_goal_navigation.sh $SIDE";;
        esac;;
    *) echo '用法：run_nano.sh image|build|shell|bench|mission|tracking|navigation [left|right]' >&2; exit 2;;
esac
mkdir -p "$PROJECT_DIR/runtime_logs"
task_devices=()
# Device mount is read/write intentionally; host permissions and hardware still matter.
for task_device in /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS* /dev/i2c-* /dev/video*; do
    if [[ -c "$task_device" ]]; then task_devices+=(--device "$task_device:$task_device"); fi
done
task_tty=()
if [[ -t 0 && -t 1 ]]; then task_tty=(-it); else task_tty=(-i); fi
exec docker run --rm --init --runtime=runc --network host \
    "${task_tty[@]}" "${task_devices[@]}" \
    -e DRY_RUN="$DRY_RUN" -e WEB_PORT="${WEB_PORT:-8080}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" \
    -v "$PROJECT_DIR:/workspace/usv_runtime_portable" \
    -v "$PROJECT_DIR/runtime_logs:/root/Downloads" \
    "$IMAGE" bash -lc "cd /workspace/usv_runtime_portable; $COMMAND"
