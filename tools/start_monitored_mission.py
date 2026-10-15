#!/usr/bin/env python3
"""Prepare diagnostic logging, then exec the original ROS launch (same PID/signals)."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

from capture_runtime_health import load_config, parent_identity, read


def start(args):
    project = Path(args.project_dir).expanduser().resolve()
    config_path = project/'tools/runtime_monitor.yaml'
    config = load_config(config_path)
    output = Path(config['log_directory']).expanduser()
    if not output.is_absolute():
        output = project/output
    output = output/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    command = ['ros2', 'launch', 'usv_mission_manager', 'mission.launch.py',
               f'goal_side:={args.goal_side}', f'web_port:={args.web_port}', f'dry_run:={args.dry_run}']
    env = os.environ.copy()
    # Persist every node's stdout/stderr, including automatic faulthandler dumps.
    # Do not put the control launch behind a tee pipe or a Python supervisor.
    env.update(ROS_LOG_DIR=str(output/'ros'), OVERRIDE_LAUNCH_PROCESS_OUTPUT='both', PYTHONUNBUFFERED='1')
    metadata = dict(unix_ns=time.time_ns(), monotonic=time.monotonic(),
                    hostname=socket.gethostname(), boot_id=read(Path('/proc/sys/kernel/random/boot_id')),
                    launch_pid=os.getpid(), launch_start_ticks=parent_identity(os.getpid()),
                    project=str(project), command=command, monitor_config=config,
                    ros_environment={k: env.get(k) for k in
                                     ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION', 'ROS_LOG_DIR')},
                    control_log_roots=['~/Downloads/usv_mission_logs', '~/Downloads/usv_navigation_logs',
                                       '~/Downloads/usv_logs'])
    (output/'session.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2))
    # Diagnostic config is live as well: changes need no colcon build.
    (output/'runtime_monitor.yaml').write_text(config_path.read_text())
    with (output/'monitor.log').open('a', buffering=1) as monitor_log:
        monitor = subprocess.Popen([sys.executable, str(project/'tools/capture_runtime_health.py'),
                                    '--duration', '0', '--watch-parent', str(os.getpid()),
                                    '--output-directory', str(output), '--web-port', str(args.web_port),
                                    '--config', str(config_path)],
                                   stdout=monitor_log, stderr=monitor_log, start_new_session=True)
    keyboard = None
    try:
        deadline = time.monotonic()+5.
        while not (output/'ready.json').exists():
            if monitor.poll() is not None:
                raise RuntimeError(f'系统监控启动失败，请查看 {output}/monitor.log；控制节点尚未启动。')
            if time.monotonic() >= deadline:
                raise RuntimeError(f'系统监控启动超时，请查看 {output}/monitor.log；控制节点尚未启动。')
            time.sleep(.05)
        if monitor.poll() is not None:
            raise RuntimeError(f'系统监控已退出，请查看 {output}/monitor.log；控制节点尚未启动。')
        print(f'本次系统级日志：{output}', flush=True)
        print('系统采样独立运行；完整ROS终端输出保存在该目录的 ros/ 下。', flush=True)
        os.chdir(project/'ros2_ws')
        if sys.stdin.isatty():
            keyboard = subprocess.Popen(
                [sys.executable, str(project/'tools/mission_escape_keyboard.py'),
                 '--watch-parent', str(os.getpid())], env=env)
        os.execvpe(command[0], command, env)
    except BaseException:
        if keyboard is not None and keyboard.poll() is None:
            keyboard.terminate()
            keyboard.wait(timeout=3.)
        if monitor.poll() is None:
            monitor.terminate()  # only our sampler, no ROS nodes have been started by us
            try:
                monitor.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                monitor.kill()
                monitor.wait(timeout=1.)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-dir', required=True)
    parser.add_argument('--goal-side', choices=('left', 'right'), default='left')
    parser.add_argument('--web-port', type=int, default=8080)
    parser.add_argument('--dry-run', choices=('true', 'false'), default='true')
    args = parser.parse_args()
    if not 1 <= args.web_port <= 65535:
        parser.error('web port must be in 1..65535')
    start(args)


if __name__ == '__main__':
    main()
