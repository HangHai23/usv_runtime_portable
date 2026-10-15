#!/usr/bin/env python3
"""B-key service client; never publishes propulsion or changes control mode."""
import argparse
import os
import select
import signal
import sys
import termios
import time
import tty

import rclpy
from std_srvs.srv import Trigger

from capture_runtime_health import parent_identity


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--watch-parent', type=int, required=True)
    args = parser.parse_args()
    if not sys.stdin.isatty():
        print('无交互终端，B键脱困未启用。', flush=True)
        return
    identity = parent_identity(args.watch_parent)
    fd = sys.stdin.fileno()
    previous = termios.tcgetattr(fd)
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    rclpy.init()
    node = rclpy.create_node('mission_escape_keyboard')
    client = node.create_client(Trigger, '/mission/escape_boundary')
    pending = None
    last_press = -float('inf')
    try:
        tty.setcbreak(fd)  # Keep Ctrl+C as a terminal signal, no Enter required.
        print('B键脱困已启用：自动模式下按B；CH5急停、CH8手动接管优先。', flush=True)
        while running and rclpy.ok():
            if identity is None or parent_identity(args.watch_parent) != identity:
                break
            rclpy.spin_once(node, timeout_sec=0.)
            if pending is not None and pending.done():
                try:
                    result = pending.result()
                    print(f'脱困请求：{result.message}', flush=True)
                except Exception as exc:
                    print(f'脱困请求失败：{exc}', flush=True)
                pending = None
            readable, _, _ = select.select([fd], [], [], .1)
            if not readable:
                continue
            key = os.read(fd, 1)
            if not key:
                break
            if key not in (b'b', b'B'):
                continue
            now = time.monotonic()
            if pending is not None or now-last_press < 1.:
                continue
            last_press = now
            if not client.service_is_ready():
                print('脱困服务尚未就绪，本次按键未执行、不会排队执行。', flush=True)
                continue
            pending = client.call_async(Trigger.Request())
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, previous)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
