#!/usr/bin/env python3
"""Independent, read-only stall evidence collector; never publishes or signals ROS nodes."""
import argparse
import copy
from datetime import datetime
import faulthandler
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import urllib.request

NAMES = {'imu_node', 'udp_position_node', 'camera_node', 'yolo_detector_node',
         'target_state_estimator', 'ball_follow_controller', 'goal_navigation_node',
         'mission_manager', 'web_monitor', 'crsf_receiver_node', 'differential_mixer',
         'control_mode_arbiter', 'pwm_actuator_node', 'telemetry_recorder'}
DEFAULTS = dict(sample_interval_sec=1., process_detail_interval_sec=5.,
                web_timeout_sec=.5, startup_grace_sec=10.,
                system_cpu_warning_percent=95., memory_available_warning_mb=512.,
                io_pressure_warning_percent=20., process_blocked_warning_sec=2.,
                web_failure_warning_count=3, ros_loop_warning_ms=1000.,
                sample_gap_warning_sec=2.5, kernel_logging_enabled=True)


def read(path):
    try:
        return path.read_text().strip()
    except (OSError, UnicodeError):
        return None


def key_values(raw):
    values = {}
    for line in (raw or '').splitlines():
        try:
            key, value = line.split(':', 1)
            values[key] = int(value.strip().split()[0])
        except (ValueError, IndexError):
            continue
    return values


def parse_pressure(raw):
    result = {}
    for line in (raw or '').splitlines():
        fields = line.split()
        try:
            result[fields[0]] = {key: float(value) for key, value in
                                 (field.split('=', 1) for field in fields[1:])}
        except (ValueError, IndexError):
            continue
    return result


def process_stat(raw):
    if not raw:
        return None
    try:
        fields = raw[raw.rfind(')') + 2:].split()
        return dict(state=fields[0], ppid=int(fields[1]),
                    cpu_ticks=int(fields[11]) + int(fields[12]),
                    threads=int(fields[17]), start_ticks=int(fields[19]),
                    rss_pages=int(fields[21]))
    except (IndexError, ValueError):
        return None


def parent_identity(pid):
    stat = process_stat(read(Path('/proc') / str(pid) / 'stat'))
    return stat['start_ticks'] if stat and stat['state'] != 'Z' else None


def is_descendant(pid, parent, stats):
    seen = set()
    while pid in stats and pid not in seen:
        if pid == parent:
            return True
        seen.add(pid)
        pid = stats[pid]['ppid']
    return False


def load_config(path=None):
    config = dict(DEFAULTS)
    if path:
        import yaml
        with Path(path).open() as source:
            supplied = yaml.safe_load(source) or {}
        if not isinstance(supplied, dict):
            raise ValueError('monitor config must be a mapping')
        config.update(supplied)
    for key in DEFAULTS:
        value = config[key]
        if key == 'kernel_logging_enabled':
            if not isinstance(value, bool):
                raise ValueError(f'{key} must be true/false')
        elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be finite and positive')
    if config['sample_interval_sec'] < .2 or config['web_timeout_sec'] > 5:
        raise ValueError('sample interval must be >=0.2s; web timeout must be <=5s')
    return config


class WebProbe:
    """HTTP never blocks the /proc sampler; bypass LAN/system proxy settings."""
    def __init__(self, url, timeout, stop):
        self.url, self.timeout, self.stop = url, timeout, stop
        self.lock = threading.Lock()
        self.result = None
        self.thread = threading.Thread(target=self.run, daemon=True, name='web-health-probe')

    def run(self):
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        failures = 0
        while not self.stop.is_set():
            started = time.monotonic()
            try:
                with opener.open(self.url, timeout=self.timeout) as response:
                    raw = response.read(65537)
                    if len(raw) > 65536:
                        raise ValueError('health response too large')
                    health = json.loads(raw)
                    if not isinstance(health, dict):
                        raise ValueError('health response is not an object')
                    # Reject non-finite values before passing them to the strict logger.
                    json.dumps(health, allow_nan=False)
                failures = 0
                result = dict(ok=True, health=health)
            except (OSError, ValueError, TimeoutError) as error:
                failures += 1
                result = dict(ok=False, error=str(error))
            result.update(monotonic=time.monotonic(), latency_ms=(time.monotonic()-started)*1000,
                          consecutive_failures=failures)
            with self.lock:
                self.result = result
            self.stop.wait(max(.01, 1.-(time.monotonic()-started)))

    def snapshot(self):
        with self.lock:
            result = copy.deepcopy(self.result)
        if result:
            result['probe_age_sec'] = time.monotonic()-result['monotonic']
        return result


class Sampler:
    def __init__(self, parent=None, proc_root=Path('/proc')):
        self.parent, self.proc_root = parent, Path(proc_root)
        self.ticks, self.page = os.sysconf('SC_CLK_TCK'), os.sysconf('SC_PAGE_SIZE')
        self.previous_cpu, self.previous_processes = {}, {}
        self.details, self.last_details, self.log_paths = {}, float('-inf'), set()

    def snapshot(self, detail_interval=5.):
        now = time.monotonic()
        cpu = {}
        for line in (read(self.proc_root/'stat') or '').splitlines():
            fields = line.split()
            if not fields or not fields[0].startswith('cpu'):
                continue
            counters = [int(value) for value in fields[1:9]]  # exclude double-counted guest time
            if len(counters) < 5:
                continue
            total, idle, wait = sum(counters), counters[3], counters[4]
            old = self.previous_cpu.get(fields[0])
            delta = total-old[0] if old else 0
            cpu[fields[0]] = dict(busy_percent=100.*(delta-idle+old[1]-wait+old[2])/delta if delta > 0 else None,
                                 iowait_percent=100.*(wait-old[2])/delta if delta > 0 else None)
            self.previous_cpu[fields[0]] = (total, idle, wait)
        stats = {}
        for root in self.proc_root.iterdir():
            if root.name.isdigit():
                stat = process_stat(read(root/'stat'))
                if stat:
                    stats[int(root.name)] = stat
        processes, previous = [], {}
        collect_details = now-self.last_details >= detail_interval
        if collect_details:
            self.last_details = now
        for pid, stat in sorted(stats.items()):
            if self.parent and not is_descendant(pid, self.parent, stats):
                continue
            root = self.proc_root/str(pid)
            try:
                args = (root/'cmdline').read_bytes().decode(errors='replace').split('\0')
            except OSError:
                continue
            name = ('ros2_launch' if pid == self.parent else
                    next((Path(arg).name for arg in args if arg and Path(arg).name in NAMES), None))
            if not name:
                continue
            key = (pid, stat['start_ticks'])  # PID reuse must not produce false CPU/IO rates
            io = key_values(read(root/'io'))
            old = self.previous_processes.get(key)
            elapsed = now-old[0] if old else 0
            record = dict(stat, pid=pid, name=name, rss_mb=stat['rss_pages']*self.page/1048576,
                          wchan=read(root/'wchan'), io=io,
                          cpu_percent=100.*(stat['cpu_ticks']-old[1])/self.ticks/elapsed if elapsed > 0 else None,
                          io_bytes_per_sec={k: (io[k]-old[2][k])/elapsed for k in
                                            ('read_bytes', 'write_bytes', 'rchar', 'wchar')
                                            if elapsed > 0 and k in io and k in old[2]})
            previous[key] = (now, stat['cpu_ticks'], io)
            if collect_details:
                fds = []
                try:
                    for fd in list((root/'fd').iterdir())[:256]:
                        try:
                            target = str(fd.readlink())
                        except OSError:
                            continue
                        if target.endswith(('.jsonl', '.csv', '.log')):
                            fds.append(target)
                            self.log_paths.add(target)
                except OSError:
                    pass
                waits = {}
                try:
                    for task in list((root/'task').iterdir())[:32]:
                        thread_stat = process_stat(read(task/'stat'))
                        waits[task.name] = dict(state=thread_stat['state'] if thread_stat else None,
                                               wchan=read(task/'wchan'))
                except OSError:
                    pass
                self.details[key] = dict(log_files=sorted(set(fds)), thread_waits=waits,
                                         detail_monotonic=now)
            record.update(self.details.get(key, {}))
            if stat['state'] == 'D':
                record['kernel_stack'] = read(root/'stack')  # may require permission; never sudo
            processes.append(record)
        self.previous_processes = previous
        self.details = {key: detail for key, detail in self.details.items() if key in previous}
        mem = key_values(read(self.proc_root/'meminfo'))
        temperatures = {}
        if collect_details:
            for zone in Path('/sys/class/thermal').glob('thermal_zone*'):
                try:
                    raw = read(zone/'temp')
                    value = float(raw)/1000
                    if math.isfinite(value):
                        temperatures[read(zone/'type') or zone.name] = value
                except (TypeError, ValueError):
                    continue
        return dict(unix_ns=time.time_ns(), monotonic=now, cpu=cpu, meminfo_kb=mem,
                    memory_available_mb=mem['MemAvailable']/1024 if 'MemAvailable' in mem else None,
                    swap_used_mb=(mem.get('SwapTotal', 0)-mem.get('SwapFree', 0))/1024,
                    pressure={name: parse_pressure(read(self.proc_root/'pressure'/name))
                              for name in ('cpu', 'io', 'memory')},
                    loadavg=read(self.proc_root/'loadavg'), temperatures_c=temperatures or None,
                    diskstats=read(self.proc_root/'diskstats'), vmstat=read(self.proc_root/'vmstat'),
                    netdev=read(self.proc_root/'net/dev'),
                    net_snmp=read(self.proc_root/'net/snmp'), processes=processes)


class Evidence:
    """Onset/clear markers, not an assertion of root cause and never a control action."""
    def __init__(self, config):
        self.config, self.active, self.blocked = config, set(), {}

    def update(self, data, elapsed):
        c, now, reasons = self.config, data['monotonic'], set()
        busy = data.get('cpu', {}).get('cpu', {}).get('busy_percent')
        if busy is not None and busy >= c['system_cpu_warning_percent']:
            reasons.add('high_system_cpu')
        available = data.get('memory_available_mb')
        if available is not None and available < c['memory_available_warning_mb']:
            reasons.add('low_available_memory')
        if data.get('pressure', {}).get('io', {}).get('some', {}).get('avg10', 0) >= c['io_pressure_warning_percent']:
            reasons.add('high_io_pressure')
        current = {}
        for process in data.get('processes', []):
            if process['state'] == 'D':
                key = (process['pid'], process['start_ticks'])
                current[key] = self.blocked.get(key, now)
                if now-current[key] >= c['process_blocked_warning_sec']:
                    reasons.add(f"blocked_process:{process['name']}:{process['pid']}")
        self.blocked = current
        if (data.get('sample_gap_sec') or 0) > c['sample_gap_warning_sec']:
            reasons.add('system_sampler_delayed')
        web = data.get('web')
        if elapsed >= c['startup_grace_sec']:
            if not web or web.get('probe_age_sec', 0) > 3:
                reasons.add('web_probe_stale')
            elif web.get('consecutive_failures', 0) >= c['web_failure_warning_count']:
                reasons.add('web_unresponsive')
            age = (web or {}).get('health', {}).get('rosLoopAgeMs')
            if isinstance(age, (int, float)) and age > c['ros_loop_warning_ms']:
                reasons.add('web_ros_callbacks_delayed')
        events = [dict(event='warning_started', reason=r) for r in sorted(reasons-self.active)]
        events += [dict(event='warning_cleared', reason=r) for r in sorted(self.active-reasons)]
        self.active = reasons
        return events


def run(args, config):
    output = (Path(args.output_directory).expanduser() if args.output_directory else
              Path.home()/'Downloads/usv_diagnostics'/datetime.now().strftime('runtime_%Y%m%d_%H%M%S_%f'))
    output.mkdir(parents=True, exist_ok=True)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    identity = parent_identity(args.watch_parent) if args.watch_parent else None
    if args.watch_parent and identity is None:
        raise ValueError('launch parent has already exited')
    sampler, evidence = Sampler(args.watch_parent), Evidence(config)
    probe = WebProbe(f'http://127.0.0.1:{args.web_port}/healthz', config['web_timeout_sec'], stop)
    probe.thread.start()
    kernel = None
    with (output/'system.jsonl').open('a', buffering=1) as samples, \
            (output/'events.jsonl').open('a', buffering=1) as events, \
            (output/'kernel.log').open('a', buffering=1) as kernel_log, \
            (output/'monitor_threads.log').open('a', buffering=1) as stacks:
        faulthandler.enable(file=stacks)

        def event(item):
            item = dict(item, unix_ns=time.time_ns(), monotonic=time.monotonic())
            events.write(json.dumps(item, ensure_ascii=False, allow_nan=False)+'\n')

        if config['kernel_logging_enabled']:
            try:
                kernel = subprocess.Popen(['journalctl', '-k', '-f', '--since', 'now', '--no-pager', '-o', 'short-iso'],
                                          stdout=kernel_log, stderr=kernel_log)
                event(dict(event='kernel_collection_started', pid=kernel.pid))
            except OSError as error:
                event(dict(event='kernel_collection_unavailable', error=str(error)))
        (output/'ready.json').write_text(json.dumps(dict(pid=os.getpid(), parent=args.watch_parent,
                                                      parent_start_ticks=identity, unix_ns=time.time_ns())))
        (output/'related_logs.json').write_text('[]')
        print(f'独立系统记录：{output}；不会修改控制模式或动力。', flush=True)
        event(dict(event='monitor_started', config=config))
        started, last, known_logs = time.monotonic(), None, set()
        ended_reason = 'duration_elapsed'
        try:
            while not stop.is_set():
                now = time.monotonic()
                if args.watch_parent and parent_identity(args.watch_parent) != identity:
                    ended_reason = 'launch_parent_exited'
                    break
                if args.duration and now-started >= args.duration:
                    break
                faulthandler.dump_traceback_later(10., repeat=True, file=stacks)
                try:
                    data = sampler.snapshot(config['process_detail_interval_sec'])
                    data.update(web=probe.snapshot(), sample_gap_sec=now-last if last is not None else None)
                    last = now
                    disk = os.statvfs(output)
                    data['log_disk_free_mb'] = disk.f_bavail*disk.f_frsize/1048576
                    data['collection_ms'] = (time.monotonic()-now)*1000
                    samples.write(json.dumps(data, ensure_ascii=False, allow_nan=False)+'\n')
                    for item in evidence.update(data, now-started):
                        event(item)
                    if sampler.log_paths != known_logs:
                        known_logs = set(sampler.log_paths)
                        (output/'related_logs.json').write_text(json.dumps(sorted(known_logs), ensure_ascii=False, indent=2))
                except Exception as error:
                    event(dict(event='sampling_error', error=repr(error)))
                if kernel and kernel.poll() is not None:
                    event(dict(event='kernel_collection_ended', returncode=kernel.returncode,
                               note='Check kernel.log for permissions; other sampling continues.'))
                    kernel = None
                stop.wait(max(.01, config['sample_interval_sec']-(time.monotonic()-now)))
            if stop.is_set():
                ended_reason = 'monitor_signal'
        finally:
            faulthandler.cancel_dump_traceback_later()
            stop.set()
            probe.thread.join(timeout=config['web_timeout_sec']+.2)
            if kernel and kernel.poll() is None:
                kernel.terminate()  # only our journalctl child, never a ROS process
                try:
                    kernel.wait(timeout=1.)
                except subprocess.TimeoutExpired:
                    kernel.kill()
                    kernel.wait(timeout=1.)
            event(dict(event='monitor_stopped', reason=ended_reason))
            faulthandler.disable()
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=120., help='0 means until watched launch exits')
    parser.add_argument('--watch-parent', type=int)
    parser.add_argument('--output-directory')
    parser.add_argument('--web-port', type=int, default=8080)
    parser.add_argument('--config')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 <= args.duration <= 86400 or (args.duration == 0 and not args.watch_parent):
        parser.error('duration must be positive <=86400, or 0 with --watch-parent')
    if not 1 <= args.web_port <= 65535 or (args.watch_parent is not None and args.watch_parent <= 0):
        parser.error('invalid web port or parent PID')
    return run(args, load_config(args.config))


if __name__ == '__main__':
    raise SystemExit(main())
