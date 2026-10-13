"""Append-only raw telemetry for both manual excitation and automatic runs."""
import json
import time
from pathlib import Path
from datetime import datetime
from rosidl_runtime_py.convert import message_to_ordereddict


class Recorder:
    def __init__(self, directory, parameters):
        self.directory = Path(directory).expanduser() / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.directory.mkdir(parents=True)
        (self.directory / 'parameters.json').write_text(json.dumps(parameters, indent=2))
        source_root = Path(__file__).resolve().parents[2]
        for ancestor in Path(__file__).resolve().parents:
            if (ancestor / 'src' / 'usv_pwm_actuator' / 'config').is_dir():
                source_root = ancestor / 'src'
                break
        configurations = {}
        for package in ('usv_pwm_actuator', 'usv_udp_position', 'usv_imu_driver', 'usv_rc_teleop'):
            for path in (source_root / package / 'config').glob('*.yaml'):
                configurations[str(path)] = path.read_text()
        (self.directory / 'source_configurations.json').write_text(json.dumps(configurations, indent=2))
        self.file = (self.directory / 'telemetry.jsonl').open('w', buffering=1024*256)
        self.last_flush = time.monotonic()
        self.write('session_start', parameters=parameters)

    def write(self, event, **values):
        self.file.write(json.dumps(dict(event=event, monotonic=time.monotonic(),
                         unix_ns=time.time_ns(), **values), allow_nan=False) + '\n')
        if time.monotonic() - self.last_flush > 1:
            self.file.flush()
            self.last_flush = time.monotonic()

    def message(self, event, msg):
        self.write(event, message=message_to_ordereddict(msg))

    def close(self):
        self.write('session_end')
        self.file.close()
