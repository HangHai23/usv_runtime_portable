"""Resolve model paths independently of the terminal working directory."""
from pathlib import Path


def resolve_engine_path(value, anchor=__file__):
    value = value.strip()
    if not value:
        return ''
    path = Path(value).expanduser()
    if path.is_absolute():
        return str(path)
    for ancestor in Path(anchor).resolve().parents:
        if (ancestor / 'ros2_ws' / 'src' / 'usv_yolo_detector').is_dir():
            return str(ancestor / path)
    raise ValueError('Cannot locate project root; set engine_path to an absolute path for standalone installs')
