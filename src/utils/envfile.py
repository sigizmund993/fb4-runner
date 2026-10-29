"""Settings from the .env file in the repository root (see .env.example).

A variable set in the process environment overrides the file. Standard library only:
flash_firmware.py uses it and runs with the system python3, without the venv.
"""
import os
from pathlib import Path

ENV_FILE = Path(os.environ.get("FB4_ENV_FILE", Path(__file__).resolve().parents[2] / ".env"))
TEMPLATE = Path(__file__).resolve().parents[2] / ".env.example"


def _parse_line(line: str) -> tuple[str, str] | None:
    line = line.strip()
    if not line or line.startswith("#") or "=" not in line:
        return None
    key, value = line.removeprefix("export ").split("=", 1)
    value = value.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
        value = value[1:-1]
    else:
        value = value.split(" #", 1)[0].strip()
    return key.strip(), value


def read(path: Path = ENV_FILE) -> dict[str, str]:
    try:
        lines = path.read_text().splitlines()
    except FileNotFoundError:
        return {}
    return dict(kv for kv in map(_parse_line, lines) if kv is not None)


def get(key: str, default: str | None = None, path: Path = ENV_FILE) -> str | None:
    """The environment's value, else the file's; default when unset or empty in both."""
    value = os.environ.get(key) or read(path).get(key)
    return value if value else default


def put(key: str, value: str, path: Path = ENV_FILE, template: Path = TEMPLATE) -> None:
    """Sets key in the file, keeping the other lines and comments; a missing file starts as template."""
    try:
        lines = path.read_text().splitlines()
    except FileNotFoundError:
        lines = template.read_text().splitlines() if template.exists() else []
    new_line = f"{key}={value}"
    for i, line in enumerate(lines):
        parsed = _parse_line(line)
        if parsed is not None and parsed[0] == key:
            lines[i] = new_line
            break
    else:
        lines.append(new_line)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text("\n".join(lines) + "\n")
    tmp.replace(path)  # a power cut never leaves a half-written file
