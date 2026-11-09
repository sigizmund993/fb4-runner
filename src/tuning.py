"""Controller coefficients from the strategy: SET_COEFFICIENTS with Coefficients (robot/control.proto).

The runner keeps all it was sent in COEFFICIENTS_FILE, so they survive restarts. The STM32 fields go over UART
(protocol.uart.pack_settings), the others are Tuning used by command_receiver. The telemetry reports
coefficients_crc of them, the strategy sends SET_COEFFICIENTS only when it differs from its own.
"""
import json
import math
import os
import struct
import zlib
from dataclasses import dataclass, fields
from pathlib import Path

import config
from ssl_packet_package.protopy.spbunited.robot import control_pb2

# Not in git: what the strategy sent to this robot
COEFFICIENTS_FILE = Path(__file__).resolve().parents[1] / "coefficients.json"
COEFFICIENT_NAMES = tuple(f.name for f in control_pb2.Coefficients.DESCRIPTOR.fields)


def coefficients_crc(values: dict[str, float]) -> int:
    """CRC32 of the set coefficients by name, values as float32 (the proto's float). The strategy computes the
    same from FB4_COEFFICIENTS (coefficients_crc in its router_processor.py): change both together."""
    return zlib.crc32(b"".join(name.encode() + struct.pack("<f", values[name]) for name in sorted(values)))


@dataclass
class Tuning:
    """The runner's coefficients; the defaults from config until the strategy sends others."""

    position_kp: float = config.POSITION_KP
    position_max_vel: float = config.POSITION_MAX_VEL
    ricochet_gain: float = config.RICOCHET_CAM_GAIN
    catch_gain: float = config.RICOCHET_CAM_GAIN
    onboard_max_vel: float = config.RICOCHET_MAX_VEL
    kicker_face_x: float = config.KICKER_FACE_X
    min_approach_vel: float = config.RICOCHET_MIN_APPROACH_VEL
    freeze_s: float = config.RICOCHET_FREEZE_S


# The command_receiver process's coefficients
tuning = Tuning()


class CoefficientStore:
    """All coefficients the strategy has sent, kept in COEFFICIENTS_FILE; the runner's ones go to tuning."""

    def __init__(self, path: Path = COEFFICIENTS_FILE) -> None:
        self.path = path
        self.values: dict[str, float] = {}
        try:
            saved = json.loads(path.read_text())
        except FileNotFoundError:
            saved = {}
        except (OSError, ValueError) as e:
            print(f"can't read {path} ({e}), the coefficients are the defaults")
            saved = {}
        for name, value in saved.items():
            if name in COEFFICIENT_NAMES and isinstance(value, (int, float)) and value >= 0:
                self.values[name] = float(value)
        self._apply_tuning()
        if self.values:
            print(f"coefficients from {path}: {self.values}")

    def _apply_tuning(self) -> None:
        for f in fields(tuning):
            if f.name in self.values:
                setattr(tuning, f.name, self.values[f.name])

    def update(self, coefficients: control_pb2.Coefficients) -> list[str]:
        """Takes the set fields (NaN and negative are ignored, as in the firmware), saves them; returns the
        changed ones as "name old -> new"."""
        changed = []
        for name in COEFFICIENT_NAMES:
            if not coefficients.HasField(name):
                continue
            value = getattr(coefficients, name)
            if math.isnan(value) or value < 0 or self.values.get(name) == value:
                continue
            old = self.values.get(name)
            changed.append(f"{name} {'default' if old is None else f'{old:g}'} -> {value:g}")
            self.values[name] = value
        if changed:
            self._apply_tuning()
            self._save()
        return changed

    def _save(self) -> None:
        tmp = self.path.with_suffix(".tmp")
        try:
            tmp.write_text(json.dumps(self.values, indent=1, sort_keys=True))
            os.replace(tmp, self.path)  # a power cut leaves the old file, not half of it
        except OSError as e:
            print(f"can't save {self.path}: {e}")

    def crc(self) -> int:
        return coefficients_crc(self.values)
