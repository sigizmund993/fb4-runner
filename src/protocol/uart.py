"""UART protocol between RPi and STM32 firmware.

Frame (both directions): [SYNC_BYTE][payload][XOR checksum of SYNC_BYTE + payload].
Must match src/devices/uart_control in FootBot4_Firmware.
"""
import math
import struct
from dataclasses import dataclass
from typing import Optional

SYNC_BYTE = 0xA5
SETTINGS_SYNC_BYTE = 0x5A  # the settings frame: the STM32 fields of Coefficients (robot/control.proto)
NO_ROBOT_ID = 0xFF  # robot_id in a command: no id (diagnostics), the STM32 keeps its own

# vel_x, vel_y, w, dribbler, kicker [V] (fp16), kicker_mode (u8), angle_mode (bool), beep (bool),
# robot_id (u8)
CMD_FMT = "<5eB??B"
CMD_FRAME_LEN = 1 + struct.calcsize(CMD_FMT) + 1

# The settings frame, float32 each; NaN: the firmware's default
SETTINGS_FIELDS = (
    "angle_k", "angle_b", "max_linear_vel", "max_linear_accel", "max_angular_vel", "max_angular_accel",
    "wheel_vel_p", "wheel_vel_i",
)
SETTINGS_FMT = f"<{len(SETTINGS_FIELDS)}f"

# robot_id (u8), voltage (fp16), ball_deep (bool), ball_front (bool)
TEL_FMT = "<Be??"
TEL_FRAME_LEN = 1 + struct.calcsize(TEL_FMT) + 1


def compute_checksum(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
    return crc


def make_frame(payload: bytes, sync: int = SYNC_BYTE) -> bytes:
    body = bytes([sync]) + payload
    return body + bytes([compute_checksum(body)])


def pack_command(
    vel_x: float,
    vel_y: float,
    w: float,
    dribbler: float,
    kicker: float,
    kicker_mode: int,
    angle_mode: bool,
    beep: bool = False,
    robot_id: int = NO_ROBOT_ID,
) -> bytes:
    return make_frame(
        struct.pack(CMD_FMT, vel_x, vel_y, w, dribbler, kicker, kicker_mode, angle_mode, beep, robot_id)
    )


def pack_settings(values: dict[str, float]) -> Optional[bytes]:
    """The settings frame for coefficients by name (Coefficients fields); None if none of the STM32 ones is set."""
    if not any(name in values for name in SETTINGS_FIELDS):
        return None
    payload = struct.pack(SETTINGS_FMT, *(values.get(name, math.nan) for name in SETTINGS_FIELDS))
    return make_frame(payload, SETTINGS_SYNC_BYTE)


@dataclass
class UartCommand:
    vel_x: float = 0.0
    vel_y: float = 0.0
    w: float = 0.0
    dribbler_speed: float = 0.0
    kicker_voltage: float = 0.0  # [V] capacitor target 0..330, 0 - don't charge
    kicker_mode: int = 0
    angle_mode: bool = True  # False - angular velocity, True - delta angle
    beep: bool = False
    robot_id: int = NO_ROBOT_ID

    def to_bytes(self) -> bytes:
        return pack_command(
            self.vel_x,
            self.vel_y,
            self.w,
            self.dribbler_speed,
            self.kicker_voltage,
            self.kicker_mode,
            self.angle_mode,
            self.beep,
            self.robot_id,
        )


@dataclass
class Telemetry:
    robot_id: int
    voltage: float
    ball_deep: bool
    ball_front: bool


class FrameParser:
    """Extracts valid frames of a fixed payload format from a byte stream."""

    def __init__(self, fmt: str):
        self.fmt = fmt
        self.frame_len = 1 + struct.calcsize(fmt) + 1
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[tuple]:
        self._buf.extend(data)
        frames = []
        while len(self._buf) >= self.frame_len:
            idx = self._buf.find(SYNC_BYTE)
            if idx < 0:
                self._buf.clear()
                break
            if idx > 0:
                del self._buf[:idx]
                continue

            frame = bytes(self._buf[: self.frame_len])
            if compute_checksum(frame[:-1]) != frame[-1]:
                del self._buf[0]  # false sync byte, keep searching
                continue

            del self._buf[: self.frame_len]
            frames.append(struct.unpack(self.fmt, frame[1:-1]))
        return frames
