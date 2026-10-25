"""UART protocol between RPi and STM32 firmware.

Frame (both directions): [SYNC_BYTE][payload][XOR checksum of SYNC_BYTE + payload].
Must match src/devices/uart_control in FootBot4_Firmware.
"""
import struct
from dataclasses import dataclass

SYNC_BYTE = 0xA5

# vel_x, vel_y, w, dribbler, kicker (fp16), kicker_mode (u8), angle_mode (bool)
CMD_FMT = "<5eB?"
CMD_FRAME_LEN = 1 + struct.calcsize(CMD_FMT) + 1

# robot_id (u8), voltage (fp16), ball_deep (bool), ball_front (bool)
TEL_FMT = "<Be??"
TEL_FRAME_LEN = 1 + struct.calcsize(TEL_FMT) + 1


def compute_checksum(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
    return crc


def make_frame(payload: bytes) -> bytes:
    body = bytes([SYNC_BYTE]) + payload
    return body + bytes([compute_checksum(body)])


def pack_command(
    vel_x: float,
    vel_y: float,
    w: float,
    dribbler: float,
    kicker: float,
    kicker_mode: int,
    angle_mode: bool,
) -> bytes:
    return make_frame(struct.pack(CMD_FMT, vel_x, vel_y, w, dribbler, kicker, kicker_mode, angle_mode))


@dataclass
class UartCommand:
    vel_x: float = 0.0
    vel_y: float = 0.0
    w: float = 0.0
    dribbler_speed: float = 0.0
    kicker_voltage: float = 0.0
    kicker_mode: int = 0
    angle_mode: bool = True  # False - angular velocity, True - delta angle

    def to_bytes(self) -> bytes:
        return pack_command(
            self.vel_x,
            self.vel_y,
            self.w,
            self.dribbler_speed,
            self.kicker_voltage,
            self.kicker_mode,
            self.angle_mode,
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
