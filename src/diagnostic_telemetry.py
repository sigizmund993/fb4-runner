"""Serialize runner samples only while the diagnostics stream has subscribers."""

from dataclasses import asdict
import json
import math

from diagnostic_stream import DiagnosticStream, TELEMETRY_SOCKET


def finite(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def encode_sample(sample):
    robot_id, monotonic_time, wall_time, uart, pi_temp, crc, stm_status, pixel, ball = sample
    status = {
        "rpi_temp": pi_temp,
        "coefficients_crc": crc,
        "battery_voltage": None,
        "drive_pcb_temp": None,
        "drive_motor_temp": None,
    }
    if stm_status is not None:
        status.update(
            battery_voltage=finite(stm_status.battery_voltage),
            drive_pcb_temp=[finite(value) for value in stm_status.pcb_temp],
            drive_motor_temp=[finite(value) for value in stm_status.motor_temp],
        )
    snapshot = {
        "version": 1,
        "robot_id": robot_id,
        "monotonic": monotonic_time,
        "time": wall_time,
        "uart": {name: finite(value) for name, value in asdict(uart).items()},
        "status": status,
        "ball": {
            "pixel": [finite(pixel.x), finite(pixel.y)] if pixel is not None else None,
            "robot_m": [finite(ball.x), finite(ball.y)] if ball is not None else None,
        },
    }
    return json.dumps(snapshot, separators=(",", ":"), allow_nan=False).encode()


def start_telemetry_stream():
    try:
        return DiagnosticStream(TELEMETRY_SOCKET, encode_sample)
    except OSError as error:
        print(f"FB4 diagnostics telemetry unavailable: {error}", flush=True)
        return None
