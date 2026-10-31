"""Sends a few test commands to the STM32 over UART and prints its telemetry.

Runs on the RPi instead of main.py, which holds the same port: systemctl stop fb4-runner.
The robot drives and turns: put it on a stand.

    make test-uart
    make test-uart PORT=/dev/ttyUSB0
"""
import time

import serial

import config
from protocol.uart import TEL_FMT, FrameParser, Telemetry, UartCommand

STEPS = (
    ("stop", UartCommand(angle_mode=False)),
    ("dribbler", UartCommand(dribbler_speed=8, angle_mode=False)),
    ("forward 0.3 m/s", UartCommand(vel_x=0.3, angle_mode=False)),
    ("left 0.3 m/s", UartCommand(vel_y=0.3, angle_mode=False)),
    ("turn 2 rad/s", UartCommand(w=2.0, angle_mode=False)),
    ("stop", UartCommand(angle_mode=False)),
)
STEP_S = 2.0
PRINT_PERIOD_S = 0.5
MIN_TELEMETRY_PER_S = 30  # the firmware sends a frame every main loop, ~45..100/s


def run_step(ser: serial.Serial, name: str, cmd: UartCommand) -> bool:
    """Sends cmd for STEP_S at the runner's rate, prints the telemetry, says whether it kept coming."""
    print(f"--- {name}")
    frame = cmd.to_bytes()
    parser = FrameParser(TEL_FMT)
    count = 0
    last = None
    next_print = time.monotonic()
    deadline = time.monotonic() + STEP_S
    while time.monotonic() < deadline:
        ser.write(frame)
        for raw in parser.feed(ser.read(ser.in_waiting)):
            count += 1
            last = Telemetry(*raw)
        if last is not None and time.monotonic() >= next_print:
            print(f"  id {last.robot_id}  kicker {last.voltage:5.1f} V  "
                  f"ball_deep {int(last.ball_deep)}  ball_front {int(last.ball_front)}")
            next_print += PRINT_PERIOD_S
        time.sleep(1 / config.CMD_RATE_HZ)
    rate = count / STEP_S
    ok = rate >= MIN_TELEMETRY_PER_S
    print(f"  telemetry {rate:.0f}/s {'ok' if ok else 'TOO LOW'}")
    return ok


def main():
    print(f"port {config.UART_PORT} @ {config.BAUD_RATE}")
    try:
        ser = serial.Serial(config.UART_PORT, config.BAUD_RATE, timeout=0)
    except serial.SerialException as e:
        raise SystemExit(f"{e}; is main.py running? systemctl stop fb4-runner")
    with ser:
        ser.reset_input_buffer()
        try:
            results = [run_step(ser, name, cmd) for name, cmd in STEPS]
        finally:
            # The firmware stops by itself after 200 ms without frames, but don't rely on it
            ser.write(UartCommand(angle_mode=False).to_bytes())
    if not all(results):
        raise SystemExit("FAIL: no or too little telemetry from the STM32")
    print("OK")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
