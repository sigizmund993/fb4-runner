"""Step-by-step UART link diagnostics against the real STM32.

Skipped unless FB4_HW_TEST=1. Works from the RPi or from a PC with a USB-UART bridge:

    make test-hw                          # RPi, config.UART_PORT (stop main.py first)
    make test-hw PORT=/dev/ttyUSB0        # PC: bridge TX -> PA1 (UART4 RX), bridge RX <- PA0, GND
    FB4_HW_MOTION=1 make test-hw PORT=... # also drives the wheels (robot on a stand!)
    FB4_HW_MOTION=1 make test-hw K=wheels # only the matching steps (unittest -k)

Disconnect the RPi from PA0/PA1 when using a bridge: two transmitters on one line corrupt each other.
Steps run in order; the first failing one tells where the link is broken. When run from a
terminal the test asks to confirm what can only be seen (LED, wheels).

The firmware keeps executing the last command if the link drops, so every test ends with a
burst of stop commands, even on failure.
"""
import math
import os
import random
import struct
import sys
import time
import unittest

import helpers  # noqa: F401  (sets up sys.path)

if os.environ.get("FB4_HW_TEST") != "1":
    raise unittest.SkipTest("hardware test; set FB4_HW_TEST=1 to write to the real UART")

import serial  # noqa: E402

import config  # noqa: E402
from protocol.uart import CMD_FRAME_LEN, SYNC_BYTE, TEL_FMT, FrameParser, Telemetry, UartCommand  # noqa: E402

# Same layout as TEL_FMT, but ball sensors as raw bytes: struct's "?" turns any nonzero byte
# into True, which would hide a shifted firmware layout
TEL_RAW_FMT = "<BeBB"
assert struct.calcsize(TEL_RAW_FMT) == struct.calcsize(TEL_FMT)

STOP = UartCommand(angle_mode=False)
STOP_BURST_S = 0.3
MIN_TELEMETRY_PER_S = 30  # one frame per main loop; measured ~45/s on the robot, 100/s nominal
OTHER_BAUDRATES = (9600, 19200, 38400, 57600, 230400, 460800, 921600)
LED = "DATA_TRANSFER_1 (PG7)"
MAX_KICKER_V = 140 * 3.3  # kicker ADC full scale (Kicker::get_voltage); charging target is <= 330 V
# Optional: robot_id the MCU must report (its EEPROM id, shown on the OLED)
EXPECTED_MCU_ID = int(os.environ["FB4_MCU_ROBOT_ID"]) if os.environ.get("FB4_MCU_ROBOT_ID") else None


class UartLinkTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        print(f"\n  port {config.UART_PORT} @ {config.BAUD_RATE}")
        cls.ser = serial.Serial(config.UART_PORT, config.BAUD_RATE, timeout=0.05)
        cls.mcu_robot_id = EXPECTED_MCU_ID  # first id seen; all later frames must match

    @classmethod
    def tearDownClass(cls):
        cls.ser.close()

    def setUp(self):
        self.ser.reset_input_buffer()
        self.addCleanup(self.send_for, STOP, STOP_BURST_S)

    # --- helpers ---------------------------------------------------------

    def read_raw(self, duration_s: float) -> bytes:
        data = bytearray()
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            data += self.ser.read(self.ser.in_waiting or 1)
        return bytes(data)

    def send_for(self, cmd: UartCommand, duration_s: float) -> list[Telemetry]:
        """Sends cmd at the runner's rate for duration_s and collects telemetry meanwhile."""
        frame = cmd.to_bytes()
        parser = FrameParser(TEL_RAW_FMT)
        telemetry = []
        period = 1 / config.CMD_RATE_HZ
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self.assertEqual(self.ser.write(frame), CMD_FRAME_LEN, "short write to the port")
            self.ser.flush()
            telemetry += self.checked_telemetry(parser.feed(self.ser.read(self.ser.in_waiting)))
            time.sleep(period)
        return telemetry

    def read_telemetry(self, duration_s: float) -> list[Telemetry]:
        return self.checked_telemetry(FrameParser(TEL_RAW_FMT).feed(self.read_raw(duration_s)))

    def checked_telemetry(self, raw_frames: list[tuple]) -> list[Telemetry]:
        """Validates every field of raw telemetry frames and converts them."""
        telemetry = []
        for robot_id, voltage, ball_deep, ball_front in raw_frames:
            frame = (robot_id, voltage, ball_deep, ball_front)
            self.assertLess(robot_id, 16, f"robot_id out of range: {frame}")
            if self.mcu_robot_id is None:
                type(self).mcu_robot_id = robot_id
            self.assertEqual(robot_id, self.mcu_robot_id, f"robot_id changed or unexpected: {frame}")
            self.assertTrue(math.isfinite(voltage), f"voltage is not a number: {frame}")
            self.assertTrue(0 <= voltage <= MAX_KICKER_V, f"voltage outside 0..{MAX_KICKER_V:.0f} V: {frame}")
            self.assertIn(ball_deep, (0, 1), f"ball_deep byte is not a bool: {frame}")
            self.assertIn(ball_front, (0, 1), f"ball_front byte is not a bool: {frame}")
            telemetry.append(Telemetry(robot_id, voltage, bool(ball_deep), bool(ball_front)))
        return telemetry

    def scan_baudrates(self) -> str:
        try:
            for baud in OTHER_BAUDRATES:
                self.ser.baudrate = baud
                self.ser.reset_input_buffer()
                if FrameParser(TEL_FMT).feed(self.read_raw(0.3)):
                    return f"valid frames found at {baud} baud: baudrate mismatch"
            return "no valid frames at other baudrates either: noise, wrong pin or wrong firmware"
        finally:
            self.ser.baudrate = config.BAUD_RATE

    def confirm(self, question: str):
        if not sys.stdin.isatty():
            # Can't confirm without a terminal: don't report a pass nobody verified
            raise unittest.SkipTest(f"needs visual check, run from a terminal: {question}")
        answer = input(f"  {question} [y/n] ").strip().lower()
        self.assertIn(answer, ("y", "yes", "д", "да"), question)

    def prompt(self, action: str):
        if not sys.stdin.isatty():
            raise unittest.SkipTest(f"needs a person at the robot, run from a terminal: {action}")
        input(f"  {action}, then press Enter ")
        self.ser.reset_input_buffer()

    def assert_telemetry_flowing(self, telemetry: list[Telemetry], duration_s: float, when: str):
        rate = len(telemetry) / duration_s
        self.assertGreaterEqual(rate, MIN_TELEMETRY_PER_S, f"telemetry {rate:.0f}/s {when}: MCU stalled or reset?")

    # --- steps -----------------------------------------------------------

    def test_1_mcu_sends_bytes(self):
        data = self.read_raw(1.0)
        self.assertTrue(
            data,
            "no bytes from STM32: check PA0 (UART4 TX) -> bridge RX, common GND, firmware flashed and running",
        )
        print(f"  {len(data)} bytes/s from MCU")

    def test_2_telemetry_frames_valid(self):
        data = self.read_raw(1.0)
        raw_frames = FrameParser(TEL_RAW_FMT).feed(data)
        if not raw_frames:
            self.fail(f"{len(data)} bytes but no valid telemetry frames ({data[:24].hex(' ')}); {self.scan_baudrates()}")
        self.assertGreaterEqual(len(raw_frames), MIN_TELEMETRY_PER_S, f"only {len(raw_frames)} frames/s")
        telemetry = self.checked_telemetry(raw_frames)
        volts = [t.voltage for t in telemetry]
        print(
            f"  {len(telemetry)} frames/s; robot_id={self.mcu_robot_id}, voltage {min(volts):.1f}..{max(volts):.1f} V, "
            f"ball_deep {sum(t.ball_deep for t in telemetry)}/{len(telemetry)}, "
            f"ball_front {sum(t.ball_front for t in telemetry)}/{len(telemetry)}"
        )

    def test_3_ball_sensors(self):
        # Kicker LEDs mirror the sensors: DRV2 = deep, DRV3 = front
        cases = (
            ("Take the ball away from the robot", False, False),
            ("Put the ball against the dribbler (front sensor, LED DRV3)", None, True),
            ("Push the ball deep into the dribbler (deep sensor, LED DRV2)", True, None),
        )
        for action, deep, front in cases:
            self.prompt(action)
            telemetry = self.read_telemetry(1.0)
            self.assertTrue(telemetry, "no telemetry")
            for name, expected in (("ball_deep", deep), ("ball_front", front)):
                if expected is None:
                    continue
                share = sum(getattr(t, name) == expected for t in telemetry) / len(telemetry)
                self.assertGreaterEqual(share, 0.9, f"{action}: {name} should be {expected}, was {share:.0%} of frames")

    def test_4_commands_reach_mcu(self):
        print(f"\n  watch LED {LED}: blinks ~5 Hz for 3 s, off/steady for 2 s, blinks again for 3 s")
        telemetry = self.send_for(STOP, 3.0)
        self.assert_telemetry_flowing(telemetry, 3.0, "while receiving commands")
        self.read_raw(2.0)
        self.send_for(STOP, 3.0)
        self.confirm(f"Did LED {LED} blink while sending and stop blinking during the pause?")

    def test_5_mcu_survives_garbage(self):
        # No sync byte in the noise, so it can never form a command that moves the robot
        noise = bytes(random.choice([b for b in range(256) if b != SYNC_BYTE]) for _ in range(2000))
        bad_checksum = bytearray(STOP.to_bytes())
        bad_checksum[-1] ^= 0xFF
        self.ser.write(noise + bytes(bad_checksum) * 20 + STOP.to_bytes()[:5])
        self.ser.flush()
        telemetry = self.send_for(STOP, 1.0)
        self.assert_telemetry_flowing(telemetry, 1.0, "after garbage")

    @unittest.skipUnless(os.environ.get("FB4_HW_MOTION") == "1", "set FB4_HW_MOTION=1 to drive the wheels")
    def test_6_wheels_move(self):
        # Spin first: it must work straight from idle, without a linear command before it
        print("\n  robot must be on a stand: spin 2 rad/s for 1 s, then forward 0.3 m/s for 1 s")
        self.send_for(UartCommand(w=2.0, angle_mode=False), 1.0)
        self.send_for(UartCommand(vel_x=0.3, angle_mode=False), 1.0)
        self.send_for(STOP, STOP_BURST_S)
        self.confirm("Did the wheels spin at full speed, then drive forward?")

    @unittest.skipUnless(os.environ.get("FB4_HW_MOTION") == "1", "set FB4_HW_MOTION=1 to drive the wheels")
    def test_7_stops_on_link_loss(self):
        print("\n  robot must be on a stand: forward 0.3 m/s for 1 s, then commands stop arriving")
        self.send_for(UartCommand(vel_x=0.3, angle_mode=False), 1.0)
        time.sleep(1.5)  # firmware link timeout is 200 ms
        self.confirm("Did the wheels stop by themselves right after the commands stopped?")

if __name__ == "__main__":
    unittest.main()
