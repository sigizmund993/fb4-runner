"""Hardware test: writes real commands to the STM32 over UART.

Skipped unless FB4_HW_TEST=1. Stop main.py first: it holds the same port.

    make test-hw                  # link check: stop commands + telemetry back
    FB4_HW_MOTION=1 make test-hw  # also spins the robot slowly for 1 s

The firmware keeps executing the last command if the link drops, so every
test ends with a burst of stop commands, even on failure.
"""
import math
import os
import time
import unittest

import helpers  # noqa: F401  (sets up sys.path)

if os.environ.get("FB4_HW_TEST") != "1":
    raise unittest.SkipTest("hardware test; set FB4_HW_TEST=1 to write to the real UART")

import serial  # noqa: E402

import config  # noqa: E402
from protocol.uart import CMD_FRAME_LEN, TEL_FMT, FrameParser, Telemetry, UartCommand  # noqa: E402

STOP = UartCommand(angle_mode=False)
SPIN_W = 1.0  # rad/s, slow in-place rotation
STOP_BURST_S = 0.3


class UartHardwareTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ser = serial.Serial(config.UART_PORT, config.BAUD_RATE, timeout=0.05)
        cls.ser.reset_input_buffer()

    @classmethod
    def tearDownClass(cls):
        cls.ser.close()

    def setUp(self):
        self.addCleanup(self.send_for, STOP, STOP_BURST_S)

    def send_for(self, cmd: UartCommand, duration_s: float) -> list[Telemetry]:
        """Sends cmd at the runner's rate for duration_s and collects telemetry meanwhile."""
        frame = cmd.to_bytes()
        parser = FrameParser(TEL_FMT)
        telemetry = []
        period = 1 / config.CMD_RATE_HZ
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            written = self.ser.write(frame)
            self.assertEqual(written, CMD_FRAME_LEN)
            self.ser.flush()
            telemetry += [Telemetry(*f) for f in parser.feed(self.ser.read(self.ser.in_waiting))]
            time.sleep(period)
        return telemetry

    def assert_telemetry_sane(self, telemetry: list[Telemetry]):
        self.assertTrue(telemetry, f"no telemetry from MCU on {config.UART_PORT}")
        for tel in telemetry:
            self.assertLess(tel.robot_id, 16)
            self.assertTrue(math.isfinite(tel.voltage) and tel.voltage >= 0, tel)

    def test_stop_command_and_telemetry(self):
        telemetry = self.send_for(STOP, 1.0)
        self.assert_telemetry_sane(telemetry)
        print(f"\n  {len(telemetry)} telemetry frames/s, last: {telemetry[-1]}")

    @unittest.skipUnless(os.environ.get("FB4_HW_MOTION") == "1", "set FB4_HW_MOTION=1 to move the robot")
    def test_spin_command(self):
        print(f"\n  spinning at {SPIN_W} rad/s for 1 s, watch the robot")
        telemetry = self.send_for(UartCommand(w=SPIN_W, angle_mode=False), 1.0)
        self.assert_telemetry_sane(telemetry)


if __name__ == "__main__":
    unittest.main()
