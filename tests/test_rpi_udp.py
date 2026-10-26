"""End-to-end test from a PC: UDP commands to the RPi runner, UDP telemetry back.

PC --UDP:5555--> RPi (main.py) --UART--> STM32 --UART--> RPi --UDP broadcast:5556--> PC

Skipped unless FB4_RPI_TEST=1. main.py must be running on the RPi, the strategy must be
stopped (it would fight over the commands), and the PC must be on the RPi's subnet to get
its broadcast telemetry.

    make test-rpi                               # host fb4-<ROBOT_ID>.local
    make test-rpi HOST=192.168.1.42             # explicit address
    FB4_HW_MOTION=1 make test-rpi               # also drives the wheels (robot on a stand!)

Steps run in order; the first failing one tells where the chain is broken. When run from a
terminal the test asks to confirm what can only be seen (LED, wheels).
"""
import math
import os
import socket
import sys
import time
import unittest

import helpers

if os.environ.get("FB4_RPI_TEST") != "1":
    raise unittest.SkipTest("network test; set FB4_RPI_TEST=1 to send UDP commands to the RPi")
helpers.require_protobuf()

import config  # noqa: E402
from ssl_packet_package.protopy.spbunited.robot import control_pb2, telemetry_pb2  # noqa: E402

HOST = os.environ.get("FB4_RPI_HOST") or f"fb4-{config.ROBOT_ID:02d}.local"
SEND_RATE_HZ = 50
MIN_TELEMETRY_PER_S = 20  # RPi forwards one packet per MCU frame (~45-100/s)
LED = "DATA_TRANSFER_1 (PG7)"


def make_command(vel_x=0.0, vel_y=0.0, angular_velocity=0.0) -> control_pb2.RobotCommand:
    cmd = control_pb2.RobotCommand(robot_id=config.ROBOT_ID, robot_control_type=control_pb2.RobotControlType.NEW_FORMAT)
    cmd.new_format.speed_control.vel_x = vel_x
    cmd.new_format.speed_control.vel_y = vel_y
    cmd.new_format.speed_control.angular_velocity = angular_velocity
    kad = cmd.new_format.kicker_and_dribbler
    kad.kicker_mode = control_pb2.KickerMode.IDLE
    kad.kicker_setting = 0
    kad.dribbler_setting = 0
    return cmd


STOP = make_command()


class RpiUdpTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.addr = (socket.gethostbyname(HOST), config.CMD_PORT)
        except socket.gaierror as e:
            raise unittest.SkipTest(f"can't resolve {HOST} ({e}); set FB4_RPI_HOST=<ip>") from e
        print(f"\n  RPi {HOST} -> {cls.addr[0]}")
        cls.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        cls.tel_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        cls.tel_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        cls.tel_sock.bind(("", config.TEL_PORT))
        cls.tel_sock.setblocking(False)

    @classmethod
    def tearDownClass(cls):
        cls.cmd_sock.close()
        cls.tel_sock.close()

    def setUp(self):
        self.drain_telemetry()
        self.addCleanup(self.send_for, STOP, 0.3)

    # --- helpers ---------------------------------------------------------

    def drain_telemetry(self) -> list[tuple[telemetry_pb2.RobotTelemetry, str]]:
        packets = []
        while True:
            try:
                data, (src, _) = self.tel_sock.recvfrom(1024)
            except BlockingIOError:
                return packets
            try:
                packets.append((telemetry_pb2.RobotTelemetry.FromString(data), src))
            except Exception:
                continue  # some other sender on the port

    def own_telemetry(self, packets) -> list[telemetry_pb2.RobotTelemetry]:
        return [p for p, src in packets if src == self.addr[0]]

    def collect_telemetry(self, duration_s: float) -> list[telemetry_pb2.RobotTelemetry]:
        packets = []
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            packets += self.drain_telemetry()
            time.sleep(0.01)
        return self.own_telemetry(packets)

    def send_for(self, cmd: control_pb2.RobotCommand, duration_s: float) -> list[telemetry_pb2.RobotTelemetry]:
        """Sends cmd like the strategy does and collects the RPi's telemetry meanwhile."""
        data = cmd.SerializeToString()
        packets = []
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self.cmd_sock.sendto(data, self.addr)
            packets += self.drain_telemetry()
            time.sleep(1 / SEND_RATE_HZ)
        return self.own_telemetry(packets)

    def confirm(self, question: str):
        if not sys.stdin.isatty():
            raise unittest.SkipTest(f"needs visual check, run from a terminal: {question}")
        answer = input(f"  {question} [y/n] ").strip().lower()
        self.assertIn(answer, ("y", "yes", "д", "да"), question)

    # --- steps -----------------------------------------------------------

    def test_1_telemetry_from_rpi(self):
        packets = self.collect_telemetry(2.0)
        self.assertTrue(
            packets,
            f"no telemetry from {self.addr[0]}:{config.TEL_PORT}: main.py not running, PC not on the RPi's "
            "subnet/firewall, or no UART frames from the MCU to the RPi (try `make test-hw` on the RPi)",
        )
        rate = len(packets) / 2.0
        self.assertGreaterEqual(rate, MIN_TELEMETRY_PER_S, f"only {rate:.0f} telemetry packets/s")
        last = packets[-1]
        self.assertEqual(last.robot_id, config.ROBOT_ID, "RPi reports another robot id (FB4_ROBOT_ID?)")
        self.assertTrue(math.isfinite(last.strategy_telemetry.kicker_voltage))
        print(f"  {rate:.0f} packets/s, last: robot_id={last.robot_id} {str(last.strategy_telemetry).strip()!r}")

    def test_2_commands_reach_mcu(self):
        print(f"\n  watch LED {LED}: blinks ~5 Hz for 3 s, steady for 2 s, blinks again for 3 s")
        packets = self.send_for(STOP, 3.0)
        self.assertGreaterEqual(len(packets) / 3.0, MIN_TELEMETRY_PER_S, "telemetry stopped while sending commands")
        self.collect_telemetry(2.0)
        self.send_for(STOP, 3.0)
        self.confirm(f"Did LED {LED} blink while sending and stop blinking during the pause?")

    def test_3_bad_packets_ignored(self):
        for payload in (b"\xff\xff\xff", control_pb2.RobotCommand(robot_id=config.ROBOT_ID).SerializePartialToString()):
            self.cmd_sock.sendto(payload, self.addr)
        packets = self.send_for(STOP, 1.0)
        self.assertGreaterEqual(len(packets), MIN_TELEMETRY_PER_S, "RPi stopped forwarding after bad packets")

    @unittest.skipUnless(os.environ.get("FB4_HW_MOTION") == "1", "set FB4_HW_MOTION=1 to drive the wheels")
    def test_4_wheels_move(self):
        print("\n  robot must be on a stand: spin 2 rad/s for 1 s, then forward 0.3 m/s for 1 s")
        self.send_for(make_command(angular_velocity=2.0), 1.0)
        self.send_for(make_command(vel_x=0.3), 1.0)
        self.send_for(STOP, 0.3)
        self.confirm("Did the wheels spin, then drive forward?")

    @unittest.skipUnless(os.environ.get("FB4_HW_MOTION") == "1", "set FB4_HW_MOTION=1 to drive the wheels")
    def test_5_stops_when_strategy_silent(self):
        print("\n  robot must be on a stand: forward 0.3 m/s for 1 s, then the PC goes silent")
        self.send_for(make_command(vel_x=0.3), 1.0)
        time.sleep(1.5)  # runner times out after CMD_TIMEOUT_S
        self.confirm(f"Did the wheels stop by themselves within ~{config.CMD_TIMEOUT_S} s?")


if __name__ == "__main__":
    unittest.main()
