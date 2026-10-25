"""Command path: UDP protobuf from strategy -> UART frame for the STM32."""
import math
import select
import socket
import unittest

import helpers

helpers.require_protobuf()

from command_receiver import CommandReceiver, build_uart_command  # noqa: E402
from protocol.uart import CMD_FMT, FrameParser  # noqa: E402
from ssl_packet_package.protopy.spbunited.robot import control_pb2  # noqa: E402
from utils.auxiliary import aux  # noqa: E402
from vision.vision import Robot  # noqa: E402
from vision.vision_receiver import Team  # noqa: E402

CT = control_pb2.RobotControlType


def make_robot(x=0.0, y=0.0, ang=0.0) -> Robot:
    return Robot(aux.Point(x, y), ang, 5, Team.YELLOW)


def new_format_cmd(vel_x=0.0, vel_y=0.0, angular_velocity=None, delta_angle=None, kicker_mode=0, kicker=0, dribbler=0):
    cmd = control_pb2.RobotCommand(robot_id=5, robot_control_type=CT.NEW_FORMAT)
    spdc = cmd.new_format.speed_control
    spdc.vel_x = vel_x
    spdc.vel_y = vel_y
    if angular_velocity is not None:
        spdc.angular_velocity = angular_velocity
    if delta_angle is not None:
        spdc.delta_angle = delta_angle
    kad = cmd.new_format.kicker_and_dribbler
    kad.kicker_mode = kicker_mode
    kad.kicker_setting = kicker
    kad.dribbler_setting = dribbler
    return cmd


def old_format_cmd(**flags):
    cmd = control_pb2.RobotCommand(robot_id=5, robot_control_type=CT.OLD_FORMAT)
    old = cmd.old_format
    old.vel_x = 1
    old.vel_y = -2
    old.angular_velocity_or_delta_angle = 3
    old.kicker_setting = 10
    old.dribbler_setting = 7
    old.dribbler_is_enabled = flags.pop("dribbler_is_enabled", True)
    old.angvel_angle_toggle = flags.pop("angvel_angle_toggle", False)
    old.high_voltage = False
    for name in ("autokick_straight", "autokick_high", "kick_straight", "kick_high"):
        setattr(old, name, flags.pop(name, False))
    assert not flags, flags
    return cmd


def decode_uart(frame: bytes) -> tuple:
    """Parses a command frame the same way the firmware does."""
    frames = FrameParser(CMD_FMT).feed(frame)
    assert len(frames) == 1, frames
    return frames[0]


class BuildUartCommandTest(unittest.TestCase):
    def test_new_format_angular_velocity(self):
        uart, reached = build_uart_command(
            new_format_cmd(vel_x=1.0, vel_y=-0.5, angular_velocity=2.0, kicker_mode=1, kicker=8, dribbler=12),
            make_robot(),
        )
        self.assertFalse(reached)
        self.assertEqual(decode_uart(uart.to_bytes()), (1.0, -0.5, 2.0, 12.0, 8.0, 1, False))

    def test_new_format_delta_angle(self):
        uart, _ = build_uart_command(new_format_cmd(delta_angle=0.5), make_robot())
        self.assertEqual((uart.w, uart.angle_mode), (0.5, True))

    def test_new_format_without_angle_holds_heading(self):
        uart, _ = build_uart_command(new_format_cmd(vel_x=1.0), make_robot())
        self.assertEqual((uart.w, uart.angle_mode), (0, True))

    def test_speed_control_only(self):
        cmd = control_pb2.RobotCommand(robot_id=5, robot_control_type=CT.SPEED_CONTROL)
        cmd.speed_control.vel_x = 0.25
        cmd.speed_control.vel_y = 0.75
        cmd.speed_control.angular_velocity = -1.0
        uart, _ = build_uart_command(cmd, make_robot())
        self.assertEqual(decode_uart(uart.to_bytes()), (0.25, 0.75, -1.0, 0.0, 0.0, 0, False))

    def test_kicker_and_dribbler_only(self):
        cmd = control_pb2.RobotCommand(robot_id=5, robot_control_type=CT.KICKER_AND_DRIBBLER)
        cmd.kicker_and_dribbler.kicker_mode = control_pb2.KickerMode.AUTOKICK_HIGH
        cmd.kicker_and_dribbler.kicker_setting = 15
        cmd.kicker_and_dribbler.dribbler_setting = 4
        uart, _ = build_uart_command(cmd, make_robot())
        self.assertEqual((uart.vel_x, uart.vel_y, uart.w), (0, 0, 0))
        self.assertEqual((uart.kicker_mode, uart.kicker_voltage, uart.dribbler_speed), (4, 15, 4))

    def test_old_format_kicker_modes(self):
        cases = {
            "kick_straight": 1,
            "kick_high": 2,
            "autokick_straight": 3,
            "autokick_high": 4,
        }
        for flag, mode in cases.items():
            with self.subTest(flag=flag):
                uart, _ = build_uart_command(old_format_cmd(**{flag: True}), make_robot())
                self.assertEqual(uart.kicker_mode, mode)
        uart, _ = build_uart_command(old_format_cmd(), make_robot())
        self.assertEqual(uart.kicker_mode, 0)

    def test_old_format_fields(self):
        uart, _ = build_uart_command(old_format_cmd(angvel_angle_toggle=True), make_robot())
        self.assertEqual(
            (uart.vel_x, uart.vel_y, uart.w, uart.kicker_voltage, uart.dribbler_speed, uart.angle_mode),
            (1, -2, 3, 10, 7, True),
        )
        uart, _ = build_uart_command(old_format_cmd(dribbler_is_enabled=False), make_robot())
        self.assertEqual(uart.dribbler_speed, 0)

    def test_global_coordinates_drives_in_robot_frame(self):
        cmd = control_pb2.RobotCommand(robot_id=5, robot_control_type=CT.GLOBAL_COORDINATES)
        cmd.global_coordinates.x = 1.0
        cmd.global_coordinates.y = 0.0
        cmd.global_coordinates.angle = math.pi / 2
        # Robot faces +y, so a target along global +x is to its right (local -y)
        uart, reached = build_uart_command(cmd, make_robot(ang=math.pi / 2))
        self.assertFalse(reached)
        self.assertAlmostEqual(uart.vel_x, 0.0, places=6)
        self.assertAlmostEqual(uart.vel_y, -1.0, places=6)
        self.assertAlmostEqual(uart.w, 0.0, places=6)
        self.assertTrue(uart.angle_mode)

    def test_global_coordinates_target_reached(self):
        cmd = control_pb2.RobotCommand(robot_id=5, robot_control_type=CT.GLOBAL_COORDINATES)
        cmd.global_coordinates.x = 1.01
        cmd.global_coordinates.y = 0.0
        cmd.global_coordinates.angle = 0.0
        uart, reached = build_uart_command(cmd, make_robot(x=1.0))
        self.assertTrue(reached)
        self.assertEqual((uart.vel_x, uart.vel_y, uart.w), (0, 0, 0))

    def test_unimplemented_types_send_stop(self):
        for control_type in (CT.COORDINATE_CONTROL, CT.CAP_VEL_AND_ACCEL):
            with self.subTest(control_type=control_type):
                cmd = control_pb2.RobotCommand(robot_id=5, robot_control_type=control_type)
                uart, _ = build_uart_command(cmd, make_robot())
                self.assertEqual(decode_uart(uart.to_bytes())[:5], (0.0, 0.0, 0.0, 0.0, 0.0))


class CommandReceiverUdpTest(unittest.TestCase):
    def setUp(self):
        self.receiver = CommandReceiver("127.0.0.1", 0)
        self.addr = self.receiver.sock.getsockname()
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tearDown(self):
        self.receiver.sock.close()
        self.sender.close()

    def send(self, payload: bytes):
        self.sender.sendto(payload, self.addr)

    def update(self):
        # Loopback delivery is asynchronous; wait until the datagrams are readable
        select.select([self.receiver.sock], [], [], 1.0)
        self.receiver.update()

    def test_default_command_is_stop(self):
        self.receiver.update()
        uart, _ = build_uart_command(self.receiver.last_cmd, make_robot())
        self.assertEqual(decode_uart(uart.to_bytes())[:5], (0.0, 0.0, 0.0, 0.0, 0.0))

    def test_udp_command_reaches_uart_frame(self):
        self.send(new_format_cmd(vel_x=0.5, angular_velocity=1.5, kicker_mode=2, kicker=6).SerializeToString())
        self.update()
        uart, _ = build_uart_command(self.receiver.last_cmd, make_robot())
        self.assertEqual(decode_uart(uart.to_bytes()), (0.5, 0.0, 1.5, 0.0, 6.0, 2, False))

    def test_latest_command_wins(self):
        for vel in (0.25, 0.5, 1.0):
            self.send(new_format_cmd(vel_x=vel).SerializeToString())
        self.update()
        # A second update would pick up anything still in flight on a loaded machine
        self.receiver.update()
        self.assertEqual(self.receiver.last_cmd.new_format.speed_control.vel_x, 1.0)

    def test_invalid_packets_are_ignored(self):
        self.send(new_format_cmd(vel_x=0.5).SerializeToString())
        self.update()
        self.send(b"\xff\xff\xff")  # not a protobuf
        self.send(control_pb2.RobotCommand(robot_id=5).SerializePartialToString())  # missing required field
        self.update()
        self.assertEqual(self.receiver.last_cmd.new_format.speed_control.vel_x, 0.5)

    def test_no_new_data_keeps_last_command(self):
        self.send(new_format_cmd(vel_x=0.5).SerializeToString())
        self.update()
        self.receiver.update()
        self.receiver.update()
        self.assertEqual(self.receiver.last_cmd.new_format.speed_control.vel_x, 0.5)


if __name__ == "__main__":
    unittest.main()
