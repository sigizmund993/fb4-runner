import socket
from time import monotonic, sleep, time
from typing import Optional

import serial
from google.protobuf.message import DecodeError

import config
from ball_detector import BallDetector
from protocol.uart import NO_ROBOT_ID, UartCommand
from ssl_packet_package.protopy.spbunited.robot import control_pb2
from utils.auxiliary import aux
from vision.vision_receiver import Team, VisionClient
from vision.vision import Field, Robot


# Commands with their own end (a target): one packet is enough, they don't expire when the
# strategy goes silent. They run until reached or replaced.
TARGET_COMMANDS = (control_pb2.RobotControlType.GLOBAL_COORDINATES, control_pb2.RobotControlType.RICOCHET)


class CommandReceiver:
    def __init__(self, host: str = config.BIND_HOST, port: int = config.CMD_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.last_cmd = control_pb2.RobotCommand()
        self.last_rx_time = float("-inf")

    def update(self) -> None:
        """Drains all pending datagrams and keeps the latest valid command."""
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
            except BlockingIOError:
                return
            cmd = control_pb2.RobotCommand()
            try:
                cmd.ParseFromString(data)
            except DecodeError:
                continue
            # ParseFromString does not enforce proto2 required fields
            if not cmd.IsInitialized():
                continue
            self.last_cmd = cmd
            self.last_rx_time = monotonic()

    def current_command(self, now: float | None = None) -> control_pb2.RobotCommand:
        """Latest command, or a stop command once the strategy has gone silent (target commands never expire)."""
        if self.last_cmd.robot_control_type in TARGET_COMMANDS:
            return self.last_cmd
        if (monotonic() if now is None else now) - self.last_rx_time > config.CMD_TIMEOUT_S:
            return control_pb2.RobotCommand()
        return self.last_cmd


def apply_speed_control(spdc: control_pb2.SpeedControl, uart_command: UartCommand) -> None:
    uart_command.vel_x = spdc.vel_x
    uart_command.vel_y = spdc.vel_y
    if spdc.angular_velocity:
        uart_command.w = spdc.angular_velocity
        uart_command.angle_mode = False
    elif spdc.delta_angle:
        uart_command.w = spdc.delta_angle
        uart_command.angle_mode = True
    elif spdc.global_angle:
        # TODO: implement global angle control
        pass
    else:
        uart_command.w = 0
        uart_command.angle_mode = True


def apply_kicker_and_dribbler(kad: control_pb2.KickerAndDribbler, uart_command: UartCommand) -> None:
    uart_command.dribbler_speed = kad.dribbler_setting
    uart_command.kicker_voltage = kad.kicker_setting
    uart_command.kicker_mode = kad.kicker_mode


def apply_old_format(cmd, uart_command: UartCommand) -> None:
    uart_command.vel_x = cmd.vel_x
    uart_command.vel_y = cmd.vel_y
    uart_command.w = cmd.angular_velocity_or_delta_angle
    uart_command.kicker_voltage = cmd.kicker_setting
    uart_command.dribbler_speed = cmd.dribbler_setting * cmd.dribbler_is_enabled
    uart_command.angle_mode = cmd.angvel_angle_toggle
    if cmd.kick_straight:
        uart_command.kicker_mode = 1
    elif cmd.kick_high:
        uart_command.kicker_mode = 2
    elif cmd.autokick_straight:
        uart_command.kicker_mode = 3
    elif cmd.autokick_high:
        uart_command.kicker_mode = 4
    else:
        uart_command.kicker_mode = 0


def go_to_point_ignore(robot: Robot, tgt_pos: aux.Point, tgt_ang: float, uart_command: UartCommand) -> None:
    glob_vel = (tgt_pos - robot.pos) * 1
    local_vel = aux.rotate(glob_vel, -robot.ang)
    uart_command.vel_x = local_vel.x
    uart_command.vel_y = local_vel.y
    uart_command.w = aux.wind_down_angle(tgt_ang - robot.ang)
    uart_command.angle_mode = True


def ricochet(cmd: control_pb2.Ricochet, robot: Robot, ball: Optional[aux.Point], uart_command: UartCommand) -> None:
    """Holds the target angle and drives to the target; once the camera sees the ball,
    the sideways speed follows the ball instead, so it bounces off at the set angle."""
    go_to_point_ignore(robot, aux.Point(cmd.x, cmd.y), cmd.angle, uart_command)
    if ball is not None:
        uart_command.vel_y = config.RICOCHET_CAM_GAIN * ball.y
    uart_command.vel_x = aux.minmax(uart_command.vel_x, config.RICOCHET_MAX_VEL)
    uart_command.vel_y = aux.minmax(uart_command.vel_y, config.RICOCHET_MAX_VEL)
    if cmd.HasField("kicker_and_dribbler"):
        apply_kicker_and_dribbler(cmd.kicker_and_dribbler, uart_command)


def build_uart_command(
    cmd: control_pb2.RobotCommand, robot: Robot, ball: Optional[aux.Point] = None, now: Optional[float] = None
) -> tuple[UartCommand, bool]:
    """Translates a protobuf command into a UART command.

    ball is the ball seen by the onboard camera, in the robot frame [m], or None.
    Target commands send a stop while vision doesn't see the robot (its position would be stale)
    and resume when it is seen again.
    Returns the command and whether the current target is reached and
    the stored command should be reset.
    """
    uart_command = UartCommand(beep=cmd.beep)
    target_reached = False
    if cmd.robot_control_type in TARGET_COMMANDS and not robot.is_visible(
        time() if now is None else now, config.VISION_TIMEOUT_S
    ):
        return uart_command, False
    match cmd.robot_control_type:
        case control_pb2.RobotControlType.NEW_FORMAT:
            apply_speed_control(cmd.new_format.speed_control, uart_command)
            apply_kicker_and_dribbler(cmd.new_format.kicker_and_dribbler, uart_command)
        case control_pb2.RobotControlType.KICKER_AND_DRIBBLER:
            apply_kicker_and_dribbler(cmd.kicker_and_dribbler, uart_command)
        case control_pb2.RobotControlType.SPEED_CONTROL:
            apply_speed_control(cmd.speed_control, uart_command)
        case control_pb2.RobotControlType.OLD_FORMAT:
            apply_old_format(cmd.old_format, uart_command)
        case control_pb2.RobotControlType.COORDINATE_CONTROL:
            pass
        case control_pb2.RobotControlType.GLOBAL_COORDINATES:
            tgt = aux.Point(cmd.global_coordinates.x, cmd.global_coordinates.y)
            if aux.dist(tgt, robot.pos) > 0.05:
                go_to_point_ignore(robot, tgt, cmd.global_coordinates.angle, uart_command)
            else:
                target_reached = True
        case control_pb2.RobotControlType.RICOCHET:
            # Never "reached": the robot waits at the target for the ball
            ricochet(cmd.ricochet, robot, ball, uart_command)
        case control_pb2.RobotControlType.CAP_VEL_AND_ACCEL:
            # TODO: implement
            pass
    return uart_command, target_reached


def command_receiver(client: VisionClient, ball_detector: BallDetector, robot_id):
    """robot_id: multiprocessing.Value shared with telemetry_transceiver, which may change it; -1 = unknown."""
    field = Field(client)
    receiver = CommandReceiver()
    ser = serial.Serial(config.UART_PORT, config.BAUD_RATE)
    team = Team[config.ROBOT_TEAM.upper()]
    current_id = None
    period = 1 / config.CMD_RATE_HZ

    next_tick = time()
    while True:
        if robot_id.value != current_id:
            current_id = robot_id.value
            # Without an id there is no robot to look up in vision: a never seen one stops target commands
            robot = field.get_robot(team, current_id) if current_id >= 0 else Robot(aux.Point(0, 0), 0, -1, team)
        field.update()
        receiver.update()
        ball_detector.update()
        uart_command, target_reached = build_uart_command(receiver.current_command(), robot, ball_detector.r_pos)
        if target_reached:
            receiver.last_cmd = control_pb2.RobotCommand()
        uart_command.robot_id = current_id if current_id >= 0 else NO_ROBOT_ID
        ser.write(uart_command.to_bytes())

        next_tick += period
        delay = next_tick - time()
        if delay > 0:
            sleep(delay)
        else:
            next_tick = time()
