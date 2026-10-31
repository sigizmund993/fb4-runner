import math
import socket
from time import monotonic, sleep, time
from typing import Optional

import serial
from google.protobuf.message import DecodeError

import config
from ball_detector import BallDetector
from protocol.uart import UartCommand
from ssl_packet_package.protopy.spbunited.robot import control_pb2
from utils.auxiliary import aux
from vision.vision_receiver import Team, VisionClient
from vision.vision import Field, Robot


# Commands with their own end (a target): one packet is enough, they don't expire when the
# strategy goes silent. They run until reached or replaced.
TARGET_COMMANDS = (
    control_pb2.RobotControlType.GLOBAL_COORDINATES,
    control_pb2.RobotControlType.RICOCHET,
    control_pb2.RobotControlType.CATCH_BALL,
)


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


def kicker_setting_to_voltage(setting: int) -> float:
    """OldFormat's kicker level: 0 - don't charge, 1..15 - KICKER_MIN_SETTING_V..KICKER_MAX_V."""
    if setting == 0:
        return 0.0
    span = config.KICKER_MAX_V - config.KICKER_MIN_SETTING_V
    return config.KICKER_MIN_SETTING_V + (min(setting, 15) - 1) / (15 - 1) * span


def apply_kicker_and_dribbler(kad: control_pb2.KickerAndDribbler, uart_command: UartCommand) -> None:
    uart_command.dribbler_speed = kad.dribbler_setting
    # The firmware clamps too, but fp16 can't even hold a huge uint32
    uart_command.kicker_voltage = min(kad.kicker_voltage, config.KICKER_MAX_V)
    uart_command.kicker_mode = kad.kicker_mode


def apply_old_format(cmd, uart_command: UartCommand) -> None:
    uart_command.vel_x = cmd.vel_x
    uart_command.vel_y = cmd.vel_y
    uart_command.w = cmd.angular_velocity_or_delta_angle
    uart_command.kicker_voltage = kicker_setting_to_voltage(cmd.kicker_setting)
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


def hold_heading(uart_command: UartCommand) -> None:
    uart_command.w = 0
    uart_command.angle_mode = True


def predicted_hit_y(ball: aux.Point, ball_vel: Optional[aux.Point]) -> float:
    """Robot-frame y where the ball will reach the kicker face; its current y unless it approaches."""
    if ball_vel is None or ball_vel.x > -config.RICOCHET_MIN_APPROACH_VEL:
        return ball.y
    t = max(ball.x - config.KICKER_FACE_X, 0.0) / -ball_vel.x
    return ball.y + ball_vel.y * min(t, config.RICOCHET_MAX_PREDICT_S)


def follow_ball(ball: Optional[aux.Point], ball_vel: Optional[aux.Point], uart_command: UartCommand) -> None:
    """Sideways speed towards where the ball seen by the onboard camera will hit the kicker; both speeds limited."""
    if ball is not None:
        uart_command.vel_y = config.RICOCHET_CAM_GAIN * predicted_hit_y(ball, ball_vel)
    uart_command.vel_x = aux.minmax(uart_command.vel_x, config.RICOCHET_MAX_VEL)
    uart_command.vel_y = aux.minmax(uart_command.vel_y, config.RICOCHET_MAX_VEL)


def ricochet_goal(field: Field) -> aux.Point:
    """Centre of the goal RICOCHET aims at; vision geometry is in mm, 0 until it arrives."""
    half_length = (field.geometry.length / 1000 or config.FIELD_LENGTH_M) / 2
    return aux.Point(config.RICOCHET_GOAL_SIGN * half_length, 0)


def ricochet_heading(hit: aux.Point, goal: aux.Point, ball_vel: aux.Point) -> float:
    """Kicker heading that sends the ball from hit into goal. The kick adds speed along the heading,
    the ball keeps part of its speed along the kicker face, so the heading is turned against that drift."""
    to_goal = (goal - hit).arg()
    heading = to_goal
    for _ in range(4):  # the drift depends on the heading itself
        face = aux.rotate(aux.Point(0, 1), heading)
        drift = config.RICOCHET_TANGENT_KEEP * aux.scal_mult(ball_vel, face)
        heading = to_goal - math.atan2(drift, config.RICOCHET_KICK_SPEED)
    return aux.wind_down_angle(heading)


def ricochet_angle(cmd: control_pb2.Ricochet, robot: Robot, field: Optional[Field], now: float) -> float:
    """The command's angle, else the heading into the goal (the current one without the field)."""
    if cmd.HasField("angle"):
        return cmd.angle
    if field is None:
        return robot.ang
    ball_vel = field.ball.vel if field.ball.is_visible(now, config.VISION_TIMEOUT_S) else aux.Point(0, 0)
    return ricochet_heading(aux.Point(cmd.x, cmd.y), ricochet_goal(field), ball_vel)


def ricochet(
    cmd: control_pb2.Ricochet,
    robot: Optional[Robot],
    ball: Optional[aux.Point],
    uart_command: UartCommand,
    angle: float = 0.0,
    ball_vel: Optional[aux.Point] = None,
) -> None:
    """Holds angle and drives to the target; once the camera sees the ball,
    the sideways speed follows the ball instead, so it bounces off at that angle.
    robot is None without vision: then it stays where it is, keeps its heading and only follows the ball."""
    if robot is not None:
        go_to_point_ignore(robot, aux.Point(cmd.x, cmd.y), angle, uart_command)
    else:
        hold_heading(uart_command)
    follow_ball(ball, ball_vel, uart_command)
    if cmd.HasField("kicker_and_dribbler"):
        apply_kicker_and_dribbler(cmd.kicker_and_dribbler, uart_command)


def catch_ball(
    cmd: control_pb2.CatchBall,
    robot: Optional[Robot],
    ball: Optional[aux.Point],
    uart_command: UartCommand,
    ball_vel: Optional[aux.Point] = None,
) -> None:
    """Turns to the angle and doesn't drive forward; the sideways speed follows the ball seen by the camera.
    robot is None without vision: then it keeps its current heading."""
    if robot is not None:
        uart_command.w = aux.wind_down_angle(cmd.angle - robot.ang)
        uart_command.angle_mode = True
    else:
        hold_heading(uart_command)
    follow_ball(ball, ball_vel, uart_command)
    apply_kicker_and_dribbler(cmd.kicker_and_dribbler, uart_command)


def build_uart_command(
    cmd: control_pb2.RobotCommand,
    robot: Robot,
    ball: Optional[aux.Point] = None,
    now: Optional[float] = None,
    field: Optional[Field] = None,
    ball_vel: Optional[aux.Point] = None,
) -> tuple[UartCommand, bool]:
    """Translates a protobuf command into a UART command.

    ball is the ball seen by the onboard camera, in the robot frame [m], or None; ball_vel its
    velocity relative to the robot [m/s].
    field (SSL vision: ball velocity, geometry) lets RICOCHET without angle aim at the goal.
    Target commands send a stop while vision doesn't see the robot (its position would be stale)
    and resume when it is seen again; RICOCHET and CATCH_BALL then keep the heading and only follow
    the ball with the onboard camera.
    Returns the command and whether the current target is reached and
    the stored command should be reset.
    """
    uart_command = UartCommand(beep=cmd.beep)
    target_reached = False
    now = time() if now is None else now
    if cmd.robot_control_type in TARGET_COMMANDS and not robot.is_visible(now, config.VISION_TIMEOUT_S):
        if cmd.robot_control_type == control_pb2.RobotControlType.RICOCHET:
            ricochet(cmd.ricochet, None, ball, uart_command, ball_vel=ball_vel)
        elif cmd.robot_control_type == control_pb2.RobotControlType.CATCH_BALL:
            catch_ball(cmd.catch_ball, None, ball, uart_command, ball_vel)
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
            angle = ricochet_angle(cmd.ricochet, robot, field, now)
            ricochet(cmd.ricochet, robot, ball, uart_command, angle, ball_vel)
        case control_pb2.RobotControlType.CATCH_BALL:
            # Never "reached": the robot waits for the ball until the strategy sends another command
            catch_ball(cmd.catch_ball, robot, ball, uart_command, ball_vel)
        case control_pb2.RobotControlType.CAP_VEL_AND_ACCEL:
            # TODO: implement
            pass
    return uart_command, target_reached


def command_receiver(client: VisionClient, ball_detector: BallDetector, robot_id):
    """robot_id: multiprocessing.Value shared with telemetry_transceiver, which may change it."""
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
            robot = field.get_robot(team, current_id)
        field.update()
        receiver.update()
        ball_detector.update()
        uart_command, target_reached = build_uart_command(
            receiver.current_command(), robot, ball_detector.r_pos, field=field, ball_vel=ball_detector.r_vel
        )
        if target_reached:
            receiver.last_cmd = control_pb2.RobotCommand()
        uart_command.robot_id = current_id
        ser.write(uart_command.to_bytes())

        next_tick += period
        delay = next_tick - time()
        if delay > 0:
            sleep(delay)
        else:
            next_tick = time()
