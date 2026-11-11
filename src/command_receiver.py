import math
import socket
from dataclasses import dataclass
from time import monotonic, sleep
from typing import Optional

import serial
from google.protobuf.message import DecodeError

import config
from ball_detector import BallDetector, BallPath
from protocol.uart import UartCommand, pack_settings
from ricochet_log import RicochetLog
from tuning import CoefficientStore, tuning
from ssl_packet_package.protopy.spbunited.robot import control_pb2
from utils.auxiliary import aux
from vision.vision_receiver import Team, VisionClient
from vision.vision import Ball, Field, Robot


# Commands with their own end (a target): one packet is enough, they don't expire when the
# strategy goes silent. They run until reached or replaced.
TARGET_COMMANDS = (
    control_pb2.RobotControlType.GLOBAL_COORDINATES,
    control_pb2.RobotControlType.RICOCHET,
    control_pb2.RobotControlType.CATCH_BALL,
)


class CommandReceiver:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((config.BIND_HOST, config.CMD_PORT))
        self.sock.setblocking(False)
        self.last_cmd = control_pb2.RobotCommand()
        self.last_rx_time = float("-inf")
        self.coefficients: Optional[control_pb2.Coefficients] = None  # not applied yet

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
            if cmd.robot_control_type == control_pb2.RobotControlType.SET_COEFFICIENTS:
                # Not a motion command: the robot keeps running the last one
                self.coefficients = cmd.coefficients
                continue
            self.last_cmd = cmd
            self.last_rx_time = monotonic()

    def current_command(self) -> control_pb2.RobotCommand:
        """Latest command, or a stop command once the strategy has gone silent (target commands never expire)."""
        if self.last_cmd.robot_control_type in TARGET_COMMANDS:
            return self.last_cmd
        if monotonic() - self.last_rx_time > config.CMD_TIMEOUT_S:
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
    glob_vel = (tgt_pos - robot.pos) * tuning.position_kp
    if glob_vel.mag() > tuning.position_max_vel:
        glob_vel = glob_vel.unity() * tuning.position_max_vel
    local_vel = aux.rotate(glob_vel, -robot.ang)
    uart_command.vel_x = local_vel.x
    uart_command.vel_y = local_vel.y
    uart_command.w = aux.wind_down_angle(tgt_ang - robot.ang)
    uart_command.angle_mode = True


def hold_heading(uart_command: UartCommand) -> None:
    uart_command.w = 0
    uart_command.angle_mode = True


def limit_speed(uart_command: UartCommand) -> None:
    """RICOCHET and CATCH_BALL: the speed is not more than onboard_max_vel, its direction is kept."""
    vel = aux.Point(uart_command.vel_x, uart_command.vel_y)
    if vel.mag() > tuning.onboard_max_vel:
        vel = vel.unity() * tuning.onboard_max_vel
    uart_command.vel_x, uart_command.vel_y = vel.x, vel.y


def follow_ball(ball: Optional[aux.Point], uart_command: UartCommand) -> None:
    """Sideways speed towards the current y of the ball seen by the onboard camera (CATCH_BALL while
    the ball doesn't come to the robot)."""
    if ball is not None:
        uart_command.vel_y = tuning.catch_gain * ball.y


@dataclass
class BallLine:
    """The ball path in the robot frame and how it misses the kicker."""
    source: str  # "<point>/<direction>": point "camera" or "vision", direction "vision" or "fit" (camera frames)
    point: aux.Point  # [m] a point of the path
    vel: aux.Point  # [m/s] the ball velocity along it
    coming: bool  # the ball comes to the kicker (faster than min_approach_vel)
    miss: aux.Point  # [m] from the path to the kicker centre, across the path
    time_left: float  # [s] until the ball comes to the kicker
    mode: str = ""  # set by align_with_ball_path: "align", "freeze"; "hold" by hold_after_coming


def vision_ball_vel_ok(vel: aux.Point, fit: Optional[tuple[aux.Point, aux.Point, float]]) -> bool:
    """The SSL vision ball velocity (robot frame) is not a glitch: not faster than BALL_MAX_VEL and, when the
    camera frames give a direction, not more than BALL_DIR_TOL_DEG off it."""
    if vel.mag() > config.BALL_MAX_VEL:
        return False
    if fit is None or fit[1].mag() < tuning.min_approach_vel or vel.mag() == 0:
        return True
    cos = aux.scal_mult(vel.unity(), fit[1].unity())
    return cos >= math.cos(math.radians(config.BALL_DIR_TOL_DEG))


def ball_line(path: BallPath, robot: Optional[Robot], vision_ball: Ball) -> Optional[BallLine]:
    """How the ball misses the kicker; None without a ball in front of the robot within CAM_MAX_DIST. The ball
    may come at any angle, so the miss is across the path, not along the kicker.

    The point of the path: the latest camera frame within CAM_TRUST_DIST (the camera is accurate only near the
    robot), else the SSL vision ball in the robot frame (by the robot's vision pose), else the camera frame.
    The direction: the SSL vision ball velocity, on the field (the robot's own motion doesn't tilt it) and
    filtered along the whole path. Without it (vision lost the ball or the robot, or a glitch: see
    vision_ball_vel_ok) the line through the camera frames, which needs BALL_PATH_MIN_FRAMES of them."""
    cam = path.last()
    fit = path.fit()
    vis_point = vis_vel = None
    if robot is not None and vision_ball.is_visible(config.VISION_TIMEOUT_S):
        vis_point = aux.rotate(vision_ball.pos - robot.pos, -robot.ang)
        if vis_point.x <= tuning.kicker_face_x or vis_point.mag() > config.CAM_MAX_DIST:
            vis_point = None  # beside or behind the kicker, or out of the camera's range
        vis_vel = aux.rotate(vision_ball.vel, -robot.ang)
        if not vision_ball_vel_ok(vis_vel, fit):
            vis_vel = None

    if vis_vel is None:
        if fit is None:
            return None
        point, vel, t = fit
        source = "camera/fit"
    else:
        vel = vis_vel
        if cam is not None and (cam[0].mag() <= config.CAM_TRUST_DIST or vis_point is None):
            (point, t), source = cam, "camera/vision"
        elif vis_point is not None:
            point, t, source = vis_point, vision_ball.last_update, "vision/vision"
        else:
            return None
    to_kicker = aux.Point(tuning.kicker_face_x, 0) - point
    coming = to_kicker.mag() > 0 and aux.scal_mult(vel, to_kicker.unity()) >= tuning.min_approach_vel
    direction = vel.unity()
    along = aux.scal_mult(to_kicker, direction)
    time_left = along / vel.mag() - (monotonic() - t) if vel.mag() > 0 else float("inf")
    return BallLine(source, point, vel, coming, to_kicker - direction * along, time_left)


def robot_frame_vel(robot: Optional[Robot]) -> aux.Point:
    """The robot's speed by SSL vision in its own frame; zero without vision."""
    return aux.rotate(robot.vel, -robot.ang) if robot is not None else aux.Point(0, 0)


def align_with_ball_path(line: BallLine, robot_vel: aux.Point, min_gain: float, uart_command: UartCommand) -> None:
    """Moves only across the ball path, so the kicker centre gets onto it by the time the ball comes.

    The robot follows a command ROBOT_RESPONSE_S late, meanwhile it keeps its current speed robot_vel (robot
    frame): the miss is predicted that far ahead and closed by freeze_s + ROBOT_RESPONSE_S before the ball.
    Later it doesn't move (a command then would act only after the ball, it only shakes the robot)."""
    direction = line.vel.unity()
    vel_across = robot_vel - direction * aux.scal_mult(robot_vel, direction)
    miss = line.miss + vel_across * config.ROBOT_RESPONSE_S
    time_to_close = line.time_left - config.ROBOT_RESPONSE_S - tuning.freeze_s
    if time_to_close < 0:
        vel = aux.Point(0, 0)
        line.mode = "freeze"
    else:
        gain = max(min_gain, 1 / (time_to_close + 1 / config.CMD_RATE_HZ))
        vel = -miss * gain
        line.mode = "align"
    uart_command.vel_x, uart_command.vel_y = vel.x, vel.y


# monotonic() when a ball last came to the kicker (hold_after_coming)
_last_coming = float("-inf")


def hold_after_coming(line: Optional[BallLine], uart_command: UartCommand) -> bool:
    """Right after the ball stopped "coming" (lost for a frame, a vision glitch) the robot stands still for
    COMING_HOLD_S rather than driving off to the target. True when it holds."""
    global _last_coming
    if line is not None and line.coming:
        _last_coming = monotonic()
        return False
    if monotonic() - _last_coming > config.COMING_HOLD_S:
        return False
    uart_command.vel_x = uart_command.vel_y = 0
    if line is not None:
        line.mode = "hold"
    return True


def ricochet(
    cmd: control_pb2.Ricochet,
    robot: Optional[Robot],
    ball_path: BallPath,
    vision_ball: Ball,
    uart_command: UartCommand,
) -> Optional[BallLine]:
    """Holds the angle (the current heading without one) and drives to the target the strategy chose for the
    bounce; once the ball comes to the kicker, moves only across its path so it hits the kicker centre.
    robot is None without vision: then it keeps its heading and moves only by the camera."""
    if robot is not None:
        go_to_point_ignore(robot, aux.Point(cmd.x, cmd.y), cmd.angle, uart_command)
        if not cmd.HasField("angle"):
            hold_heading(uart_command)
    else:
        hold_heading(uart_command)
    line = ball_line(ball_path, robot, vision_ball)
    if line is not None and line.coming:
        align_with_ball_path(line, robot_frame_vel(robot), tuning.ricochet_gain, uart_command)
    hold_after_coming(line, uart_command)
    limit_speed(uart_command)
    if cmd.HasField("kicker_and_dribbler"):
        apply_kicker_and_dribbler(cmd.kicker_and_dribbler, uart_command)
    return line


def catch_ball(
    cmd: control_pb2.CatchBall,
    robot: Optional[Robot],
    ball: Optional[aux.Point],
    ball_path: BallPath,
    vision_ball: Ball,
    uart_command: UartCommand,
) -> Optional[BallLine]:
    """Turns to the angle; once the ball comes to the kicker, moves only across its path, before that
    sideways to the ball seen by the camera, without driving forward.
    robot is None without vision: then it keeps its current heading."""
    if robot is not None:
        uart_command.w = aux.wind_down_angle(cmd.angle - robot.ang)
        uart_command.angle_mode = True
    else:
        hold_heading(uart_command)
    line = ball_line(ball_path, robot, vision_ball)
    if line is not None and line.coming:
        align_with_ball_path(line, robot_frame_vel(robot), tuning.catch_gain, uart_command)
    elif not hold_after_coming(line, uart_command):
        follow_ball(ball, uart_command)
    limit_speed(uart_command)
    apply_kicker_and_dribbler(cmd.kicker_and_dribbler, uart_command)
    return line


def build_uart_command(
    cmd: control_pb2.RobotCommand,
    robot: Optional[Robot],
    ball: Optional[aux.Point],
    ball_path: BallPath,
    vision_ball: Ball,
) -> tuple[UartCommand, bool, Optional[BallLine]]:
    """Translates a protobuf command into a UART command.

    robot is None while SSL vision doesn't see it. ball is the ball seen by the onboard camera, in the robot
    frame [m], or None; ball_path its path from the last camera frames; vision_ball the ball from SSL vision.
    Target commands send a stop while vision doesn't see the robot (its position would be stale)
    and resume when it is seen again; RICOCHET and CATCH_BALL then keep the heading and only follow
    the ball with the onboard camera.
    Returns the command, whether the current target is reached and the stored command should be reset,
    and the ball path of RICOCHET and CATCH_BALL (for the log).
    """
    uart_command = UartCommand(beep=cmd.beep)
    target_reached = False
    line = None
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
            if robot is None:
                pass  # stop until vision sees the robot again
            elif aux.dist(tgt, robot.pos) > 0.05:
                go_to_point_ignore(robot, tgt, cmd.global_coordinates.angle, uart_command)
            else:
                target_reached = True
        case control_pb2.RobotControlType.RICOCHET:
            # Never "reached": the robot waits at the target for the ball
            line = ricochet(cmd.ricochet, robot, ball_path, vision_ball, uart_command)
        case control_pb2.RobotControlType.CATCH_BALL:
            # Never "reached": the robot waits for the ball until the strategy sends another command
            line = catch_ball(cmd.catch_ball, robot, ball, ball_path, vision_ball, uart_command)
        case control_pb2.RobotControlType.CAP_VEL_AND_ACCEL:
            # TODO: implement
            pass
    return uart_command, target_reached, line


def command_receiver(client: VisionClient, ball_detector: BallDetector, robot_id, coefficients_crc):
    """robot_id: multiprocessing.Value shared with telemetry_transceiver, which may change it;
    coefficients_crc: multiprocessing.Value, the CRC of the coefficients for the telemetry."""
    field = Field(client)
    receiver = CommandReceiver()
    ser = serial.Serial(config.UART_PORT, config.BAUD_RATE)
    team = Team[config.ROBOT_TEAM.upper()]
    current_id = None
    period = 1 / config.CMD_RATE_HZ
    ball_path = BallPath()
    robot_vel = aux.Point(0, 0)  # commanded at the last tick
    log = RicochetLog()
    coefficients = CoefficientStore()
    coefficients_crc.value = coefficients.crc()
    next_settings = monotonic()  # the STM32 doesn't keep them over a reset: repeat now and then

    next_tick = monotonic()
    while True:
        if robot_id.value != current_id:
            current_id = robot_id.value
            robot = field.get_robot(team, current_id)
        field.update()
        receiver.update()
        if receiver.coefficients is not None:
            changed = coefficients.replace(receiver.coefficients)
            receiver.coefficients = None
            if changed:
                print("coefficients: " + ", ".join(changed))
                coefficients_crc.value = coefficients.crc()
                next_settings = monotonic()
        if monotonic() >= next_settings:
            frame = pack_settings(coefficients.values)
            if frame is not None:
                ser.write(frame)  # the firmware logs only changes
            next_settings = monotonic() + config.SETTINGS_RESEND_S
        new_frame = ball_detector.update()
        ball_path.update(ball_detector.r_pos, new_frame, robot_vel)
        seen_robot = robot if robot.is_visible(config.VISION_TIMEOUT_S) else None
        cmd = receiver.current_command()
        uart_command, target_reached, line = build_uart_command(
            cmd, seen_robot, ball_detector.r_pos, ball_path, field.ball
        )
        log.write(cmd, seen_robot, field.ball, ball_detector, new_frame, ball_path, line, uart_command)
        robot_vel = aux.Point(uart_command.vel_x, uart_command.vel_y)
        if target_reached:
            receiver.last_cmd = control_pb2.RobotCommand()
        uart_command.robot_id = current_id
        ser.write(uart_command.to_bytes())

        next_tick += period
        delay = next_tick - monotonic()
        if delay > 0:
            sleep(delay)
        else:
            next_tick = monotonic()
