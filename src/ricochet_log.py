"""CSV log of RICOCHET and CATCH_BALL: one row per command tick while one of them runs and LOG_TAIL_S after
(to see how it ended). One file per runner start, logs/ricochet_<date>.csv, the last LOG_KEEP are kept.

Robot frame: x forward, y left [m]; field: SSL vision [m]. Empty cells: no data.
"""
import csv
from datetime import datetime
from pathlib import Path
from time import monotonic
from typing import Optional

from ssl_packet_package.protopy.spbunited.robot import control_pb2

LOG_DIR = Path(__file__).resolve().parents[1] / "logs"
LOG_KEEP = 20
LOG_TAIL_S = 0.5
FLUSH_PERIOD_S = 1.0
LOGGED = {control_pb2.RobotControlType.RICOCHET: "RICOCHET", control_pb2.RobotControlType.CATCH_BALL: "CATCH_BALL"}

COLUMNS = (
    "t", "dt", "episode", "cmd",  # dt: since the last tick, 0.01 when the loop keeps up
    "cmd_x", "cmd_y", "cmd_angle",  # RICOCHET target, CATCH_BALL angle; field
    "robot_x", "robot_y", "robot_ang", "robot_vx", "robot_vy",  # SSL vision, field; empty while not seen
    "vball_age", "vball_x", "vball_y", "vball_vx", "vball_vy",  # SSL vision ball, field; age [s] of its last capture
    "cam_cx", "cam_cy", "cam_x", "cam_y", "cam_new", "path_frames",  # onboard camera: pixel, robot frame
    # robot frame; line_src: "<point>/<direction>" (command_receiver.BallLine)
    "line_src", "line_x", "line_y", "line_vx", "line_vy", "coming", "miss_x", "miss_y", "time_left",
    "mode",  # align: moves across the ball path, freeze: too late to correct, hold: the ball just stopped coming,
    # empty: the command's own
    "out_vx", "out_vy", "out_w", "out_angle_mode", "out_dribbler", "out_kicker_v",  # sent over UART
)


def _f(v: Optional[float], digits: int = 3) -> str:
    return "" if v is None else f"{v:.{digits}f}"


class RicochetLog:
    def __init__(self) -> None:
        self._out = None
        self._file = None
        self._episode = 0
        self._last_cmd: Optional[str] = None
        self._last_logged = float("-inf")  # monotonic() of the last RICOCHET or CATCH_BALL tick
        self._last_tick = monotonic()
        self._next_flush = 0.0

    def _open(self) -> None:
        LOG_DIR.mkdir(exist_ok=True)
        for old in sorted(LOG_DIR.glob("ricochet_*.csv"))[:-LOG_KEEP + 1]:
            old.unlink()
        path = LOG_DIR / f"ricochet_{datetime.now():%Y-%m-%d_%H-%M-%S}.csv"
        print(f"RICOCHET/CATCH_BALL log: {path}")
        self._file = open(path, "w", newline="")
        self._out = csv.writer(self._file)
        self._out.writerow(COLUMNS)

    def write(self, cmd, robot, vision_ball, detector, new_frame: bool, path, line, uart) -> None:
        """cmd: RobotCommand; robot: vision Robot or None; vision_ball: vision Ball; detector: BallDetector;
        path: BallPath; line: BallLine or None; uart: UartCommand sent this tick."""
        now = monotonic()
        dt, self._last_tick = now - self._last_tick, now
        kind = LOGGED.get(cmd.robot_control_type)
        if kind is not None:
            if kind != self._last_cmd:
                self._episode += 1
            self._last_logged = now
        elif now - self._last_logged > LOG_TAIL_S:
            if self._last_cmd is not None:  # the tail has just ended
                self._file.flush()
                self._last_cmd = None
            return
        self._last_cmd = kind or self._last_cmd
        if self._file is None:
            self._open()

        if kind == "RICOCHET":
            r = cmd.ricochet
            target = (r.x, r.y, r.angle if r.HasField("angle") else None)
        elif kind == "CATCH_BALL":
            target = (None, None, cmd.catch_ball.angle)
        else:  # the tail after the command: what the strategy sends now
            target = (None, None, None)
        robot_cols = (
            (robot.pos.x, robot.pos.y, robot.ang, robot.vel.x, robot.vel.y) if robot is not None else (None,) * 5
        )
        vball_age = now - vision_ball.last_update
        vball_cols = (
            (vision_ball.pos.x, vision_ball.pos.y, vision_ball.vel.x, vision_ball.vel.y)
            if vball_age < float("inf") else (None,) * 4
        )
        px, pos = detector.p_pos, detector.r_pos
        cam_cols = (
            px.x if px else "", px.y if px else "", _f(pos.x if pos else None), _f(pos.y if pos else None),
            int(new_frame), len(path.frames),
        )
        if line is not None:
            line_cols = (
                line.source, _f(line.point.x), _f(line.point.y), _f(line.vel.x), _f(line.vel.y), int(line.coming),
                _f(line.miss.x), _f(line.miss.y), _f(line.time_left),
            )
            mode = line.mode
        else:
            line_cols, mode = ("",) * 9, ""

        self._out.writerow((
            f"{now:.3f}", _f(dt), self._episode,
            kind or "after:" + control_pb2.RobotControlType.Name(cmd.robot_control_type),
            *(_f(v) for v in target),
            *(_f(v) for v in robot_cols),
            _f(vball_age if vball_age < float("inf") else None), *(_f(v) for v in vball_cols),
            *cam_cols,
            *line_cols,
            mode,
            _f(uart.vel_x), _f(uart.vel_y), _f(uart.w), int(uart.angle_mode), _f(uart.dribbler_speed, 1),
            _f(uart.kicker_voltage, 0),
        ))
        if now >= self._next_flush:
            self._file.flush()
            self._next_flush = now + FLUSH_PERIOD_S
