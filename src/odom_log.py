"""CSV log to compare the robot's motion by the STM32 wheel odometry, its IMU and SSL vision (FB4_ODOM_LOG=1).
One file per runner start, logs/odom_<date>.csv, the last LOG_KEEP are kept. A row is an event; rows come
roughly in time order, sort by t:
  kind=tel  an STM32 telemetry frame (the Telemetry columns, protocol/uart.py: the odometries by the wheel
            positions, the wheel speeds and the accelerometer, raw IMU), t when read from the UART
  kind=vis  a new SSL vision detection of this robot: vis_x, vis_y [m], vis_ang [rad] (field), the vision PC's
            t_capture and t_sent [s], camera_id; t when command_receiver took it (up to a tick after it came)
  kind=cmd  the command the runner sent over the UART this tick: rpi_vel_x, rpi_vel_y [m/s], rpi_w (angle_mode 1:
            a heading change [rad], 0: [rad/s]), rpi_angle_mode, rpi_cmd (the strategy's command type)
t: monotonic() on the RPi (the same clock in all processes); wall: time.time() at t, to put t_capture and t_sent
on the RPi clock (the vision PC's clock may be off).
"""
import csv
import multiprocessing
import queue
from dataclasses import fields
from datetime import datetime
from pathlib import Path
from time import monotonic, time
from typing import Optional

from protocol.uart import Telemetry, UartCommand

LOG_DIR = Path(__file__).resolve().parents[1] / "logs"
LOG_KEEP = 10
FLUSH_PERIOD_S = 1.0
QUEUE_SIZE = 1000  # telemetry frames between the processes, ~10 s at 100 Hz

TEL_COLUMNS = tuple(f.name for f in fields(Telemetry))
VIS_COLUMNS = ("vis_x", "vis_y", "vis_ang", "t_capture", "t_sent", "camera_id")
CMD_COLUMNS = ("rpi_vel_x", "rpi_vel_y", "rpi_w", "rpi_angle_mode", "rpi_cmd")
COLUMNS = ("kind", "t", "wall") + TEL_COLUMNS + VIS_COLUMNS + CMD_COLUMNS


def make_queue() -> multiprocessing.Queue:
    """telemetry_transceiver puts (t, wall, Telemetry fields) per frame, command_receiver writes them."""
    return multiprocessing.Queue(QUEUE_SIZE)


def put_frame(q: Optional[multiprocessing.Queue], t: float, wall: float, frame: tuple) -> None:
    """telemetry_transceiver: one frame; dropped when the log is off or the queue is full."""
    if q is None:
        return
    try:
        q.put_nowait((t, wall, frame))
    except queue.Full:
        pass


class OdomLog:
    def __init__(self, tel_queue: multiprocessing.Queue) -> None:
        self._queue = tel_queue
        LOG_DIR.mkdir(exist_ok=True)
        for old in sorted(LOG_DIR.glob("odom_*.csv"))[: -LOG_KEEP + 1]:
            old.unlink()
        path = LOG_DIR / f"odom_{datetime.now():%Y-%m-%d_%H-%M-%S}.csv"
        print(f"odometry log: {path}")
        self._file = open(path, "w", newline="")
        self._out = csv.writer(self._file)
        self._out.writerow(COLUMNS)
        self._next_flush = monotonic() + FLUSH_PERIOD_S

    def _row(self, kind: str, t: float, wall: float, tel=(), vis=(), cmd=()) -> None:
        self._out.writerow(
            (kind, f"{t:.4f}", f"{wall:.4f}")
            + (tuple(tel) or ("",) * len(TEL_COLUMNS))
            + (tuple(vis) or ("",) * len(VIS_COLUMNS))
            + (tuple(cmd) or ("",) * len(CMD_COLUMNS))
        )

    def write(self, robot_detections, uart: UartCommand, rpi_cmd: str) -> None:
        """A command_receiver tick: the queued telemetry frames, robot_detections (this robot's new vision
        detections, vision_receiver.RobotDetection) and the command sent."""
        now, wall = monotonic(), time()
        while True:
            try:
                t, w, frame = self._queue.get_nowait()
            except queue.Empty:
                break
            self._row("tel", t, w, tel=(f"{v:.4f}" if isinstance(v, float) else int(v) for v in frame))
        for det in robot_detections:
            fi = det.frame_info
            self._row(
                "vis", now, wall,
                vis=(f"{det.pos.x / 1000:.4f}", f"{det.pos.y / 1000:.4f}", f"{det.orientation:.4f}",
                     f"{fi.t_capture:.4f}", f"{fi.t_sent:.4f}", fi.camera_id),
            )
        self._row(
            "cmd", now, wall,
            cmd=(f"{uart.vel_x:.3f}", f"{uart.vel_y:.3f}", f"{uart.w:.3f}", int(uart.angle_mode), rpi_cmd),
        )
        if now >= self._next_flush:
            self._file.flush()
            self._next_flush = now + FLUSH_PERIOD_S
