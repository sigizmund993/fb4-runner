import struct
from collections import deque
from multiprocessing.shared_memory import SharedMemory
from pathlib import Path
from time import monotonic, sleep
from typing import Optional

import config
from to_real_coords.coord_converter import CoordConverter
from utils.auxiliary import aux
from vision.vision import Robot

CALIB_DIR = Path(__file__).parent / "to_real_coords"
# Written by yolo26_post.cpp when no ball is detected
NO_BALL = (1281, 961)
# The detector (run.sh) creates the shared memory once the Hailo model is loaded
SHM_WAIT_S = 30.0
# The shared memory has no frame counter: a new frame is a changed pixel, or this long without one
# (the ball stands still); the camera runs at 30 fps (run.sh)
SAME_PIXEL_S = 1.5 / 30


def attach_shm() -> SharedMemory:
    deadline = monotonic() + SHM_WAIT_S
    while True:
        try:
            # track=False: else the resource tracker unlinks the segment when this process exits, and the
            # detector keeps writing to a segment nobody can attach to any more
            return SharedMemory(name=config.SHM_NAME, track=False)
        # ValueError: created but not sized yet (ftruncate in yolo26_post.cpp)
        except (FileNotFoundError, ValueError):
            if monotonic() > deadline:
                raise
            sleep(0.05)


def read_pixel(shm: SharedMemory) -> tuple[int, int]:
    """The ball's bbox centre [px] from yolo26_post.cpp, NO_BALL when not detected."""
    return struct.unpack("<HH", shm.buf[:4])


class BallDetector:
    def __init__(self):
        self.shm = attach_shm()
        self.coord_converter = CoordConverter(
            str(CALIB_DIR / "camera_params_fisheye.npz"),
            str(CALIB_DIR / "homography_matrix.npy"),
        )
        self.p_pos: Optional[aux.Point] = None  # [px]
        self.r_pos: Optional[aux.Point] = None  # [m], robot frame: x forward, y left
        self._last_px: Optional[tuple[int, int]] = None
        self._last_sample = float("-inf")

    def update(self) -> bool:
        """Reads the detector; True on a new camera frame with the ball. A ball behind the robot or further than
        CAM_MAX_DIST is no ball (r_pos None), p_pos keeps its pixel for the log."""
        cx, cy = read_pixel(self.shm)
        if (cx, cy) == NO_BALL:
            self.r_pos = None
            self.p_pos = None
            self._last_px = None
            return False
        now = monotonic()
        if (cx, cy) == self._last_px and now - self._last_sample < SAME_PIXEL_S:
            return False  # still the same camera frame
        self._last_px, self._last_sample = (cx, cy), now
        self.p_pos = aux.Point(cx, cy)
        pos = calib_to_robot_frame(self.coord_converter.get_coords(self.p_pos))
        if pos.x < 0 or pos.mag() > config.CAM_MAX_DIST:
            self.r_pos = None  # above the horizon: a false detection, or too far to tell where
            return False
        self.r_pos = pos
        return True


class BallPath:
    """The ball's path from the last camera frames, given in the robot frame (x forward, y left).

    A frame is dated by its capture, CAM_DELAY_S before it is read. With the robot's SSL vision pose the frames
    are kept on the field: each camera point is put there by the pose at its capture, so neither the robot's
    motion nor its turns tilt the path, and the path is brought to the robot's current pose when asked.
    Without the pose (vision doesn't see the robot) the frames are kept in the robot frame, shifted back by the
    robot's estimated motion (the command through the robot's lag, see command_receiver), the heading taken as
    held."""

    def __init__(self) -> None:
        # (capture monotonic(), ball in the robot frame (shifted by the estimated motion), ball on the field or
        # None without the robot's pose)
        self.frames: deque[tuple[float, aux.Point, Optional[aux.Point]]] = deque()
        self._last_update = monotonic()

    def update(self, ball: Optional[aux.Point], new_frame: bool, robot: Optional[Robot], robot_vel: aux.Point) -> None:
        """ball, new_frame: BallDetector.r_pos, update(); robot: the robot seen by SSL vision or None;
        robot_vel: its estimated speed (robot frame) since the last call."""
        now = monotonic()
        shift = robot_vel * (now - self._last_update)
        self._last_update = now
        if ball is None:
            self.frames.clear()
            return
        self.frames = deque(
            (t, p - shift, f) for t, p, f in self.frames if now - t <= config.BALL_PATH_S + config.CAM_DELAY_S
        )
        if new_frame:
            t = now - config.CAM_DELAY_S
            on_field = None
            if robot is not None:
                pos, ang = robot_pose_at(robot, t)
                on_field = pos + aux.rotate(ball, ang)
            self.frames.append((t, ball, on_field))

    def _robot_frame(self, robot: Optional[Robot]) -> list[tuple[float, aux.Point]]:
        """The frames in the current robot frame: by the field points and the robot's current pose when all
        frames have them, else the shifted ones."""
        if robot is not None and all(f is not None for _, _, f in self.frames):
            return [(t, aux.rotate(f - robot.pos, -robot.ang)) for t, _, f in self.frames]
        return [(t, p) for t, p, _ in self.frames]

    def last(self, robot: Optional[Robot]) -> Optional[tuple[aux.Point, float]]:
        """The latest frame: (ball position now in the robot frame, its capture monotonic()); None without
        frames."""
        if not self.frames:
            return None
        t, p = self._robot_frame(robot)[-1]
        return p, t

    def fit(self, robot: Optional[Robot]) -> Optional[tuple[aux.Point, aux.Point, float]]:
        """Least squares line through the frames: (a point of the path, the ball velocity [m/s], monotonic()
        when the ball was at that point), robot frame; None with fewer than BALL_PATH_MIN_FRAMES frames."""
        frames = self._robot_frame(robot)
        n = len(frames)
        if n < config.BALL_PATH_MIN_FRAMES:
            return None
        t_mean = sum(t for t, _ in frames) / n
        p_mean = aux.Point(sum(p.x for _, p in frames) / n, sum(p.y for _, p in frames) / n)
        t_var = sum((t - t_mean) ** 2 for t, _ in frames)
        if t_var == 0:
            return None
        vel = aux.Point(
            sum((t - t_mean) * (p.x - p_mean.x) for t, p in frames),
            sum((t - t_mean) * (p.y - p_mean.y) for t, p in frames),
        ) / t_var
        return p_mean, vel, t_mean


def robot_pose_at(robot: Robot, t: float) -> tuple[aux.Point, float]:
    """The robot's SSL vision pose (position, heading) at monotonic() t, from its last one and its speed."""
    dt = t - robot.last_update
    return robot.pos + robot.vel * dt, robot.ang + robot.angle_vel * dt


def calib_to_robot_frame(p: aux.Point) -> aux.Point:
    """The calibration (homography_matrix.npy) has y to the right of the image; the robot's y is left."""
    return aux.Point(p.x, -p.y)
