import struct
from collections import deque
from multiprocessing.shared_memory import SharedMemory
from pathlib import Path
from time import monotonic, sleep
from typing import Optional

import config
from to_real_coords.coord_converter import CoordConverter
from utils.auxiliary import aux

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
        """Reads the detector; True on a new camera frame with the ball."""
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
        self.r_pos = calib_to_robot_frame(self.coord_converter.get_coords(self.p_pos))
        return True


class BallPath:
    """The ball's path on the field from the last camera frames, robot frame (x forward, y left).

    The robot moves meanwhile: the stored positions are shifted back by its own motion (the commanded speed),
    so they stay where the ball was on the field and the path is not tilted by the robot's speed.
    The heading is taken as held: RICOCHET and CATCH_BALL turn to their angle before the ball comes."""

    def __init__(self) -> None:
        self.frames: deque[tuple[float, aux.Point]] = deque()  # (monotonic, ball position)
        self._last_update = monotonic()

    def update(self, ball: Optional[aux.Point], new_frame: bool, robot_vel: aux.Point) -> None:
        """ball, new_frame: BallDetector.r_pos, update(); robot_vel: the speed commanded since the last call."""
        now = monotonic()
        shift = robot_vel * (now - self._last_update)
        self._last_update = now
        if ball is None:
            self.frames.clear()
            return
        self.frames = deque((t, p - shift) for t, p in self.frames if now - t <= config.BALL_PATH_S)
        if new_frame:
            self.frames.append((now, ball))

    def last(self) -> Optional[tuple[aux.Point, float]]:
        """The latest frame: (ball position, monotonic() of the frame); None without frames."""
        if not self.frames:
            return None
        t, p = self.frames[-1]
        return p, t

    def fit(self) -> Optional[tuple[aux.Point, aux.Point, float]]:
        """Least squares line through the frames: (a point of the path, the ball velocity [m/s], monotonic()
        when the ball was at that point); None with fewer than BALL_PATH_MIN_FRAMES frames."""
        n = len(self.frames)
        if n < config.BALL_PATH_MIN_FRAMES:
            return None
        t_mean = sum(t for t, _ in self.frames) / n
        p_mean = aux.Point(sum(p.x for _, p in self.frames) / n, sum(p.y for _, p in self.frames) / n)
        t_var = sum((t - t_mean) ** 2 for t, _ in self.frames)
        if t_var == 0:
            return None
        vel = aux.Point(
            sum((t - t_mean) * (p.x - p_mean.x) for t, p in self.frames),
            sum((t - t_mean) * (p.y - p_mean.y) for t, p in self.frames),
        ) / t_var
        return p_mean, vel, t_mean


def calib_to_robot_frame(p: aux.Point) -> aux.Point:
    """The calibration (homography_matrix.npy) has y to the right of the image; the robot's y is left."""
    return aux.Point(p.x, -p.y)
