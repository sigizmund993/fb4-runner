import struct
from multiprocessing.shared_memory import SharedMemory
from pathlib import Path
from time import monotonic, sleep
from typing import Optional

import config
from to_real_coords.coord_converter import CoordConverter
from utils.auxiliary import aux
from vision.filters import AlphaBetaFilter

CALIB_DIR = Path(__file__).parent / "to_real_coords"
# Written by yolo26_post.cpp when no ball is detected
NO_BALL = (1281, 961)
# The detector (run.sh) creates the shared memory once the Hailo model is loaded
SHM_WAIT_S = 30.0
# The shared memory has no frame counter: a new frame is a changed pixel, or this long without one
# (the ball stands still); the camera runs at 30 fps (run.sh)
SAME_PIXEL_S = 1.5 / 30


class BallDetector:
    def __init__(self):
        self.shm = self._attach_shm()
        self.coord_converter = CoordConverter(
            str(CALIB_DIR / "camera_params_fisheye.npz"),
            str(CALIB_DIR / "homography_matrix.npy"),
        )
        self.p_pos: Optional[aux.Point] = None  # [px]
        self.r_pos: Optional[aux.Point] = None  # [m], robot frame: x forward, y left
        self.r_vel: Optional[aux.Point] = None  # [m/s], robot frame, relative to the robot
        self._filters: Optional[tuple[AlphaBetaFilter, AlphaBetaFilter]] = None
        self._last_px: Optional[tuple[int, int]] = None
        self._last_sample = float("-inf")

    @staticmethod
    def _attach_shm() -> SharedMemory:
        deadline = monotonic() + SHM_WAIT_S
        while True:
            try:
                return SharedMemory(name=config.SHM_NAME)
            # ValueError: created but not sized yet (ftruncate in yolo26_post.cpp)
            except (FileNotFoundError, ValueError):
                if monotonic() > deadline:
                    raise
                sleep(0.05)

    def update(self):
        cx, cy = struct.unpack("<HH", self.shm.buf[:4])
        if (cx, cy) == NO_BALL:
            self.r_pos = None
            self.p_pos = None
            self.r_vel = None
            self._filters = None
            return
        now = monotonic()
        if (cx, cy) == self._last_px and now - self._last_sample < SAME_PIXEL_S:
            return  # still the same camera frame
        self._last_px, self._last_sample = (cx, cy), now
        self.p_pos = aux.Point(cx, cy)
        self.r_pos = calib_to_robot_frame(self.coord_converter.get_coords(self.p_pos))
        if self._filters is None:  # the ball has just appeared
            self._filters = (AlphaBetaFilter(0.6, 0.15, self.r_pos.x), AlphaBetaFilter(0.6, 0.15, self.r_pos.y))
            self.r_vel = aux.Point(0, 0)
        else:
            fx, fy = self._filters
            fx.update(self.r_pos.x)
            fy.update(self.r_pos.y)
            self.r_vel = aux.Point(fx.dx, fy.dx)


def calib_to_robot_frame(p: aux.Point) -> aux.Point:
    """The calibration (homography_matrix.npy) has y to the right of the image; the robot's y is left."""
    return aux.Point(p.x, -p.y)
