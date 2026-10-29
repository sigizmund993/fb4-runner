import struct
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


class BallDetector:
    def __init__(self):
        self.shm = self._attach_shm()
        self.coord_converter = CoordConverter(
            str(CALIB_DIR / "camera_params_fisheye.npz"),
            str(CALIB_DIR / "homography_matrix.npy"),
        )
        self.p_pos: Optional[aux.Point] = None
        self.r_pos: Optional[aux.Point] = None

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
        else:
            self.p_pos = aux.Point(cx, cy)
            self.r_pos = self.coord_converter.get_coords(self.p_pos)
