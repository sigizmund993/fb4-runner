import struct
from multiprocessing.shared_memory import SharedMemory
from pathlib import Path
from time import sleep
from typing import Optional

import config
from to_real_coords.coord_converter import CoordConverter
from utils.auxiliary import aux

CALIB_DIR = Path(__file__).parent / "to_real_coords"
# Written by yolo26_post.cpp when no ball is detected
NO_BALL = (1281, 961)


class BallDetector:
    def __init__(self):
        sleep(0.1)
        self.shm = SharedMemory(name=config.SHM_NAME)
        self.coord_converter = CoordConverter(
            str(CALIB_DIR / "camera_params_fisheye.npz"),
            str(CALIB_DIR / "homography_matrix.npy"),
        )
        self.p_pos: Optional[aux.Point] = None
        self.r_pos: Optional[aux.Point] = None

    def update(self):
        cx, cy = struct.unpack("<HH", self.shm.buf[:4])
        if (cx, cy) == NO_BALL:
            self.r_pos = None
            self.p_pos = None
        else:
            self.p_pos = aux.Point(cx, cy)
            self.r_pos = self.coord_converter.get_coords(self.p_pos)
