from time import sleep
from multiprocessing.shared_memory import SharedMemory
from to_real_coords.coord_converter import CoordConverter
from typing import Optional
from utils.auxiliary import aux
import config
import struct
class BallDetector:
    def __init__(self):
        sleep(0.1)
        self.shm = SharedMemory(name=config.SHM_NAME)
        self.coord_converter = CoordConverter("to_real_coords/camera_params_fisheye.npz","to_real_coords/homography_matrix.npy")
        self.p_pos:Optional[aux.Point] = None
        self.r_pos:Optional[aux.Point] = None
    def update(self):
        raw_bytes = self.shm.buf[:4]
        cx, cy = struct.unpack("HH", raw_bytes)#Ye
        if cx == 1281 and cy == 961:
            self.r_pos = None
            self.p_pos = None
        else:
            self.p_pos = aux.Point(cx,cy)
            self.r_pos = self.coord_converter.get_coords(self.p_pos)