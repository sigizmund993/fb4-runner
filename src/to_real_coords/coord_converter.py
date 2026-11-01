import cv2
import numpy as np
from utils.auxiliary.aux import Point

W, H = 1280, 960  # the detector's frame (run.sh, yolo26_post.cpp)
BALANCE = 1.0


def load_camera(cam_params_path: str) -> tuple[np.ndarray, np.ndarray]:
    with np.load(cam_params_path) as data:
        return data['mtx'], data['dist']


def undistort(pixels: np.ndarray, mtx: np.ndarray, dist: np.ndarray) -> np.ndarray:
    """(N, 2) fisheye pixels -> (N, 2) pixels of the undistorted image the homography is made for."""
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(mtx, dist, (W, H), np.eye(3), balance=BALANCE)
    pts = pixels.astype(np.float32).reshape(-1, 1, 2)
    return cv2.fisheye.undistortPoints(pts, mtx, dist, R=np.eye(3), P=new_K).reshape(-1, 2)


def apply_homography(H_mat: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """(N, 2) undistorted pixels -> (N, 2) [m], x forward, y right."""
    homog = np.hstack((pts, np.ones((pts.shape[0], 1), dtype=pts.dtype)))
    real = homog @ H_mat.T
    return real[:, :2] / real[:, 2:]


class CoordConverter:
    def __init__(self,cam_params_path:str,hom_matrix_path:str):
        mtx, dist = load_camera(cam_params_path)
        H_mat = np.load(hom_matrix_path)

        self.h, self.w = H, W

        grid_y, grid_x = np.mgrid[0:self.h, 0:self.w]
        real = apply_homography(H_mat, undistort(np.stack((grid_x, grid_y), axis=-1).reshape(-1, 2), mtx, dist))

        self.LUT_X = real[:, 0].reshape(self.h, self.w)
        self.LUT_Y = real[:, 1].reshape(self.h, self.w)

    def get_coords(self, p:Point)->Point:
        u_idx = max(0, min(p.x, self.w - 1))
        v_idx = max(0, min(p.y, self.h - 1))

        x_m = self.LUT_X[v_idx, u_idx]
        y_m = self.LUT_Y[v_idx, u_idx]

        return Point(round(float(x_m), 3), round(float(y_m), 3))
