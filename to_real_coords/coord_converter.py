import time
import cv2
import numpy as np
class CoordConverter:
    def __init__(self,cam_params_path:str,hom_matrix_path:str):
        with np.load(cam_params_path) as data:
            mtx = data['mtx']
            dist = data['dist']
        H_mat = np.load(hom_matrix_path)

        self.h, self.w = 960, 1280 
        balance = 0.0


        grid_y, grid_x = np.mgrid[0:self.h, 0:self.w]
        distorted_pts = np.stack((grid_x, grid_y), axis=-1).astype(np.float32).reshape(-1, 1, 2)

        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(mtx, dist, (self.w, self.h), np.eye(3), balance=balance)

        undist_pts = cv2.fisheye.undistortPoints(distorted_pts, mtx, dist, R=np.eye(3), P=new_K).reshape(-1, 2)

        ones = np.ones((undist_pts.shape[0], 1), dtype=np.float32)
        homog_pts = np.hstack((undist_pts, ones))

        real_pts_raw = np.dot(H_mat, homog_pts.T).T

        X_real = real_pts_raw[:, 0] / real_pts_raw[:, 2]
        Y_real = real_pts_raw[:, 1] / real_pts_raw[:, 2]

        self.LUT_X = X_real.reshape(self.h, self.w)
        self.LUT_Y = Y_real.reshape(self.h, self.w)

    def get_coords(self, u:int , v:int):
        u_idx = max(0, min(u, self.w - 1))
        v_idx = max(0, min(v, self.h - 1))
        
        x_m = self.LUT_X[v_idx, u_idx]
        y_m = self.LUT_Y[v_idx, u_idx]
        
        return round(float(x_m), 3), round(float(y_m), 3)

