import math
from time import time


class AlphaBetaFilter:
    def __init__(self, alpha: float, beta: float, x: float, is_angle: bool = False) -> None:
        self.alpha = alpha
        self.beta = beta
        self.x = x
        self.dx = 0.0
        self.is_angle = is_angle
        self.last_update = time()

    def update(self, new_x: float) -> None:
        now = time()
        dt = now - self.last_update
        self.last_update = now
        pred_x = self.x + self.dx * dt
        if self.is_angle:
            res_x = math.atan2(math.sin(new_x - pred_x), math.cos(new_x - pred_x))
        else:
            res_x = new_x - pred_x
        self.x = pred_x + self.alpha * res_x
        if dt > 0:
            self.dx = self.dx + (self.beta / dt) * res_x
