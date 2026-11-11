import math
from time import monotonic
from typing import Optional

# Velocity is not updated from captures closer than this: two SSL cameras see the ball in their overlap at
# almost the same moment, a centimetre apart, and that over a millisecond is metres per second
MIN_VEL_DT_S = 0.01
# A measurement this much older than the last one is not a late frame: the clock has restarted (SSL vision)
CLOCK_RESET_S = 1.0


class AlphaBetaFilter:
    def __init__(self, alpha: float, beta: float, x: float, is_angle: bool = False) -> None:
        self.alpha = alpha
        self.beta = beta
        self.x = x
        self.dx = 0.0
        self.is_angle = is_angle
        # Time of the last measurement: t_capture when given, else monotonic(); None until the first one
        self.last_update: Optional[float] = None

    def reset(self, x: float, t: Optional[float] = None) -> None:
        self.x = x
        self.dx = 0.0
        self.last_update = monotonic() if t is None else t

    def predict(self, t: float) -> float:
        return self.x if self.last_update is None else self.x + self.dx * (t - self.last_update)

    def update(self, new_x: float, t: Optional[float] = None) -> None:
        """t: when new_x was measured (vision t_capture), else now. An older measurement than the last one
        (another SSL camera's frame arriving late) is dropped."""
        now = monotonic() if t is None else t
        if self.last_update is None or now - self.last_update < -CLOCK_RESET_S:
            self.reset(new_x, now)
            return
        dt = now - self.last_update
        if dt < 0:
            return
        self.last_update = now
        pred_x = self.x + self.dx * dt
        if self.is_angle:
            res_x = math.atan2(math.sin(new_x - pred_x), math.cos(new_x - pred_x))
        else:
            res_x = new_x - pred_x
        self.x = pred_x + self.alpha * res_x
        if self.is_angle:
            self.x = math.atan2(math.sin(self.x), math.cos(self.x))
        if dt > 0:
            self.dx = self.dx + (self.beta / max(dt, MIN_VEL_DT_S)) * res_x
