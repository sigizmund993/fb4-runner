from time import monotonic, sleep

from utils.auxiliary import aux
from vision.vision_receiver import Geometry, Team, VisionClient
from vision.filters import AlphaBetaFilter

# Field._update_ball: a detection further than this from where the ball should be is another ball
BALL_GATE_M = 0.5
# Not seen this long, the ball is taken anew wherever it is
BALL_LOST_S = 0.2


class Ball:
    def __init__(self, pos: aux.Point):
        self.pos = pos
        self.vel = aux.Point(0, 0)
        self.last_update = float("-inf")  # monotonic() of the last vision detection; never seen yet
        self.filter_x = AlphaBetaFilter(0.6, 0.15, self.pos.x)
        self.filter_y = AlphaBetaFilter(0.6, 0.15, self.pos.y)

    def update(self, pos: aux.Point, t_capture: float):
        self.filter_x.update(pos.x, t_capture)
        self.filter_y.update(pos.y, t_capture)
        self._set_state()

    def reset(self, pos: aux.Point, t_capture: float):
        """Starts over at pos with zero velocity: another ball, or the ball seen again after being lost."""
        self.filter_x.reset(pos.x, t_capture)
        self.filter_y.reset(pos.y, t_capture)
        self._set_state()

    def predict(self, t_capture: float) -> aux.Point:
        return aux.Point(self.filter_x.predict(t_capture), self.filter_y.predict(t_capture))

    def _set_state(self):
        self.vel = aux.Point(self.filter_x.dx, self.filter_y.dx)
        self.pos = aux.Point(self.filter_x.x, self.filter_y.x)
        self.last_update = monotonic()

    def is_visible(self, timeout: float) -> bool:
        return monotonic() - self.last_update <= timeout


class Robot:
    def __init__(self, pos: aux.Point, ang: float, r_id: int, team: Team) -> None:
        self.pos = pos
        self.ang = ang
        self.id = r_id
        self.team = team
        self.vel = aux.Point(0, 0)
        self.angle_vel = 0.0
        self.last_update = float("-inf")  # monotonic() of the last vision detection; never seen yet
        self.filter_x = AlphaBetaFilter(0.6, 0.15, self.pos.x)
        self.filter_y = AlphaBetaFilter(0.6, 0.15, self.pos.y)
        self.filter_ang = AlphaBetaFilter(0.6, 0.15, self.ang, True)

    def update(self, pos: aux.Point, ang: float, t_capture: float) -> None:
        self.filter_x.update(pos.x, t_capture)
        self.filter_y.update(pos.y, t_capture)
        self.filter_ang.update(ang, t_capture)
        self.vel = aux.Point(self.filter_x.dx, self.filter_y.dx)
        self.pos = aux.Point(self.filter_x.x, self.filter_y.x)
        self.angle_vel = self.filter_ang.dx
        self.ang = self.filter_ang.x
        self.last_update = monotonic()

    def is_visible(self, timeout: float) -> bool:
        return monotonic() - self.last_update <= timeout


class Field:
    def __init__(self, client: VisionClient) -> None:
        self.client = client
        self.ball = Ball(aux.Point(0, 0))
        self.yellow_robots = [Robot(aux.Point(0, 0), 0, i, Team.YELLOW) for i in range(16)]
        self.blue_robots = [Robot(aux.Point(0, 0), 0, i, Team.BLUE) for i in range(16)]
        self.geometry = Geometry(0, 0, 0, 0, 0, 0, 0, 0)
        self._ball_capture = None  # t_capture of the last ball fed to the filter
        self._robot_captures: dict[tuple[int, int], float] = {}
        sleep(1)
        self.update()

    def get_robot(self, team: Team, robot_id: int) -> Robot:
        robots = self.yellow_robots if team.value == Team.YELLOW.value else self.blue_robots
        return robots[robot_id]

    def update(self):
        """Feeds only new captures: the detection keeps the last ball and robots between vision frames, and
        this runs every tick, more often than vision. Repeats would pull the filtered velocity to zero and keep
        a lost ball or robot "visible". The filters run on t_capture, not on when the packet came."""
        detection = self.client.get_detection()
        if detection.balls and detection.balls[0].frame_info.t_capture != self._ball_capture:
            self._ball_capture = detection.balls[0].frame_info.t_capture
            self._update_ball(detection.balls)
        for rbt_det in detection.robots:
            key = (rbt_det.team.value, rbt_det.robot_id)
            if self._robot_captures.get(key) != rbt_det.frame_info.t_capture:
                self._robot_captures[key] = rbt_det.frame_info.t_capture
                self.get_robot(rbt_det.team, rbt_det.robot_id).update(
                    rbt_det.pos / 1000, rbt_det.orientation, rbt_det.frame_info.t_capture
                )
        if detection.geometry:
            self.geometry = detection.geometry

    def _update_ball(self, balls) -> None:
        """Follows one ball: of a frame's balls the one nearest to where the tracked ball should be now, and only
        within BALL_GATE_M of it. Another ball (or a false one) elsewhere on the field, seen by another camera
        every other frame, would else make the ball jump back and forth at tens of m/s. A ball lost for
        BALL_LOST_S is taken anew, the nearest to where it was."""
        t = balls[0].frame_info.t_capture
        lost = not self.ball.is_visible(BALL_LOST_S)
        expected = self.ball.pos if lost else self.ball.predict(t)
        nearest = min(balls, key=lambda b: aux.dist(b.pos / 1000, expected))
        pos = nearest.pos / 1000
        if lost:
            self.ball.reset(pos, t)
        elif aux.dist(pos, expected) <= BALL_GATE_M:
            self.ball.update(pos, t)
