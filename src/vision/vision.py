from time import sleep, time

from utils.auxiliary import aux
from vision.vision_receiver import Geometry, Team, VisionClient
from vision.filters import AlphaBetaFilter


class Ball:
    def __init__(self, pos: aux.Point):
        self.pos = pos
        self.vel = aux.Point(0, 0)
        self.last_update = float("-inf")  # time() of the last vision detection; never seen yet
        self.filter_x = AlphaBetaFilter(0.6, 0.15, self.pos.x)
        self.filter_y = AlphaBetaFilter(0.6, 0.15, self.pos.y)

    def update(self, pos: aux.Point):
        self.filter_x.update(pos.x)
        self.filter_y.update(pos.y)
        self.vel = aux.Point(self.filter_x.dx, self.filter_y.dx)
        self.pos = aux.Point(self.filter_x.x, self.filter_y.x)
        self.last_update = time()

    def is_visible(self, now: float, timeout: float) -> bool:
        return now - self.last_update <= timeout


class Robot:
    def __init__(self, pos: aux.Point, ang: float, r_id: int, team: Team) -> None:
        self.pos = pos
        self.ang = ang
        self.id = r_id
        self.team = team
        self.vel = aux.Point(0, 0)
        self.angle_vel = 0.0
        self.last_update = float("-inf")  # time() of the last vision detection; never seen yet
        self.filter_x = AlphaBetaFilter(0.6, 0.15, self.pos.x)
        self.filter_y = AlphaBetaFilter(0.6, 0.15, self.pos.y)
        self.filter_ang = AlphaBetaFilter(0.6, 0.15, self.ang, True)

    def update(self, pos: aux.Point, ang: float) -> None:
        self.filter_x.update(pos.x)
        self.filter_y.update(pos.y)
        self.filter_ang.update(ang)
        self.vel = aux.Point(self.filter_x.dx, self.filter_y.dx)
        self.pos = aux.Point(self.filter_x.x, self.filter_y.x)
        self.angle_vel = self.filter_ang.dx
        self.ang = self.filter_ang.x
        self.last_update = time()

    def is_visible(self, now: float, timeout: float) -> bool:
        return now - self.last_update <= timeout


class Field:
    def __init__(self, client: VisionClient) -> None:
        self.client = client
        self.ball = Ball(aux.Point(0, 0))
        self.yellow_robots = [Robot(aux.Point(0, 0), 0, i, Team.YELLOW) for i in range(16)]
        self.blue_robots = [Robot(aux.Point(0, 0), 0, i, Team.BLUE) for i in range(16)]
        self.geometry = Geometry(0, 0, 0, 0, 0, 0, 0, 0)
        sleep(1)
        self.update()

    def get_robot(self, team: Team, robot_id: int) -> Robot:
        robots = self.yellow_robots if team.value == Team.YELLOW.value else self.blue_robots
        return robots[robot_id]

    def update(self):
        detection = self.client.get_detection()
        if len(detection.balls) > 0:
            self.ball.update(detection.balls[0].pos / 1000)
        for rbt_det in detection.robots:
            self.get_robot(rbt_det.team, rbt_det.robot_id).update(rbt_det.pos / 1000, rbt_det.orientation)
        if detection.geometry:
            self.geometry = detection.geometry
