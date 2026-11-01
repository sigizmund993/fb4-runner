"""Records data for the camera calibration: the ball pixel from the onboard detector next to the ball and
the robot from SSL vision, one row per vision packet that has both.

Runs on the RPi with the detector running (run.sh, it is part of fb4-runner.service). main.py may run too:
both read the vision multicast and /ball_shm, so the robot can be driven with teleop meanwhile.

    make calib-record [TEAM=blue] [ID=3]

Writes calib_data/<date>.csv; make calib-fit turns it into homography_matrix.npy.
"""
import argparse
import csv
import time
from datetime import datetime
from pathlib import Path

import attr

import config
from ball_detector import NO_BALL, attach_shm, read_pixel
from ssl_packet_package.protopy.ssl.vision.ssl_vision_wrapper_pb2 import SSL_WrapperPacket
from vision.vision_receiver import SocketReader, VisionClient

DATA_DIR = Path(__file__).resolve().parents[2] / "calib_data"
COLUMNS = ("t", "cx", "cy", "robot_x", "robot_y", "robot_ang", "ball_x", "ball_y")
# The robot and the ball may come from different SSL cameras: pair them only when captured this close
MAX_CAPTURE_GAP_S = 0.05
PRINT_PERIOD_S = 1.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--team", default=config.ROBOT_TEAM, choices=("yellow", "blue"))
    parser.add_argument("--id", type=int, default=config.ROBOT_ID)
    args = parser.parse_args()

    shm = attach_shm()
    vision = attr.fields(VisionClient)
    reader = SocketReader(ip=vision.multicast_ip.default, port=vision.multicast_port.default)
    DATA_DIR.mkdir(exist_ok=True)
    path = DATA_DIR / f"{datetime.now():%Y-%m-%d_%H-%M-%S}.csv"
    print(f"robot {args.team} {args.id}, writing {path}, Ctrl+C to stop")

    robot = ball = None  # (x [m], y [m], ang [rad] | None, t_capture)
    counts = dict(rows=0, no_robot=0, no_ball=0, many_balls=0, no_pixel=0)
    next_print = time.monotonic()
    with open(path, "w", newline="") as f:
        out = csv.writer(f)
        out.writerow(COLUMNS)
        while True:
            packet = SSL_WrapperPacket.FromString(reader.read_package())
            if not packet.HasField("detection"):
                continue
            det = packet.detection
            robots = det.robots_blue if args.team == "blue" else det.robots_yellow
            mine = [r for r in robots if r.robot_id == args.id]
            if mine:
                robot = (mine[0].x / 1000, mine[0].y / 1000, mine[0].orientation, det.t_capture)
            if len(det.balls) == 1:
                ball = (det.balls[0].x / 1000, det.balls[0].y / 1000, None, det.t_capture)
            elif det.balls:
                counts["many_balls"] += 1  # can't tell which one the camera sees

            if robot is None or det.t_capture - robot[3] > MAX_CAPTURE_GAP_S:
                counts["no_robot"] += 1
            elif ball is None or abs(robot[3] - ball[3]) > MAX_CAPTURE_GAP_S:
                counts["no_ball"] += 1
            elif (px := read_pixel(shm)) == NO_BALL:
                counts["no_pixel"] += 1
            else:
                out.writerow((f"{time.monotonic():.3f}", *px, *(f"{v:.4f}" for v in robot[:3]), f"{ball[0]:.4f}",
                              f"{ball[1]:.4f}"))
                counts["rows"] += 1

            if time.monotonic() >= next_print:
                f.flush()
                print("  ".join(f"{k} {v}" for k, v in counts.items()))
                next_print += PRINT_PERIOD_S


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
