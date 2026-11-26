#!/usr/bin/env python3
"""Drives the robot through odometry calibration runs while the runner logs (FB4_ODOM_LOG=1).

Runs on the robot's RPi next to the runner: sends commands to it on localhost like the strategy does
(stop the strategy and teleop for this robot first) and reads SSL vision itself for the safety box.
Before each run the robot is put at its start with GLOBAL_COORDINATES (the runner's go-to-point by
vision); the runs themselves are open-loop speed commands, so the log shows what the odometry
makes of known motions:

  rot    spinning in place: 3 turns each way slow and fast, then direction reversals every second
  lines  straight there and back without turning (heading held), along x, y and both diagonals,
         at 0.5, 1.0 and 1.5 m/s
  arcs   circles (constant speed in the robot frame) both ways at several speeds and radii
  spin   straight across the field while spinning (constant speed in the field frame)
  free   random go-to-point targets with random headings, for checking the fit

A run stops (zero speed, then back to the centre) when the robot leaves the box or vision loses it.
The box is the field (vision geometry) shrunk by --margin, or --box. Ctrl+C stops the robot.
Run marks go to logs/odom_drive_<date>.csv on the same monotonic clock as the odometry log.

  cd /root/fb4-runner && PYTHONPATH=. venv/bin/python tools/odom_drive.py [--runs rot,lines] [--dry-run]
"""
import argparse
import csv
import math
import random
import socket
import struct
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

import config  # noqa: E402
from ssl_packet_package.protopy.spbunited.robot import control_pb2 as pb  # noqa: E402
from ssl_packet_package.protopy.ssl.vision.ssl_vision_wrapper_pb2 import SSL_WrapperPacket  # noqa: E402

RATE_HZ = 50
VISION_STALE_S = 0.3  # a run stops when vision hasn't seen the robot this long
BOX_SLACK = 0.15  # [m] past the box before a run stops (the box already has the margin)
CAM_DELAY_S = 0.03  # vision pose -> now, for the field-frame speed of the spin run
GOTO_TOL_M, GOTO_TOL_RAD, GOTO_TIMEOUT_S = 0.05, math.radians(5), 10.0
ALL_RUNS = ("rot", "lines", "arcs", "spin", "free")


class Vision(threading.Thread):
    """The robot's latest SSL vision pose (x, y [m], heading [rad]) and the field size."""

    def __init__(self, team: str, robot_id: int):
        super().__init__(daemon=True)
        self.team, self.robot_id = team, robot_id
        self.pose = None
        self.t_pose = -math.inf
        self.field = None  # (length, width) [m]
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("224.5.23.2", 10006))
        mreq = struct.pack("4sl", socket.inet_aton("224.5.23.2"), socket.INADDR_ANY)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    def run(self):
        while True:
            try:
                p = SSL_WrapperPacket.FromString(self.sock.recv(65536))
            except Exception:
                continue
            if p.HasField("geometry") and p.geometry.field.field_length > 0:
                self.field = (p.geometry.field.field_length / 1000, p.geometry.field.field_width / 1000)
            if p.HasField("detection"):
                robots = p.detection.robots_yellow if self.team == "yellow" else p.detection.robots_blue
                for r in robots:
                    if r.robot_id == self.robot_id:
                        with self.lock:
                            self.pose = (r.x / 1000, r.y / 1000, r.orientation)
                            self.t_pose = time.monotonic()

    def get(self):
        """The pose, None when vision hasn't seen the robot for VISION_STALE_S."""
        with self.lock:
            return self.pose if time.monotonic() - self.t_pose < VISION_STALE_S else None


class Abort(Exception):
    pass


class Driver:
    def __init__(self, args, vision: Vision, box):
        self.vision, self.box, self.dry = vision, box, args.dry_run
        self.addr = (args.host, args.port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cx, self.cy = (box[0] + box[1]) / 2, (box[2] + box[3]) / 2
        self.hx, self.hy = (box[1] - box[0]) / 2, (box[3] - box[2]) / 2
        self.marks = None
        if not self.dry:
            logs = ROOT / "logs"
            logs.mkdir(exist_ok=True)
            path = logs / f"odom_drive_{datetime.now():%Y-%m-%d_%H-%M-%S}.csv"
            self.marks_file = open(path, "w", newline="")
            self.marks = csv.writer(self.marks_file)
            self.marks.writerow(("t", "wall", "event", "run", "detail"))
            print(f"run marks: {path}")
        self.total = 0.0  # [s] planned duration (dry run)

    # --- commands
    def _send(self, msg):
        msg.robot_id = config.ROBOT_ID
        if not self.dry:
            self.sock.sendto(msg.SerializeToString(), self.addr)

    def send_vel(self, vx: float, vy: float, w: float):
        """Robot frame [m/s], w [rad/s]; w = 0 holds the heading."""
        msg = pb.RobotCommand(robot_control_type=pb.RobotControlType.NEW_FORMAT)
        s = msg.new_format.speed_control
        s.vel_x, s.vel_y, s.angular_velocity = vx, vy, w
        kad = msg.new_format.kicker_and_dribbler
        kad.kicker_mode, kad.kicker_voltage, kad.dribbler_setting = 0, 0, 0
        self._send(msg)

    def send_turn(self, delta: float):
        """Stand and turn by delta [rad] from the current heading (the firmware's angle mode)."""
        msg = pb.RobotCommand(robot_control_type=pb.RobotControlType.NEW_FORMAT)
        s = msg.new_format.speed_control
        s.vel_x, s.vel_y, s.delta_angle = 0, 0, delta
        kad = msg.new_format.kicker_and_dribbler
        kad.kicker_mode, kad.kicker_voltage, kad.dribbler_setting = 0, 0, 0
        self._send(msg)

    def send_goto(self, x: float, y: float, ang: float):
        msg = pb.RobotCommand(robot_control_type=pb.RobotControlType.GLOBAL_COORDINATES)
        g = msg.global_coordinates
        g.x, g.y, g.angle = x, y, ang
        self._send(msg)

    def mark(self, event: str, run: str = "", detail: str = ""):
        if self.dry and event != "start":
            return
        print(f"  {run} {detail}" if self.dry else f"  {event:6s} {run} {detail}")
        if self.marks:
            self.marks.writerow((f"{time.monotonic():.4f}", f"{time.time():.4f}", event, run, detail))
            self.marks_file.flush()

    # --- primitives
    def loop(self, duration: float, step):
        """Calls step(t) at RATE_HZ for duration [s]; step sends the command, may raise Abort."""
        self.total += duration
        if self.dry:
            return
        t0 = time.monotonic()
        while (t := time.monotonic() - t0) < duration:
            step(t)
            time.sleep(1 / RATE_HZ)

    def check(self):
        pose = self.vision.get()
        if pose is None:
            raise Abort("vision lost the robot")
        x, y, _ = pose
        b = self.box
        if not (b[0] - BOX_SLACK <= x <= b[1] + BOX_SLACK and b[2] - BOX_SLACK <= y <= b[3] + BOX_SLACK):
            raise Abort(f"left the box at ({x:.2f}, {y:.2f})")
        return pose

    def stand(self, duration: float):
        self.loop(duration, lambda t: self.send_vel(0, 0, 0))

    def goto(self, x: float, y: float, ang: float = 0.0):
        self.total += 3.0
        if self.dry:
            return
        t0, settled = time.monotonic(), None
        while time.monotonic() - t0 < GOTO_TIMEOUT_S:
            pose = self.vision.get()
            at_point = pose is not None and math.hypot(pose[0] - x, pose[1] - y) < GOTO_TOL_M
            d_ang = math.remainder(ang - pose[2], 2 * math.pi) if pose is not None else 0.0
            if at_point:
                # the runner's go-to-point stops within 5 cm without turning: turn by the heading error
                self.send_turn(d_ang)
            else:
                self.send_goto(x, y, ang)
            near = at_point and abs(d_ang) < GOTO_TOL_RAD
            settled = (settled or time.monotonic()) if near else None
            if settled and time.monotonic() - settled > 0.3:
                return
            time.sleep(1 / RATE_HZ)
        print(f"  goto ({x:.2f}, {y:.2f}) timed out, going on from where the robot is")

    def drive_robot(self, duration: float, vx: float, vy: float, w: float):
        """Constant speed in the robot frame."""
        def step(t):
            self.check()
            self.send_vel(vx, vy, w)
        self.loop(duration, step)

    def drive_field(self, duration: float, vfx: float, vfy: float, w: float):
        """Constant speed in the field frame while turning at w: the robot-frame speed follows the
        vision heading (extrapolated by the commanded w over the vision delay)."""
        def step(t):
            _, _, th = self.check()
            th += w * (CAM_DELAY_S + 1 / RATE_HZ)
            c, s = math.cos(th), math.sin(th)
            self.send_vel(vfx * c + vfy * s, -vfx * s + vfy * c, w)
        self.loop(duration, step)

    def run(self, name: str, detail: str, start, body):
        """Goto start, stand 2 s (the odometry fit anchors here), body(), stand. Abort: stop, recentre."""
        self.mark("start", name, detail)
        try:
            self.goto(*start)
            self.stand(2.0)
            body()
            self.stand(1.5)
            self.mark("end", name, detail)
        except Abort as e:
            self.mark("abort", name, f"{detail}: {e}")
            self.stand(1.0)
            self.goto(self.cx, self.cy, 0.0)

    # --- runs
    def rot(self):
        c = (self.cx, self.cy, 0.0)
        for w, turns in ((1.5, 3), (-1.5, 3), (4.0, 3), (-4.0, 3)):
            self.run("rot", f"w={w:+.1f} turns={turns}", c,
                     lambda w=w, turns=turns: self.drive_robot(turns * 2 * math.pi / abs(w), 0, 0, w))

        def reversals():
            for i in range(10):
                self.drive_robot(1.0, 0, 0, 3.0 if i % 2 == 0 else -3.0)
        self.run("rot", "reversals w=+-3.0 every 1 s", c, reversals)

    def lines(self):
        r = math.sqrt(0.5)
        for ux, uy, axis in ((1, 0, "x"), (0, 1, "y"), (r, r, "diag+"), (r, -r, "diag-")):
            half = min(self.hx / abs(ux) if ux else math.inf, self.hy / abs(uy) if uy else math.inf)
            length = min(2.0, 1.6 * half)
            start = (self.cx - ux * length / 2, self.cy - uy * length / 2, 0.0)
            for v in (0.5, 1.0, 1.5):
                def body(ux=ux, uy=uy, v=v, length=length):
                    self.drive_robot(length / v, ux * v, uy * v, 0)
                    self.stand(1.5)
                    self.drive_robot(length / v, -ux * v, -uy * v, 0)
                self.run("lines", f"{axis} v={v} L={length:.2f}", start, body)

    def arcs(self):
        for v, w in ((0.5, 1.5), (0.5, 3.0), (1.0, 1.5), (1.0, 3.0)):
            radius = v / w
            if radius > 0.8 * min(self.hx, self.hy):
                continue
            for sign in (1, -1):
                # the circle is centred on the box centre: start on it, facing +x
                start = (self.cx, self.cy - sign * radius, 0.0)
                self.run("arcs", f"v={v} w={sign * w:+.1f} R={radius:.2f}", start,
                         lambda v=v, w=sign * w: self.drive_robot(1.5 * 2 * math.pi / abs(w), v, 0, w))

    def spin(self):
        length = min(2.0, 1.6 * self.hx)
        start = (self.cx - length / 2, self.cy, 0.0)
        for v in (0.5, 1.0):
            for w in (3.0, -3.0):
                def body(v=v, w=w):
                    self.drive_field(length / v, v, 0, w)
                    self.stand(1.5)
                    self.drive_field(length / v, -v, 0, w)
                self.run("spin", f"v={v} w={w:+.1f} L={length:.2f}", start, body)

    def free(self, duration: float):
        def body():
            rnd = random.Random(1)
            t_end = time.monotonic() + duration
            self.total += duration
            while self.dry is False and time.monotonic() < t_end:
                x = rnd.uniform(self.box[0] + 0.2, self.box[1] - 0.2)
                y = rnd.uniform(self.box[2] + 0.2, self.box[3] - 0.2)
                ang = rnd.uniform(-math.pi, math.pi)
                t0 = time.monotonic()
                while time.monotonic() - t0 < 3.0:
                    self.check()
                    self.send_goto(x, y, ang)
                    pose = self.vision.get()
                    if pose and math.hypot(pose[0] - x, pose[1] - y) < 0.1:
                        break
                    time.sleep(1 / RATE_HZ)
        self.run("free", f"{duration:.0f} s of random go-to-point", (self.cx, self.cy, 0.0), body)

    def stop(self):
        """NEW_FORMAT zeros (a GLOBAL_COORDINATES target never expires in the runner)."""
        for _ in range(RATE_HZ // 2):
            self.send_vel(0, 0, 0)
            time.sleep(1 / RATE_HZ)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", default=",".join(ALL_RUNS), help=f"comma-separated, of {', '.join(ALL_RUNS)}")
    ap.add_argument("--box", help="x0,x1,y0,y1 [m] field area to drive in (default: the field minus --margin)")
    ap.add_argument("--margin", type=float, default=0.4, help="[m] kept from the field edge (default 0.4)")
    ap.add_argument("--free-s", type=float, default=60.0, help="[s] of the free run (default 60)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=config.CMD_PORT)
    ap.add_argument("--dry-run", action="store_true", help="print the plan and its duration, send nothing")
    ap.add_argument("--yes", action="store_true", help="don't ask before driving")
    args = ap.parse_args()
    runs = [r for r in args.runs.split(",") if r]
    if unknown := set(runs) - set(ALL_RUNS):
        sys.exit(f"unknown runs: {', '.join(sorted(unknown))}")

    vision = Vision(config.ROBOT_TEAM, config.ROBOT_ID)
    vision.start()
    if args.box:
        box = tuple(float(v) for v in args.box.split(","))
    else:
        t0 = time.monotonic()
        while vision.field is None and time.monotonic() - t0 < 3.0:
            time.sleep(0.05)
        if vision.field is None:
            sys.exit("no field geometry from vision in 3 s: pass --box x0,x1,y0,y1")
        hl, hw = vision.field[0] / 2 - args.margin, vision.field[1] / 2 - args.margin
        box = (-hl, hl, -hw, hw)
    print(f"robot {config.ROBOT_TEAM} {config.ROBOT_ID}, box x {box[0]:.2f}..{box[1]:.2f} y {box[2]:.2f}..{box[3]:.2f} m")

    plan = Driver(argparse.Namespace(**{**vars(args), "dry_run": True}), vision, box)
    for r in runs:
        plan.free(args.free_s) if r == "free" else getattr(plan, r)()
    print(f"runs: {', '.join(runs)}, about {plan.total / 60:.1f} min")
    if args.dry_run:
        return

    t0 = time.monotonic()
    while vision.get() is None:
        if time.monotonic() - t0 > 3.0:
            sys.exit("vision doesn't see the robot")
        time.sleep(0.05)
    print(f"vision pose {tuple(round(v, 2) for v in vision.get())}")
    if not args.yes:
        input("stop the strategy/teleop for this robot, clear the box, Enter to drive (Ctrl+C stops) ")

    drv = Driver(args, vision, box)
    try:
        for r in runs:
            drv.free(args.free_s) if r == "free" else getattr(drv, r)()
        drv.goto(drv.cx, drv.cy, 0.0)
        drv.mark("done")
    except KeyboardInterrupt:
        drv.mark("abort", "", "Ctrl+C")
    finally:
        drv.stop()


if __name__ == "__main__":
    main()
