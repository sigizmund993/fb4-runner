"""Fits homography_matrix.npy to the data of calib_record.py and compares it with the current one.

The fisheye parameters (camera_params_fisheye.npz) stay: only the plane the ball lies on is refitted.
Uses only moments when the ball pixel and the ball in the robot frame hold still: the camera and
SSL vision have different delays.

    make calib-fit [DATA="calib_data/a.csv calib_data/b.csv"] [SAVE=1]
"""
import argparse
import csv
from pathlib import Path

import cv2
import numpy as np

from to_real_coords.calib_record import DATA_DIR
from to_real_coords.coord_converter import apply_homography, load_camera, undistort

CALIB_DIR = Path(__file__).parent
STILL_S = 0.25  # half of the window that has to hold still
STILL_PX = 3  # [px] the detector's bbox centre jitters by a pixel or two
STILL_M = 0.01
CELL_PX = 10  # one point per image cell, else spots where the robot stood longer weigh more
RANSAC_M = 0.03
MIN_POINTS = 15
DIST_BINS_M = (0.0, 0.25, 0.5, 0.75, 1.0, 1.5, np.inf)


def load(path: Path) -> dict[str, np.ndarray]:
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]} if rows else {}


def ball_in_robot_frame(d: dict[str, np.ndarray]) -> np.ndarray:
    """(N, 2) [m] in the calibration's frame: x forward, y right (ball_detector.calib_to_robot_frame)."""
    dx, dy = d["ball_x"] - d["robot_x"], d["ball_y"] - d["robot_y"]
    c, s = np.cos(d["robot_ang"]), np.sin(d["robot_ang"])
    return np.stack((c * dx + s * dy, s * dx - c * dy), axis=1)


def still_points(d: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """(pixels, ball) of the rows inside a window of +-STILL_S where both hold still."""
    t, px, ball = d["t"], np.stack((d["cx"], d["cy"]), axis=1), ball_in_robot_frame(d)
    lo = np.searchsorted(t, t - STILL_S)
    hi = np.searchsorted(t, t + STILL_S, side="right")
    keep = np.zeros(len(t), dtype=bool)
    for i in range(len(t)):
        a, b = lo[i], hi[i]
        keep[i] = (t[b - 1] - t[a] >= 1.5 * STILL_S  # no gap in the data around the row
                   and np.ptp(px[a:b], axis=0).max() <= STILL_PX
                   and np.ptp(ball[a:b], axis=0).max() <= STILL_M)
    return px[keep], ball[keep]


def per_cell(px: np.ndarray, ball: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    cells: dict[tuple[int, int], list[int]] = {}
    for i, key in enumerate(map(tuple, (px // CELL_PX).astype(int))):
        cells.setdefault(key, []).append(i)
    idx = list(cells.values())
    return np.array([np.median(px[i], axis=0) for i in idx]), np.array([np.median(ball[i], axis=0) for i in idx])


def report(name: str, H_mat: np.ndarray, undist: np.ndarray, ball: np.ndarray) -> None:
    err = apply_homography(H_mat, undist) - ball
    norm = np.linalg.norm(err, axis=1) * 100
    dist = np.linalg.norm(ball, axis=1)
    print(f"{name}: error [cm] median {np.median(norm):.1f}  p90 {np.percentile(norm, 90):.1f}  "
          f"max {norm.max():.1f}  mean bias x {err[:, 0].mean() * 100:+.1f} y(right) {err[:, 1].mean() * 100:+.1f}")
    for a, b in zip(DIST_BINS_M, DIST_BINS_M[1:]):
        sel = (dist >= a) & (dist < b)
        if sel.any():
            print(f"    {a:.1f}..{b:.1f} m  n {sel.sum():4d}  median {np.median(norm[sel]):5.1f}  "
                  f"p90 {np.percentile(norm[sel], 90):5.1f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("data", nargs="*", type=Path, help=f"default: all of {DATA_DIR}")
    parser.add_argument("--save", action="store_true", help="write homography_matrix.npy")
    args = parser.parse_args()

    paths = args.data or sorted(DATA_DIR.glob("*.csv"))
    pxs, balls = [], []
    for path in paths:
        d = load(path)
        px, ball = still_points(d) if d else (np.empty((0, 2)), np.empty((0, 2)))
        print(f"{path}: {len(d.get('t', []))} rows, {len(px)} still")
        pxs.append(px)
        balls.append(ball)
    px, ball = per_cell(np.concatenate(pxs), np.concatenate(balls))
    print(f"{len(px)} points ({CELL_PX} px cells)")
    if len(px) < MIN_POINTS:
        raise SystemExit(f"need at least {MIN_POINTS}: put the ball in more places of the image")

    undist = undistort(px, *load_camera(str(CALIB_DIR / "camera_params_fisheye.npz")))
    H_new, inliers = cv2.findHomography(undist, ball, cv2.RANSAC, RANSAC_M)
    H_new /= H_new[2, 2]
    inliers = inliers.ravel().astype(bool)
    print(f"RANSAC ({RANSAC_M * 100:.0f} cm): {inliers.sum()} inliers, {(~inliers).sum()} outliers")
    for i in np.flatnonzero(~inliers):
        print(f"    outlier px ({px[i, 0]:.0f}, {px[i, 1]:.0f})  vision ({ball[i, 0]:.2f}, {ball[i, 1]:.2f}) m")

    H_old = np.load(CALIB_DIR / "homography_matrix.npy")
    print("\nall points")
    report("old", H_old, undist, ball)
    report("new", H_new, undist, ball)
    print("\ninliers")
    report("old", H_old, undist[inliers], ball[inliers])
    report("new", H_new, undist[inliers], ball[inliers])

    if args.save:
        np.save(CALIB_DIR / "homography_matrix.npy", H_new)
        print(f"\nsaved {CALIB_DIR / 'homography_matrix.npy'}")
    else:
        print("\nnot saved, run with SAVE=1")


if __name__ == "__main__":
    main()
