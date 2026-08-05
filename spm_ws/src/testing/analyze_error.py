#!/usr/bin/env python3
"""
analyze_error.py — compute e = ||R_d^T R_a - I||_F over a trial.

Input: the CSV produced by run_trial.py (columns: t_rel_s, source, line).

Parses two things out of the TEENSY/DONGLE lines:
  - Desired rotation matrix R_d: a single TEENSY line with 9 comma-separated
    floats (row-major, trailing comma after every entry), from the ik.cpp
    print block:
        Serial.printf("%.4f,", R[i][j]);  (for i in 0..2, j in 0..2)
        Serial.println();  // after the outer loop
  - Actual orientation from IMU quaternion: DONGLE lines of the form
        "%.4f,%.4f,%.4f,%.4f" -> quat_i, quat_j, quat_k, quat_real
    i.e. (x, y, z, w) in the usual Hamilton convention.

Each R_d sample is matched to the nearest-in-time IMU quaternion sample
(by t_rel_s). Matches farther apart than --max-dt are dropped and reported
as skipped, rather than silently paired.

Output: a CSV with columns t_rel_s, error, dt_match, plus a quick summary
printed to stdout, and optionally a plot.

Usage:
    python analyze_error.py --in trial01.csv --out error01.csv --plot error01.png
"""

import argparse
import csv
import re
import sys

import numpy as np

MATRIX_LINE_RE = re.compile(
    r"^(-?\d+\.\d{4},){9}$"
)
QUAT_RE = re.compile(
    r"^(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)$"
)


def quat_to_R(x, y, z, w):
    """Quaternion (x,y,z,w) -> 3x3 rotation matrix."""
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return None
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


# Fixed IMU-mount axis correction: the BNO085 frame is rotated relative to
# the IK/platform frame about z. Positive angle = counterclockwise viewed
# from +z (same right-hand convention as ik.cpp's Rz()). Change the degrees
# value below to adjust; negative = clockwise.
IMU_Z_CORRECTION_DEG = -90.0
_theta = np.radians(IMU_Z_CORRECTION_DEG)
_c = np.cos(_theta)
_s = np.sin(_theta)
IMU_AXIS_CORRECTION = np.array([
    [_c, -_s, 0.0],
    [_s,  _c, 0.0],
    [0.0, 0.0, 1.0],
])


def parse_trial(path):
    """Return (list of (t, R_d) desired matrices, list of (t, R_a) actual matrices)."""
    rows = []
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append((float(row["t_rel_s"]), row["source"], row["line"]))

    desired = []
    actual = []

    for t, src, line in rows:
        if src == "TEENSY" and MATRIX_LINE_RE.match(line):
            vals = [float(v) for v in line.strip(",").split(",")]
            if len(vals) == 9:
                R_d = np.array(vals).reshape(3, 3)
                desired.append((t, R_d))
            continue
        if src == "DONGLE":
            m = QUAT_RE.match(line)
            if m:
                qi, qj, qk, qreal = (float(g) for g in m.groups())
                R_a = quat_to_R(qi, qj, qk, qreal)
                if R_a is not None:
                    actual.append((t, R_a))

    return desired, actual


def nearest_match(t, actual_times, actual_mats):
    """Find index of nearest actual sample to time t. actual_times must be sorted."""
    import bisect
    pos = bisect.bisect_left(actual_times, t)
    candidates = []
    if pos > 0:
        candidates.append(pos - 1)
    if pos < len(actual_times):
        candidates.append(pos)
    if not candidates:
        return None
    best = min(candidates, key=lambda idx: abs(actual_times[idx] - t))
    return best


def find_calibration_offset(desired, actual_times, actual_mats, max_dt):
    """Use the first matched (R_d, R_a) pair as a home-pose sample to derive
    a fixed correction: R_offset such that R_offset @ R_a ~= R_d.
    Returns (R_offset, t_used, dt_used) or (None, None, None) if no match found.
    """
    for t, R_d in desired:
        idx = nearest_match(t, actual_times, actual_mats)
        if idx is None:
            continue
        dt = abs(actual_times[idx] - t)
        if dt > max_dt:
            continue
        R_a = actual_mats[idx]
        R_offset = R_d @ R_a.T
        return R_offset, t, dt
    return None, None, None


def rotation_angle_deg(R):
    """Angle (deg) of the rotation represented by R, from its trace."""
    c = (np.trace(R) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    return np.degrees(np.arccos(c))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--in", dest="infile", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-dt", type=float, default=0.05,
                     help="Max seconds between R_d sample and nearest IMU sample to accept a match")
    ap.add_argument("--plot", default=None, help="Optional path to save a PNG plot")
    ap.add_argument("--calibrate", action="store_true",
                     help="Derive the IMU axis/mount correction empirically from the first "
                          "matched sample in this trial (assumes the trial starts at home / "
                          "a known R_d), instead of using the fixed IMU_Z_CORRECTION_DEG guess.")
    args = ap.parse_args()

    desired, actual = parse_trial(args.infile)
    print(f"Parsed {len(desired)} desired rotation matrices, {len(actual)} IMU quaternion samples.")

    if not desired or not actual:
        print("[ERROR] Not enough data parsed — check the CSV / regexes against your actual print format.")
        sys.exit(1)

    actual_times = [t for t, _ in actual]
    actual_mats = [R for _, R in actual]

    if args.calibrate:
        R_offset, t_used, dt_used = find_calibration_offset(desired, actual_times, actual_mats, args.max_dt)
        if R_offset is None:
            print("[ERROR] --calibrate requested but no matched sample found within --max-dt "
                  "to derive an offset from.")
            sys.exit(1)
        angle = rotation_angle_deg(R_offset)
        print(f"Calibrated offset from sample at t={t_used:.4f}s (dt_match={dt_used:.4f}s): "
              f"equivalent rotation angle = {angle:.2f} deg")
        print(f"R_offset =\n{R_offset}")
        correction = R_offset
    else:
        correction = IMU_AXIS_CORRECTION

    results = []
    skipped = 0
    for t, R_d in desired:
        idx = nearest_match(t, actual_times, actual_mats)
        if idx is None:
            skipped += 1
            continue
        dt = abs(actual_times[idx] - t)
        if dt > args.max_dt:
            skipped += 1
            continue
        R_a = actual_mats[idx]
        R_a = correction @ R_a
        M = R_d.T @ R_a - np.eye(3)
        e = float(np.linalg.norm(M, ord="fro"))
        results.append((t, e, dt))

    print(f"Matched {len(results)} samples, skipped {skipped} (no match within {args.max_dt}s).")

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_rel_s", "error_frobenius", "dt_match_s"])
        for t, e, dt in results:
            w.writerow([f"{t:.6f}", f"{e:.6f}", f"{dt:.6f}"])
    print(f"Wrote {len(results)} rows to {args.out}")

    if results:
        errs = np.array([e for _, e, _ in results])
        print(f"error: mean={errs.mean():.4f}  max={errs.max():.4f}  "
              f"steady-state (last 20%)={errs[int(0.8*len(errs)):].mean():.4f}")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        ts = [t for t, _, _ in results]
        es = [e for _, e, _ in results]
        plt.figure(figsize=(8, 4))
        plt.plot(ts, es)
        plt.xlabel("t (s)")
        plt.ylabel(r"$e = \|R_d^T R_a - I\|_F$")
        plt.title("Orientation tracking error")
        plt.tight_layout()
        plt.savefig(args.plot, dpi=150)
        print(f"Saved plot to {args.plot}")


if __name__ == "__main__":
    main()