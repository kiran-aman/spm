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
    """Quaternion (x,y,z,w) -> 3x3 BODY-TO-WORLD rotation matrix.

    NOTE: an earlier version of this function computed the transpose of
    this (labeled "world-to-body"). Since R^T = R^-1 for a rotation matrix,
    that reports the SAME rotation axis but a NEGATED angle for every
    sample, regardless of the true physical rotation direction. That
    produced a persistent desired/actual mirror that looked like a firmware
    sign bug even after the firmware pitch-sign bug was actually fixed and
    confirmed physically (platform direction reversed correctly when the
    firmware sign was flipped, but the analyzed "actual" stayed mirrored
    either way — the tell that the bug had moved into this function).
    This is the standard SH-2/BNO08x convention: sensor orientation
    expressed in its reference/world frame.
    """
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
# value below to adjust; negative = clockwise. Only used when --calibrate
# is NOT passed — now that the mount has been physically re-aligned, try
# running WITHOUT --calibrate and WITHOUT this correction first (set to 0)
# to see how much residual error remains from mounting alone.
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


def find_calibration_offset(desired, actual_times, actual_mats, max_dt, side="right"):
    """Use the first matched (R_d, R_a) pair as a home-pose sample to derive
    a fixed correction.
      side="right" (body-frame / mounting correction): R_d = R_a @ R_offset
          -> R_offset = R_a.T @ R_d ; applied as R_a_corrected = R_a @ R_offset
      side="left"  (world-frame correction): R_d = R_offset @ R_a
          -> R_offset = R_d @ R_a.T ; applied as R_a_corrected = R_offset @ R_a
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
        if side == "right":
            R_offset = R_a.T @ R_d
        else:
            R_offset = R_d @ R_a.T
        return R_offset, t, dt
    return None, None, None


def rotation_angle_deg(R):
    """Angle (deg) of the rotation represented by R, from its trace."""
    c = (np.trace(R) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    return np.degrees(np.arccos(c))


def euler_from_R(R):
    """Extract roll, pitch, yaw (degrees) assuming R = Rx(roll) @ Ry(pitch) @ Rz(yaw),
    matching ik.cpp's rotation_matrix() convention exactly. Standard formulas:
      pitch = asin(R[0][2])
      roll  = atan2(-R[1][2], R[2][2])
      yaw   = atan2(-R[0][1], R[0][0])
    Degenerates near pitch = +/-90 deg (gimbal lock) — fine for small-angle
    validation trajectories, not meant for large/singular motions.

    NOTE: if your rotation_matrix() actually builds R = Rx(yaw) @ Ry(pitch) @ Rz(roll)
    (swapped roll/yaw axis assignment), the "roll" and "yaw" values this
    function returns are swapped relative to your firmware's field names —
    "roll" here is really your commanded yaw field's effect (about x), and
    "yaw" here is really your commanded roll field's effect (about z).
    Pitch is unaffected by that swap either way.
    """
    r02 = max(-1.0, min(1.0, R[0][2]))
    pitch = np.arcsin(r02)
    roll = np.arctan2(-R[1][2], R[2][2])
    yaw = np.arctan2(-R[0][1], R[0][0])
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


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
    ap.add_argument("--calibrate-side", choices=["left", "right"], default="right",
                     help="'right' = body-frame/mounting correction R_a @ R_offset (default, "
                          "correct for a sensor bolted on rotated relative to platform body axes). "
                          "'left' = world-frame correction R_offset @ R_a (correct for a "
                          "misaligned reference/world axis).")
    ap.add_argument("--no-correction", action="store_true",
                     help="Skip any fixed/calibrated correction entirely — compare R_d directly "
                          "against raw R_a. Useful now that the BNO085 mount has been physically "
                          "re-aligned, to see the residual error with no correction applied at all.")
    ap.add_argument("--euler-out", default=None,
                     help="Optional CSV path: per-sample roll/pitch/yaw (deg) for R_d vs "
                          "corrected R_a, side by side, to diagnose which axis is diverging.")
    ap.add_argument("--euler-plot", default=None,
                     help="Optional PNG path: plot roll/pitch/yaw of R_d vs corrected R_a over time.")
    args = ap.parse_args()

    desired, actual = parse_trial(args.infile)
    print(f"Parsed {len(desired)} desired rotation matrices, {len(actual)} IMU quaternion samples.")

    if not desired or not actual:
        print("[ERROR] Not enough data parsed — check the CSV / regexes against your actual print format.")
        sys.exit(1)

    actual_times = [t for t, _ in actual]
    actual_mats = [R for _, R in actual]

    if args.no_correction:
        correction = np.eye(3)
        correction_side = "right"
        print("Skipping correction entirely (--no-correction): comparing R_d directly against raw R_a.")
    elif args.calibrate:
        R_offset, t_used, dt_used = find_calibration_offset(
            desired, actual_times, actual_mats, args.max_dt, side=args.calibrate_side)
        if R_offset is None:
            print("[ERROR] --calibrate requested but no matched sample found within --max-dt "
                  "to derive an offset from.")
            sys.exit(1)
        angle = rotation_angle_deg(R_offset)
        print(f"Calibrated ({args.calibrate_side}-side) offset from sample at t={t_used:.4f}s "
              f"(dt_match={dt_used:.4f}s): equivalent rotation angle = {angle:.2f} deg")
        print(f"R_offset =\n{R_offset}")
        correction = R_offset
        correction_side = args.calibrate_side
    else:
        correction = IMU_AXIS_CORRECTION
        correction_side = "left"

    results = []
    euler_rows = []
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
        R_a = actual_mats[idx].T # THIS MIGHT NOT BE RIGHT
        if correction_side == "right":
            R_a = R_a @ correction
        else:
            R_a = correction @ R_a
        M = R_d.T @ R_a - np.eye(3)
        e = float(np.linalg.norm(M, ord="fro"))
        results.append((t, e, dt))

        if args.euler_out or args.euler_plot:
            roll_d, pitch_d, yaw_d = euler_from_R(R_d)
            roll_a, pitch_a, yaw_a = euler_from_R(R_a)
            euler_rows.append(
                (t, roll_d, pitch_d, yaw_d,
                    roll_a, pitch_a, yaw_a)
            )

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

    # ------------------------------------------------------------
    # Optional Euler-angle CSV
    # ------------------------------------------------------------
    if args.euler_out:
        with open(args.euler_out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                "t_rel_s",
                "roll_des_deg", "pitch_des_deg", "yaw_des_deg",
                "roll_act_deg", "pitch_act_deg", "yaw_act_deg",
                "roll_err_deg", "pitch_err_deg", "yaw_err_deg"
            ])

            for row in euler_rows:
                t, rd, pd, yd, ra, pa, ya = row
                w.writerow([
                    f"{t:.6f}",
                    f"{rd:.6f}", f"{pd:.6f}", f"{yd:.6f}",
                    f"{ra:.6f}", f"{pa:.6f}", f"{ya:.6f}",
                    f"{rd-ra:.6f}",
                    f"{pd-pa:.6f}",
                    f"{yd-ya:.6f}",
                ])

        print(f"Saved Euler CSV to {args.euler_out}")

    # ------------------------------------------------------------
    # Optional Euler-angle plots
    # ------------------------------------------------------------
    if args.euler_plot and len(euler_rows) > 0:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        ts = np.array([r[0] for r in euler_rows])

        roll_d  = np.array([r[1] for r in euler_rows])
        pitch_d = np.array([r[2] for r in euler_rows])
        yaw_d   = np.array([r[3] for r in euler_rows])

        roll_a  = np.array([r[4] for r in euler_rows])
        pitch_a = np.array([r[5] for r in euler_rows])
        yaw_a   = np.array([r[6] for r in euler_rows])

        fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

        ax[0].plot(ts, roll_d, label="Desired")
        ax[0].plot(ts, roll_a, "--", label="Actual")
        ax[0].set_ylabel("Roll (deg)")
        ax[0].grid(True)
        ax[0].legend()

        ax[1].plot(ts, pitch_d)
        ax[1].plot(ts, pitch_a, "--")
        ax[1].set_ylabel("Pitch (deg)")
        ax[1].grid(True)

        ax[2].plot(ts, yaw_d)
        ax[2].plot(ts, yaw_a, "--")
        ax[2].set_ylabel("Yaw (deg)")
        ax[2].set_xlabel("Time (s)")
        ax[2].grid(True)

        fig.suptitle("Desired vs Actual Euler Angles")
        fig.tight_layout()
        fig.savefig(args.euler_plot, dpi=150)

        print(f"Saved Euler plot to {args.euler_plot}")


if __name__ == "__main__":
    main()