import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import spm_core as core


def find_near_singular_points(n_grid=60, margin_band=(0.0, 0.08)):
    """Search on the normalized Type I margin (fair across poses),
    at roll=0, over the task-relevant (pitch, yaw) plane."""
    pitches = np.linspace(-np.pi / 2, np.pi / 2, n_grid)
    yaws = np.linspace(-np.pi / 2, np.pi / 2, n_grid)

    candidates = []
    for p in pitches:
        for y in yaws:
            m = core.worst_leg_margin_normalized(0.0, p, y)
            if margin_band[0] < m < margin_band[1]:
                candidates.append((p, y, m))
    return candidates


def sweep_selfmotion(pitch, yaw, n=361):
    rolls = np.linspace(0, 2 * np.pi / 3, n)
    norm_margins = np.full(n, np.nan)
    det_mags = np.full(n, np.nan)

    for k, r in enumerate(rolls):
        norm_margins[k] = core.worst_leg_margin_normalized(r, pitch, yaw)
        d = core.det_Jx(r, pitch, yaw)
        if d is not None:
            det_mags[k] = abs(d)

    return rolls, norm_margins, det_mags


def analyze_point(pitch, yaw, start_roll=0.0):
    rolls, norm_margins, det_mags = sweep_selfmotion(pitch, yaw)
    period = 2 * np.pi / 3

    def travel_to_best(values):
        idx = np.nanargmax(values)
        best_roll = rolls[idx]
        delta = (best_roll - start_roll) % period
        delta = min(delta, period - delta)
        return values[idx], np.degrees(delta), idx

    start_idx = 0  # rolls[0] == start_roll == 0.0
    norm_at_start = norm_margins[start_idx]
    det_at_start = det_mags[start_idx]

    norm_best, norm_travel, norm_idx = travel_to_best(norm_margins)
    det_best, det_travel, det_idx = travel_to_best(det_mags)

    return {
        "pitch_deg": np.degrees(pitch),
        "yaw_deg": np.degrees(yaw),
        "norm_at_start": norm_at_start,
        "norm_best": norm_best,
        "norm_travel_deg": norm_travel,
        "det_at_start": det_at_start,
        "det_best": det_best,
        "det_travel_deg": det_travel,
        "same_direction": abs(norm_idx - det_idx) < len(rolls) * 0.05,
        "rolls": rolls,
        "norm_margins": norm_margins,
        "det_mags": det_mags,
    }


if __name__ == "__main__":
    print("Running spm_core validation first...")
    if not core.validate():
        print("Core validation failed — stopping.")
        raise SystemExit
    print()

    print("Searching for near-singular (pitch, yaw) configurations "
          "(Type I, normalized margin)...")
    candidates = find_near_singular_points()
    print(f"Found {len(candidates)} candidates.\n")

    if not candidates:
        print("No candidates found — widen margin_band or n_grid.")
        raise SystemExit

    idxs = np.linspace(0, len(candidates) - 1, min(6, len(candidates)), dtype=int)
    sample = [candidates[i] for i in idxs]

    results = []
    fig, axes = plt.subplots(len(sample), 1, figsize=(8, 3.2 * len(sample)), sharex=True)
    if len(sample) == 1:
        axes = [axes]

    for ax, (p, y, m0) in zip(axes, sample):
        res = analyze_point(p, y, start_roll=0.0)
        results.append(res)

        ax2 = ax.twinx()
        l1, = ax.plot(np.degrees(res["rolls"]), res["norm_margins"],
                       color="tab:blue", label="Type I (normalized margin)")
        l2, = ax2.plot(np.degrees(res["rolls"]), res["det_mags"],
                        color="tab:orange", label="Type II |det(Jx)|")
        ax.set_title(
            f"pitch={res['pitch_deg']:.1f}deg  yaw={res['yaw_deg']:.1f}deg  |  "
            f"TypeI: {res['norm_at_start']:.3f}->{res['norm_best']:.3f} "
            f"(travel {res['norm_travel_deg']:.1f}deg)  |  "
            f"TypeII: {res['det_at_start']:.4f}->{res['det_best']:.4f} "
            f"(travel {res['det_travel_deg']:.1f}deg)",
            fontsize=9
        )
        ax.set_ylabel("Type I margin", color="tab:blue")
        ax2.set_ylabel("|det(Jx)|", color="tab:orange")
        ax.legend(handles=[l1, l2], loc="upper right", fontsize=7)

    axes[-1].set_xlabel("self-motion angle 'roll' (deg, one 120deg period)")
    plt.tight_layout()
    out_path = "singularity_type_comparison.png"
    plt.savefig(out_path, dpi=150)
    print(f"Saved plot to {out_path}\n")

    print(f"{'pitch':>7} {'yaw':>7} {'TypeI@0':>9} {'TypeI best':>11} "
          f"{'TypeI trav':>10} {'TypeII@0':>10} {'TypeII best':>12} "
          f"{'TypeII trav':>11} {'same dir?':>10}")
    for res in results:
        print(f"{res['pitch_deg']:7.1f} {res['yaw_deg']:7.1f} "
              f"{res['norm_at_start']:9.4f} {res['norm_best']:11.4f} "
              f"{res['norm_travel_deg']:10.1f} {res['det_at_start']:10.5f} "
              f"{res['det_best']:12.5f} {res['det_travel_deg']:11.1f} "
              f"{str(res['same_direction']):>10}")