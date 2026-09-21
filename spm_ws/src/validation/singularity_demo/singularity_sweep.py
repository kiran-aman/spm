import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import spm_core as core


# shared sweep primitive
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


# type I: search (pitch, yaw) at roll=0
def find_near_singular_points(n_grid=60, margin_band=(0.0, 0.08)):
    """search normalized type I margin over (pitch, yaw) at roll=0"""
    pitches = np.linspace(-np.pi / 2, np.pi / 2, n_grid)
    yaws = np.linspace(-np.pi / 2, np.pi / 2, n_grid)

    candidates = []
    for p in pitches:
        for y in yaws:
            m = core.worst_leg_margin_normalized(0.0, p, y)
            if margin_band[0] < m < margin_band[1]:
                candidates.append((p, y, m))
    return candidates


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


def plot_type1_vs_type2_tradeoff(results, out_path):
    """parametric plot of (type I margin, type II margin) pairs as roll
    sweeps its 120deg period. loops/backtracking => real trade-off."""
    n = len(results)
    ncols = 3
    nrows = int(np.ceil(n / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(4.5 * ncols, 4 * nrows))
    axes = np.atleast_1d(axes).flatten()

    for ax, res in zip(axes, results):
        x = res["norm_margins"]
        y = res["det_mags"]

        # color by roll angle to show direction of travel
        roll_deg = np.degrees(res["rolls"])
        sc = ax.scatter(x, y, c=roll_deg, cmap="viridis", s=8)

        # mark start (roll=0) and the type I-best point
        ax.scatter(x[0], y[0], marker="x", s=90, color="red",
                   label="start (roll=0)", zorder=5)
        best_i_idx = np.nanargmax(x)
        ax.scatter(x[best_i_idx], y[best_i_idx], marker="*", s=140,
                   color="black", label="TypeI-best roll", zorder=5)

        ax.set_title(
            f"pitch={res['pitch_deg']:.1f}deg  yaw={res['yaw_deg']:.1f}deg",
            fontsize=9
        )
        ax.set_xlabel("Type I margin (normalized)")
        ax.set_ylabel("Type II |det(Jx)|")
        ax.legend(fontsize=6, loc="best")
        fig.colorbar(sc, ax=ax, label="roll (deg)", fraction=0.046, pad=0.04)

    # hide unused subplot axes
    for ax in axes[n:]:
        ax.axis("off")

    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f"Saved Type I vs Type II trade-off plot to {out_path}")


# type II: det(Jx) depends on (roll, pitch, yaw), so this searches
# the full 3D space; each candidate carries its own starting roll.
def find_near_type2_singular_points(n_roll=24, n_grid=40, margin_band=(0.0, 0.13),
                                     pitch_range=(-np.pi / 2, np.pi / 2),
                                     yaw_range=(-np.pi / 2, np.pi / 2)):
    rolls = np.linspace(0, 2 * np.pi / 3, n_roll)
    pitches = np.linspace(*pitch_range, n_grid)
    yaws = np.linspace(*yaw_range, n_grid)

    candidates = []
    for r in rolls:
        for p in pitches:
            for y in yaws:
                d = core.det_Jx(r, p, y)
                if d is None:
                    continue
                ad = abs(d)
                if margin_band[0] < ad < margin_band[1]:
                    candidates.append((r, p, y, ad))
    candidates.sort(key=lambda c: c[3])  # smallest |det(Jx)| first
    return candidates


def analyze_point_type2(pitch, yaw, start_roll):
    rolls, norm_margins, det_mags = sweep_selfmotion(pitch, yaw)
    period = 2 * np.pi / 3

    # nearest sampled roll to the candidate's own start
    start_idx = int(np.argmin(np.abs((rolls - start_roll + period / 2) % period - period / 2)))

    def travel_to_best(values):
        idx = np.nanargmax(values)
        best_roll = rolls[idx]
        delta = (best_roll - start_roll) % period
        delta = min(delta, period - delta)
        return values[idx], np.degrees(delta), idx

    norm_at_start = norm_margins[start_idx]
    det_at_start = det_mags[start_idx]

    det_best, det_travel, det_idx = travel_to_best(det_mags)
    norm_best, norm_travel, norm_idx = travel_to_best(norm_margins)

    return {
        "pitch_deg": np.degrees(pitch),
        "yaw_deg": np.degrees(yaw),
        "start_roll_deg": np.degrees(start_roll),
        "det_at_start": det_at_start,
        "det_best": det_best,
        "det_travel_deg": det_travel,
        "norm_at_start": norm_at_start,
        "norm_best": norm_best,
        "norm_travel_deg": norm_travel,
        "rolls": rolls,
        "norm_margins": norm_margins,
        "det_mags": det_mags,
        "start_idx": start_idx,
    }


def run_type2_analysis(pitch_range=(-np.pi / 2, np.pi / 2),
                        yaw_range=(-np.pi / 2, np.pi / 2),
                        margin_band=(0.0, 0.13),
                        out_path="singularity_demo/singularity_near_type-2_comparison.png"):
    print("Searching for near-singular (roll, pitch, yaw) configurations "
          "(Type II, |det(Jx)|)...")
    candidates = find_near_type2_singular_points(
        margin_band=margin_band, pitch_range=pitch_range, yaw_range=yaw_range
    )
    print(f"Found {len(candidates)} candidates.")
    if candidates:
        print(f"Smallest |det(Jx)| found: {candidates[0][3]:.5f} "
              f"at roll={np.degrees(candidates[0][0]):.1f}deg "
              f"pitch={np.degrees(candidates[0][1]):.1f}deg "
              f"yaw={np.degrees(candidates[0][2]):.1f}deg")
    print()

    if not candidates:
        print("No candidates found — widen margin_band or n_grid.")
        return []

    # take up to 6 closest-to-singular distinct (pitch, yaw) candidates
    seen = set()
    sample = []
    for r, p, y, d in candidates:
        key = (round(np.degrees(p)), round(np.degrees(y)))
        if key in seen:
            continue
        seen.add(key)
        sample.append((r, p, y, d))
        if len(sample) == 6:
            break

    results = []
    fig, axes = plt.subplots(len(sample), 1, figsize=(8, 3.2 * len(sample)), sharex=True)
    if len(sample) == 1:
        axes = [axes]

    for ax, (r0, p, y, d0) in zip(axes, sample):
        res = analyze_point_type2(p, y, start_roll=r0)
        results.append(res)

        ax2 = ax.twinx()
        l1, = ax.plot(np.degrees(res["rolls"]), res["norm_margins"],
                       color="tab:blue", label="Type I (normalized margin)")
        l2, = ax2.plot(np.degrees(res["rolls"]), res["det_mags"],
                        color="tab:orange", label="Type II |det(Jx)|")
        ax2.axvline(res["start_roll_deg"], color="red", linestyle="--",
                    linewidth=1, label="start roll")
        ax.set_title(
            f"pitch={res['pitch_deg']:.1f}deg  yaw={res['yaw_deg']:.1f}deg  "
            f"start_roll={res['start_roll_deg']:.1f}deg  |  "
            f"TypeII: {res['det_at_start']:.4f}->{res['det_best']:.4f} "
            f"(travel {res['det_travel_deg']:.1f}deg)",
            fontsize=9
        )
        ax.set_ylabel("Type I margin", color="tab:blue")
        ax2.set_ylabel("|det(Jx)|", color="tab:orange")
        ax.legend(handles=[l1, l2], loc="upper right", fontsize=7)

    axes[-1].set_xlabel("self-motion angle 'roll' (deg, one 120deg period)")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f"Saved plot to {out_path}\n")

    print(f"{'pitch':>7} {'yaw':>7} {'start_roll':>10} {'TypeII@0':>10} "
          f"{'TypeII best':>12} {'TypeII trav':>11}")
    for res in results:
        print(f"{res['pitch_deg']:7.1f} {res['yaw_deg']:7.1f} "
              f"{res['start_roll_deg']:10.1f} {res['det_at_start']:10.5f} "
              f"{res['det_best']:12.5f} {res['det_travel_deg']:11.1f}")

    return results


if __name__ == "__main__":
    print("Running spm_core validation first...")
    if not core.validate():
        print("Core validation failed — stopping.")
        raise SystemExit
    print()

    # type I analysis
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
    out_path = "singularity_demo/singularity_near_type-1_comparison.png"
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

    # parametric Type I vs Type II trade-off plot, reusing `results`
    plot_type1_vs_type2_tradeoff(
        results, "singularity_demo/type1_vs_type2_tradeoff.png"
    )

    print()
    print("=" * 70)
    print()

    # type II analysis, wide +/-90deg search -- see caveat above
    run_type2_analysis()