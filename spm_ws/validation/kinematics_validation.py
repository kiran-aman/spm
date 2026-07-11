import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import spm_kinematics as spm

# 3rd order polynomial trajectory generation
# ---> ADD TRAPEZOIDAL VELOCITY TRAJECTORIES (later) <---
def cubic_trajectory(t, t_f, q_f):
    """
    starts and ends at zero velocity
    q(t) = q_f * (3*(t/t_f)^2 - 2*(t/t_f)^3)
    """
    s = t / t_f
    return q_f * (3*s**2 - 2*s**3)

# validation
def main():
    print("=" * 60)
    print("IK/FK Validation — Cruz-Reyes et al.")
    print(f"alpha1={np.degrees(spm.ALPHA1):.1f} deg  "
          f"alpha2={np.degrees(spm.ALPHA2):.1f} deg  "
          f"beta={np.degrees(spm.BETA):.1f} deg")
    print("=" * 60)

    # Trajectory: yaw 0 -> 30 deg over 1 second, roll=pitch=0
    t_f   = 1.0
    roll_f = np.radians(30.0)
    N     = 100
    times = np.linspace(0, t_f, N)

    roll_traj    = np.array([cubic_trajectory(t, t_f, roll_f) for t in times])
    theta_traj  = np.zeros((N, 3))
    rpy_fk_traj = np.zeros((N, 3))

    print("\nRunning IK along trajectory...")
    for k, (t, roll) in enumerate(zip(times, roll_traj)):
        thetas = spm.ik(roll, 0.0, 0.0)
        if thetas is None:
            print(f"  IK failed at t={t:.3f}s, roll={np.degrees(roll):.2f} deg")
            return
        theta_traj[k] = thetas

    print("IK complete. Running FK verification...")
    for k, thetas in enumerate(theta_traj):
        rpy = spm.fk(thetas)
        rpy_fk_traj[k] = rpy

    # round trip error
    roll_fk_deg    = np.degrees(rpy_fk_traj[:, 0])
    roll_input_deg = np.degrees(roll_traj)
    error_deg     = roll_fk_deg - roll_input_deg

    print(f"\nRound-trip error (IK → FK):")
    print(f"  Max roll error: {np.max(np.abs(error_deg)):.6f} deg")
    print(f"  RMS roll error: {np.sqrt(np.mean(error_deg**2)):.6f} deg")

    # sample values
    print(f"\nSample IK results (every 10th point):")
    print(f"{'t':>6} {'roll_in':>10} {'θ1':>10} {'θ2':>10} {'θ3':>10}")
    for k in range(0, N, 10):
        print(f"{times[k]:6.2f} "
              f"{np.degrees(roll_traj[k]):10.4f} "
              f"{np.degrees(theta_traj[k,0]):10.4f} "
              f"{np.degrees(theta_traj[k,1]):10.4f} "
              f"{np.degrees(theta_traj[k,2]):10.4f}")

    # plots
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("ik/fk validation using third-order polynomial trajectories; roll 0°->30° (based on Cruz-Reyes et al.)\n"
                 f"α₁={np.degrees(spm.ALPHA1):.0f}°, "
                 f"α₂={np.degrees(spm.ALPHA2):.0f}°, "
                 f"β={np.degrees(spm.BETA):.0f}°", fontsize=13)

    # Input trajectory
    ax = axes[0, 0]
    ax.plot(times, np.degrees(roll_traj), 'b-')
    ax.set_xlabel("time (s)")
    ax.set_ylabel("roll (deg)")
    ax.set_title("input trajectory (roll)")
    ax.grid(True)

    # IK joint angles
    ax = axes[0, 1]
    for i in range(3):
        ax.plot(times, np.degrees(theta_traj[:, i]), label=f'θ{i+1}')
    ax.set_xlabel("time (s)")
    ax.set_ylabel("joint angle (deg)")
    ax.set_title("ik: joint angles")
    ax.legend()
    ax.grid(True)

    # FK verification
    ax = axes[1, 0]
    ax.plot(times, roll_input_deg, 'b-', label='input roll (same as above)')
    ax.plot(times, roll_fk_deg, 'r--', label='fk recovered roll')
    ax.set_xlabel("time (s)")
    ax.set_ylabel("roll (deg)")
    ax.set_title("fk verification (round-trip)")
    ax.legend()
    ax.grid(True)

    # Round-trip error
    ax = axes[1, 1]
    ax.plot(times, error_deg, 'r-')
    ax.set_xlabel("time (s)")
    ax.set_ylabel("error (deg)")
    ax.set_title("round-trip error (fk-ik)")
    ax.grid(True)

    plt.tight_layout()
    plt.savefig("/home/kiran-aman/Documents/spm/spm_ws/validation/ik_validation.png", dpi=150)
    plt.show()
    print("\nPlot saved to validation/ik_validation.png")

if __name__ == "__main__":
    main()