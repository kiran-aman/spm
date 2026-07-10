import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# machine parameters
ALPHA1 = np.radians(45.0)   # actuated joint cone angle
ALPHA2 = np.radians(90.0)   # distal link angle
BETA   = np.radians(90.0)   # platform joint angle

# <--- FIX RPY AXIS MIX UP <---

# rotation matrices
def Rx(a):
    return np.array([
        [1,  0,          0        ],
        [0,  np.cos(a), -np.sin(a)],
        [0,  np.sin(a),  np.cos(a)]
    ])

def Ry(a):
    return np.array([
        [ np.cos(a), 0, np.sin(a)],
        [ 0,         1, 0        ],
        [-np.sin(a), 0, np.cos(a)]
    ])

def Rz(a):
    return np.array([
        [np.cos(a), -np.sin(a), 0],
        [np.sin(a),  np.cos(a), 0],
        [0,          0,         1]
    ])

def rotation_matrix(roll, pitch, yaw):
    """
    rpy rotation matrix — Cruz-Reyes eq (4)
    R = Rx(yaw) * Ry(pitch) * Rz(roll)
    """
    return Rx(yaw) @ Ry(pitch) @ Rz(roll)

# home configuration vectors (v_i,h)
def get_vi_home(beta):
    """
    Platform joint unit vectors in home configuration.
    Evenly spaced 120 deg apart, at cone angle beta from Z axis.
    """
    vi_home = []
    for i in range(3):
        eta_i = 2 * i * np.pi / 3
        v = np.array([
            np.cos(eta_i) * np.sin(beta),
            np.sin(eta_i) * np.sin(beta),
            np.cos(beta)
        ])
        vi_home.append(v)
    return vi_home

# inverse kinematics
def ik(roll, pitch, yaw, alpha1=ALPHA1, alpha2=ALPHA2, beta=BETA):
    """
    given desired platform orientation (RPY), returns joint angles theta.
    returns: [theta1, theta2, theta3] in radians, or None if no solution.
    """
    R = rotation_matrix(roll, pitch, yaw)
    vi_home = get_vi_home(beta)
    thetas = []

    for i in range(3):
        eta_i = 2 * i * np.pi / 3

        # platform joint vector in current orientation eq (3)
        vi = R @ vi_home[i]

        # physical constants for coefficients (A, B, C)
        sa1 = np.sin(alpha1)
        ca1 = np.cos(alpha1)
        ce  = np.cos(eta_i)
        se  = np.sin(eta_i)

        A = sa1 * (ce * vi[0] + se * vi[1])
        B = sa1 * (se * vi[0] - ce * vi[1])
        C = np.cos(alpha2) + ca1 * vi[2]

        # Weierstrass half-angle substitution: t = tan(theta/2)
        # (C + A)*t^2 - 2*B*t + (C - A) = 0
        qa = C + A
        qb = -2 * B
        qc = C - A

        discriminant = qb**2 - 4*qa*qc

        if discriminant < 0:
            print(f"[IK] No real solution for joint {i+1} (discriminant={discriminant:.4f})")
            return None

        if abs(qa) < 1e-10:
            # degenerate case
            if abs(qb) < 1e-10:
                print(f"[IK] Degenerate equation for joint {i+1}")
                return None
            t = -qc / qb
            theta_i = 2 * np.arctan(t)
        else:
            t1 = (-qb + np.sqrt(discriminant)) / (2 * qa)
            t2 = (-qb - np.sqrt(discriminant)) / (2 * qa)

            # 2 solutions (2 assembly modes); select t1 (positive)
            # solve for theta_i
            theta_i = 2 * np.arctan(t1)

        thetas.append(theta_i)

    return np.array(thetas)

# forward kinematics
def fk(thetas, alpha1=ALPHA1, alpha2=ALPHA2, beta=BETA):
    """
    given joint angles (theta_i), recover platform orientation.
    numerical solver (Newton-Raphson) since fk for SPM has no simple closed form.
    returns: (roll, pitch, yaw) in radians
    """
    from scipy.optimize import fsolve

    vi_home = get_vi_home(beta)

    def residuals(rpy):
        roll, pitch, yaw = rpy
        R = rotation_matrix(roll, pitch, yaw)
        res = []
        for i in range(3):
            eta_i = 2 * i * np.pi / 3
            vi = R @ vi_home[i]
            # reconstruct wi from known theta_i
            phi_i = eta_i - thetas[i]
            wi = np.array([
                np.cos(phi_i) * np.sin(alpha1),
                np.sin(phi_i) * np.sin(alpha1),
                -np.cos(alpha1)
            ])
            res.append(np.dot(wi, vi) - np.cos(alpha2))
        return res

    # initial guess — home position
    # ---> MODIFY FOR BETTER INITIAL GUESS <---- (later)
    rpy0 = np.array([0.0, 0.0, 0.0])
    rpy_sol, info, ier, msg = fsolve(residuals, rpy0, full_output=True)

    if ier != 1:
        print(f"[FK] Warning: solver did not converge — {msg}")

    return rpy_sol  # [roll, pitch, yaw]

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
    print(f"alpha1={np.degrees(ALPHA1):.1f} deg  "
          f"alpha2={np.degrees(ALPHA2):.1f} deg  "
          f"beta={np.degrees(BETA):.1f} deg")
    print("=" * 60)

    # Trajectory: yaw 0 -> 30 deg over 1 second, roll=pitch=0
    t_f   = 1.0
    yaw_f = np.radians(30.0)
    N     = 100
    times = np.linspace(0, t_f, N)

    yaw_traj    = np.array([cubic_trajectory(t, t_f, yaw_f) for t in times])
    theta_traj  = np.zeros((N, 3))
    rpy_fk_traj = np.zeros((N, 3))

    print("\nRunning IK along trajectory...")
    for k, (t, yaw) in enumerate(zip(times, yaw_traj)):
        thetas = ik(0.0, 0.0, yaw)
        if thetas is None:
            print(f"  IK failed at t={t:.3f}s, yaw={np.degrees(yaw):.2f} deg")
            return
        theta_traj[k] = thetas

    print("IK complete. Running FK verification...")
    for k, thetas in enumerate(theta_traj):
        rpy = fk(thetas)
        rpy_fk_traj[k] = rpy

    # round trip error
    yaw_fk_deg    = np.degrees(rpy_fk_traj[:, 2])
    yaw_input_deg = np.degrees(yaw_traj)
    error_deg     = yaw_fk_deg - yaw_input_deg

    print(f"\nRound-trip error (IK → FK):")
    print(f"  Max yaw error: {np.max(np.abs(error_deg)):.6f} deg")
    print(f"  RMS yaw error: {np.sqrt(np.mean(error_deg**2)):.6f} deg")

    # sample values
    print(f"\nSample IK results (every 10th point):")
    print(f"{'t':>6} {'yaw_in':>10} {'θ1':>10} {'θ2':>10} {'θ3':>10}")
    for k in range(0, N, 10):
        print(f"{times[k]:6.2f} "
              f"{np.degrees(yaw_traj[k]):10.4f} "
              f"{np.degrees(theta_traj[k,0]):10.4f} "
              f"{np.degrees(theta_traj[k,1]):10.4f} "
              f"{np.degrees(theta_traj[k,2]):10.4f}")

    # plots
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("ik/fk validation using third-order polynomial trajectories; yaw 0°->30° (based on Cruz-Reyes et al.)\n"
                 f"α₁={np.degrees(ALPHA1):.0f}°, "
                 f"α₂={np.degrees(ALPHA2):.0f}°, "
                 f"β={np.degrees(BETA):.0f}°", fontsize=13)

    # Input trajectory
    ax = axes[0, 0]
    ax.plot(times, np.degrees(yaw_traj), 'b-')
    ax.set_xlabel("time (s)")
    ax.set_ylabel("yaw (deg)")
    ax.set_title("input trajectory (yaw)")
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
    ax.plot(times, yaw_input_deg, 'b-', label='input yaw (same as above)')
    ax.plot(times, yaw_fk_deg, 'r--', label='fk recovered yaw')
    ax.set_xlabel("time (s)")
    ax.set_ylabel("yaw (deg)")
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