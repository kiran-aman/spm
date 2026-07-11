import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# machine parameters
ALPHA1 = np.radians(45.0)   # actuated joint cone angle
ALPHA2 = np.radians(90.0)   # distal link angle
BETA   = np.radians(90.0)   # platform joint angle

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
    return Rx(roll) @ Ry(pitch) @ Rz(yaw) # REVERSE (RZ FIRST FOR GLOBAL FRAME; RX FIRST FOR LOCAL FRAME [OFF AXIS ROT])

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

def get_robot_coordinates(thetas, roll, pitch, yaw, alpha1=ALPHA1, beta=BETA):
    """Computes absolute 3D coordinates for all links/joints for plotting."""
    R = rotation_matrix(roll, pitch, yaw)
    vi_home = get_vi_home(beta)
    r_base, r_plat = 1.0, 0.6
    coords = []
    for i in range(3):
        eta_i = 2 * i * np.pi / 3
        base = np.array([0.0, 0.0, 0.0])
        phi_i = eta_i - thetas[i]
        wi_unit = np.array([
            np.cos(phi_i) * np.sin(alpha1),
            np.sin(phi_i) * np.sin(alpha1),
            -np.cos(alpha1)
        ])
        elbow = base + (r_base * wi_unit)
        vi_unit = R @ vi_home[i]
        platform_joint = r_plat * vi_unit
        coords.append({'base': base, 'elbow': elbow, 'platform': platform_joint})
    return coords
