import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from scipy.optimize import least_squares

# machine parameters
ALPHA1 = np.radians(50.0)   # actuated joint cone angle
ALPHA2 = np.radians(54.566)   # distal link angle
BETA   = np.radians(70.0)   # platform joint angle

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
    return Rx(yaw) @ Ry(pitch) @ Rz(roll) # REVERSE (RZ FIRST FOR GLOBAL FRAME; RX FIRST FOR LOCAL FRAME [OFF AXIS ROT])

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
            -np.cos(beta)
        ])
        vi_home.append(v)
    return vi_home


# inverse kinematics
def ik(roll, pitch, yaw, alpha1=ALPHA1, alpha2=ALPHA2, beta=BETA, prev_thetas=None):
    R = rotation_matrix(roll, pitch, yaw)
    vi_home = get_vi_home(beta)
    thetas = []

    for i in range(3):
        eta_i = 2 * i * np.pi / 3
        vi = R @ vi_home[i]

        sa1, ca1 = np.sin(alpha1), np.cos(alpha1)
        ce, se = np.cos(eta_i), np.sin(eta_i)

        A = sa1 * (ce * vi[0] + se * vi[1])
        B = sa1 * (se * vi[0] - ce * vi[1])
        C = np.cos(alpha2) + ca1 * vi[2]
        # print("A" + str(A))
        # print("B" + str(B))
        # print("C" + str(C))

        qa, qb, qc = C+A, -2*B, C-A
        discriminant = qb**2 - 4*qa*qc

        if discriminant < 0:
            print(f"[IK] No real solution for joint {i+1}")
            return None

        if abs(qa) < 1e-10:
            t = -qc / qb
            sin_theta = 2*t / (1 + t**2)
            cos_theta = (1 - t**2) / (1 + t**2)
            theta_i = np.arctan2(sin_theta, cos_theta)
        else:
            t1 = (-qb + np.sqrt(discriminant)) / (2*qa)
            t2 = (-qb - np.sqrt(discriminant)) / (2*qa)

            sin_t1 = 2*t1 / (1 + t1**2)
            cos_t1 = (1 - t1**2) / (1 + t1**2)
            theta_t1 = np.arctan2(sin_t1, cos_t1)

            sin_t2 = 2*t2 / (1 + t2**2)
            cos_t2 = (1 - t2**2) / (1 + t2**2)
            theta_t2 = np.arctan2(sin_t2, cos_t2)

            if prev_thetas is None:
                theta_i = theta_t1
            else:
                # normalize deltas to (-pi, pi)
                delta_t1 = (theta_t1 - prev_thetas[i] + np.pi) % (2*np.pi) - np.pi
                delta_t2 = (theta_t2 - prev_thetas[i] + np.pi) % (2*np.pi) - np.pi

                # accumulate smallest delta onto prev_theta
                # keeps output continuous regardless of arctan2 wrapping
                if abs(delta_t1) < abs(delta_t2):
                    theta_i = prev_thetas[i] + delta_t1
                else:
                    theta_i = prev_thetas[i] + delta_t2

        thetas.append(theta_i)

    return np.array(thetas)

# forward kinematics
def fk(thetas, alpha1=ALPHA1, alpha2=ALPHA2, beta=BETA, initial_guess=None):
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

    # initial guess w/ warm start (previous position conventionally)
    if initial_guess is None:
        rpy0 = np.array([0.0, 0.0, 0.0])  # Break initial symmetry
    else:
        rpy0 = np.array(initial_guess)

    # least-squares Levenberg–Marquardt algorithm
    result = least_squares(residuals, rpy0, method='lm', xtol=1e-12, ftol=1e-12, gtol=1e-12)
    rpy_sol = result.x
    msg = result.message

    if not result.success:
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

# DEBUGGING
# def main():
#     # Test 1: home position should give thetas = [0, 0, 0]
#     thetas = ik(0, 0, 0)
#     print("Home thetas (expect [0,0,0]):", np.degrees(thetas))

#     # Test 2: check A, B, C coefficients at home for joint 1
#     R = rotation_matrix(0, 0, 0)
#     vi_home = get_vi_home(BETA)
#     vi = R @ vi_home[0]
#     print("vi[0] at home:", vi)

#     eta_i = 0.0
#     sa1 = np.sin(ALPHA1)
#     ca1 = np.cos(ALPHA1)
#     ce  = np.cos(eta_i)
#     se  = np.sin(eta_i)

#     A = sa1 * (ce * vi[0] + se * vi[1])
#     B = sa1 * (se * vi[0] - ce * vi[1])
#     C = np.cos(ALPHA2) + ca1 * vi[2]

#     print(f"A={A:.6f} B={B:.6f} C={C:.6f}")
#     print(f"qa={C+A:.6f} qb={-2*B:.6f} qc={C-A:.6f}")

#     disc = (-2*B)**2 - 4*(C+A)*(C-A)
#     print(f"discriminant={disc:.6f}")

#     t1 = (2*B + np.sqrt(disc)) / (2*(C+A))
#     t2 = (2*B - np.sqrt(disc)) / (2*(C+A))
#     print(f"t1={t1:.6f} -> theta={np.degrees(2*np.arctan(t1)):.4f} deg")
#     print(f"t2={t2:.6f} -> theta={np.degrees(2*np.arctan(t2)):.4f} deg")

#     # Test 3: constraint check — wi . vi should equal cos(alpha2)
#     theta_i = 2*np.arctan(t1)
#     phi_i = eta_i - theta_i
#     wi = np.array([
#         np.cos(phi_i) * np.sin(ALPHA1),
#         np.sin(phi_i) * np.sin(ALPHA1),
#         -np.cos(ALPHA1)
#     ])
#     print(f"wi.vi = {np.dot(wi, vi):.6f} (expect cos(alpha2)={np.cos(ALPHA2):.6f})")

# if __name__ == "__main__":
#     main()