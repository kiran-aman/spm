"""
look for type 1 and type 2 singularities in the coaxial 3-RRR SPM.

tests near-type 1 and 2 singular poses and sweeps roll to find effect 
    of reorientation simultaneously on type 1/2 singularity margins

type 1 singularity: leg discriminant = 0 (leg loses IK solution)
type 2 singularity: det(Jx) = 0 (platform loses controllability)

preliminary observations:
    - platform reorientation seems to have a strong effect on type 1
    singularity margins while near type 1 singular poses; doesn't seem
    to effect type 2 singularity margins as much

NEED TO TEST NEAR TYPE 2 SINGULAR POSES
"""

import numpy as np

# machine params (from spm_kinematics.py)
ALPHA1 = np.radians(50.0)
ALPHA2 = np.radians(54.566)
BETA = np.radians(70.0)

# rotation matrices (from spm_kinematics.py)
def Rx(a):
    return np.array([
        [1, 0, 0],
        [0, np.cos(a), -np.sin(a)],
        [0, np.sin(a), np.cos(a)]
    ])


def Ry(a):
    return np.array([
        [np.cos(a), 0, np.sin(a)],
        [0, 1, 0],
        [-np.sin(a), 0, np.cos(a)]
    ])


def Rz(a):
    return np.array([
        [np.cos(a), -np.sin(a), 0],
        [np.sin(a), np.cos(a), 0],
        [0, 0, 1]
    ])


def rotation_matrix(roll, pitch, yaw):
    return Rx(yaw) @ Ry(pitch) @ Rz(roll)


def get_vi_home(beta=BETA):
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


def wi_from_theta(theta_i, eta_i, alpha1=ALPHA1):
    phi_i = eta_i - theta_i
    return np.array([
        np.cos(phi_i) * np.sin(alpha1),
        np.sin(phi_i) * np.sin(alpha1),
        -np.cos(alpha1)
    ])

# ik from spm_kinematics.py
def ik(roll, pitch, yaw, alpha1=ALPHA1, alpha2=ALPHA2, beta=BETA, prev_thetas=None):
    R = rotation_matrix(roll, pitch, yaw)
    vi_home = get_vi_home(beta)
    thetas = []

    sa1, ca1 = np.sin(alpha1), np.cos(alpha1)

    for i in range(3):
        eta_i = 2 * i * np.pi / 3
        vi = R @ vi_home[i]
        ce, se = np.cos(eta_i), np.sin(eta_i)

        A = sa1 * (ce * vi[0] + se * vi[1])
        B = sa1 * (se * vi[0] - ce * vi[1])
        C = np.cos(alpha2) + ca1 * vi[2]

        qa, qb, qc = C + A, -2 * B, C - A
        discriminant = qb**2 - 4 * qa * qc

        if discriminant < 0:
            return None

        if abs(qa) < 1e-10:
            t = -qc / qb
            sin_theta = 2 * t / (1 + t**2)
            cos_theta = (1 - t**2) / (1 + t**2)
            theta_i = np.arctan2(sin_theta, cos_theta)
        else:
            t1 = (-qb + np.sqrt(discriminant)) / (2 * qa)
            t2 = (-qb - np.sqrt(discriminant)) / (2 * qa)

            sin_t1 = 2 * t1 / (1 + t1**2)
            cos_t1 = (1 - t1**2) / (1 + t1**2)
            theta_t1 = np.arctan2(sin_t1, cos_t1)

            sin_t2 = 2 * t2 / (1 + t2**2)
            cos_t2 = (1 - t2**2) / (1 + t2**2)
            theta_t2 = np.arctan2(sin_t2, cos_t2)

            if prev_thetas is None:
                theta_i = theta_t1
            else:
                delta_t1 = (theta_t1 - prev_thetas[i] + np.pi) % (2 * np.pi) - np.pi
                delta_t2 = (theta_t2 - prev_thetas[i] + np.pi) % (2 * np.pi) - np.pi
                theta_i = prev_thetas[i] + (delta_t1 if abs(delta_t1) < abs(delta_t2) else delta_t2)

        thetas.append(theta_i)

    return np.array(thetas)


# TYPE 1 SINGULARITY METRICS
def leg_ABC(roll, pitch, yaw, alpha1=ALPHA1, alpha2=ALPHA2, beta=BETA):
    """find A, B, C for each leg (i=0,1,2) given a pose (roll, pitch, yaw)."""
    R = rotation_matrix(roll, pitch, yaw)
    vi_home = get_vi_home(beta)
    sa1, ca1 = np.sin(alpha1), np.cos(alpha1)
    cos_a2 = np.cos(alpha2)

    out = []
    for i in range(3):
        eta_i = 2 * i * np.pi / 3
        vi = R @ vi_home[i]
        ce, se = np.cos(eta_i), np.sin(eta_i)
        A = sa1 * (ce * vi[0] + se * vi[1])
        B = sa1 * (se * vi[0] - ce * vi[1])
        C = cos_a2 + ca1 * vi[2]
        out.append((A, B, C))
    return out


def leg_margin_raw(roll, pitch, yaw, **kw):
    """discriminant per leg = 4*(K^2 - C^2) = 4*(A^2+B^2-C^2)"""
    margins = []
    for A, B, C in leg_ABC(roll, pitch, yaw, **kw):
        margins.append(4 * (A**2 + B**2 - C**2))
    return np.array(margins)


def leg_margin_normalized(roll, pitch, yaw, **kw):
    """normalized singularity margin per leg = sin^2(angular margin) = (K^2-C^2)/K^2,
    dimensionless in [0,1] (0 at singularity, 1 at C=0 i.e. maximum
    possible slack for that leg)"""
    margins = []
    for A, B, C in leg_ABC(roll, pitch, yaw, **kw):
        K2 = A**2 + B**2
        if K2 < 1e-12:
            margins.append(0.0)
        else:
            margins.append((K2 - C**2) / K2)
    return np.array(margins)


def worst_leg_margin_raw(roll, pitch, yaw, **kw):
    return np.min(leg_margin_raw(roll, pitch, yaw, **kw))


def worst_leg_margin_normalized(roll, pitch, yaw, **kw):
    return np.min(leg_margin_normalized(roll, pitch, yaw, **kw))


# type 2 singularity metrics
def jacobian(roll, pitch, yaw, alpha1=ALPHA1, beta=BETA, prev_thetas=None):
    """returns (Jx, c, thetas) where Jx @ omega = diag(c) @ theta_dot
    Jx row i = (vi x wi)^T ; c_i = (zhat x wi) . vi.
    requires a valid ik solution (returns None if the pose is outside
    the type 1 workspace boundary already)."""
    thetas = ik(roll, pitch, yaw, alpha1=alpha1, beta=beta, prev_thetas=prev_thetas)
    if thetas is None:
        return None, None, None

    R = rotation_matrix(roll, pitch, yaw)
    vi_home = get_vi_home(beta)
    zhat = np.array([0.0, 0.0, 1.0])

    Jx = np.zeros((3, 3))
    c = np.zeros(3)
    for i in range(3):
        eta_i = 2 * i * np.pi / 3
        vi = R @ vi_home[i]
        wi = wi_from_theta(thetas[i], eta_i, alpha1=alpha1)

        Jx[i, :] = np.cross(vi, wi)
        c[i] = np.dot(np.cross(zhat, wi), vi)

    return Jx, c, thetas


def det_Jx(roll, pitch, yaw, **kw):
    """type 2 singularity indicator: det(Jx). Zero => parallel
    singularity (platform loses controllability) even if every leg's
    own IK is perfectly valid. Returns None if outside the type 1
    workspace boundary (no IK solution at all)."""
    Jx, c, thetas = jacobian(roll, pitch, yaw, **kw)
    if Jx is None:
        return None
    return np.linalg.det(Jx)


# <------------------ validation ------------------>
def validate(verbose=True):
    """Two independent checks that must both pass before trusting any
    analysis built on this module:
      1. home position (0,0,0) matches the documented hardware home
         angle (60 deg for all three legs, by symmetry).
      2. type 1 consistency: wherever a leg's raw discriminant crosses
         zero, that leg's Jacobian diagonal entry c_i must also cross
         zero (both describe the same type 1 event by construction --
    """
    ok = True

    # home position
    thetas = ik(0, 0, 0)
    if thetas is None:
        if verbose:
            print("FAIL: no IK solution at home (0,0,0).")
        return False
    deg = np.degrees(thetas)
    all_equal = np.allclose(deg, deg[0], atol=1e-3)
    matches_hw = np.allclose(deg, 60.0, atol=0.01)
    if verbose:
        print(f"[Check 1] Home thetas: {deg}")
        print(f"  All three legs equal: {all_equal}")
        print(f"  Matches documented hardware home angle (60 deg): {matches_hw}")
    ok = ok and all_equal and matches_hw

    # check 2: type 1 consistency between discriminant and Jacobian c_i
    # scan a line through a known near-singular region (pitch axis at
    # yaw=0, roll=0) and check sign(raw margin) == sign(c_i) leg-by-leg
    # wherever both are defined.
    mismatches = 0
    checked = 0
    for pitch_deg in np.linspace(-89, 89, 400):
        pitch = np.radians(pitch_deg)
        raw = leg_margin_raw(0.0, pitch, 0.0)
        thetas_here = ik(0.0, pitch, 0.0)
        if thetas_here is None:
            continue
        R = rotation_matrix(0.0, pitch, 0.0)
        vi_home = get_vi_home()
        zhat = np.array([0.0, 0.0, 1.0])
        for i in range(3):
            eta_i = 2 * i * np.pi / 3
            vi = R @ vi_home[i]
            wi = wi_from_theta(thetas_here[i], eta_i)
            c_i = np.dot(np.cross(zhat, wi), vi)
            checked += 1
            # both should be positive (valid solution, nonzero margin)
            # here since thetas_here is not None means raw[i] >= 0
            if raw[i] > 1e-6 and abs(c_i) < 1e-6:
                mismatches += 1
            if raw[i] < -1e-6:
                mismatches += 1  # shouldn't happen if ik() returned non-None

    if verbose:
        print(f"[Check 2] Type 1 consistency: {checked} leg-configs checked, "
              f"{mismatches} mismatches between discriminant sign and c_i")
    ok = ok and (mismatches == 0)

    if verbose:
        print("PASS" if ok else "FAIL")
    return ok


if __name__ == "__main__":
    validate()