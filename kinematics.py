"""
kinematics.py - Forward & Inverse Kinematics and Trajectory Interpolation
for a 6-Axis Industrial Robot Arm (ABB IRB-style, Z-up coordinate system).

Supports:
  - Forward Kinematics (FK)
  - Analytical Inverse Kinematics (IK)
  - MoveJ: Joint-space cosine-eased interpolation
  - MoveL: Cartesian linear interpolation
  - MoveC: 3D circular arc interpolation through 3 points (Start -> Via -> End)
"""

import math
from typing import List, Dict, Any

# Robot geometric link parameters (mm)
L1 = 400.0     # Base height to J2 shoulder joint
L2 = 500.0     # Lower arm length (J2 to J3)
L3 = 450.0     # Forearm length (J3 to J5 wrist center)
L_TCP = 150.0  # Wrist center (J5) to Tool Center Point (TCP)

# Joint limits (degrees)
JOINT_LIMITS = [
    (-180.0, 180.0),  # J1: Base rotation
    (-110.0, 110.0),  # J2: Shoulder pitch
    (-135.0, 150.0),  # J3: Elbow pitch
    (-180.0, 180.0),  # J4: Wrist roll
    (-120.0, 120.0),  # J5: Wrist pitch
    (-360.0, 360.0),  # J6: Tool flange rotation
]

HOME_JOINTS = [0.0, 15.0, 75.0, 0.0, 90.0, 0.0]


class KinematicsError(Exception):
    """Raised when a target position is outside the robot's reachable workspace."""
    pass


def _clamp(val: float, low: float, high: float) -> float:
    return max(low, min(high, val))


def _vec_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _vec_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def _vec_scale(a, s):
    return [a[0] * s, a[1] * s, a[2] * s]


def _vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _vec_cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _vec_norm(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def forward_kinematics(joints: List[float]) -> Dict[str, Any]:
    """
    Compute the 3D position (X, Y, Z in mm) of the TCP and intermediate joints
    given 6 joint angles [j1, j2, j3, j4, j5, j6] in degrees.
    """
    j1, j2, j3, j4, j5, j6 = [math.radians(a) for a in joints]
    c1, s1 = math.cos(j1), math.sin(j1)

    x2, y2, z2 = 0.0, 0.0, L1

    r3 = L2 * math.sin(j2)
    x3, y3, z3 = r3 * c1, r3 * s1, L1 + L2 * math.cos(j2)

    r5 = r3 + L3 * math.sin(j2 + j3)
    x5, y5, z5 = r5 * c1, r5 * s1, z3 + L3 * math.cos(j2 + j3)

    r_tcp = r5 + L_TCP * math.sin(j2 + j3 + j5)
    x_tcp, y_tcp, z_tcp = r_tcp * c1, r_tcp * s1, z5 + L_TCP * math.cos(j2 + j3 + j5)

    return {
        "tcp": [round(x_tcp, 2), round(y_tcp, 2), round(z_tcp, 2)],
        "wrist": [round(x5, 2), round(y5, 2), round(z5, 2)],
        "elbow": [round(x3, 2), round(y3, 2), round(z3, 2)],
        "shoulder": [round(x2, 2), round(y2, 2), round(z2, 2)],
    }


def inverse_kinematics(x: float, y: float, z: float, tool_roll: float = 0.0) -> List[float]:
    """
    Analytical Inverse Kinematics for a 6-axis articulated arm with the tool
    pointing vertically downward (-Z direction).
    Returns [j1, j2, j3, j4, j5, j6] in degrees.
    """
    wx, wy, wz = x, y, z + L_TCP

    r = math.hypot(wx, wy)
    j1_rad = 0.0 if r < 1e-6 else math.atan2(wy, wx)

    h = wz - L1
    d = math.hypot(r, h)

    max_reach = L2 + L3 - 1.0
    min_reach = abs(L2 - L3) + 1.0

    if d > max_reach or d < min_reach:
        raise KinematicsError(
            f"Target [{x:.1f}, {y:.1f}, {z:.1f}] mm is out of reach "
            f"(wrist distance {d:.1f} mm, valid range {min_reach:.0f}-{max_reach:.0f} mm)."
        )

    cos_j3 = _clamp((d * d - L2 * L2 - L3 * L3) / (2.0 * L2 * L3), -1.0, 1.0)
    j3_rad = math.acos(cos_j3)

    alpha = math.atan2(r, h)
    cos_beta = _clamp((L2 * L2 + d * d - L3 * L3) / (2.0 * L2 * d), -1.0, 1.0)
    beta = math.acos(cos_beta)
    j2_rad = alpha - beta

    j1 = math.degrees(j1_rad)
    j2 = math.degrees(j2_rad)
    j3 = math.degrees(j3_rad)
    j4 = 0.0
    j5 = 180.0 - (j2 + j3)
    j6 = tool_roll

    joints = [round(j1, 3), round(j2, 3), round(j3, 3), round(j4, 3), round(j5, 3), round(j6, 3)]

    for idx, (val, (low, high)) in enumerate(zip(joints, JOINT_LIMITS), start=1):
        if val < low - 0.5 or val > high + 0.5:
            raise KinematicsError(
                f"Target [{x:.1f}, {y:.1f}, {z:.1f}] mm exceeds J{idx} limit "
                f"({val:.1f} deg, allowed [{low:.0f}, {high:.0f}])."
            )

    return joints


def interpolate_movej(
    start_joints: List[float],
    end_joints: List[float],
    speed_mm_s: float,
    fps: int = 30,
) -> List[Dict[str, Any]]:
    """Joint-space interpolation (MoveJ) with cosine ease-in/ease-out."""
    max_delta = max(abs(e - s) for s, e in zip(start_joints, end_joints))
    deg_per_sec = max(25.0, min(220.0, speed_mm_s * 0.22))
    duration = max(0.25, max_delta / deg_per_sec)
    steps = max(8, int(duration * fps))

    frames = []
    for i in range(1, steps + 1):
        u = i / steps
        t = 0.5 - 0.5 * math.cos(math.pi * u)
        joints = [round(s + (e - s) * t, 3) for s, e in zip(start_joints, end_joints)]
        fk = forward_kinematics(joints)
        frames.append({"joints": joints, "tcp": fk["tcp"]})
    return frames


def interpolate_movel(
    start_tcp: List[float],
    end_tcp: List[float],
    speed_mm_s: float,
    fps: int = 30,
) -> List[Dict[str, Any]]:
    """Cartesian linear interpolation (MoveL)."""
    dx = end_tcp[0] - start_tcp[0]
    dy = end_tcp[1] - start_tcp[1]
    dz = end_tcp[2] - start_tcp[2]
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)

    duration = max(0.2, dist / max(40.0, speed_mm_s))
    steps = max(8, int(duration * fps))

    frames = []
    for i in range(1, steps + 1):
        u = i / steps
        t = 0.5 - 0.5 * math.cos(math.pi * u)
        x = start_tcp[0] + dx * t
        y = start_tcp[1] + dy * t
        z = start_tcp[2] + dz * t
        joints = inverse_kinematics(x, y, z)
        frames.append({"joints": joints, "tcp": [round(x, 2), round(y, 2), round(z, 2)]})
    return frames


def interpolate_movec(
    start_tcp: List[float],
    via_tcp: List[float],
    end_tcp: List[float],
    speed_mm_s: float,
    fps: int = 30,
) -> List[Dict[str, Any]]:
    """
    3D circular arc interpolation (MoveC) passing from start_tcp through
    via_tcp to end_tcp. Falls back to two MoveL segments if points are collinear.
    """
    p1, p2, p3 = start_tcp, via_tcp, end_tcp
    v1 = _vec_sub(p2, p1)
    v2 = _vec_sub(p3, p1)
    normal = _vec_cross(v1, v2)
    n_len = _vec_norm(normal)

    if n_len < 1e-3:
        # Collinear points -> fall back to linear motion
        return interpolate_movel(p1, p2, speed_mm_s, fps) + interpolate_movel(p2, p3, speed_mm_s, fps)

    # Local 2D orthonormal basis (e_x, e_y) in the circle's plane
    u_len = _vec_norm(v1)
    e_x = _vec_scale(v1, 1.0 / u_len)
    e_z = _vec_scale(normal, 1.0 / n_len)
    e_y = _vec_cross(e_z, e_x)

    # 2D coordinates of p1=(0,0), p2=(x2,0), p3=(x3,y3)
    x2 = u_len
    x3 = _vec_dot(v2, e_x)
    y3 = _vec_dot(v2, e_y)

    # Circumcenter in 2D plane
    cx = x2 / 2.0
    cy = ((x3 - cx) ** 2 + y3 ** 2 - cx ** 2) / (2.0 * y3)
    radius = math.hypot(cx, cy)

    center = _vec_add(p1, _vec_add(_vec_scale(e_x, cx), _vec_scale(e_y, cy)))

    # Angles of p1, p2, p3 around center
    a1 = math.atan2(-cy, -cx)
    a2 = math.atan2(-cy, x2 - cx)
    a3 = math.atan2(y3 - cy, x3 - cx)

    def _norm_pos(ang):
        return ang % (2.0 * math.pi)

    d12 = _norm_pos(a2 - a1)
    d13 = _norm_pos(a3 - a1)
    if d12 < d13:
        total_angle = d13
    else:
        total_angle = -(2.0 * math.pi - d13)

    arc_len = abs(total_angle) * radius
    duration = max(0.3, arc_len / max(40.0, speed_mm_s))
    steps = max(14, int(duration * fps))

    frames = []
    for i in range(1, steps + 1):
        u = i / steps
        t = 0.5 - 0.5 * math.cos(math.pi * u)
        ang = a1 + total_angle * t
        pt = _vec_add(
            center,
            _vec_add(_vec_scale(e_x, radius * math.cos(ang)), _vec_scale(e_y, radius * math.sin(ang))),
        )
        joints = inverse_kinematics(pt[0], pt[1], pt[2])
        frames.append({
            "joints": joints,
            "tcp": [round(pt[0], 2), round(pt[1], 2), round(pt[2], 2)],
        })
    return frames
