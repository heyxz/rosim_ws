"""GP12 kinematics taken from the project URDF (numpy only).

The demo model was a hand transcription that missed the two fixed joints
before j1 (``rail_joint`` 0.09 m + ``carriage_base_mount`` 0.077 m), so its TCP
sat 167 mm lower than the real robot's ``tool_tip``. This module keeps the same
chain (verified joint-by-joint against ``robot.urdf.xacro``) and adds that base
offset, so TCP positions here match MoveIt's ``tool_tip`` frame exactly.

    base -> rail(0.09) -> mount(0.077) -> j1(Z) -> j2(Y) -> j3(-Y)
         -> j4(-X) -> j5(-Y) -> j6(-X) -> flange(0.1) -> tool_tip

Collision is a capsule per link segment against spheres, the same approximation
used for training; MoveIt/FCL remains the authority at execution time.
"""

from __future__ import annotations

import numpy as np

# --- chain (parent-link coordinates, identical to robot.urdf.xacro) ----------
JOINTS = [
    ("z", np.array([0.000, 0.000, 0.000])),
    ("y", np.array([0.155, 0.000, 0.450])),
    ("y", np.array([0.000, 0.000, 0.614])),
    ("x", np.array([0.640, 0.000, 0.200])),
    ("y", np.array([0.000, 0.000, 0.000])),
    ("x", np.array([0.000, 0.000, 0.000])),
]
AXIS_SIGN = np.array([1.0, 1.0, -1.0, -1.0, -1.0, -1.0])

FLANGE_TOOL0 = (np.array([0.100, 0.000, 0.000]), np.array([np.pi, -np.pi / 2, 0.0]))
TOOL0_TO_TIP = (np.array([-0.081476, 0.000022, 0.364356]), np.array([0.0, 0.6283185, np.pi]))

# rail_joint (0.09) + carriage_base_mount (0.077): present in the URDF, missing
# from the demo model. Everything else already matched.
BASE_OFFSET = np.array([0.0, 0.0, 0.167])

# URDF limits, with |j4| and |j6| clipped to pi (their URDF range is wider than
# one turn; keeping them inside +-pi removes the wrap ambiguity for learning).
LIMITS = np.array([
    [-2.967, 2.967],
    [-1.571, 2.705],
    [-1.484, 2.618],
    [-3.000, 3.000],
    [-2.618, 2.618],
    [-3.000, 3.000],
])

# Capsule radius per link segment (base->j2, j2->j3, ..., flange->tcp).
# 8 segments: base->j1->j2->j3->j4->j5->j6->flange->tcp (forward returns 9 points)
CAPSULE_RADII = np.array([0.10, 0.09, 0.08, 0.07, 0.06, 0.05, 0.04, 0.03])

Q_HOME = np.array([0.0, -0.6, 0.0, 0.0, 0.6, 0.0])


def rot(axis: str, angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy(r: float, p: float, y: float) -> np.ndarray:
    return rot("z", y) @ rot("y", p) @ rot("x", r)


def tf(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def forward(q: np.ndarray):
    """Return (link origins [8x3] incl. BASE_OFFSET, tcp position [3]).

    Points are base, j2, j3, j4, j5, j6, flange, tcp - the capsule chain.
    """
    T = np.eye(4)
    pts = [T[:3, 3].copy()]
    for i, (axis, origin) in enumerate(JOINTS):
        T = T @ tf(np.eye(3), origin)
        T = T @ tf(rot(axis, AXIS_SIGN[i] * float(q[i])), np.zeros(3))
        pts.append(T[:3, 3].copy())
    T = T @ tf(rpy(*FLANGE_TOOL0[1]), FLANGE_TOOL0[0])
    pts.append(T[:3, 3].copy())
    T = T @ tf(rpy(*TOOL0_TO_TIP[1]), TOOL0_TO_TIP[0])
    pts.append(T[:3, 3].copy())
    pts = np.array(pts) + BASE_OFFSET
    return pts, pts[-1].copy()


def segment_distance(p0: np.ndarray, p1: np.ndarray, c: np.ndarray) -> float:
    d = p1 - p0
    denom = float(d @ d)
    if denom < 1e-12:
        return float(np.linalg.norm(c - p0))
    t = np.clip(float((c - p0) @ d) / denom, 0.0, 1.0)
    return float(np.linalg.norm(c - (p0 + t * d)))


def min_clearance(pts: np.ndarray, obstacles: np.ndarray) -> float:
    """Signed clearance (m) to the nearest sphere surface; <0 means contact.

    Vectorised over the 7 capsules x N spheres: this runs once per env step, so
    the Python loop version was the training bottleneck.
    """
    if len(obstacles) == 0:
        return 1.0
    p0 = pts[:-1][:, None, :]            # (7, 1, 3)
    p1 = pts[1:][:, None, :]
    c = obstacles[None, :, :3]           # (1, N, 3)
    r = obstacles[None, :, 3]            # (1, N)
    d = p1 - p0
    denom = np.maximum((d * d).sum(-1, keepdims=True), 1e-12)
    t = np.clip(((c - p0) * d).sum(-1, keepdims=True) / denom, 0.0, 1.0)
    dist = np.linalg.norm(c - (p0 + t * d), axis=-1)
    clr = dist - CAPSULE_RADII[:, None] - r
    return float(clr.min())


def box_spheres(centre, half, radius=0.05):
    """Fill a box with a grid of spheres - a cheap stand-in for mesh collision."""
    nx = max(1, int(np.ceil(2 * half[0] / (1.6 * radius))))
    ny = max(1, int(np.ceil(2 * half[1] / (1.6 * radius))))
    nz = max(1, int(np.ceil(2 * half[2] / (1.6 * radius))))
    xs = np.linspace(centre[0] - half[0] + radius, centre[0] + half[0] - radius, nx) if nx > 1 else [centre[0]]
    ys = np.linspace(centre[1] - half[1] + radius, centre[1] + half[1] - radius, ny) if ny > 1 else [centre[1]]
    zs = np.linspace(centre[2] - half[2] + radius, centre[2] + half[2] - radius, nz) if nz > 1 else [centre[2]]
    out = []
    for x in xs:
        for y in ys:
            for z in zs:
                out.append([x, y, z, radius])
    return np.array(out, dtype=np.float64)
