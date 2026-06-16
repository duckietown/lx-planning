"""
Pure-numpy SE2 rigid-body math.

Drop-in replacement for the `geometry` (PyGeometry-z6) and `duckietown_world`
SE2 helpers, removing all zuper/duckietown-world-ente transitive dependencies.
"""
from __future__ import annotations

from typing import Tuple

import numpy as np

# Matches the geometry.SE2value type (a 3x3 numpy array)
SE2value = np.ndarray


# ---------------------------------------------------------------------------
# Conversion between FriendlyPose and 3x3 SE2 homogeneous matrices
# ---------------------------------------------------------------------------

def pose_from_friendly(pose) -> np.ndarray:
    """FriendlyPose(x, y, theta_deg) → 3×3 SE2 homogeneous matrix."""
    theta = np.deg2rad(pose.theta_deg)
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, pose.x],
        [s,  c, pose.y],
        [0,  0,    1.0],
    ])


def friendly_from_pose(q: np.ndarray):
    """3×3 SE2 homogeneous matrix → FriendlyPose(x, y, theta_deg)."""
    from collision_protocol import FriendlyPose
    return FriendlyPose(
        x=float(q[0, 2]),
        y=float(q[1, 2]),
        theta_deg=float(np.rad2deg(np.arctan2(q[1, 0], q[0, 0]))),
    )


# ---------------------------------------------------------------------------
# Point transformation
# ---------------------------------------------------------------------------

def SE2_apply_R2(q: np.ndarray, point) -> Tuple[float, float]:
    """Apply a 3×3 SE2 transformation to a 2D point, returning (x, y)."""
    p = np.array([float(point[0]), float(point[1]), 1.0])
    r = q @ p
    return float(r[0]), float(r[1])


# ---------------------------------------------------------------------------
# Lie algebra / group conversions
# ---------------------------------------------------------------------------

def se2_from_linear_angular(linear, angular: float) -> np.ndarray:
    """Build an se2 Lie algebra element from linear [vx, vy] and angular velocity."""
    vx, vy = float(linear[0]), float(linear[1])
    w = float(angular)
    return np.array([
        [0.0,  -w,  vx],
        [  w, 0.0,  vy],
        [0.0, 0.0, 0.0],
    ])


def SE2_from_translation_angle(t, angle: float) -> np.ndarray:
    """Build a 3×3 SE2 matrix from translation (x, y) and angle in radians."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [c, -s, float(t[0])],
        [s,  c, float(t[1])],
        [0,  0,         1.0],
    ])


def translation_angle_from_SE2(q: np.ndarray) -> Tuple[np.ndarray, float]:
    """Extract (translation_vector, angle_rad) from a 3×3 SE2 matrix."""
    t = q[:2, 2].copy()
    angle = float(np.arctan2(q[1, 0], q[0, 0]))
    return t, angle


# ---------------------------------------------------------------------------
# SE2 group namespace  (mirrors the geometry.SE2 API used in the codebase)
# ---------------------------------------------------------------------------

class SE2:
    """SE2 group operations — pure numpy, no external dependencies."""

    @staticmethod
    def identity() -> np.ndarray:
        return np.eye(3)

    @staticmethod
    def inverse(q: np.ndarray) -> np.ndarray:
        R, t = q[:2, :2], q[:2, 2]
        inv = np.eye(3)
        inv[:2, :2] = R.T
        inv[:2, 2] = -R.T @ t
        return inv

    @staticmethod
    def multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        return q1 @ q2

    @staticmethod
    def group_from_algebra(V: np.ndarray) -> np.ndarray:
        """Closed-form matrix exponential for an se2 Lie algebra element."""
        w = float(V[1, 0])
        vx, vy = float(V[0, 2]), float(V[1, 2])
        if abs(w) < 1e-10:
            return np.array([
                [1.0, 0.0, vx],
                [0.0, 1.0, vy],
                [0.0, 0.0, 1.0],
            ])
        c, s = np.cos(w), np.sin(w)
        return np.array([
            [c,  -s,  (s * vx - (1.0 - c) * vy) / w],
            [s,   c,  ((1.0 - c) * vx + s * vy) / w],
            [0.0, 0.0, 1.0],
        ])

    @staticmethod
    def friendly(q: np.ndarray) -> str:
        """Human-readable string for a pose matrix."""
        t, angle = translation_angle_from_SE2(q)
        return f"(x={t[0]:.3f}, y={t[1]:.3f}, θ={np.rad2deg(angle):.1f}°)"
