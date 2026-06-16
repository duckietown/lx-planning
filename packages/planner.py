import math
from typing import List, Optional, Tuple

import numpy as np
from numpy.random import RandomState

from collision_checker import CSpaceChecker
from collision_protocol import FriendlyPose, PlanningQuery, PlanningResult, PlanningSetup, PlanStep
from utils import more_granular, normalize_angle, simulate


# ── Motion-model constants (provided — do not change) ────────────────────────

# Orientations are quantized to multiples of this value.
# 360 / 5 = 72 distinct slices — all pre-computed once at startup.
_THETA_STEP_DEG: float = 5.0
_N_THETA: int = int(round(360.0 / _THETA_STEP_DEG))  # 72

# Maximum rotation per single turn step (must be a multiple of _THETA_STEP_DEG).
_MAX_TURN_DEG: float = 30.0

# Maximum translation per single straight step (meters).
_STEP_DIST_M: float = 0.3

# Drive straight if heading error is below this threshold.
_TURN_THRESHOLD_DEG: float = _THETA_STEP_DEG / 2.0   # 2.5°


def _snap_theta(theta_deg: float) -> float:
    """Round theta to the nearest multiple of _THETA_STEP_DEG in [0, 360).
    Provided — no need to modify."""
    return (round(theta_deg / _THETA_STEP_DEG) * _THETA_STEP_DEG) % 360.0


# ── Main planner ──────────────────────────────────────────────────────────────

def plan(
    ps: PlanningSetup,
    q: PlanningQuery,
    max_iter: int = 5000,
    goal_bias: float = 0.10,
    check_dt: float = 0.10,
    seed: int = 42,
) -> PlanningResult:
    """
    Plan a path from q.start to q.target using RRT.

    Suggested structure:

      1. Create a CSpaceChecker from ps.environment and ps.body.

      2. Pre-warm the cache — call checker._slice(i * _THETA_STEP_DEG) for
         i in range(_N_THETA).  This computes all 72 C-space slices up front
         so that every collision query during the RRT loop is an O(1) cache
         hit.  Without this, each new orientation encountered triggers an
         expensive Minkowski-sum computation.

      3. Snap start and target orientations to the grid with _snap_theta.
         Return PlanningResult(False, None) if either is in collision.

      4. Initialise the RRT:
           nodes:      [q_start]  — list of FriendlyPose (snapped theta)
           parents:    [None]
           edge_steps: [None]

      5. Loop up to max_iter times:
           a. Sample q_rand: with probability goal_bias use q_target;
              otherwise draw x,y in ps.bounds and theta from the discrete
              set {0, 5, 10, ..., 355} using rs.randint(0, _N_THETA).
           b. nearest_idx = _nearest(nodes, q_rand)
           c. steer_result = _steer(nodes[nearest_idx], q_rand,
                                    ps.max_linear_velocity_m_s,
                                    ps.max_angular_velocity_deg_s)
              — skip if None
           d. steps, q_new = steer_result
           e. If _is_path_collision_free(checker, nodes[nearest_idx], steps, check_dt):
                - Append q_new / nearest_idx / steps.
                - new_idx = len(nodes) - 1
                - If _reached(q_new, q_target, ps.tolerance_xy_m,
                              ps.tolerance_theta_deg):
                    return PlanningResult(True,
                                          _extract_path(parents, edge_steps, new_idx))

      6. Return PlanningResult(False, None).
    """
    # TODO: implement RRT planner
    raise NotImplementedError


# ── RRT helpers ───────────────────────────────────────────────────────────────

def _nearest(nodes: List[FriendlyPose], q: FriendlyPose) -> int:
    """
    Return the index of the node in nodes closest to q.

    Hint: sqrt(dx²+dy²) + 0.1 * |normalize_angle(deg2rad(Δtheta))|
    """
    # TODO: implement nearest-node search
    raise NotImplementedError


def _steer(
    q_near: FriendlyPose,
    q_rand: FriendlyPose,
    max_linear_velocity_m_s: float,
    max_angular_velocity_deg_s: float,
) -> Optional[Tuple[List[PlanStep], FriendlyPose]]:
    """
    Produce a SINGLE motion primitive from q_near toward q_rand.

    Primitives:
      - Turn left/right : velocity_x=0,    angular_velocity=±max_w
      - Drive straight  : velocity_x=max_v, angular_velocity=0

    Decision logic when dist(q_near, q_rand) > 1e-4:
      1. target_hdg = degrees(atan2(dy, dx))
      2. dtheta_deg = wrapped difference between target_hdg and q_near.theta_deg
      3. snapped    = round(dtheta_deg / _THETA_STEP_DEG) * _THETA_STEP_DEG
      4. capped     = clamp(snapped, -_MAX_TURN_DEG, +_MAX_TURN_DEG)
      5. If |capped| > _TURN_THRESHOLD_DEG: return a turn step of capped degrees.
         Else: return a straight step of min(dist, _STEP_DIST_M) meters.

    When dist <= 1e-4 (same position): turn to match q_rand's orientation
    using the same snap/cap logic; return None if already aligned.

    After computing the step, simulate to get q_new and snap its theta:
      q_new = FriendlyPose(raw.x, raw.y, _snap_theta(raw.theta_deg))

    Return ([step], q_new).
    """
    # TODO: implement single-primitive steer with snapped turns
    raise NotImplementedError


def _is_path_collision_free(
    checker: CSpaceChecker,
    start: FriendlyPose,
    steps: List[PlanStep],
    dt: float = 0.10,
) -> bool:
    """
    Return True if no pose along the path is in collision.

    Turn steps (velocity_x ≈ 0): check at every _THETA_STEP_DEG increment
    of the rotation.  All intermediate orientations are multiples of
    _THETA_STEP_DEG and therefore already in the pre-warmed cache.

      total_turn = angular_velocity_deg_s * duration   (signed degrees)
      n_checks   = round(|total_turn| / _THETA_STEP_DEG) + 1
      for i in range(n_checks):
          frac  = i / (n_checks - 1)
          theta = _snap_theta(start.theta_deg + frac * total_turn)
          if checker.check(FriendlyPose(start.x, start.y, theta)):
              return False

    Straight steps (angular_velocity ≈ 0): use more_granular(steps, dt) +
    simulate, then check each pose.  Orientation is constant and cached.
    """
    # TODO: implement collision check (two-case: turn vs straight)
    raise NotImplementedError


def _reached(
    q: FriendlyPose,
    target: FriendlyPose,
    tol_xy: float,
    tol_theta_deg: float,
) -> bool:
    """
    Return True if q is within tolerance of target.

    Check Euclidean (x, y) distance <= tol_xy AND
    wrapped angular difference in degrees <= tol_theta_deg.
    """
    # TODO: implement goal check
    raise NotImplementedError


def _extract_path(
    parents: List[Optional[int]],
    edge_steps: List[Optional[List[PlanStep]]],
    goal_idx: int,
) -> List[PlanStep]:
    """
    Trace back from goal_idx to root via parent pointers,
    collecting edge_steps.  Reverse and concatenate.
    """
    # TODO: implement path extraction
    raise NotImplementedError
