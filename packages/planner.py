import math
from typing import Callable, List, Optional, Tuple

import numpy as np
from numpy.random import RandomState

from collision_checker import CSpaceChecker
from collision_protocol import FriendlyPose, PlanningQuery, PlanningResult, PlanningSetup, PlanStep
from utils import more_granular, normalize_angle, simulate


# ── Motion-model constants ────────────────────────────────────────────────────

# Orientations are quantized to multiples of this value.
# 360 / 5 = 72 distinct slices — all pre-computed once at startup.
_THETA_STEP_DEG: float = 5.0
_N_THETA: int = int(round(360.0 / _THETA_STEP_DEG))  # 72

# Maximum rotation per single turn step (must be a multiple of _THETA_STEP_DEG).
_MAX_TURN_DEG: float = 30.0

# Maximum translation per single straight step (meters).
# Small enough to maneuver through tight gaps between signs/holes (~0.2-0.4 m).
_STEP_DIST_M: float = 0.12

# Drive straight if heading error is below this threshold.
_TURN_THRESHOLD_DEG: float = _THETA_STEP_DEG / 2.0   # 2.5°


# ── Utility ───────────────────────────────────────────────────────────────────

def _snap_theta(theta_deg: float) -> float:
    """Round theta to the nearest multiple of _THETA_STEP_DEG in [0, 360)."""
    return (round(theta_deg / _THETA_STEP_DEG) * _THETA_STEP_DEG) % 360.0


# ── Main planner ──────────────────────────────────────────────────────────────

def plan(
    ps: PlanningSetup,
    q: PlanningQuery,
    max_iter: int = 5000,
    goal_bias: float = 0.10,
    check_dt: float = 0.10,
    seed: int = 42,
    on_extend: Optional[Callable[[FriendlyPose, FriendlyPose], None]] = None,
) -> PlanningResult:
    """
    Plan a path from q.start to q.target using RRT.

    All node orientations are kept on a discrete _THETA_STEP_DEG grid.
    The CSpaceChecker's _N_THETA slices are pre-computed once at startup
    so every collision query during the RRT loop hits the cache in O(1).
    """
    # TODO: implement the RRT planner.  Suggested structure:
    #   1. checker = CSpaceChecker(ps.environment, ps.body); pre-warm every slice:
    #      for i in range(_N_THETA): checker._slice(i * _THETA_STEP_DEG)
    #   2. Snap start/target theta with _snap_theta; if checker.check(start) or
    #      checker.check(target): return PlanningResult(False, None).
    #   3. Tree state as parallel lists: nodes=[start], parents=[None], edge_steps=[None].
    #      For fast nearest-neighbour also keep a preallocated feats buffer with rows
    #      [x, y, cos(theta), sin(theta)] parallel to `nodes` (see _nearest).
    #   4. Loop up to max_iter:
    #        a. With prob goal_bias sample q_rand = target, else sample x,y in ps.bounds
    #           and theta from {0,5,...,355} via rs.randint(0, _N_THETA).
    #        b. nearest_idx = _nearest(feats, len(nodes), q_rand); q_near = nodes[nearest_idx].
    #        c. steer_result = _steer(q_near, q_rand, ps.max_linear_velocity_m_s,
    #           ps.max_angular_velocity_deg_s); skip if None.
    #        d. steps, q_new = steer_result; skip if not
    #           _is_path_collision_free(checker, q_near, steps, check_dt).
    #        e. Append q_new / nearest_idx / steps (and its feats row).  If on_extend is
    #           not None, call on_extend(q_near, q_new)  # powers the notebook animation.
    #        f. If _reached(q_new, q_target, ps.tolerance_xy_m, ps.tolerance_theta_deg):
    #           return PlanningResult(True, _extract_path(parents, edge_steps, new_idx)).
    #   5. return PlanningResult(False, None).
    raise NotImplementedError


# ── RRT helpers ───────────────────────────────────────────────────────────────

def _nearest(feats: np.ndarray, n: int, q: FriendlyPose) -> int:
    """Return the index of the node closest to q among the first n nodes.

    ``feats`` has columns [x, y, cos(theta), sin(theta)]. This is a vectorized
    form of the original metric sqrt(dx^2 + dy^2) + 0.1 * |normalize_angle(dtheta)|:
    the angular term equals arccos(cos_p*cos_q + sin_p*sin_q), i.e. the absolute
    angle between the two heading unit vectors (always in [0, pi]).
    """
    # TODO: implement nearest-node search over feats[:n]
    #   - position term: sqrt((feats[:n,0]-q.x)**2 + (feats[:n,1]-q.y)**2)
    #   - angular term:  0.1 * arccos(clip(feats[:n,2]*cos(q) + feats[:n,3]*sin(q), -1, 1))
    #   - return int(np.argmin(position + angular))
    raise NotImplementedError


def _steer(
    q_near: FriendlyPose,
    q_rand: FriendlyPose,
    max_linear_velocity_m_s: float,
    max_angular_velocity_deg_s: float,
) -> Optional[Tuple[List[PlanStep], FriendlyPose]]:
    """
    Produce a SINGLE motion primitive from q_near toward q_rand.

    The turn angle is snapped to the nearest multiple of _THETA_STEP_DEG and
    capped at _MAX_TURN_DEG, keeping every resulting node orientation on the
    discrete grid so all C-space queries hit the pre-warmed cache.

    Primitives:
      - Turn left/right : velocity_x=0,   angular_velocity=±max_w
      - Drive straight  : velocity_x=max_v, angular_velocity=0
    """
    # TODO: implement single-primitive steer with snapped, capped turns
    #   - dx, dy, dist to q_rand.  If dist > 1e-4: target_hdg = degrees(atan2(dy, dx));
    #     dtheta = wrapped(target_hdg - q_near.theta_deg); snap to _THETA_STEP_DEG and
    #     cap to +/-_MAX_TURN_DEG.  If |capped| > _TURN_THRESHOLD_DEG -> a turn step,
    #     else a straight step of min(dist, _STEP_DIST_M).
    #   - If dist <= 1e-4: turn toward q_rand.theta_deg (same snap/cap); None if aligned.
    #   - simulate the step, snap q_new's theta, return ([step], q_new).
    raise NotImplementedError


def _is_path_collision_free(
    checker: CSpaceChecker,
    start: FriendlyPose,
    steps: List[PlanStep],
    dt: float,
) -> bool:
    """
    Return True if no pose along the path is in collision.

    Turn steps are checked at every _THETA_STEP_DEG increment (all cached).
    Straight steps are sampled at time-step dt (orientation is constant
    and already cached for the node's snapped theta).
    """
    # TODO: implement the two-case collision check
    #   - Turn step (velocity_x ~ 0): total_turn = angular_velocity_deg_s * duration;
    #     sample n = max(2, round(|total_turn|/_THETA_STEP_DEG)+1) orientations across
    #     the turn (snap each) and checker.check at (cur.x, cur.y, theta).
    #   - Straight step: fine = more_granular([step], dt); checker.check each pose of
    #     simulate(cur, fine).  Then advance cur = simulate(cur, [step]).poses[-1].
    #   Return False on any collision, else True.
    raise NotImplementedError


def _reached(
    q: FriendlyPose,
    target: FriendlyPose,
    tol_xy: float,
    tol_theta_deg: float,
) -> bool:
    """Return True if q is within tolerance of target."""
    # TODO: return True iff Euclidean (x, y) distance <= tol_xy AND the wrapped
    #       angular difference (in degrees) <= tol_theta_deg.
    raise NotImplementedError


def _extract_path(
    parents: List[Optional[int]],
    edge_steps: List[Optional[List[PlanStep]]],
    goal_idx: int,
) -> List[PlanStep]:
    """Trace back from goal_idx to root, collect steps, reverse, and concatenate."""
    # TODO: walk parents from goal_idx back to the root collecting edge_steps,
    #       reverse, and concatenate into a single List[PlanStep].
    raise NotImplementedError
