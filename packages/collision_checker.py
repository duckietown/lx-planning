from typing import Dict, List

from shapely.affinity import rotate, scale, translate
from shapely.geometry import Point as SPoint, box
from shapely import unary_union

from collision_protocol import (
    Circle,
    FriendlyPose,
    PlacedPrimitive,
    Rectangle,
)


# ---------------------------------------------------------------------------
# Utility helpers (provided — no need to modify)
# ---------------------------------------------------------------------------

def _primitive_shapely(p):
    """Convert a Primitive to a Shapely geometry centred at the origin."""
    if isinstance(p, Circle):
        return SPoint(0.0, 0.0).buffer(p.radius)
    return box(minx=p.xmin, maxx=p.xmax, miny=p.ymin, maxy=p.ymax)


def _obs_shapely(obs: PlacedPrimitive):
    """Convert an obstacle PlacedPrimitive to its world-frame Shapely geometry."""
    shape = _primitive_shapely(obs.primitive)
    shape = rotate(shape, obs.pose.theta_deg, origin=(0, 0))
    return translate(shape, obs.pose.x, obs.pose.y)


# ---------------------------------------------------------------------------
# Geometry helpers — implement these as part of the exercise
# ---------------------------------------------------------------------------

def _body_at_theta(body_part: PlacedPrimitive, theta_deg: float):
    """
    Return the Shapely shape of robot body part when the robot center is at the
    origin with global orientation theta_deg.

    Steps:
      1. Get the primitive shape at the origin (_primitive_shapely).
      2. Apply the body part's local pose: rotate by body_part.pose.theta_deg,
         then translate by (body_part.pose.x, body_part.pose.y).
      3. Rotate the whole thing by theta_deg about the origin.

    Hint: use rotate(..., origin=(0, 0)) and translate from shapely.affinity.
    """
    # TODO: implement this function
    raise NotImplementedError


def _cspace_obstacle(obs: PlacedPrimitive, body_reflected):
    """
    Return the C-space obstacle for one (obstacle, reflected-body-part) pair.

    The C-space obstacle is obs ⊕ body_reflected, i.e., the set of robot-centre
    positions at which the body (at its current orientation) would touch obs.

    Two cases depending on the obstacle primitive type:

    - Circle obstacle (radius r, centre (cx, cy)):
        Exact result: translate(body_reflected.buffer(r), cx, cy)
        (Minkowski sum with a disk = dilation, then shift to the circle's centre.)

    - Rectangle obstacle (convex polygon):
        Exact result for two convex polygons: translate the obstacle to every vertex
        of body_reflected and take the convex hull of the union.

    Hint: use _obs_shapely, body_reflected.exterior.coords, translate, unary_union,
          and .convex_hull.
    """
    # TODO: implement this function
    raise NotImplementedError


# ---------------------------------------------------------------------------
# Main class — implement _compute_slice as part of the exercise
# ---------------------------------------------------------------------------

class CSpaceChecker:
    """
    Collision checker that builds a 2D C-space slice per queried orientation, lazily and cached.

    For a fixed orientation θ, the C-space forbidden region is the set of (x, y) positions
    where the robot body placed at (x, y, θ) intersects any obstacle.  It is computed once
    via Minkowski sums and reused for all subsequent queries at the same θ.

    Usage:
        checker = CSpaceChecker(environment, robot_body)
        if checker.check(pose):
            ...
        d = checker.distance(pose)   # clearance, 0 if in collision
    """

    def __init__(self, environment: List[PlacedPrimitive], robot_body: List[PlacedPrimitive]):
        self._environment = environment
        self._robot_body = robot_body
        self._cache: Dict[float, object] = {}

    def check(self, robot_pose: FriendlyPose) -> bool:
        """Return True if the robot at robot_pose collides with any obstacle."""
        forbidden = self._slice(robot_pose.theta_deg)
        return forbidden is not None and not forbidden.disjoint(SPoint(robot_pose.x, robot_pose.y))

    def distance(self, robot_pose: FriendlyPose) -> float:
        """Minimum distance from robot_pose to the nearest C-space obstacle (0.0 if in collision)."""
        forbidden = self._slice(robot_pose.theta_deg)
        if forbidden is None:
            return float("inf")
        return float(forbidden.distance(SPoint(robot_pose.x, robot_pose.y)))

    def _slice(self, theta_deg: float):
        """Return the cached forbidden region for theta_deg, computing it on first use."""
        if theta_deg not in self._cache:
            self._cache[theta_deg] = self._compute_slice(theta_deg)
        return self._cache[theta_deg]

    def _compute_slice(self, theta_deg: float):
        """
        Build the forbidden (x, y) region for a robot at orientation theta_deg.

        For each (body_part, obstacle) pair, compute the C-space obstacle via the
        Minkowski sum formula:

            CO(θ) = obstacle ⊕ (−body_at_theta)

        where body_at_theta is the body part shape when the robot sits at the origin
        facing theta_deg, and −body_at_theta is its reflection through the origin.

        Return the union of all C-space obstacles, or None if the environment is empty.

        Hint: use _body_at_theta, scale(..., xfact=-1, yfact=-1, origin=(0,0)),
              _cspace_obstacle, and unary_union.
        """
        # TODO: implement this method
        raise NotImplementedError


# ---------------------------------------------------------------------------
# Convenience wrapper
# ---------------------------------------------------------------------------

def check_collision(
    environment: List[PlacedPrimitive], robot_body: List[PlacedPrimitive], robot_pose: FriendlyPose
) -> bool:
    """Single-query collision check. For repeated queries at the same environment/body, use CSpaceChecker."""
    return CSpaceChecker(environment, robot_body).check(robot_pose)
