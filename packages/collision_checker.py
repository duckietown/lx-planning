from typing import Dict, List

from shapely.affinity import rotate, scale, translate
from shapely.geometry import Point as SPoint
from shapely import unary_union
from shapely.prepared import prep

from collision_protocol import Circle, FriendlyPose, PlacedPrimitive
from shapely_utils import placed_primitive_to_shapely


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _body_at_theta(body_part: PlacedPrimitive, theta_deg: float):
    """Robot body part shape in the robot frame when global orientation is theta_deg."""
    # TODO: implement this function
    #   1. shape = placed_primitive_to_shapely(body_part)  # applies the part's local pose
    #   2. return rotate(shape, theta_deg, origin=(0, 0))
    raise NotImplementedError


def _cspace_obstacle(obs: PlacedPrimitive, body_reflected):
    """
    C-space obstacle for one (obstacle, reflected-body-part) pair.

    For a circle obstacle the exact result is: dilate the reflected body by the
    circle's radius and translate to the circle's centre.

    For a polygon obstacle (and convex body) the exact result is: translate the
    obstacle polygon to every vertex of the reflected body and take the convex hull.
    """
    # TODO: implement this function
    #   - Circle obstacle: translate(body_reflected.buffer(obs.primitive.radius),
    #     obs.pose.x, obs.pose.y)
    #   - Polygon obstacle: obs_shape = placed_primitive_to_shapely(obs); translate
    #     obs_shape to each (dx, dy) in body_reflected.exterior.coords[:-1],
    #     unary_union them, then take .convex_hull
    raise NotImplementedError


# ---------------------------------------------------------------------------
# Main class (depends on all helpers above)
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
        # theta_deg -> (forbidden_geometry, prepared_geometry). The prepared
        # geometry carries a spatial index so the many point-in-region queries
        # during planning are fast; the raw geometry is kept for distance().
        self._cache: Dict[float, tuple] = {}

    def check(self, robot_pose: FriendlyPose) -> bool:
        """Return True if the robot at robot_pose collides with any obstacle."""
        _, prepared = self._slice(robot_pose.theta_deg)
        return prepared is not None and not prepared.disjoint(SPoint(robot_pose.x, robot_pose.y))

    def distance(self, robot_pose: FriendlyPose) -> float:
        """Minimum distance from robot_pose to the nearest C-space obstacle (0.0 if in collision)."""
        forbidden, _ = self._slice(robot_pose.theta_deg)
        if forbidden is None:
            return float("inf")
        return float(forbidden.distance(SPoint(robot_pose.x, robot_pose.y)))

    def _slice(self, theta_deg: float):
        if theta_deg not in self._cache:
            forbidden = self._compute_slice(theta_deg)
            prepared = prep(forbidden) if forbidden is not None else None
            self._cache[theta_deg] = (forbidden, prepared)
        return self._cache[theta_deg]

    def _compute_slice(self, theta_deg: float):
        """Build and cache the forbidden (x, y) region for a robot at orientation theta_deg."""
        # TODO: implement this method
        #   parts = []
        #   for body_part in self._robot_body:
        #       body_shape = _body_at_theta(body_part, theta_deg)
        #       body_reflected = scale(body_shape, xfact=-1, yfact=-1, origin=(0, 0))
        #       for obs in self._environment:
        #           parts.append(_cspace_obstacle(obs, body_reflected))
        #   return unary_union(parts) if parts else None
        raise NotImplementedError


# ---------------------------------------------------------------------------
# Convenience wrapper (preserves the original check_collision interface)
# ---------------------------------------------------------------------------

def check_collision(
    environment: List[PlacedPrimitive], robot_body: List[PlacedPrimitive], robot_pose: FriendlyPose
) -> bool:
    """Single-query collision check. For repeated queries at the same environment/body, use CSpaceChecker."""
    return CSpaceChecker(environment, robot_body).check(robot_pose)
