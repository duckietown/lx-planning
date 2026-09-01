import random
from dataclasses import dataclass
from typing import Iterator, List, Optional, Tuple
import math
import numpy as np

from matplotlib import pyplot

from collision_protocol import (
    FriendlyPose,
    Circle,
    CollisionCheckQuery,
    CollisionCheckResult,
    MapDefinition,
    Rectangle,
    Appearance,
    PlacedPrimitive,
    Point,
    Primitive
)

from collision_drawing import plot_geometry

from shapely_utils import as_shapely_cached, transform_shapely

from se2_utils import SE2, SE2_apply_R2, pose_from_friendly

__all__ = ["sample_environment", "sample_pose", "sample_body", "get_ground_truth", "get_ground_truth2"]

COLOR_BG = "#ccc9c6"
COLOR_COLLISION = "red"
COLOR_COLLISION_WRONG = "pink"
COLOR_NOCOLLISION = "blue"
COLOR_NOCOLLISION_WRONG = "orange"


def is_inside(x: Tuple[float, float], p: Primitive):
    return {Rectangle: is_inside_rect, Circle: is_inside_circle}[type(p)](x, p)


def is_inside_rect(p: Tuple[float, float], r: Rectangle):
    return (r.xmin <= p[0] <= r.xmax) and (r.ymin <= p[1] <= r.ymax)


def is_inside_circle(p: Tuple[float, float], c: Circle):
    dist = np.hypot(p[0], p[1])
    return dist <= c.radius


def random_sequence_2d(n) -> Iterator[Tuple[float, float]]:
    for _ in range(n):
        yield random.uniform(0, 1), random.uniform(0, 1)


def enum_points(p: Primitive, s: Iterator[Tuple[float, float]]):
    return {Rectangle: enum_points_rectangle, Circle: enum_points_circle}[type(p)](p, s)


def enum_points_circle(c: Circle, s: Iterator[Tuple[float, float]]) -> Iterator[Tuple[float, float]]:
    for x, y in s:
        theta = x * np.pi * 2
        dist = y * c.radius
        yield math.cos(theta) * dist, math.sin(theta) * dist


def enum_points_rectangle(r: Rectangle, s: Iterator[Tuple[float, float]]) -> Iterator[Tuple[float, float]]:
    for x, y in s:
        px = r.xmin + x * (r.xmax - r.xmin)
        py = r.ymin + y * (r.ymax - r.ymin)
        yield px, py


def enum_points_rectangle_border(
    r: Rectangle, s: Iterator[Tuple[float, float]]
) -> Iterator[Tuple[float, float]]:
    for x, y in s:
        px = r.xmin + x * (r.xmax - r.xmin)
        py = r.ymin + y * (r.ymax - r.ymin)
        yield px, py


@dataclass
class MyCollisionCheckResult(CollisionCheckResult):
    proof: Optional[Point]
    distance: float


def get_ground_truth(md: MapDefinition, q: CollisionCheckQuery, n_samples: int) -> CollisionCheckResult:
    placed_p1: PlacedPrimitive
    for placed_p1 in md.body:
        for x in enum_points(placed_p1.primitive, random_sequence_2d(n_samples)):
            x_world = apply(q.pose, x)

            res = collides_environment(x_world, md.environment)
            if res.collision:
                return res

    return MyCollisionCheckResult(False, None, 0.0)


def get_ground_truth2(md: MapDefinition, q: CollisionCheckQuery) -> MyCollisionCheckResult:
    placed_p1: PlacedPrimitive
    min_distance = 1000
    for placed_p1 in md.body:
        body = as_shapely_cached(placed_p1)
        body = transform_shapely(q.pose, body)
        for pp in md.environment:
            s = as_shapely_cached(pp)

            if s.intersects(body):
                return MyCollisionCheckResult(True, None, distance=0.0)

            d = s.distance(body)
            min_distance = min(d, min_distance)

            # intersection = s.intersection(body)
            # if intersection.area > 0:
            #     # logger.info(f'collision {body} {s} {intersection}')
            #     return MyCollisionCheckResult(True, None)

    return MyCollisionCheckResult(False, None, distance=min_distance)


def collides_environment(x: Tuple[float, float], p: List[PlacedPrimitive]):
    for posed_shape in p:
        x_p2ref = apply_inv(posed_shape.pose, x)
        if is_inside(x_p2ref, posed_shape.primitive):
            return MyCollisionCheckResult(True, Point(x[0], x[1]), 0.0)
    return MyCollisionCheckResult(False, None, 0.0)


def apply(pose: FriendlyPose, x: Tuple[float, float]) -> Tuple[float, float]:
    q = pose_from_friendly(pose)
    x2 = SE2_apply_R2(q, np.array(x))
    return float(x2[0]), float(x2[1])


def apply_inv(pose: FriendlyPose, x: Tuple[float, float]) -> Tuple[float, float]:
    q = pose_from_friendly(pose)
    q = SE2.inverse(q)
    x2 = SE2_apply_R2(q, np.array(x))
    return float(x2[0]), float(x2[1])



def sample_environment(area) -> List[PlacedPrimitive]:
    e = []
    brown = Appearance("#6F4E37")
    n_circles = random.randint(5, 20)
    for _ in range(n_circles):
        pose = sample_pose(area)
        radius = random.uniform(0.05, 0.8)
        circle = Circle(radius)
        e.append(PlacedPrimitive(pose, circle, appearance=brown))
    n_rects = random.randint(5, 20)
    for _ in range(n_rects):
        pose = sample_pose(area)
        Q = random.uniform(0.15, 0.4)
        M = random.uniform(0.15, 0.4)
        r = Rectangle(xmin=-M, xmax=+M, ymin=-Q, ymax=+Q)
        e.append(PlacedPrimitive(pose, r, appearance=brown))

    return e

def sample_pose(area) -> FriendlyPose:
    x = random.uniform(area.xmin, area.xmax)
    y = random.uniform(area.ymin, area.ymax)
    theta_deg = float(random.uniform(0, 360))
    return FriendlyPose(float(x), float(y), theta_deg)

def sample_body() -> List[PlacedPrimitive]:
    pose = FriendlyPose(0.0, 0.0, 0.0)
    b = Rectangle(xmin=-0.1, xmax=0.15, ymin=-0.07, ymax=0.07)
    return [PlacedPrimitive(pose, b, appearance=Appearance("blue"))]



