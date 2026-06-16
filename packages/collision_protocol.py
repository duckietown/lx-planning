from typing import List, Optional, Union

from dataclasses import dataclass

__all__ = [
    "FriendlyPose",
    "Circle",
    "Rectangle",
    "Primitive",
    "MapDefinition",
    "CollisionCheckQuery",
    "CollisionCheckResult",
    "PlacedPrimitive",
    "Point",
    "PlanningQuery",
    "PlanningResult",
    "PlanningSetup",
    "PlanStep",
    "Appearance",
]

@dataclass
class FriendlyPose:
    x: float
    y: float
    theta_deg: float


@dataclass
class Circle:
    radius: float


@dataclass
class Point:
    x: float
    y: float


@dataclass
class Rectangle:
    xmin: float
    ymin: float
    xmax: float
    ymax: float


Primitive = Union[Circle, Rectangle]


@dataclass
class Appearance:
    fillcolor: str
    rel_zorder: int = 0


@dataclass
class PlanStep:
    duration: float
    velocity_x_m_s: float
    angular_velocity_deg_s: float


@dataclass
class PlacedPrimitive:
    pose: FriendlyPose
    primitive: Primitive
    appearance: Optional[Appearance] = None


@dataclass
class MapDefinition:
    environment: List[PlacedPrimitive]
    body: List[PlacedPrimitive]


@dataclass
class CollisionCheckQuery:
    pose: FriendlyPose


@dataclass
class CollisionCheckResult:
    collision: bool


@dataclass
class PlanningSetup(MapDefinition):
    bounds: Rectangle
    max_linear_velocity_m_s: float
    min_linear_velocity_m_s: float
    max_angular_velocity_deg_s: float
    max_curvature: float
    tolerance_xy_m: float
    tolerance_theta_deg: float


@dataclass
class PlanningQuery:
    start: FriendlyPose
    target: FriendlyPose


@dataclass
class PlanningResult:
    feasible: bool
    plan: Optional[List[PlanStep]]

