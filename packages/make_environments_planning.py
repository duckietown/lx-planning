from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
from numpy.random import RandomState
from se2_utils import translation_angle_from_SE2, pose_from_friendly

from make_environments import get_ground_truth2, COLOR_BG
from collision_protocol import (
    Appearance,
    Circle,
    CollisionCheckQuery,
    MapDefinition,
    PlacedPrimitive,
    PlanningSetup,
    Rectangle,
    FriendlyPose,
)

__all__ = [
    "PlanningParams",
    "SamplePlanningParams",
    "FixedPlanningParams",
    "EnvInfo",
    "EnvSampler",
    "GeoSampler",
    "PoseSampler",
    "RandomPoseSampler",
    "FixedBody",
    "RandomEnv",
    "SamplingEnvInfo",
    "sample_pose",
    "get_room_sides",
]


# ── Planning parameters (kinematic constraints + tolerances) ──────────────────

@dataclass
class PlanningParams:
    tolerance_xy_m: float
    tolerance_theta_deg: float
    max_curvature: float
    max_linear_velocity_m_s: float
    min_linear_velocity_m_s: float
    max_angular_velocity_deg_s: float


class SamplePlanningParams(ABC):
    @abstractmethod
    def sample(self, rs: RandomState) -> PlanningParams:
        pass


class FixedPlanningParams(SamplePlanningParams):
    def __init__(self, dp: PlanningParams):
        self.dp = dp

    def sample(self, rs: RandomState) -> PlanningParams:
        return self.dp


# ── Environment factories ─────────────────────────────────────────────────────

@dataclass
class SamplingEnvInfo:
    n_circles: int
    n_rectangles: int
    min_distance: float
    interval_radius: Tuple[float, float]
    interval_sides: Tuple[float, float]


@dataclass
class EnvInfo:
    bounds: Rectangle
    obstacles: List[PlacedPrimitive]


def sample_pose(area: Rectangle, rs: RandomState) -> FriendlyPose:
    x = rs.uniform(area.xmin, area.xmax)
    y = rs.uniform(area.ymin, area.ymax)
    theta_deg = float(rs.uniform(0, 360))
    return FriendlyPose(float(x), float(y), theta_deg)


def get_room_sides(area: Rectangle) -> List[PlacedPrimitive]:
    """Return four thin wall rectangles that border the given area."""
    zero = FriendlyPose(0.0, 0.0, 0.0)
    brown = Appearance("brown")
    M = 0.1
    return [
        PlacedPrimitive(zero, Rectangle(xmin=area.xmin, xmax=area.xmin + M,
                                        ymin=area.ymin, ymax=area.ymax), appearance=brown),
        PlacedPrimitive(zero, Rectangle(xmin=area.xmax - M, xmax=area.xmax,
                                        ymin=area.ymin, ymax=area.ymax), appearance=brown),
        PlacedPrimitive(zero, Rectangle(xmin=area.xmin, xmax=area.xmax,
                                        ymin=area.ymax - M, ymax=area.ymax), appearance=brown),
        PlacedPrimitive(zero, Rectangle(xmin=area.xmin, xmax=area.xmax,
                                        ymin=area.ymin, ymax=area.ymin + M), appearance=brown),
    ]


def _sample_environment(area: Rectangle, rs: RandomState, sei: SamplingEnvInfo) -> List[PlacedPrimitive]:
    e = get_room_sides(area)
    zero = FriendlyPose(0.0, 0.0, 0.0)
    brown = Appearance("brown")

    def is_acceptable(pr: PlacedPrimitive) -> bool:
        md = MapDefinition(e, [pr])
        result = get_ground_truth2(md, CollisionCheckQuery(zero))
        d = 0.0 if result.collision else result.distance
        return d >= sei.min_distance

    for _ in range(sei.n_circles):
        for _attempt in range(200):
            pose = sample_pose(area, rs)
            radius = rs.uniform(sei.interval_radius[0], sei.interval_radius[1])
            pp = PlacedPrimitive(pose, Circle(radius), appearance=brown)
            if is_acceptable(pp):
                e.append(pp)
                break

    for _ in range(sei.n_rectangles):
        for _attempt in range(200):
            pose = sample_pose(area, rs)
            Q = rs.uniform(sei.interval_sides[0], sei.interval_sides[1])
            M = rs.uniform(sei.interval_sides[0], sei.interval_sides[1])
            pp = PlacedPrimitive(pose, Rectangle(xmin=-M, xmax=+M, ymin=-Q, ymax=+Q), appearance=brown)
            if is_acceptable(pp):
                e.append(pp)
                break

    return e


class GeoSampler(ABC):
    @abstractmethod
    def sample(self, rs: RandomState) -> List[PlacedPrimitive]:
        pass


class EnvSampler(ABC):
    @abstractmethod
    def sample(self, rs: RandomState) -> EnvInfo:
        pass


class PoseSampler(ABC):
    @abstractmethod
    def sample_pair(self, ps: PlanningSetup, rs: RandomState) -> Tuple[FriendlyPose, FriendlyPose]:
        pass


class FixedBody(GeoSampler):
    """A small rectangular robot body (Duckiebot-like proportions)."""

    def sample(self, rs: RandomState) -> List[PlacedPrimitive]:
        pose = FriendlyPose(0.0, 0.0, 0.0)
        body_app = Appearance(fillcolor="blue", rel_zorder=+1)
        wheel_app = Appearance(fillcolor="black", rel_zorder=-1)
        chassis = Rectangle(xmin=-0.13, xmax=0.07, ymin=-0.045, ymax=0.045)
        wheels = Rectangle(xmin=-0.03, xmax=0.03, ymin=-0.065, ymax=0.065)
        return [
            PlacedPrimitive(pose, chassis, appearance=body_app),
            PlacedPrimitive(pose, wheels, appearance=wheel_app),
        ]


class RandomEnv(EnvSampler):
    def __init__(self, bounds: Rectangle, sei: SamplingEnvInfo):
        self.bounds = bounds
        self.sei = sei

    def sample(self, rs: RandomState) -> EnvInfo:
        obstacles = _sample_environment(self.bounds, rs, self.sei)
        return EnvInfo(self.bounds, obstacles)


class RandomPoseSampler(PoseSampler):
    def __init__(self, min_distance: float, min_pose_distance: float):
        self.min_distance = min_distance
        self.min_pose_distance = min_pose_distance

    def sample_pair(self, ps: PlanningSetup, rs: RandomState) -> Tuple[FriendlyPose, FriendlyPose]:
        while True:
            a = _sample_feasible_pose(ps, rs, min_distance=0.2)
            b = _sample_feasible_pose(ps, rs, min_distance=0.2)
            q = _pose_diff_dist(a, b)
            if q > self.min_pose_distance:
                return a, b


def _sample_feasible_pose(ps: PlanningSetup, rs: RandomState, min_distance: float) -> FriendlyPose:
    while True:
        fp = sample_pose(ps.bounds, rs=rs)
        gt = get_ground_truth2(ps, CollisionCheckQuery(fp))
        if not gt.collision and gt.distance >= min_distance:
            return fp


def _pose_diff_dist(a: FriendlyPose, b: FriendlyPose) -> float:
    q1 = pose_from_friendly(a)
    q2 = pose_from_friendly(b)
    from utils import pose_diff
    qr = pose_diff(q1, q2)
    t, _ = translation_angle_from_SE2(qr)
    return float(np.linalg.norm(t))
