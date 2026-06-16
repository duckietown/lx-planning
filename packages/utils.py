from dataclasses import dataclass
from typing import List

import math
import numpy as np
from numpy.testing import assert_allclose

from collision_protocol import FriendlyPose, PlanStep
from se2_utils import SE2, SE2value, se2_from_linear_angular, friendly_from_pose, pose_from_friendly


__all__ = [
    "SimulationResult",
    "pose_diff",
    "simulate",
    "normalize_angle",
    "assert_fp_close",
    "more_granular",
]


def assert_fp_close(a: FriendlyPose, b: FriendlyPose, atol=1e-10):
    assert_allclose(a.x, b.x, atol=atol)
    assert_allclose(a.y, b.y, atol=atol)

    t1 = normalize_angle(np.deg2rad(a.theta_deg))
    t2 = normalize_angle(np.deg2rad(b.theta_deg))
    assert_allclose(t1, t2, atol=atol)


def normalize_angle(x: float) -> float:
    c = np.cos(x)
    s = np.sin(x)
    return np.arctan2(s, c)


@dataclass
class SimulationResult:
    poses: List[FriendlyPose]
    ts: List[float]


def simulate(start: FriendlyPose, steps: List[PlanStep]) -> SimulationResult:
    q = pose_from_friendly(start)
    res = [start]
    ts = [0.0]
    t = 0.0
    for step in steps:
        if step.duration < 0:
            print(f"Invalid duration: step={step}")
            duration = 0.0
        else:
            duration = step.duration
        v = step.velocity_x_m_s
        w = np.deg2rad(step.angular_velocity_deg_s)
        V = se2_from_linear_angular([v, 0.0], w)
        dq = SE2.group_from_algebra(V * duration)
        q = q @ dq
        res.append(friendly_from_pose(q))
        t += duration
        ts.append(t)
    return SimulationResult(res, ts)


def pose_diff(q1: SE2value, q2: SE2value):
    d = SE2.multiply(SE2.inverse(q1), q2)
    return d



@dataclass
class GranularPlanStep(PlanStep):
    subindex: int


def more_granular(plan: List[PlanStep], dt: float) -> List[GranularPlanStep]:
    assert dt > 0, dt
    res = []
    for p in plan:
        res.extend(more_granular_(p, dt))
    return res


def more_granular_(p: PlanStep, dt: float) -> List[GranularPlanStep]:
    dt = float(dt)
    # logger.info(p=p, dt=dt)
    n = int(np.ceil(p.duration * 1.0 / dt))
    ts = [i * dt for i in range(n + 3)]
    ts.append(p.duration)
    ts = [_ for _ in ts if _ <= p.duration]
    ts = sorted(set(ts))

    dts = [float(ts[i + 1] - ts[i]) for i in range(len(ts) - 1)]
    # d2 = sum(dts)
    # logger.info(dts=dts, d2=d2, d=p.duration)

    return [
        GranularPlanStep(
            duration=_,
            angular_velocity_deg_s=p.angular_velocity_deg_s,
            velocity_x_m_s=p.velocity_x_m_s,
            subindex=i,
        )
        for i, _ in enumerate(dts)
    ]
