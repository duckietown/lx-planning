from dataclasses import dataclass
from typing import List, Optional

import math
import numpy as np
from se2_utils import SE2value, translation_angle_from_SE2, friendly_from_pose, pose_from_friendly

from collision_protocol import PlanStep, FriendlyPose
from utils import normalize_angle, pose_diff

__all__ = [
    "connect_dd",
    "connect_fps",
    "ConnectParams",
]


@dataclass
class ConnectResult:
    min_time: float
    steps: List[PlanStep]


@dataclass
class ConnectParams:
    max_linear_velocity_m_s: float
    min_linear_velocity_m_s: float
    max_angular_velocity_deg_s: float
    max_curvature: float
    avoid_weird: bool




def connect_fps(q: List[FriendlyPose], cp: ConnectParams) -> List[PlanStep]:
    n = len(q)
    res = []
    for i in range(n - 1):
        q0 = pose_from_friendly(q[i])
        q1 = pose_from_friendly(q[i + 1])
        qd = pose_diff(q0, q1)
        p = connect_dd(qd, cp)
        if p is None:
            print("Cannot connect")
        res.extend(p.steps)
    return res


def connect_dd(
    q: SE2value,
    cp: ConnectParams,
):
    plans = []
    p1 = connect_dd_(q, cp, False)
    if p1:
        plans.append(p1)
    p2 = connect_dd_(q, cp, True)
    if p2:
        plans.append(p2)
    if not plans:
        return None

    def k(_: ConnectResult):
        return _.min_time

    best = sorted(plans, key=k)
    return best[0]


def connect_dd_(
    q: SE2value,
    cp: ConnectParams,
    backward: bool,
) -> Optional[ConnectResult]:
    forward = not backward

    p, goal_angle = translation_angle_from_SE2(q)
    if backward and cp.min_linear_velocity_m_s >= 0:
        return None
    if forward and cp.max_linear_velocity_m_s <= 0:
        return None

    if backward:
        if p[0] > 0:
            return None
    else:
        if p[0] < 0:
            return None

    if cp.avoid_weird:
        if p[0] >= 0:
            if p[1] >= 0:
                d = +1
            else:
                d = -1
        else:
            if p[1] >= 0:
                d = -1
            else:
                d = +1
        if d * goal_angle < 0:
            return None

    if cp.max_curvature < np.inf:
        min_circle = 1.0 / cp.max_curvature
        # logger.info(min_circle=min_circle)
        for yc in [-min_circle, +min_circle]:
            center = [0, yc]
            delta = p - center
            dist = np.hypot(delta[0], delta[1])
            if dist < min_circle:
                return None

    steps: List[PlanStep] = []
    # first rotate towards
    first_step_goal_orientation = np.arctan2(p[1], p[0])
    if backward:
        first_step_goal_orientation = first_step_goal_orientation + np.pi

    dtheta = normalize_angle(first_step_goal_orientation)

    # logger.info("where is p" ,theta=theta, p=p)
    duration = float(np.abs(np.rad2deg(dtheta)) / cp.max_angular_velocity_deg_s)
    step1 = PlanStep(
        duration,
        angular_velocity_deg_s=float(cp.max_angular_velocity_deg_s * np.sign(dtheta)),
        velocity_x_m_s=0.0,
    )
    if step1.duration > 0:
        steps.append(step1)
    #
    # assert_allclose(0.0,
    #                 normalize_angle(step1.duration * np.deg2rad(step1.angular_velocity_deg_s),
    #                 first_step_goal_orientation))
    #
    dist = np.hypot(p[0], p[1])
    if dist > 0:
        if forward:
            v = cp.max_linear_velocity_m_s
            duration2 = float(dist / math.fabs(cp.max_linear_velocity_m_s))
            step2 = PlanStep(duration2, angular_velocity_deg_s=0.0, velocity_x_m_s=v)

        else:
            v = cp.min_linear_velocity_m_s
            duration2 = float(dist / math.fabs(cp.min_linear_velocity_m_s))
            step2 = PlanStep(duration2, angular_velocity_deg_s=0.0, velocity_x_m_s=v)
        steps.append(step2)

    dtheta = normalize_angle(goal_angle - first_step_goal_orientation)
    duration = float(np.abs(np.rad2deg(dtheta)) / cp.max_angular_velocity_deg_s)
    step3 = PlanStep(
        duration,
        angular_velocity_deg_s=float(cp.max_angular_velocity_deg_s * np.sign(dtheta)),
        velocity_x_m_s=0.0,
    )

    if step3.duration > 0:
        steps.append(step3)

    for s in steps:
        total_rot = math.fabs(s.angular_velocity_deg_s) * s.duration
        if total_rot > 180:
            print("Invalid")

    w = sum(_.duration for _ in steps)

    if False:
        fps = simulate(FriendlyPose(0.0, 0.0, 0.0), steps).poses
        last = fps[-1]
        fp2 = friendly_from_pose(q)

        assert_fp_close(last, fp2)
    return ConnectResult(min_time=w, steps=steps)

    #
    # t, _a = g.translation_angle_from_SE2(d)
    # return np.linalg.norm(t)
