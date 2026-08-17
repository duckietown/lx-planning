#!/usr/bin/env python3

# Set headless matplotlib backend before any import that pulls in pyplot
import matplotlib
matplotlib.use('Agg')

import asyncio
from io import BytesIO
import json
import math
import os
import re
import subprocess
import time
from typing import Any, Dict, List, Optional, Tuple
from urllib.request import urlopen

from matplotlib import pyplot as plt

from dtps import context, ContextConfig, DTPSContext
from dtps_http import RawData
from duckietown_messages.sensors.compressed_image import CompressedImage

from collision_protocol import (
    Appearance,
    Circle,
    FriendlyPose,
    PlacedPrimitive,
    PlanningQuery,
    PlanningResult,
    PlanningSetup,
    Rectangle,
)
from make_environments import COLOR_BG
from make_environments_planning import FixedBody
from planner import plan
from utils import normalize_angle, simulate

import logging


# aiopubsub logs a cosmetic "Unhandled CancelledError" when its subscriber loops
# are cancelled during asyncio shutdown: dtps cancels the loop tasks without
# calling aiopubsub's Loop.stop() (which would clear the loop's running flag), so
# the CancelledError is treated as "unhandled". It is harmless and only fires
# after the plan has finished executing. Drop just that one record (the aiopubsub
# Loop logger is named "Loop") while leaving all other logging intact.
class _SuppressShutdownCancelledError(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        return "Unhandled CancelledError" not in record.getMessage()


logging.getLogger("Loop").addFilter(_SuppressShutdownCancelledError())


# When publishing to the engine's topics, the dtps client evaluates every URL the
# engine advertises for itself, including its local unix socket
# (http+unix:///tmp/dtps-self-*). That socket only exists in the engine's
# filesystem, so from inside the robot container it logs
# "host='/tmp/dtps-self-...' does not exist" on every publish before falling back
# to the reachable http URL. The publishes still succeed via http; this is purely
# cosmetic noise.
#
# The warning is emitted on the "dtps_http" logger (DTPSClient logs via
# logger.getChild(<id>)). Raising that logger's level is the most reliable way to
# drop the warning: it is suppressed at the source, before any handler, so it is
# immune to however logging handlers happen to be wired at runtime. (Trade-off:
# other dtps_http warnings are suppressed too; connectivity is already handled,
# so a failed connection surfaces as the agent not finding/executing a plan.)
logging.getLogger("dtps_http").setLevel(logging.ERROR)


# Belt-and-suspenders: if a different dtps build emits this exact line on another
# logger, drop just that message wherever it would be printed.
class _SuppressDtpsUnixSocketNoise(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        msg = record.getMessage()
        return not ("http+unix" in msg and "does not exist" in msg)


_dtps_noise_filter = _SuppressDtpsUnixSocketNoise()
for _handler in list(logging.getLogger().handlers) + (
    [logging.lastResort] if logging.lastResort is not None else []
):
    _handler.addFilter(_dtps_noise_filter)


_GRID: int = 8
_SIGN_RADIUS: float = 0.20
# The duckiematrix sign_stop mesh renders half a tile up-right (+x,+y) of its
# frame. Signs are placed on a tile's origin corner (integer tile coords), so
# the visible/collision center is (frame + this) tiles = the tile center.
_SIGN_RENDER_OFFSET: float = 0.5
_MAX_LINEAR_V: float = 0.30
_MIN_LINEAR_V: float = 0.05
_MAX_ANGULAR_V: float = 60.0
_MAX_CURVATURE: float = 8.0
_TOLERANCE_XY: float = 0.10
_TOLERANCE_THETA: float = 30.0

# The duckiematrix plant is the DB18 *dynamic* model (get_DB18_uncalibrated),
# NOT the kinematics_node's kinematic model. Its steady-state body velocity for
# duty cycles (mr, ml) is:
#     v     = (u_alpha / u1) * (mr + ml)      # u1=5,  u_alpha=1.5  -> v     = 0.3  * (mr+ml)
#     omega = (w_alpha / w1) * (mr - ml)      # w1=4,  w_alpha=15   -> omega = 3.75 * (mr-ml)
# We invert THAT so a commanded (v, omega) is actually held. (Inverting the
# kinematic InverseKinematics+PWM model instead makes the robot hold only ~70%
# of the commanded speed and ~44% of the commanded turn rate, which walks the
# execution off the planned path.)
_DB18_U1: float = 5.0
_DB18_U_ALPHA: float = 1.5
_DB18_W1: float = 4.0
_DB18_W_ALPHA: float = 15.0

# ── Closed-loop trajectory-following gains ─────────────────────────────────────
# Each plan primitive is servoed to its planned end pose using live pose
# feedback, so the DB18 plant's first-order lag / carry-over (which open-loop
# replay leaves as gradual over-rotation) is corrected step by step.
_CTRL_RATE_HZ: float = 20.0
_CTRL_KP_OMEGA: float = 3.0            # rad/s commanded per rad of heading error
_CTRL_KP_V: float = 1.5               # m/s commanded per m of remaining distance
_CTRL_MIN_V: float = 0.04             # m/s creep so a straight step never stalls
_CTRL_POS_TOL_M: float = 0.03         # straight step reached within this along-track
_CTRL_YAW_TOL_DEG: float = 3.0        # turn step reached within this heading error
_CTRL_GOAL_YAW_TOL_DEG: float = 8.0   # final heading-alignment tolerance
_CTRL_STEP_TIMEOUT_S: float = 6.0     # give up on a single primitive after this
_VIZ_PROGRESS_PERIOD_S: float = 0.4   # re-publish the progress image at most this often


def _resolve_duckiematrix_host() -> str:
    """Find the duckiematrix engine host.

    Resolution order:
    1. DUCKIEMATRIX_HOST env var (set by `dts code workbench -m`)
    2. HIL connection config in the virtual robot's KVStore at 127.0.0.1:11411
    3. Default gateway from `ip route` (physical host's bridge IP)
    4. Fallback: 127.0.0.1
    """
    # 1. Explicit env var
    host = os.environ.get("DUCKIEMATRIX_HOST")
    print(f"[host] DUCKIEMATRIX_HOST env: {host!r}")
    if host:
        return host

    # 2. Read from virtual robot KVStore (always accessible at 127.0.0.1:11411 from inside)
    try:
        resp = urlopen("http://127.0.0.1:11411/data/hil/connection/", timeout=3)
        data = json.loads(resp.read())
        urls = data.get("simulator", {}).get("urls", [])
        print(f"[host] KVStore hil/connection urls: {urls}")
        if urls:
            m = re.match(r"https?://([^:/]+)", urls[0])
            if m:
                return m.group(1)
    except Exception as e:
        print(f"[host] KVStore lookup failed: {e}")

    # 3. Default gateway (physical host's bridge IP inside virtual robot networking)
    try:
        out = subprocess.check_output(["ip", "route", "show", "default"], text=True, timeout=2)
        print(f"[host] ip route: {out!r}")
        parts = out.split()
        if len(parts) >= 3 and parts[1] == "via":
            return parts[2]
    except Exception as e:
        print(f"[host] ip route failed: {e}")

    print("[host] Falling back to 127.0.0.1")
    return "127.0.0.1"


_DUCKIEMATRIX_HOST: str = _resolve_duckiematrix_host()
print(f"[host] Resolved duckiematrix host: {_DUCKIEMATRIX_HOST}")
_ROBOT_KEY: str = "map_0/vehicle_0"


async def _get_once(ctx: DTPSContext, timeout: float = 30.0) -> Any:
    """Subscribe to a DTPS topic and return the first message decoded as a native object."""
    event = asyncio.Event()
    result: list = []

    async def on_data(raw: RawData) -> None:
        if not event.is_set():
            result.append(raw.get_as_native_object())
            event.set()

    sub = await ctx.configure(ContextConfig(patient=True)).subscribe(on_data)
    try:
        await asyncio.wait_for(event.wait(), timeout=timeout)
    finally:
        await sub.unsubscribe()
    return result[0]


def _quaternion_to_yaw_deg(w: float, x: float, y: float, z: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny, cosy))


def _pose_from_native(pose_native: Dict) -> FriendlyPose:
    """Convert a duckiematrix state/pose message into a FriendlyPose (m, deg)."""
    return FriendlyPose(
        x=pose_native["position"]["x"],
        y=pose_native["position"]["y"],
        theta_deg=_quaternion_to_yaw_deg(
            pose_native["rotation"]["w"],
            pose_native["rotation"]["x"],
            pose_native["rotation"]["y"],
            pose_native["rotation"]["z"],
        ),
    )


def _build_planning_setup(
    frames_native: Dict[str, Any],
    tile_maps_native: Dict[str, Any],
) -> Tuple[PlanningSetup, FriendlyPose]:
    tile_size: float = tile_maps_native["data"]["map_0"]["tile_size"]["x"]
    frames_data: Dict = frames_native["data"]
    zero = FriendlyPose(0.0, 0.0, 0.0)
    obstacles: List[PlacedPrimitive] = []

    for i in range(1, _GRID - 1):
        for j in range(1, _GRID - 1):
            frame = frames_data.get(f"map_0/tile_{i}_{j}")
            if frame and frame["pose"]["x"] < 0:
                # duckiematrix corner-anchors tiles: the tile frame (i, j) is the
                # tile's origin corner, so tile (i, j) spans [i*ts, (i+1)*ts] and
                # its center (where the robot and signs sit) is (i+0.5, j+0.5)*ts.
                obstacles.append(PlacedPrimitive(
                    zero,
                    Rectangle(
                        xmin=i * tile_size,
                        xmax=(i + 1) * tile_size,
                        ymin=j * tile_size,
                        ymax=(j + 1) * tile_size,
                    ),
                    appearance=Appearance(fillcolor="#555555"),
                ))

    for key, data in frames_data.items():
        if "sign_stop_" in key:
            pose = data["pose"]
            # Placed signs sit on a tile origin corner (tile-unit coords in
            # 0.._GRID-1); unplaced signs are parked at a negative sentinel.
            # The mesh renders +0.5 tile up-right, so the sign's true center is
            # (pose + _SIGN_RENDER_OFFSET) tiles.
            if pose["x"] > -1.0:
                obstacles.append(PlacedPrimitive(
                    FriendlyPose(
                        (pose["x"] + _SIGN_RENDER_OFFSET) * tile_size,
                        (pose["y"] + _SIGN_RENDER_OFFSET) * tile_size,
                        0.0,
                    ),
                    Circle(radius=_SIGN_RADIUS),
                    appearance=Appearance(fillcolor="red"),
                ))

    # duckiematrix corner-anchors tiles, so the tiled world spans [0, _GRID*ts].
    edge_min = 0.0
    edge_max = _GRID * tile_size

    # Ring of walls just outside the driveable area. Inflated by the body in
    # C-space, they keep the robot's whole footprint on the tiles, so the
    # planner can never route a path off the edge of the map.
    wall = 0.5 * tile_size
    for wxmin, wxmax, wymin, wymax in (
        (edge_min - wall, edge_max + wall, edge_min - wall, edge_min),  # bottom
        (edge_min - wall, edge_max + wall, edge_max, edge_max + wall),  # top
        (edge_min - wall, edge_min, edge_min - wall, edge_max + wall),  # left
        (edge_max, edge_max + wall, edge_min - wall, edge_max + wall),  # right
    ):
        obstacles.append(PlacedPrimitive(
            zero,
            Rectangle(xmin=wxmin, xmax=wxmax, ymin=wymin, ymax=wymax),
            appearance=Appearance(fillcolor="#333333"),
        ))

    bounds = Rectangle(xmin=edge_min, xmax=edge_max, ymin=edge_min, ymax=edge_max)
    ps = PlanningSetup(
        environment=obstacles,
        body=FixedBody().sample(rs=None),
        bounds=bounds,
        max_linear_velocity_m_s=_MAX_LINEAR_V,
        min_linear_velocity_m_s=_MIN_LINEAR_V,
        max_angular_velocity_deg_s=_MAX_ANGULAR_V,
        max_curvature=_MAX_CURVATURE,
        tolerance_xy_m=_TOLERANCE_XY,
        tolerance_theta_deg=_TOLERANCE_THETA,
    )
    # Goal at the center of the top-right tile (_GRID-1, _GRID-1), which in
    # tile units is (_GRID-0.5, _GRID-0.5).
    goal = FriendlyPose((_GRID - 0.5) * tile_size, (_GRID - 0.5) * tile_size, 45.0)
    return ps, goal


def _velocity_to_pwm(v_ms: float, omega_rad_s: float) -> Tuple[float, float]:
    """Chassis (v, omega) -> (left, right) motor duty cycles.

    Inverts the steady state of the duckiematrix DB18 dynamic plant so that,
    once the first-order transient settles, the robot holds the commanded body
    velocity. Each duty is clipped to the +/-1.0 motor limit.
    """
    lin = 0.5 * (_DB18_U1 / _DB18_U_ALPHA) * v_ms          # 0.5 * (mr + ml)
    ang = 0.5 * (_DB18_W1 / _DB18_W_ALPHA) * omega_rad_s    # 0.5 * (mr - ml)
    pwm_right = max(-1.0, min(1.0, lin + ang))
    pwm_left = max(-1.0, min(1.0, lin - ang))
    return pwm_left, pwm_right


def _render_plan_image(
    ps: PlanningSetup,
    pq: PlanningQuery,
    result: PlanningResult,
    robot_pose: Optional[FriendlyPose] = None,
    trail: Optional[List[Tuple[float, float]]] = None,
) -> bytes:
    """Render the environment and planned path to a JPEG image.

    When ``robot_pose`` / ``trail`` are supplied, the robot's live position
    (orange, with a heading tick) and the path it has actually driven so far
    (orange line) are overlaid on the plan, giving a progress view.
    """
    fig, ax = plt.subplots(figsize=(6, 6), dpi=100)
    fig.patch.set_facecolor(COLOR_BG)
    ax.set_aspect('equal')
    # View covers the bounds plus any obstacles that extend past them (the
    # boundary wall ring), so the whole confined area is visible.
    xlo, xhi = ps.bounds.xmin, ps.bounds.xmax
    ylo, yhi = ps.bounds.ymin, ps.bounds.ymax
    for pp in ps.environment:
        prim = pp.primitive
        if isinstance(prim, Rectangle):
            xlo = min(xlo, prim.xmin + pp.pose.x)
            xhi = max(xhi, prim.xmax + pp.pose.x)
            ylo = min(ylo, prim.ymin + pp.pose.y)
            yhi = max(yhi, prim.ymax + pp.pose.y)
    ax.set_xlim(xlo, xhi)
    ax.set_ylim(ylo, yhi)
    ax.set_facecolor(COLOR_BG)
    ax.axis('off')

    # Draw obstacles so the map is verifiable: tile gaps (Rectangles, gray) and
    # stop signs (Circles, red — drawn at the inflated collision radius). Use
    # add_patch (not add_artist) so the patches live in data coordinates and
    # render under the Agg/savefig backend.
    for pp in ps.environment:
        color = pp.appearance.fillcolor if pp.appearance is not None else "#555555"
        prim = pp.primitive
        if isinstance(prim, Rectangle):
            ax.add_patch(plt.Rectangle(
                (prim.xmin + pp.pose.x, prim.ymin + pp.pose.y),
                prim.xmax - prim.xmin,
                prim.ymax - prim.ymin,
                facecolor=color, edgecolor="black", linewidth=0.5, alpha=0.75, zorder=1,
            ))
        elif isinstance(prim, Circle):
            ax.add_patch(plt.Circle(
                (pp.pose.x, pp.pose.y), prim.radius,
                facecolor=color, edgecolor="black", linewidth=0.5, alpha=0.85, zorder=2,
            ))

    # Integrate plan steps to get path waypoints for drawing
    if result.feasible and result.plan:
        x, y = pq.start.x, pq.start.y
        theta = math.radians(pq.start.theta_deg)
        xs, ys = [x], [y]
        for step in result.plan:
            omega = math.radians(step.angular_velocity_deg_s)
            n = max(1, int(step.duration / 0.05))
            dt = step.duration / n
            for _ in range(n):
                x += step.velocity_x_m_s * dt * math.cos(theta)
                y += step.velocity_x_m_s * dt * math.sin(theta)
                theta += omega * dt
            xs.append(x)
            ys.append(y)
        ax.plot(xs, ys, '-', color='steelblue', linewidth=2, zorder=5)

    # Start (green circle) and goal (red star)
    ax.plot(pq.start.x, pq.start.y, 'o', color='green', markersize=12, zorder=10)
    ax.plot(pq.target.x, pq.target.y, '*', color='red', markersize=16, zorder=10)

    # Progress overlay: path actually driven (orange) and live robot pose.
    if trail and len(trail) >= 2:
        tx = [p[0] for p in trail]
        ty = [p[1] for p in trail]
        ax.plot(tx, ty, '-', color='darkorange', linewidth=2, zorder=7)
    if robot_pose is not None:
        rx, ry = robot_pose.x, robot_pose.y
        th = math.radians(robot_pose.theta_deg)
        tick = 0.18
        ax.plot([rx, rx + tick * math.cos(th)], [ry, ry + tick * math.sin(th)],
                '-', color='black', linewidth=2, zorder=11)
        ax.plot(rx, ry, 'o', color='darkorange', markersize=11,
                markeredgecolor='black', zorder=12)

    buf = BytesIO()
    fig.savefig(buf, format='jpeg', bbox_inches='tight', facecolor=COLOR_BG)
    plt.close(fig)
    buf.seek(0)
    return buf.read()


class PlanningAgent:

    def __init__(self):
        self.is_shutdown = False

    def spin(self):
        try:
            asyncio.run(self.run())
        except RuntimeError:
            if not self.is_shutdown:
                raise

    async def _setup_viz_publisher(self) -> Optional[Any]:
        """Create a local DTPS queue and expose it on the robot's switchboard."""
        robot_name = os.environ.get("VEHICLE_NAME", "robot")
        self_ctx = await context("self", urls=["create:http://0.0.0.0:0/"])
        viz_queue = await (self_ctx / "planning" / "jpeg").queue_create()
        viz_pub = await viz_queue.publisher()
        switchboard_ctx = await context("switchboard", urls=["http://127.0.0.1:11911/"])
        switchboard_ctx = switchboard_ctx.navigate(robot_name)
        await (switchboard_ctx / "planning" / "jpeg").expose(viz_queue)
        print(f"Visualization topic exposed at switchboard/{robot_name}/planning/jpeg")
        return viz_pub

    async def run(self):
        # Try to set up visualization publishing before anything else
        viz_pub = None
        try:
            viz_pub = await self._setup_viz_publisher()
        except Exception as e:
            print(f"Note: visualization publishing unavailable: {e}")

        print(f"Connecting to duckiematrix at {_DUCKIEMATRIX_HOST}:7501 ...")
        ctx = await context("duckiematrix", urls=[f"http://{_DUCKIEMATRIX_HOST}:7501/"])
        print("Connected.")

        print("Loading map info...")
        frames_native, tile_maps_native = await asyncio.gather(
            _get_once(ctx / "map" / "frames"),
            _get_once(ctx / "map" / "tile_maps"),
        )
        print("Map loaded.")

        print("Getting initial robot pose...")
        pose_native = await _get_once(ctx / "robot" / _ROBOT_KEY / "state" / "pose")
        print("Pose received.")

        start = _pose_from_native(pose_native)

        print("Calculating plan...")
        ps, goal = _build_planning_setup(frames_native, tile_maps_native)

        print(f"Start: ({start.x:.3f}, {start.y:.3f}, {start.theta_deg:.1f}°)  "
              f"Goal: ({goal.x:.3f}, {goal.y:.3f}, {goal.theta_deg:.1f}°)  "
              f"Obstacles: {len(ps.environment)}")

        query = PlanningQuery(start=start, target=goal)
        # Tougher maps need finer steps (smaller _STEP_DIST_M) and thus more
        # iterations; lower goal_bias favors exploration over greedy progress.
        result = plan(ps, query, max_iter=100000, goal_bias=0.05)

        if not result.feasible:
            print("Planning failed — no path found.")
            return

        print(f"Plan found: {len(result.plan)} steps.")

        # Publish visualization image
        if viz_pub is not None:
            try:
                print("Rendering plan visualization...")
                jpeg_bytes = _render_plan_image(ps, query, result)
                await viz_pub.publish(CompressedImage(format="jpeg", data=jpeg_bytes).to_rawdata())
                print("Visualization published.")
            except Exception as e:
                print(f"Warning: visualization publish failed: {e}")

        print("Executing plan (closed-loop)...")
        motors_ctx = ctx / "robot" / _ROBOT_KEY / "actuator" / "wheels" / "base" / "pwm_filtered"
        await self._execute_closed_loop(ctx, motors_ctx, ps, query, result, viz_pub)
        print("Execution complete.")

    async def _execute_closed_loop(self, ctx, motors_ctx, ps, query, result, viz_pub):
        """Servo the robot along the planned trajectory using pose feedback.

        Turn primitives rotate in place to the planned heading; straight
        primitives drive to the planned end point (steering toward it, which
        also cancels lateral drift). Because every target is an *absolute*
        planned pose, per-step errors from the DB18 plant's lag do not
        accumulate — each primitive re-references the plan.

        The plan image published to DTPS is re-rendered as the robot moves,
        overlaying the live pose and the path actually driven.
        """
        start, goal, plan_steps = query.start, query.target, result.plan
        dt = 1.0 / _CTRL_RATE_HZ
        max_omega = math.radians(_MAX_ANGULAR_V)
        yaw_tol = math.radians(_CTRL_YAW_TOL_DEG)

        state = {"pose": start}

        async def on_pose(raw: RawData) -> None:
            try:
                state["pose"] = _pose_from_native(raw.get_as_native_object())
            except Exception:
                pass

        def clamp(x: float, lo: float, hi: float) -> float:
            return lo if x < lo else hi if x > hi else x

        # Live progress overlay: accumulate the driven path, re-publish throttled.
        trail: List[Tuple[float, float]] = [(start.x, start.y)]
        last_viz = [0.0]

        async def publish_progress(pose: FriendlyPose, force: bool = False) -> None:
            trail.append((pose.x, pose.y))
            if viz_pub is None:
                return
            now = time.monotonic()
            if not force and now - last_viz[0] < _VIZ_PROGRESS_PERIOD_S:
                return
            last_viz[0] = now
            try:
                jpeg = _render_plan_image(ps, query, result, robot_pose=pose, trail=trail)
                await viz_pub.publish(CompressedImage(format="jpeg", data=jpeg).to_rawdata())
            except Exception as e:
                print(f"Warning: progress viz publish failed: {e}")

        async def drive(v: float, omega: float) -> None:
            pwm_l, pwm_r = _velocity_to_pwm(v, omega)
            await motors_ctx.publish(
                RawData.cbor_from_native_object({"left": pwm_l, "right": pwm_r})
            )
            await asyncio.sleep(dt)

        pose_ctx = ctx / "robot" / _ROBOT_KEY / "state" / "pose"
        sub = await pose_ctx.configure(ContextConfig(patient=True)).subscribe(on_pose)
        target = start
        try:
            for step in plan_steps:
                if self.is_shutdown:
                    break
                is_turn = abs(step.velocity_x_m_s) < 1e-9
                target = simulate(target, [step]).poses[-1]
                deadline = time.monotonic() + _CTRL_STEP_TIMEOUT_S
                while not self.is_shutdown:
                    pose = state["pose"]
                    await publish_progress(pose)
                    cur_yaw = math.radians(pose.theta_deg)
                    if is_turn:
                        yaw_err = normalize_angle(math.radians(target.theta_deg) - cur_yaw)
                        if abs(yaw_err) <= yaw_tol:
                            break
                        await drive(0.0, clamp(_CTRL_KP_OMEGA * yaw_err, -max_omega, max_omega))
                    else:
                        dx, dy = target.x - pose.x, target.y - pose.y
                        hdg = math.radians(target.theta_deg)
                        remaining = dx * math.cos(hdg) + dy * math.sin(hdg)  # along-track
                        if remaining <= _CTRL_POS_TOL_M:
                            break
                        yaw_err = normalize_angle(math.atan2(dy, dx) - cur_yaw)
                        omega = clamp(_CTRL_KP_OMEGA * yaw_err, -max_omega, max_omega)
                        # Ease off throttle while the heading error is large so the
                        # robot turns onto the line before accelerating along it.
                        v = clamp(_CTRL_KP_V * remaining, _CTRL_MIN_V, _MAX_LINEAR_V)
                        v *= max(0.0, math.cos(yaw_err))
                        await drive(v, omega)
                    if time.monotonic() > deadline:
                        print("  step timed out; advancing to next primitive.")
                        break

            # Final in-place alignment to the goal heading.
            goal_yaw_tol = math.radians(_CTRL_GOAL_YAW_TOL_DEG)
            deadline = time.monotonic() + _CTRL_STEP_TIMEOUT_S
            while not self.is_shutdown and time.monotonic() < deadline:
                pose = state["pose"]
                await publish_progress(pose)
                yaw_err = normalize_angle(math.radians(goal.theta_deg) - math.radians(pose.theta_deg))
                if abs(yaw_err) <= goal_yaw_tol:
                    break
                await drive(0.0, clamp(_CTRL_KP_OMEGA * yaw_err, -max_omega, max_omega))

            final = state["pose"]
            err = math.hypot(final.x - goal.x, final.y - goal.y)
            print(f"  final pose ({final.x:.3f}, {final.y:.3f}, {final.theta_deg:.1f}°), "
                  f"goal error {err * 100:.1f} cm")
            await publish_progress(final, force=True)
        finally:
            await motors_ctx.publish(RawData.cbor_from_native_object({"left": 0.0, "right": 0.0}))
            try:
                await sub.unsubscribe()
            except Exception:
                pass

    def on_shutdown(self):
        self.is_shutdown = True


if __name__ == "__main__":
    agent_node = PlanningAgent()
    agent_node.spin()
