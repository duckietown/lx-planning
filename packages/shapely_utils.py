"""
Single source of truth for converting collision-protocol primitives into
Shapely geometry.

Both `collision_checker` (C-space collision queries) and `make_environments`
(ground-truth sampling) need the same Primitive -> Shapely mapping. Keeping it
here avoids two copies drifting apart.

Note: the collision-protocol dataclasses remain the public/serializable
interface; this module only handles the internal geometry representation.
"""
from shapely.affinity import rotate, translate
from shapely.geometry import box, Point as SPoint

from collision_protocol import Circle, FriendlyPose, PlacedPrimitive, Primitive, Rectangle

__all__ = [
    "primitive_to_shapely",
    "transform_shapely",
    "placed_primitive_to_shapely",
    "as_shapely_cached",
]


def primitive_to_shapely(primitive: Primitive):
    """Convert a Primitive to a Shapely geometry centred at the origin."""
    if isinstance(primitive, Circle):
        return SPoint(0.0, 0.0).buffer(primitive.radius)
    if isinstance(primitive, Rectangle):
        return box(minx=primitive.xmin, maxx=primitive.xmax, miny=primitive.ymin, maxy=primitive.ymax)
    raise TypeError(f"Unknown primitive type: {primitive!r}")


def transform_shapely(pose: FriendlyPose, shape):
    """Place an origin-centred Shapely shape into the world frame via a FriendlyPose.

    Applies the local rotation (theta_deg, in degrees) first, then the
    translation — the convention used throughout the collision/planning code.
    """
    shape = rotate(shape, pose.theta_deg, origin=(0, 0))
    return translate(shape, pose.x, pose.y)


def placed_primitive_to_shapely(pp: PlacedPrimitive):
    """Convert a PlacedPrimitive to its world-frame Shapely geometry."""
    return transform_shapely(pp.pose, primitive_to_shapely(pp.primitive))


def as_shapely_cached(pp: PlacedPrimitive):
    """World-frame Shapely geometry for a PlacedPrimitive, memoised on the instance.

    The result is cached in a ``shapely`` attribute on the PlacedPrimitive so
    repeated queries against the same obstacle do not rebuild the geometry.
    """
    if not hasattr(pp, "shapely"):
        setattr(pp, "shapely", placed_primitive_to_shapely(pp))
    return getattr(pp, "shapely")
