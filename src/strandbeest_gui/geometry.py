from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable

Point = tuple[float, float]

EPSILON = 1e-9
LENGTH_NAMES = ("a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m")


class GeometryError(ValueError):
    """Raised when the current linkage dimensions do not admit a valid pose."""


@dataclass(frozen=True)
class LinkageParams:
    a: float = 38.0
    b: float = 41.5
    c: float = 39.3
    d: float = 40.1
    e: float = 55.8
    f: float = 39.4
    g: float = 36.7
    h: float = 65.7
    i: float = 49.0
    j: float = 50.0
    k: float = 61.9
    l: float = 7.8
    m: float = 15.0

    def as_key(self) -> tuple[float, ...]:
        return tuple(round(getattr(self, name), 4) for name in LENGTH_NAMES)


@dataclass(frozen=True)
class Pose:
    ground: Point
    crank_center: Point
    crank_end: Point
    upper_joint: Point
    lower_joint: Point
    left_joint: Point
    knee_joint: Point
    foot: Point


def distance(p0: Point, p1: Point) -> float:
    return math.hypot(p1[0] - p0[0], p1[1] - p0[1])


def circle_intersections(center0: Point, radius0: float, center1: Point, radius1: float) -> tuple[Point, Point]:
    if radius0 <= 0 or radius1 <= 0:
        raise GeometryError("All link lengths must stay positive.")

    x0, y0 = center0
    x1, y1 = center1
    dx = x1 - x0
    dy = y1 - y0
    center_distance = math.hypot(dx, dy)

    if center_distance < EPSILON:
        raise GeometryError("Circle centers coincide; pose is undefined.")
    if center_distance > radius0 + radius1 + EPSILON:
        raise GeometryError("Circles are too far apart to intersect.")
    if center_distance < abs(radius0 - radius1) - EPSILON:
        raise GeometryError("One circle is contained inside the other.")

    projection = (radius0 * radius0 - radius1 * radius1 + center_distance * center_distance) / (2.0 * center_distance)
    height_sq = radius0 * radius0 - projection * projection
    if height_sq < -EPSILON:
        raise GeometryError("No real circle intersection exists for these lengths.")
    height = math.sqrt(max(height_sq, 0.0))

    mid_x = x0 + projection * dx / center_distance
    mid_y = y0 + projection * dy / center_distance
    offset_x = -dy * height / center_distance
    offset_y = dx * height / center_distance

    return (
        (mid_x + offset_x, mid_y + offset_y),
        (mid_x - offset_x, mid_y - offset_y),
    )


def _pick(
    candidates: tuple[Point, Point],
    previous: Point | None,
    fallback: Callable[[Point], float],
) -> Point:
    if previous is not None:
        return min(candidates, key=lambda point: distance(point, previous))
    return min(candidates, key=fallback)


def solve_pose(params: LinkageParams, angle_deg: float, previous: Pose | None = None) -> Pose:
    ground = (0.0, 0.0)
    crank_center = (params.a, params.l)
    angle_rad = math.radians(angle_deg)
    crank_end = (
        crank_center[0] + params.m * math.cos(angle_rad),
        crank_center[1] + params.m * math.sin(angle_rad),
    )

    upper_joint = _pick(
        circle_intersections(ground, params.b, crank_end, params.j),
        previous.upper_joint if previous else None,
        lambda point: -point[1],
    )
    lower_joint = _pick(
        circle_intersections(ground, params.c, crank_end, params.k),
        previous.lower_joint if previous else None,
        lambda point: point[1],
    )
    left_joint = _pick(
        circle_intersections(ground, params.d, upper_joint, params.e),
        previous.left_joint if previous else None,
        lambda point: point[0],
    )
    knee_joint = _pick(
        circle_intersections(left_joint, params.f, lower_joint, params.g),
        previous.knee_joint if previous else None,
        lambda point: point[0],
    )
    foot = _pick(
        circle_intersections(knee_joint, params.h, lower_joint, params.i),
        previous.foot if previous else None,
        lambda point: point[1],
    )

    return Pose(
        ground=ground,
        crank_center=crank_center,
        crank_end=crank_end,
        upper_joint=upper_joint,
        lower_joint=lower_joint,
        left_joint=left_joint,
        knee_joint=knee_joint,
        foot=foot,
    )


def foot_path(params: LinkageParams, samples: int = 180) -> list[Point]:
    if samples < 4:
        raise ValueError("Need at least 4 samples to build a path.")

    path: list[Point] = []
    previous: Pose | None = None
    for index in range(samples):
        angle = 360.0 * index / samples
        previous = solve_pose(params, angle, previous)
        path.append(previous.foot)
    return path
