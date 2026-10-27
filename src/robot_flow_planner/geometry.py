"""Small, exact CPU collision primitives for M0 testing.

This is NOT a complete robot collision validator: self-collision, joint limits,
continuous-time sweeps, latency and stopping-distance margins come later.
All coordinates use meters and right-handed XYZ frames.
"""

from dataclasses import dataclass
import numpy as np
from numpy.typing import ArrayLike


def _vec3(value: ArrayLike, name: str) -> np.ndarray:
    a = np.asarray(value, dtype=np.float64)
    if a.shape != (3,) or not np.all(np.isfinite(a)):
        raise ValueError(f"{name} must be a finite length-3 vector")
    return a


@dataclass(frozen=True)
class OrientedBox:
    """3D box specified by center, full dimensions and local-to-world rotation."""

    center: ArrayLike
    size: ArrayLike
    rotation: ArrayLike

    def __post_init__(self) -> None:
        center = _vec3(self.center, "center").copy()
        size = _vec3(self.size, "size").copy()
        rotation = np.array(self.rotation, dtype=np.float64, copy=True)
        if np.any(size <= 0):
            raise ValueError("box size components must be strictly positive")
        if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
            raise ValueError("rotation must be a finite 3x3 matrix")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
            raise ValueError("rotation must be orthonormal")
        if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6):
            raise ValueError("rotation must be a proper rotation (det=+1)")
        for array in (center, size, rotation):
            array.setflags(write=False)
        object.__setattr__(self, "center", center)
        object.__setattr__(self, "size", size)
        object.__setattr__(self, "rotation", rotation)
        self.rotation.setflags(write=False)


@dataclass(frozen=True)
class Capsule:
    """Line segment with a swept spherical radius (robot link approximation)."""

    start: ArrayLike
    end: ArrayLike
    radius: float

    def __post_init__(self) -> None:
        start = _vec3(self.start, "start").copy()
        end = _vec3(self.end, "end").copy()
        radius = float(self.radius)
        if not np.isfinite(radius) or radius < 0:
            raise ValueError("capsule radius must be finite and nonnegative")
        start.setflags(write=False)
        end.setflags(write=False)
        object.__setattr__(self, "start", start)
        object.__setattr__(self, "end", end)
        object.__setattr__(self, "radius", radius)


def _segment_aabb_distance(a: np.ndarray, b: np.ndarray, half: np.ndarray) -> float:
    """Exact minimum Euclidean distance from a segment to a centered AABB.

    Squared point-to-box distance is piecewise quadratic along the segment.
    Evaluate the boundary crossings and the minimum of each quadratic piece.
    This avoids missing thin obstacles with coarse waypoint samples.
    """
    delta = b - a
    breaks = [0.0, 1.0]
    for axis in range(3):
        if abs(delta[axis]) > 1e-14:
            for edge in (-half[axis], half[axis]):
                t = (edge - a[axis]) / delta[axis]
                if 0.0 < t < 1.0:
                    breaks.append(float(t))
    breaks = sorted(set(breaks))

    def squared_distance(t: float) -> float:
        excess = np.maximum(np.abs(a + t * delta) - half, 0.0)
        return float(excess @ excess)

    best = min(squared_distance(t) for t in breaks)
    for lo, hi in zip(breaks[:-1], breaks[1:]):
        mid = (lo + hi) * 0.5
        point = a + mid * delta
        active = np.abs(point) > half
        if not np.any(active):
            return 0.0
        offsets = a[active] - np.sign(point[active]) * half[active]
        slopes = delta[active]
        denom = float(slopes @ slopes)
        if denom > 1e-28:
            t_star = float(np.clip(-(offsets @ slopes) / denom, lo, hi))
            best = min(best, squared_distance(t_star))
    return float(np.sqrt(max(best, 0.0)))


def capsule_box_clearance(capsule: Capsule, box: OrientedBox) -> float:
    """Signed separation in meters: positive=gap, zero=touching, negative=penetrating.

    Uses exact minimum segment-to-oriented-box distance minus capsule radius.
    If the segment intersects a box, penetration depth is NOT fully resolved:
    the return value bottoms out at negative capsule.radius.
    """
    a = box.rotation.T @ (capsule.start - box.center)
    b = box.rotation.T @ (capsule.end - box.center)
    return _segment_aabb_distance(a, b, box.size / 2.0) - capsule.radius


def capsule_box_collision(capsule: Capsule, box: OrientedBox, margin: float = 0.0) -> bool:
    """Contact or insufficient clearance counts as collision."""
    margin = float(margin)
    if not np.isfinite(margin) or margin < 0:
        raise ValueError("margin must be finite and nonnegative")
    return capsule_box_clearance(capsule, box) <= margin
