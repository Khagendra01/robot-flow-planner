"""Conservative rigid-body collider envelopes for research diagnostics only.

A link-local AABB enclosing authored mesh vertices becomes an oriented world
box under a rigid link pose. This does not reproduce PhysX cooking, contact
offsets, self-collisions, or continuous-time clearance.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike

from .geometry import OrientedBox


def quaternion_wxyz_rotation(quaternion: ArrayLike) -> np.ndarray:
    q = np.asarray(quaternion, dtype=np.float64)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("Quaternion must be a finite wxyz length-4 vector")
    length = float(np.linalg.norm(q))
    if not np.isclose(length, 1.0, atol=1e-3):
        raise ValueError("Quaternion must have unit magnitude")
    w, x, y, z = q / length
    return np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)


def link_local_mesh_envelope(vertices: ArrayLike) -> tuple[np.ndarray, np.ndarray]:
    """Return center and full size of mesh vertices in link-local coordinates."""
    verts = np.asarray(vertices, dtype=np.float64)
    if verts.ndim != 2 or verts.shape[1] != 3 or not len(verts) or not np.all(np.isfinite(verts)):
        raise ValueError("Need nonempty finite (N, 3) link-local vertices")
    lo, hi = verts.min(axis=0), verts.max(axis=0)
    return (lo + hi) / 2.0, hi - lo


def world_envelope(local_center: ArrayLike, local_size: ArrayLike,
                   link_position: ArrayLike, link_quaternion: ArrayLike,
                   padding_m: float = 0.0) -> OrientedBox:
    """Create a world oriented box from a link-local box and its rigid pose."""
    pad = float(padding_m)
    if not np.isfinite(pad) or pad < 0:
        raise ValueError("padding_m must be finite and nonnegative")
    center = np.asarray(local_center, dtype=np.float64)
    position = np.asarray(link_position, dtype=np.float64)
    size = np.asarray(local_size, dtype=np.float64)
    if any(v.shape != (3,) or not np.all(np.isfinite(v)) for v in (center, position, size)):
        raise ValueError("Center, position, and size must be finite 3-vectors")
    if np.any(size < 0):
        raise ValueError("Local box size must be nonnegative")
    rotation = quaternion_wxyz_rotation(link_quaternion)
    # Degenerate mesh bounds can occur; box class needs positive dimensions.
    expanded_size = np.maximum(size + 2 * pad, 1e-9)
    return OrientedBox(position + rotation @ center, expanded_size, rotation)


def obb_separating_gap(a: OrientedBox, b: OrientedBox) -> float:
    """Maximum signed projected gap over the 15 OBB separating axes (meters).

    Positive => OBB envelopes disjoint, <=0 => overlap or touch (subject to
    floating point tolerance). NOT Euclidean surface distance or penetration.
    """
    axes = [a.rotation[:, i] for i in range(3)]
    axes += [b.rotation[:, i] for i in range(3)]
    axes += [np.cross(a.rotation[:, i], b.rotation[:, j]) for i in range(3) for j in range(3)]
    diff = np.asarray(b.center, dtype=np.float64) - np.asarray(a.center, dtype=np.float64)
    best = -np.inf
    for axis in axes:
        norm = float(np.linalg.norm(axis))
        if norm < 1e-12:
            continue
        axis = axis / norm
        extent_a = float(np.sum(np.asarray(a.size) * np.abs(a.rotation.T @ axis)) / 2)
        extent_b = float(np.sum(np.asarray(b.size) * np.abs(b.rotation.T @ axis)) / 2)
        gap = abs(float(diff @ axis)) - extent_a - extent_b
        best = max(best, gap)
    return float(best)


def analyze_collider_envelopes(data: dict, padding_m: float = 0.0) -> dict:
    """Flag possible static-obstacle overlaps of proxy OBBs in one observation."""
    if data.get("schema_version") != "rfp.collider_bounds.v1":
        raise ValueError("Unsupported collider envelope snapshot schema")
    colliders, obstacles = data.get("colliders"), data.get("obstacles")
    if not isinstance(colliders, list) or not colliders or not isinstance(obstacles, list):
        raise ValueError("Need collider and obstacle lists")
    pairs = []
    for c in colliders:
        b = world_envelope(c["local_center_m"], c["local_size_m"],
                           c["link_world_position_m"], c["link_world_orientation_wxyz"],
                           padding_m)
        for o in obstacles:
            other = OrientedBox(o["position_m"], o["dimensions_m"],
                                quaternion_wxyz_rotation(o["orientation_wxyz"]))
            gap = obb_separating_gap(b, other)
            pairs.append({"link": c["link_owner"], "collider": c["prim_path"],
                          "obstacle": o["name"], "max_projected_gap_m": gap,
                          "proxy_overlap_or_touch": gap <= 1e-9})
    return {
        "schema_version": "rfp.envelope_analysis.v1",
        "tested_pairs": len(pairs),
        "possibly_overlapping_pairs": [p for p in pairs if p["proxy_overlap_or_touch"]],
        "max_projected_gap_min_m": min(p["max_projected_gap_m"] for p in pairs),
        "valid_as_physical_collision_label": False,
        "limitations": "Authored-mesh envelopes, not exact cooked PhysX bodies. Gap is a separating-axis projection, not Euclidean clearance; continuous-time collision/self-collision untested.",
    }
