"""Diagnostic-only arm-motion and frame-proxy-to-box proximity analysis.

NOT robot-body collision checking: link coordinate frames and straight lines
between them are not the robot's collision geometry. Positive distances do not
establish whether robot meshes/collision volumes intersect an obstacle.
"""

from __future__ import annotations

import numpy as np

from .geometry import Capsule, OrientedBox, capsule_box_clearance
from .scene_state import validate_scene_snapshot


def _quaternion_rotation(wxyz: list[float]) -> np.ndarray:
    """Convert scalar-first Isaac quaternion to local-to-world rotation."""
    q = np.asarray(wxyz, dtype=np.float64)
    q = q / np.linalg.norm(q)  # snapshot validator checks finite, near-unit q
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
            [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
            [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
        ],
        dtype=np.float64,
    )


def analyze_scene_snapshot(snapshot: dict, stationary_threshold_rad_s: float = 0.05) -> dict:
    """Report observed arm motion and *proxy* distances in meters.

    This does NOT assert a stationary robot or collision-free robot body.
    The threshold is a data-quality heuristic, NOT a safety constraint.
    """
    validate_scene_snapshot(snapshot)
    threshold = float(stationary_threshold_rad_s)
    if not np.isfinite(threshold) or threshold < 0:
        raise ValueError("stationary_threshold_rad_s must be finite and nonnegative")

    robot = snapshot["robot"]
    velocities = np.asarray(robot["joint_velocities"][:7], dtype=np.float64)
    peak_i = int(np.argmax(np.abs(velocities)))
    max_speed = float(abs(velocities[peak_i]))
    links = robot["links"]
    boxes = [
        (
            o["name"],
            OrientedBox(
                o["position_m"],
                o["dimensions_m"],
                _quaternion_rotation(o["orientation_wxyz"]),
            ),
        )
        for o in snapshot["obstacles"]
    ]

    frame_distances = []
    segment_distances = []
    for box_name, box in boxes:
        for link in links:
            p = link["position_m"]
            frame_distances.append(
                {"obstacle": box_name, "frame": link["name"],
                 "distance_m": float(capsule_box_clearance(Capsule(p, p, 0.0), box))}
            )
        for a, b in zip(links[:-1], links[1:]):
            segment_distances.append(
                {"obstacle": box_name, "frame_segment": f"{a['name']}->{b['name']}",
                 "distance_m": float(capsule_box_clearance(
                     Capsule(a["position_m"], b["position_m"], 0.0), box))}
            )

    closest_frame = min(frame_distances, key=lambda item: item["distance_m"])
    closest_segment = min(segment_distances, key=lambda item: item["distance_m"])
    return {
        "schema_version": "rfp.scene_analysis.v1",
        "source_snapshot_version": snapshot["schema_version"],
        "sim_time_s": float(snapshot["sim_time_s"]),
        "motion": {
            "peak_arm_joint": robot["joint_names"][peak_i],
            "peak_arm_joint_speed_rad_s": max_speed,
            "stationary_threshold_rad_s": threshold,
            "stationary_by_threshold": bool(np.all(np.abs(velocities) <= threshold)),
            "data_quality_note": "Heuristic only; motion may reflect gravity, drives, or commands.",
        },
        "proxy_geometry": {
            "minimum_link_frame_to_box_distance_m": closest_frame,
            "minimum_frame_segment_to_box_distance_m": closest_segment,
            "checked_link_frames": len(links),
            "checked_frame_segments": len(links) - 1,
            "checked_obstacles": len(boxes),
            "valid_for_collision_label": False,
            "warning": (
                "Frame points and straight frame-to-frame segments have zero thickness "
                "and are not the full robot collision geometry; positive distances "
                "DO NOT establish robot-body clearance or safety."
            ),
        },
    }
