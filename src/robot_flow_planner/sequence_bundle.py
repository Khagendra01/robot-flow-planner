"""Validate one-run time-indexed passive Franka simulation sequences offline.

This only checks chronology, data integrity, and consistency of our *proxy*
OBB screen. It does NOT establish continuous-time collision-free paths.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .collider_bounds import analyze_collider_envelopes
from .collection_bundle import validate_collection_bundle


def _finite_vector(value, size, label):
    try:
        arr = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(label + " must contain finite numbers") from exc
    if arr.shape != (size,) or not np.all(np.isfinite(arr)):
        raise ValueError(label + f" must contain {size} finite numbers")
    return arr


def validate_sequence_bundle(data: dict) -> dict:
    if not isinstance(data, dict) or data.get("schema_version") != "rfp.sequence_bundle.v1":
        raise ValueError("Expected rfp.sequence_bundle.v1")
    if data.get("valid_as_collision_free_training_labels") is not False:
        raise ValueError("Passive sequences cannot be assigned safety labels")
    if data.get("collision_status") != "NOT_VALIDATED":
        raise ValueError("Physical collision status must remain NOT_VALIDATED")
    if data.get("rollout_kind") != "passive_simulation_observation_no_joint_commands":
        raise ValueError("Unsupported rollout type")

    baseline = data.get("baseline")
    validate_collection_bundle(baseline)
    original = baseline["collider_bounds"]
    original_names = [o["name"] for o in original["obstacles"]]
    expected_bodies = [(c["link_owner"], c["link_prim_path"]) for c in original["colliders"]]
    frames = data.get("frames")
    meta = data.get("sampling")
    if not isinstance(meta, dict) or not isinstance(frames, list) or len(frames) < 2:
        raise ValueError("At least two frame observations and sampling metadata required")
    n = len(frames)
    if n > 10000 or meta.get("requested_frames") != n or meta.get("recorded_frames") != n:
        raise ValueError("Invalid frame count or mismatch to sampling metadata")
    if meta.get("atomic_physics_frame_sampling_proven") is not False:
        raise ValueError("No proof of atomic data sampling; flag must be false")
    updates = meta.get("kit_updates_used")
    if type(updates) is not int or updates < n:
        raise ValueError("Kit update count must cover the sampled frames")

    times, peak_speed = [], 0.0
    observed_overlap_frames = 0
    for i, f in enumerate(frames):
        if f.get("valid_as_collision_free_label") is not False:
            raise ValueError(f"Frame {i} has an invalid safety label")
        t = float(f.get("simulation_time_s", float("nan")))
        if not np.isfinite(t) or t < 0 or (times and t <= times[-1]):
            raise ValueError("Frame simulation timestamps must strictly increase")
        times.append(t)
        _finite_vector(f.get("joint_positions"), 9, f"Frame {i} joint positions")
        dq = _finite_vector(f.get("joint_velocities"), 9, f"Frame {i} joint velocities")
        peak_speed = max(peak_speed, float(np.max(np.abs(dq[:7]))))

        bodies = f.get("body_poses")
        if (not isinstance(bodies, list)
                or [(b.get("name"), b.get("prim_path")) for b in bodies] != expected_bodies):
            raise ValueError(f"Frame {i} must contain 11 ordered body poses")
        for body in bodies:
            _finite_vector(body.get("position_m"), 3, "body position")
            q = _finite_vector(body.get("orientation_wxyz"), 4, "body quaternion")
            if not np.isclose(np.linalg.norm(q), 1, atol=1e-3):
                raise ValueError("Body quaternion is not unit normalized")

        boxes = f.get("obstacles")
        if not isinstance(boxes, list) or [o.get("name") for o in boxes] != original_names:
            raise ValueError(f"Frame {i} obstacle identities changed")
        for box in boxes:
            _finite_vector(box.get("position_m"), 3, "obstacle position")
            _finite_vector(box.get("orientation_wxyz"), 4, "obstacle orientation")
            if not np.isclose(
                np.linalg.norm(np.asarray(box["orientation_wxyz"], dtype=float)),
                1, atol=1e-3
            ):
                raise ValueError("Obstacle quaternion is not unit normalized")
            size = _finite_vector(box.get("dimensions_m"), 3, "obstacle dimensions")
            if np.any(size <= 0):
                raise ValueError("Obstacle dimensions must be positive")

        moving_bounds = dict(original)
        moving_bounds["colliders"] = [
            {**old, "link_world_position_m": current["position_m"],
             "link_world_orientation_wxyz": current["orientation_wxyz"]}
            for old, current in zip(original["colliders"], bodies)
        ]
        moving_bounds["obstacles"] = boxes
        r = analyze_collider_envelopes(moving_bounds, padding_m=0.01)
        if f.get("proxy_obb_pairs_tested") != r["tested_pairs"]:
            raise ValueError("Per-frame proxy pair count mismatch")
        overlap_count = len(r["possibly_overlapping_pairs"])
        if f.get("proxy_overlap_pair_count") != overlap_count:
            raise ValueError("Per-frame proxy overlap count mismatch")
        observed_overlap_frames += bool(overlap_count)
        if not np.isclose(float(f.get("minimum_max_projected_gap_m", float("nan"))),
                          r["max_projected_gap_min_m"], atol=1e-7):
            raise ValueError("Per-frame proxy gap mismatch")

    delta = np.diff(times)
    for name, measured in (
        ("first_simulation_time_s", times[0]),
        ("last_simulation_time_s", times[-1]),
        ("mean_time_step_s", float(np.mean(delta))),
        ("min_time_step_s", float(np.min(delta))),
        ("max_time_step_s", float(np.max(delta))),
        ("peak_arm_joint_speed_rad_s", peak_speed),
    ):
        value = float(meta.get(name, float("nan")))
        if not np.isfinite(value) or not np.isclose(value, measured, atol=1e-7):
            raise ValueError(f"Incorrect {name} metadata")

    return {
        "schema_version": "rfp.sequence_bundle.v1",
        "frames": n,
        "simulation_duration_s": float(times[-1] - times[0]),
        "body_count_per_frame": len(expected_bodies),
        "obstacle_count_per_frame": len(original_names),
        "frames_with_proxy_overlap": observed_overlap_frames,
        "max_arm_joint_speed_rad_s": peak_speed,
        "valid_as_collision_free_training_labels": False,
    }


def load_sequence_bundle(path: str | Path) -> dict:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    validate_sequence_bundle(data)
    return data
