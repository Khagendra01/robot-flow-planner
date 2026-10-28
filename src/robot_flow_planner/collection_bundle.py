"""Validate unified one-run Isaac Sim data collections, without Isaac installed.

A valid data collection is NOT a collision-free motion planning training label.
"""

from __future__ import annotations

import json
from pathlib import Path
import numpy as np

from .scene_state import validate_scene_snapshot
from .collider_bounds import (
    analyze_collider_envelopes,
    validate_collider_bounds_snapshot,
)


def validate_collection_bundle(data: dict) -> dict:
    if not isinstance(data, dict) or data.get("schema_version") != "rfp.collection_bundle.v1":
        raise ValueError("Expected rfp.collection_bundle.v1")
    scene = data.get("scene_snapshot")
    inventory = data.get("collision_inventory")
    bounds = data.get("collider_bounds")
    if not isinstance(inventory, dict):
        raise ValueError("Missing collision inventory")
    scene_summary = validate_scene_snapshot(scene)
    bounds_summary = validate_collider_bounds_snapshot(bounds)
    if inventory.get("schema_version") != "rfp.collision_inventory.v1":
        raise ValueError("Unexpected collision inventory schema")
    prims = inventory.get("collision_prims")
    if (inventory.get("collision_prim_count") != 11
            or not isinstance(prims, list) or len(prims) != 11):
        raise ValueError("Need exactly 11 inventoried collision meshes")
    inv_paths = [p.get("prim_path") for p in prims]
    bounds_paths = [p.get("prim_path") for p in bounds["colliders"]]
    if len(set(inv_paths)) != 11 or set(inv_paths) != set(bounds_paths):
        raise ValueError("Inventory/geometry collision-prim mismatch")
    if any(p.get("collision_enabled_authored") is False for p in prims):
        raise ValueError("Inventory includes disabled collision prim")
    if bounds["robot_prim_path"] != scene["robot"]["prim_path"]:
        raise ValueError("Robot paths disagree across the capture")
    if {o.get("name") for o in bounds["obstacles"]} != {
        o.get("name") for o in scene["obstacles"]
    }:
        raise ValueError("Obstacle identities changed during capture")
    checks = data.get("checks")
    if not isinstance(checks, dict) or checks.get("valid_as_collision_free_training_label") is not False:
        raise ValueError("Collection must explicitly prohibit collision-free labels")
    if checks.get("inventory_matches_bounds") is not True:
        raise ValueError("Collector did not mark inventory match")
    q0 = np.asarray(scene["robot"]["joint_positions"], dtype=np.float64)
    q1 = np.asarray(bounds["joint_positions"], dtype=np.float64)
    drift = float(np.max(np.abs(q1 - q0)))
    if not np.isclose(
        float(checks.get("max_joint_position_drift_native_units", float("nan"))),
        drift, atol=1e-7, rtol=1e-6,
    ):
        raise ValueError("Reported joint drift does not match the source captures")
    if "sim_time_delta_between_source_reads_s" in checks:
        dt = float(bounds["simulation_time_s"] - scene["sim_time_s"])
        if dt < -1e-7:
            raise ValueError("Baseline simulation time went backwards")
        recorded_dt = float(checks["sim_time_delta_between_source_reads_s"])
        if not np.isclose(recorded_dt, dt, atol=1e-7):
            raise ValueError("Incorrect baseline simulation time delta")
        if checks.get("sim_time_advanced_between_source_reads") is not bool(dt > 1e-7):
            raise ValueError("Baseline time-advance flag inconsistent")

    max_arm_speed = float(max(
        np.max(np.abs(scene["robot"]["joint_velocities"][:7])),
        np.max(np.abs(bounds["joint_velocities"][:7])),
    ))
    if not np.isclose(
        float(checks.get("max_absolute_arm_speed_rad_s", float("nan"))),
        max_arm_speed, atol=1e-7, rtol=1e-6,
    ):
        raise ValueError("Reported joint speed does not match source captures")
    if checks.get("stationary_by_0_05_rad_s_heuristic") is not bool(max_arm_speed <= 0.05):
        raise ValueError("Inconsistent stationarity heuristic")
    obb = analyze_collider_envelopes(bounds, padding_m=0.01)
    saved_obb = checks.get("pairwise_obb_screen")
    if not isinstance(saved_obb, dict):
        raise ValueError("Missing pairwise OBB screen")
    if (saved_obb.get("tested_pairs") != obb["tested_pairs"]
            or len(saved_obb.get("possibly_overlapping_pairs", []))
            != len(obb["possibly_overlapping_pairs"])):
        raise ValueError("Pairwise OBB counts do not match recomputation")
    if saved_obb.get("valid_as_physical_collision_label") is not False:
        raise ValueError("OBB screening cannot establish physical collision labels")

    return {
        "schema_version": data["schema_version"],
        "joint_count": scene_summary["joint_count"],
        "link_frame_count": scene_summary["link_frame_count"],
        "collision_mesh_count": bounds_summary["collider_count"],
        "obstacle_count": bounds_summary["obstacle_count"],
        "obb_pairs_checked": obb["tested_pairs"],
        "obb_pair_overlaps": len(obb["possibly_overlapping_pairs"]),
        "max_joint_drift_mixed_units": drift,
        "max_arm_speed_rad_s": max_arm_speed,
        "safe_for_collision_free_labels": False,
    }


def load_collection_bundle(path: str | Path) -> dict:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    validate_collection_bundle(data)
    return data
