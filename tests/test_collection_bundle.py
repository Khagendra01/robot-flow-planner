"""Regression tests for one-run data collection and its safety boundary."""

from copy import deepcopy

import pytest

from robot_flow_planner.collection_bundle import validate_collection_bundle
from robot_flow_planner.collider_bounds import (
    PANDA_COLLIDER_NAMES,
    analyze_collider_envelopes,
)
from robot_flow_planner.scene_state import PANDA_JOINT_NAMES


def sample_bundle():
    names = sorted(PANDA_COLLIDER_NAMES)
    colliders = []
    inventory = []
    for name in names:
        owner = f"/World/RFP_Franka/Arm/{name}"
        mesh = f"{owner}/geometry/{name}"
        colliders.append({
            "link_owner": name,
            "link_prim_path": owner,
            "prim_path": mesh,
            "vertex_count": 8,
            "local_center_m": [0, 0, 0],
            "local_size_m": [0.2, 0.2, 0.2],
            "link_world_position_m": [0, 0, 0],
            "link_world_orientation_wxyz": [1, 0, 0, 0],
        })
        inventory.append({
            "prim_path": mesh,
            "name": name,
            "collision_enabled_authored": True,
        })

    obstacles = [
        {"name": name, "position_m": [x, 0, 0],
         "dimensions_m": [0.2, 0.2, 0.2],
         "orientation_wxyz": [1, 0, 0, 0]}
        for name, x in [("Obstacle_Left", 1.0), ("Obstacle_Right", 2.0)]
    ]
    bounds = {
        "schema_version": "rfp.collider_bounds.v2",
        "stage_meters_per_unit": 1.0,
        "robot_prim_path": "/World/RFP_Franka/Arm",
        "joint_names": list(PANDA_JOINT_NAMES),
        "joint_positions": [0.0] * 9,
        "joint_velocities": [0.0] * 9,
        "colliders": colliders,
        "obstacles": obstacles,
    }
    scene = {
        "schema_version": "rfp.scene_snapshot.v1",
        "stage_meters_per_unit": 1.0,
        "sim_time_s": 1.0,
        "robot": {
            "prim_path": "/World/RFP_Franka/Arm",
            "joint_names": list(PANDA_JOINT_NAMES),
            "joint_positions": [0.0] * 9,
            "joint_velocities": [0.0] * 9,
            "joint_position_units": ["rad"] * 7 + ["m"] * 2,
            "joint_velocity_units": ["rad/s"] * 7 + ["m/s"] * 2,
            "arm_dof_count": 7,
            "gripper_dof_count": 2,
            "links": [
                {"name": f"panda_link{i}", "position_m": [0, 0, 0],
                 "orientation_wxyz": [1, 0, 0, 0]}
                for i in range(8)
            ],
        },
        "obstacles": [{**o, "dynamic": False} for o in obstacles],
    }
    return {
        "schema_version": "rfp.collection_bundle.v1",
        "scene_snapshot": scene,
        "collision_inventory": {
            "schema_version": "rfp.collision_inventory.v1",
            "collision_prim_count": 11,
            "collision_prims": inventory,
        },
        "collider_bounds": bounds,
        "checks": {
            "inventory_matches_bounds": True,
            "max_joint_position_drift_native_units": 0.0,
            "max_absolute_arm_speed_rad_s": 0.0,
            "stationary_by_0_05_rad_s_heuristic": True,
            "pairwise_obb_screen": analyze_collider_envelopes(bounds, padding_m=0.01),
            "valid_as_collision_free_training_label": False,
        },
    }


def test_valid_one_run_bundle():
    result = validate_collection_bundle(sample_bundle())
    assert result["collision_mesh_count"] == 11
    assert result["obb_pairs_checked"] == 22
    assert result["safe_for_collision_free_labels"] is False


def test_legacy_bad_geometry_is_rejected():
    bad = sample_bundle()
    bad["collider_bounds"]["schema_version"] = "rfp.collider_bounds.v1"
    with pytest.raises(ValueError, match="v1 contained"):
        validate_collection_bundle(bad)


def test_missing_collider_is_rejected():
    bad = sample_bundle()
    bad["collision_inventory"]["collision_prims"].pop()
    with pytest.raises(ValueError, match="inventoried"):
        validate_collection_bundle(bad)


def test_false_safe_label_is_rejected():
    bad = sample_bundle()
    bad["checks"]["valid_as_collision_free_training_label"] = True
    with pytest.raises(ValueError, match="prohibit collision-free labels"):
        validate_collection_bundle(bad)


def test_joint_drift_mismatch_is_rejected():
    bad = sample_bundle()
    bad["collider_bounds"]["joint_positions"][0] = 0.5
    with pytest.raises(ValueError, match="joint drift"):
        validate_collection_bundle(bad)


def test_wrong_inventoried_prim_is_rejected():
    bad = sample_bundle()
    bad["collision_inventory"]["collision_prims"][0]["prim_path"] = "/elsewhere"
    with pytest.raises(ValueError, match="mismatch"):
        validate_collection_bundle(bad)


def test_arm_motion_capture_is_allowed_but_not_safe():
    moving = sample_bundle()
    moving["collider_bounds"]["joint_velocities"][3] = 1.4
    moving["checks"]["max_absolute_arm_speed_rad_s"] = 1.4
    moving["checks"]["stationary_by_0_05_rad_s_heuristic"] = False
    result = validate_collection_bundle(moving)
    assert result["max_arm_speed_rad_s"] == pytest.approx(1.4)
    assert not result["safe_for_collision_free_labels"]
