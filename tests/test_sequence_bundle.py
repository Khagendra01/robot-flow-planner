"""M1-A tests for frame chronology and sequence integrity."""

from copy import deepcopy

import pytest

from robot_flow_planner.collider_bounds import analyze_collider_envelopes
from robot_flow_planner.sequence_bundle import validate_sequence_bundle

from test_collection_bundle import sample_bundle


def sample_sequence():
    baseline = sample_bundle()
    bounds = baseline["collider_bounds"]
    frames = []
    for t in (1.0, 1.1):
        bodies = [
            {
                "name": c["link_owner"],
                "prim_path": c["link_prim_path"],
                "position_m": c["link_world_position_m"].copy(),
                "orientation_wxyz": c["link_world_orientation_wxyz"].copy(),
            }
            for c in bounds["colliders"]
        ]
        obstacles = deepcopy(bounds["obstacles"])
        analysis = analyze_collider_envelopes(bounds, padding_m=0.01)
        frames.append({
            "simulation_time_s": t,
            "joint_positions": [0.0] * 9,
            "joint_velocities": [0.0] * 9,
            "body_poses": bodies,
            "obstacles": obstacles,
            "proxy_obb_pairs_tested": 22,
            "proxy_overlap_pair_count": len(analysis["possibly_overlapping_pairs"]),
            "minimum_max_projected_gap_m": analysis["max_projected_gap_min_m"],
            "valid_as_collision_free_label": False,
        })
    return {
        "schema_version": "rfp.sequence_bundle.v1",
        "baseline": baseline,
        "rollout_kind": "passive_simulation_observation_no_joint_commands",
        "sampling": {
            "requested_frames": 2,
            "recorded_frames": 2,
            "kit_updates_used": 2,
            "first_simulation_time_s": 1.0,
            "last_simulation_time_s": 1.1,
            "mean_time_step_s": 0.1,
            "min_time_step_s": 0.1,
            "max_time_step_s": 0.1,
            "peak_arm_joint_speed_rad_s": 0.0,
            "atomic_physics_frame_sampling_proven": False,
        },
        "frames": frames,
        "collision_status": "NOT_VALIDATED",
        "valid_as_collision_free_training_labels": False,
    }


def test_valid_time_advanced_sequence():
    report = validate_sequence_bundle(sample_sequence())
    assert report["frames"] == 2
    assert report["body_count_per_frame"] == 11
    assert report["simulation_duration_s"] == pytest.approx(0.1)


def test_duplicate_simulation_time_rejected():
    d = sample_sequence()
    d["frames"][1]["simulation_time_s"] = 1.0
    with pytest.raises(ValueError, match="strictly increase"):
        validate_sequence_bundle(d)


def test_missing_body_rejected():
    d = sample_sequence()
    d["frames"][0]["body_poses"].pop()
    with pytest.raises(ValueError, match="11 ordered body poses"):
        validate_sequence_bundle(d)


def test_wrong_overlap_count_rejected():
    d = sample_sequence()
    d["frames"][1]["proxy_overlap_pair_count"] = 20
    with pytest.raises(ValueError, match="overlap count mismatch"):
        validate_sequence_bundle(d)


def test_invalid_safety_label_rejected():
    d = sample_sequence()
    d["frames"][0]["valid_as_collision_free_label"] = True
    with pytest.raises(ValueError, match="invalid safety label"):
        validate_sequence_bundle(d)


def test_invalid_aggregate_metadata_rejected():
    d = sample_sequence()
    d["sampling"]["mean_time_step_s"] = 0.12
    with pytest.raises(ValueError, match="mean_time_step_s"):
        validate_sequence_bundle(d)


def test_timestamp_count_mismatch_rejected():
    d = sample_sequence()
    d["sampling"]["recorded_frames"] = 3
    with pytest.raises(ValueError, match="frame count"):
        validate_sequence_bundle(d)


def test_sequence_cannot_claim_physx_validation():
    d = sample_sequence()
    d["collision_status"] = "SAFE"
    with pytest.raises(ValueError, match="NOT_VALIDATED"):
        validate_sequence_bundle(d)
