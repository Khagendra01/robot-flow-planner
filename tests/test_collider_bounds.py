"""Regression tests for collider envelopes and M0-F v2 data quality."""

import numpy as np
import pytest

from robot_flow_planner.collider_bounds import (
    link_local_mesh_envelope,
    world_envelope,
    obb_separating_gap,
    quaternion_wxyz_rotation,
    analyze_collider_envelopes,
    validate_collider_bounds_snapshot,
    PANDA_COLLIDER_NAMES,
)
from robot_flow_planner.geometry import OrientedBox
from robot_flow_planner.scene_state import PANDA_JOINT_NAMES


def sample_v2(link_origin=(0.0, 0.0, 0.0)):
    colliders = []
    for name in sorted(PANDA_COLLIDER_NAMES):
        link_path = "/World/RFP_Franka/Arm/" + name
        colliders.append({
            "link_owner": name,
            "link_prim_path": link_path,
            "prim_path": link_path + "/geometry/" + name,
            "vertex_count": 10,
            "local_center_m": [0.0, 0.0, 0.0],
            "local_size_m": [0.5, 0.4, 0.3],
            "link_world_position_m": list(link_origin),
            "link_world_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
        })
    return {
        "schema_version": "rfp.collider_bounds.v2",
        "stage_meters_per_unit": 1.0,
        "joint_names": list(PANDA_JOINT_NAMES),
        "joint_positions": [0.0] * 9,
        "joint_velocities": [0.0] * 9,
        "colliders": colliders,
        "obstacles": [
            {"name": "Obstacle_Left", "position_m": [0.3, 0, 0],
             "dimensions_m": [0.5, 0.5, 0.5], "orientation_wxyz": [1, 0, 0, 0]},
            {"name": "Obstacle_Right", "position_m": [3, 0, 0],
             "dimensions_m": [0.5, 0.5, 0.5], "orientation_wxyz": [1, 0, 0, 0]},
        ],
    }


def test_vertex_bounds():
    center, size = link_local_mesh_envelope([[-1, 0, 0], [3, 4, 2], [-1, 4, 2]])
    assert center == pytest.approx([1, 2, 1])
    assert size == pytest.approx([4, 4, 2])


def test_empty_vertices_rejected():
    with pytest.raises(ValueError):
        link_local_mesh_envelope([])


def test_rotation_and_center():
    q = [np.sqrt(0.5), 0, 0, np.sqrt(0.5)]
    b = world_envelope([1, 0, 0], [2, 1, 1], [2, 3, 4], q)
    assert b.center == pytest.approx([2, 4, 4])
    assert b.size == pytest.approx([2, 1, 1])


def test_separated_aligned_boxes():
    a = OrientedBox([0, 0, 0], [2, 2, 2], np.eye(3))
    b = OrientedBox([3, 0, 0], [2, 2, 2], np.eye(3))
    assert obb_separating_gap(a, b) == pytest.approx(1)
    assert obb_separating_gap(b, a) == pytest.approx(1)


def test_overlapping_aligned_boxes():
    a = OrientedBox([0, 0, 0], [2, 2, 2], np.eye(3))
    b = OrientedBox([0.5, 0, 0], [2, 2, 2], np.eye(3))
    assert obb_separating_gap(a, b) <= 0


def test_rotated_sat_case():
    a = OrientedBox([0, 0, 0], [2, 2, 2], np.eye(3))
    b = OrientedBox([0, 3.1, 0], [2, 2, 2],
        quaternion_wxyz_rotation([np.sqrt(0.5), 0, 0, np.sqrt(0.5)]))
    assert obb_separating_gap(a, b) == pytest.approx(1.1)


def test_world_bounds_padding():
    b = world_envelope([0, 0, 0], [1, 2, 3], [0, 0, 0], [1, 0, 0, 0], 0.1)
    assert b.size == pytest.approx([1.2, 2.2, 3.2])
    with pytest.raises(ValueError):
        world_envelope([0, 0, 0], [1, 2, 3], [0, 0, 0], [1, 0, 0, 0], -0.2)


def test_valid_v2_schema():
    r = validate_collider_bounds_snapshot(sample_v2())
    assert r["collider_count"] == 11
    assert r["geometric_bounds_plausible"]
    assert r["verified_physx_collision_geometry"] is False


def test_reject_old_v1_snapshot():
    legacy = sample_v2()
    legacy["schema_version"] = "rfp.collider_bounds.v1"
    with pytest.raises(ValueError, match="v1 contained"):
        analyze_collider_envelopes(legacy)


def test_reject_mesh_as_link_owner():
    invalid = sample_v2()
    invalid["colliders"][0]["link_prim_path"] = invalid["colliders"][0]["prim_path"]
    with pytest.raises(ValueError, match="owning link path"):
        validate_collider_bounds_snapshot(invalid)


def test_reject_oversized_local_mesh():
    invalid = sample_v2()
    invalid["colliders"][0]["local_size_m"] = [22, 19, 14]
    with pytest.raises(ValueError, match="implausible"):
        validate_collider_bounds_snapshot(invalid)


def test_envelope_analysis_flags_possible_overlap_not_physical_collision():
    report = analyze_collider_envelopes(sample_v2())
    assert report["tested_pairs"] == 22
    assert len(report["possibly_overlapping_pairs"]) == 11
    assert not report["valid_as_physical_collision_label"]


def test_envelope_analysis_reports_separation():
    report = analyze_collider_envelopes(sample_v2(link_origin=(10.0, 0, 0)))
    assert not report["possibly_overlapping_pairs"]
    assert report["max_projected_gap_min_m"] > 0
