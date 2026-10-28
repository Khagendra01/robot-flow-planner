"""Tests for conservative local mesh bounds and static OBB overlap checks."""

import numpy as np
import pytest

from robot_flow_planner.collider_bounds import (
    link_local_mesh_envelope,
    world_envelope,
    obb_separating_gap,
    quaternion_wxyz_rotation,
    analyze_collider_envelopes,
)
from robot_flow_planner.geometry import OrientedBox


def test_vertex_bounds():
    center, size = link_local_mesh_envelope(
        [[-1, 0, 0], [3, 4, 2], [-1, 4, 2]]
    )
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
    b = OrientedBox(
        [0, 3.1, 0], [2, 2, 2],
        quaternion_wxyz_rotation([np.sqrt(0.5), 0, 0, np.sqrt(0.5)])
    )
    assert obb_separating_gap(a, b) == pytest.approx(1.1)


def test_world_bounds_padding():
    b = world_envelope([0, 0, 0], [1, 2, 3], [0, 0, 0], [1, 0, 0, 0], 0.1)
    assert b.size == pytest.approx([1.2, 2.2, 3.2])
    with pytest.raises(ValueError):
        world_envelope([0, 0, 0], [1, 2, 3], [0, 0, 0], [1, 0, 0, 0], -0.2)


def test_envelope_analysis_flags_overlap_but_not_collision():
    data = {
        "schema_version": "rfp.collider_bounds.v1",
        "colliders": [{
            "local_center_m": [0, 0, 0], "local_size_m": [1, 1, 1],
            "link_world_position_m": [0, 0, 0],
            "link_world_orientation_wxyz": [1, 0, 0, 0],
            "link_owner": "panda_link1", "prim_path": "/x",
        }],
        "obstacles": [{
            "name": "box", "position_m": [0.3, 0, 0],
            "dimensions_m": [1, 1, 1], "orientation_wxyz": [1, 0, 0, 0],
        }],
    }
    report = analyze_collider_envelopes(data)
    assert report["tested_pairs"] == 1
    assert len(report["possibly_overlapping_pairs"]) == 1
    assert not report["valid_as_physical_collision_label"]


def test_envelope_analysis_reports_separation():
    data = {
        "schema_version": "rfp.collider_bounds.v1",
        "colliders": [{
            "local_center_m": [0, 0, 0], "local_size_m": [1, 1, 1],
            "link_world_position_m": [0, 0, 0],
            "link_world_orientation_wxyz": [1, 0, 0, 0],
            "link_owner": "link", "prim_path": "/link",
        }],
        "obstacles": [{
            "name": "box", "position_m": [2, 0, 0],
            "dimensions_m": [1, 1, 1], "orientation_wxyz": [1, 0, 0, 0],
        }],
    }
    report = analyze_collider_envelopes(data)
    assert not report["possibly_overlapping_pairs"]
    assert report["max_projected_gap_min_m"] == pytest.approx(1)
