"""Numerical regression tests for M0 geometric primitives."""

import numpy as np
import pytest

from robot_flow_planner import Capsule, OrientedBox, capsule_box_clearance, capsule_box_collision


def cube():
    return OrientedBox(center=[0, 0, 0], size=[2, 2, 2], rotation=np.eye(3))


def test_crossing_segment_has_zero_centerline_distance():
    cap = Capsule([-2, 0, 0], [2, 0, 0], 0.1)
    assert capsule_box_clearance(cap, cube()) == pytest.approx(-0.1)
    assert capsule_box_collision(cap, cube())


def test_parallel_segment_gap_is_exact():
    cap = Capsule([2, -2, 0], [2, 2, 0], 0.2)
    assert capsule_box_clearance(cap, cube()) == pytest.approx(0.8)
    assert not capsule_box_collision(cap, cube(), margin=0.79)
    assert capsule_box_collision(cap, cube(), margin=0.8)


def test_corner_distance():
    cap = Capsule([2, 2, 0], [2, 2, 0], 0.0)
    assert capsule_box_clearance(cap, cube()) == pytest.approx(np.sqrt(2))


def test_rotated_rectangular_box():
    rot = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    box = OrientedBox([0, 0, 0], [4, 1, 1], rot)
    cap = Capsule([0, 1.5, 0], [0, 1.8, 0], 0.1)
    assert capsule_box_collision(cap, box)
    cap2 = Capsule([1.5, 0, 0], [1.5, 0, 0], 0.1)
    assert capsule_box_clearance(cap2, box) == pytest.approx(0.9)


def test_oblique_segment_closest_point_inside_segment():
    cap = Capsule([2, -2, 0], [2, 2, 0], 0.0)
    assert capsule_box_clearance(cap, cube()) == pytest.approx(1.0)


@pytest.mark.parametrize("dimensions", [[1, 0, 1], [-1, 1, 1], [1, float("nan"), 1]])
def test_invalid_box_sizes_rejected(dimensions):
    with pytest.raises(ValueError):
        OrientedBox([0, 0, 0], dimensions, np.eye(3))


def test_invalid_rotation_rejected():
    with pytest.raises(ValueError):
        OrientedBox([0, 0, 0], [1, 1, 1], np.diag([2, 1, 1]))


def test_invalid_margin_rejected():
    with pytest.raises(ValueError):
        capsule_box_collision(Capsule([0, 0, 0], [1, 0, 0], 0.1), cube(), margin=-0.1)


def test_segment_near_tiny_box():
    # Closest distance is a 1 cm x-gap, despite a very long segment.
    box = OrientedBox([0, 0, 0], [0.01, 0.01, 0.01], np.eye(3))
    cap = Capsule([0.015, -100, 0], [0.015, 100, 0], 0.0)
    assert capsule_box_clearance(cap, box) == pytest.approx(0.01)


def test_box_does_not_freeze_caller_rotation():
    rotation = np.eye(3)
    OrientedBox([0, 0, 0], [1, 1, 1], rotation)
    assert rotation.flags.writeable
    rotation[0, 0] = 2.0
