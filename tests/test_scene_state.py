"""Contract tests for M0-D scene-state observations."""

from copy import deepcopy

import pytest

from robot_flow_planner.scene_state import (
    PANDA_JOINT_NAMES,
    validate_scene_snapshot,
)


def example_snapshot():
    pose = {"position_m": [0, 0, 0], "orientation_wxyz": [1, 0, 0, 0]}
    return {
        "schema_version": "rfp.scene_snapshot.v1",
        "stage_meters_per_unit": 1.0,
        "sim_time_s": 3.2,
        "robot": {
            "joint_names": PANDA_JOINT_NAMES.copy(),
            "joint_positions_rad": [0.0] * 9,
            "joint_velocities_rad_s": [0.0] * 9,
            "arm_dof_count": 7,
            "gripper_dof_count": 2,
            "links": [
                {"name": f"panda_link{i}", **deepcopy(pose)}
                for i in range(8)
            ],
        },
        "obstacles": [
            {"name": name, **deepcopy(pose),
             "dimensions_m": [0.2, 0.3, 0.4], "dynamic": False}
            for name in ["Obstacle_Left", "Obstacle_Right"]
        ],
    }


def test_valid_scene_snapshot():
    summary = validate_scene_snapshot(example_snapshot())
    assert summary["joint_count"] == 9
    assert summary["link_frame_count"] == 8
    assert summary["obstacle_count"] == 2


def test_invalid_joint_vector():
    s = example_snapshot()
    s["robot"]["joint_positions_rad"][2] = float("nan")
    with pytest.raises(ValueError, match="joint_positions_rad"):
        validate_scene_snapshot(s)


def test_missing_link():
    s = example_snapshot()
    s["robot"]["links"].pop()
    with pytest.raises(ValueError, match="link"):
        validate_scene_snapshot(s)


def test_invalid_obstacle_dimensions():
    s = example_snapshot()
    s["obstacles"][0]["dimensions_m"][2] = 0
    with pytest.raises(ValueError, match="dimensions"):
        validate_scene_snapshot(s)


def test_invalid_quaternion():
    s = example_snapshot()
    s["robot"]["links"][0]["orientation_wxyz"] = [0, 0, 0, 0]
    with pytest.raises(ValueError, match="unit quaternion"):
        validate_scene_snapshot(s)


def test_obstacle_names_are_required():
    s = example_snapshot()
    s["obstacles"][1]["name"] = "Obstacle_Left"
    with pytest.raises(ValueError, match="missing or duplicated"):
        validate_scene_snapshot(s)
