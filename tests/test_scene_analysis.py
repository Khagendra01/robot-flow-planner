"""Test frame-proxy calculations and snapshot motion diagnostics."""

import numpy as np
import pytest

from robot_flow_planner.scene_analysis import _quaternion_rotation, analyze_scene_snapshot
from robot_flow_planner.scene_state import PANDA_JOINT_NAMES


def observation():
    def pose(x):
        return {"position_m": [float(x), 0., 0.],
                "orientation_wxyz": [1., 0., 0., 0.]}
    return {
        "schema_version": "rfp.scene_snapshot.v1",
        "stage_meters_per_unit": 1.0,
        "sim_time_s": 1.0,
        "robot": {
            "joint_names": list(PANDA_JOINT_NAMES),
            "joint_positions": [0.] * 9,
            "joint_velocities": [0.] * 9,
            "joint_position_units": ["rad"] * 7 + ["m"] * 2,
            "joint_velocity_units": ["rad/s"] * 7 + ["m/s"] * 2,
            "arm_dof_count": 7,
            "gripper_dof_count": 2,
            "links": [{"name": f"panda_link{i}", **pose(i - 3.5)}
                      for i in range(8)],
        },
        "obstacles": [
            {"name": "Obstacle_Left", **pose(0),
             "dimensions_m": [0.1, 0.2, 0.2], "dynamic": False},
            {"name": "Obstacle_Right", **pose(10),
             "dimensions_m": [0.1, 0.2, 0.2], "dynamic": False},
        ],
    }


def test_zero_radius_segment_crosses_box_while_endpoints_remain_outside():
    report = analyze_scene_snapshot(observation())
    frame = report["proxy_geometry"]["minimum_link_frame_to_box_distance_m"]
    segment = report["proxy_geometry"]["minimum_frame_segment_to_box_distance_m"]
    assert frame["distance_m"] == pytest.approx(0.45)
    assert segment["distance_m"] == pytest.approx(0.0)
    assert report["proxy_geometry"]["valid_for_collision_label"] is False


def test_moving_joint_state_is_flagged():
    sample = observation()
    sample["robot"]["joint_velocities"][3] = -2.17
    report = analyze_scene_snapshot(sample)
    assert report["motion"]["peak_arm_joint"] == "panda_joint4"
    assert report["motion"]["peak_arm_joint_speed_rad_s"] == pytest.approx(2.17)
    assert report["motion"]["stationary_by_threshold"] is False


def test_zero_arm_velocities_pass_stability_heuristic():
    assert analyze_scene_snapshot(observation())["motion"]["stationary_by_threshold"] is True


def test_quaternion_rotation():
    q = [np.sqrt(0.5), 0, 0, np.sqrt(0.5)]
    assert _quaternion_rotation(q) @ np.array([1, 0, 0]) == pytest.approx([0, 1, 0])


def test_negative_motion_threshold_rejected():
    with pytest.raises(ValueError, match="stationary_threshold_rad_s"):
        analyze_scene_snapshot(observation(), stationary_threshold_rad_s=-1)
