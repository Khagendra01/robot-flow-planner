"""Validate saved M0-D scene snapshots without importing Isaac Sim."""

import json
from pathlib import Path

import numpy as np


SCHEMA_VERSION = "rfp.scene_snapshot.v1"
PANDA_JOINT_NAMES = [
    *(f"panda_joint{i}" for i in range(1, 8)),
    "panda_finger_joint1",
    "panda_finger_joint2",
]


def _vector(data: object, name: str, length: int) -> np.ndarray:
    a = np.asarray(data, dtype=np.float64)
    if a.shape != (length,) or not np.all(np.isfinite(a)):
        raise ValueError(f"{name} must contain {length} finite numbers")
    return a


def _pose(entry: dict, name: str) -> None:
    _vector(entry.get("position_m"), name + ".position_m", 3)
    quat = _vector(entry.get("orientation_wxyz"), name + ".orientation_wxyz", 4)
    if not np.isclose(float(np.linalg.norm(quat)), 1.0, atol=1e-3):
        raise ValueError(f"{name}.orientation_wxyz is not a unit quaternion")


def validate_scene_snapshot(snapshot: dict) -> dict:
    """Return shape summary or raise ValueError.

    Validity here means *data-contract validity*, NOT collision-free geometry.
    """
    if not isinstance(snapshot, dict):
        raise ValueError("Snapshot must be a mapping")
    if snapshot.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("Unsupported scene snapshot schema")
    if not np.isclose(float(snapshot.get("stage_meters_per_unit", -1)), 1.0):
        raise ValueError("M0 scene must use meters")
    sim_time = float(snapshot.get("sim_time_s", -1))
    if not np.isfinite(sim_time) or sim_time < 0:
        raise ValueError("Invalid simulation timestamp")
    robot = snapshot.get("robot")
    if not isinstance(robot, dict):
        raise ValueError("Missing robot object")
    if robot.get("joint_names") != PANDA_JOINT_NAMES:
        raise ValueError("Unexpected Panda joint names or order")
    _vector(robot.get("joint_positions_rad"), "joint_positions_rad", 9)
    _vector(robot.get("joint_velocities_rad_s"), "joint_velocities_rad_s", 9)
    if robot.get("arm_dof_count") != 7 or robot.get("gripper_dof_count") != 2:
        raise ValueError("Unexpected Panda DOF partition")

    links = robot.get("links")
    if not isinstance(links, list) or len(links) != 8:
        raise ValueError("Expected exactly 8 Panda link frames")
    if [link.get("name") for link in links] != [
        f"panda_link{i}" for i in range(8)
    ]:
        raise ValueError("Expected panda_link0 through panda_link7 in order")
    for link in links:
        _pose(link, link["name"])

    obstacles = snapshot.get("obstacles")
    if not isinstance(obstacles, list) or len(obstacles) != 2:
        raise ValueError("M0-C scene expects exactly two rectangular obstacles")
    names = [obj.get("name") for obj in obstacles]
    if sorted(names) != ["Obstacle_Left", "Obstacle_Right"]:
        raise ValueError("M0-C obstacle names are missing or duplicated")
    for obs in obstacles:
        _pose(obs, obs["name"])
        size = _vector(obs.get("dimensions_m"), obs["name"] + ".dimensions_m", 3)
        if np.any(size <= 0):
            raise ValueError("Obstacle dimensions must be positive")
        if obs.get("dynamic") is not False:
            raise ValueError("M0-C obstacles must be static")

    return {
        "schema_version": SCHEMA_VERSION,
        "joint_count": 9,
        "arm_joint_count": 7,
        "link_frame_count": 8,
        "obstacle_count": 2,
        "sim_time_s": sim_time,
    }


def load_scene_snapshot(path: str | Path) -> dict:
    """Read a JSON snapshot, validate its structure, and return its content."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    validate_scene_snapshot(data)
    return data
