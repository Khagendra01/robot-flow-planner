"""M0-D: Export one read-only joint/link/box observation from the live Franka scene.

Run *inside Isaac Sim 6.2 Window > Script Editor* after the Franka M0-C scene
is loaded and Play is active. Do NOT run with the project venv interpreter.

The experimental prim wrappers read runtime/Fabric transforms, avoiding stale
authored USD positions during simulation. This script does NOT command joints,
infer collision-free motion, or calculate a swept-volume clearance.
"""

from datetime import datetime, timezone
import json
from pathlib import Path
import re

import numpy as np
import omni.timeline
import omni.usd
from pxr import UsdGeom
from isaacsim.core.experimental.prims import Articulation, XformPrim

ROOT = "/World/RFP_Franka"
ARM_PATH = ROOT + "/Arm"
BOX_NAMES = ("Obstacle_Left", "Obstacle_Right")
LINK_RE = re.compile(r"^panda_link([0-7])$")


def as_numpy(x):
    """Handle NumPy, Warp, and Torch-backed data returned by Isaac APIs."""
    if hasattr(x, "detach"):
        x = x.detach().cpu()
    if hasattr(x, "numpy"):
        x = x.numpy()
    arr = np.asarray(x, dtype=np.float64)
    if not np.all(np.isfinite(arr)):
        raise ValueError("Non-finite simulator values encountered")
    return arr


def one_pose(path):
    """Read runtime world position and quaternion (scalar-first wxyz)."""
    positions, rotations = XformPrim(path).get_world_poses()
    p = as_numpy(positions).reshape(-1, 3)
    r = as_numpy(rotations).reshape(-1, 4)
    if len(p) != 1 or len(r) != 1:
        raise RuntimeError(f"Expected one transform at {path}, got {len(p)}")
    return p[0].tolist(), r[0].tolist()


def capture():
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        raise RuntimeError("Press Play in Isaac Sim before capturing a snapshot.")

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No open stage.")
    robot_root = stage.GetPrimAtPath(ARM_PATH)
    if not robot_root.IsValid():
        raise RuntimeError(f"Robot missing: {ARM_PATH}; run M0-C scene first.")

    meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
    if not np.isclose(meters_per_unit, 1.0):
        raise RuntimeError(
            f"Stage units must be meters for this prototype; got {meters_per_unit}"
        )

    arm = Articulation(ARM_PATH)
    joint_names = list(arm.dof_names)
    q = as_numpy(arm.get_dof_positions()).reshape(-1)
    dq = as_numpy(arm.get_dof_velocities()).reshape(-1)
    if len(joint_names) != 9 or len(q) != 9 or len(dq) != 9:
        raise RuntimeError(
            "Expected 9 Panda DOFs (7 arm + 2 gripper); "
            f"got names={len(joint_names)}, q={len(q)}, dq={len(dq)}"
        )

    # Resolve actual link prim paths from this particular referenced USD asset;
    # do not assume a fixed hierarchy within the Franka reference.
    link_paths = {}
    for prim in robot_root.GetAllChildren():
        pass  # prim hierarchy can contain many nested transforms
    from pxr import Usd
    for prim in Usd.PrimRange(robot_root):
        match = LINK_RE.fullmatch(prim.GetName())
        if match:
            index = int(match.group(1))
            if index in link_paths:
                raise RuntimeError(f"Duplicate panda_link{index} prims")
            link_paths[index] = str(prim.GetPath())

    if len(link_paths) != 8:
        available = sorted(
            (str(prim.GetPath()) for prim in Usd.PrimRange(robot_root)
             if "panda" in prim.GetName().lower())
        )
        raise RuntimeError(
            f"Expected panda_link0..7, found {sorted(link_paths)}. "
            f"Panda-named prims (first 25): {available[:25]}"
        )

    links = []
    for i in range(8):
        path = link_paths[i]
        position, orientation = one_pose(path)
        links.append(
            {
                "name": f"panda_link{i}",
                "prim_path": path,
                "position_m": position,
                "orientation_wxyz": orientation,
            }
        )

    obstacles = []
    for name in BOX_NAMES:
        path = ROOT + "/" + name
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"Expected obstacle missing: {path}")
        cube = UsdGeom.Cube(prim)
        if not cube:
            raise RuntimeError(f"Obstacle is not a USD Cube: {path}")

        position, orientation = one_pose(path)
        scale = as_numpy(XformPrim(path).get_world_scales()).reshape(-1, 3)
        if len(scale) != 1:
            raise RuntimeError(f"Expected one scale for {path}")
        size = abs(float(cube.GetSizeAttr().Get())) * np.abs(scale[0])
        if np.any(size <= 0):
            raise RuntimeError(f"Invalid obstacle size at {path}")
        obstacles.append(
            {
                "name": name,
                "prim_path": path,
                "position_m": position,
                "orientation_wxyz": orientation,
                "dimensions_m": size.tolist(),
                "dynamic": False,
            }
        )

    return {
        "schema_version": "rfp.scene_snapshot.v1",
        "source": "isaac-sim-script-editor",
        "stage_root": ROOT,
        "stage_meters_per_unit": meters_per_unit,
        "wall_time_utc": datetime.now(timezone.utc).isoformat(),
        "sim_time_s": float(timeline.get_current_time()),
        "robot": {
            "name": "FrankaPanda",
            "prim_path": ARM_PATH,
            "joint_names": joint_names,
            "joint_positions_rad": q.tolist(),
            "joint_velocities_rad_s": dq.tolist(),
            "arm_dof_count": 7,
            "gripper_dof_count": 2,
            "links": links,
        },
        "obstacles": obstacles,
    }


snapshot = capture()
destination = (
    Path.home()
    / "Desktop"
    / "robot-flow-planner"
    / "outputs"
    / "scene_snapshots"
)
destination.mkdir(parents=True, exist_ok=True)
path = destination / (
    "snapshot_"
    + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    + ".json"
)
path.write_text(json.dumps(snapshot, indent=2), encoding="utf-8")
print("M0-D read-only snapshot saved:", path)
print("Robot joints:", len(snapshot["robot"]["joint_names"]))
print("Robot link frames:", len(snapshot["robot"]["links"]))
print("Obstacles:", len(snapshot["obstacles"]))
print("Sim time (s):", snapshot["sim_time_s"])
print("No joint commands issued. This is NOT a collision/safety validation.")
