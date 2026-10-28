"""M0-F: Export the 11 authored Franka collision meshes as link-local AABB envelopes.

Run from Isaac Sim 6.2 > Window > Script Editor with the M0-C Franka
scene loaded and Play active. This code reads authored USD mesh vertices and
live link poses. It never moves the robot or edits the USD stage.

A link-local bounding box encloses the authored mesh vertices (and their
convex hull), but is NOT a faithful PhysX cooked collision shape. No continuous
trajectory, contact offset, or self-collision validation is performed.
"""

from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np
import omni.timeline
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from isaacsim.core.experimental.prims import Articulation, XformPrim

ROOT = "/World/RFP_Franka"
ARM = ROOT + "/Arm"
LINK_NAMES = {*(f"panda_link{i}" for i in range(8)),
              "panda_hand", "panda_leftfinger", "panda_rightfinger"}


def numpy_array(value):
    if hasattr(value, "detach"):
        value = value.detach().cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    result = np.asarray(value, dtype=np.float64)
    if not np.all(np.isfinite(result)):
        raise ValueError("Found non-finite values in simulation state")
    return result


def runtime_pose(path):
    positions, quaternions = XformPrim(path).get_world_poses()
    pos = numpy_array(positions).reshape(-1, 3)
    quat = numpy_array(quaternions).reshape(-1, 4)
    if len(pos) != 1 or len(quat) != 1:
        raise RuntimeError("Expected one runtime pose at " + path)
    return pos[0].tolist(), quat[0].tolist()


def owner_link(prim):
    current = prim
    while current and current.IsValid():
        if current.GetName() in LINK_NAMES:
            return current
        current = current.GetParent()
    return None


def capture():
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        raise RuntimeError("Press Play before exporting runtime link transforms")
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(ARM) if stage else None
    if not root or not root.IsValid():
        raise RuntimeError("Franka asset is missing; load the M0-C scene")
    if abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) > 1e-9:
        raise RuntimeError("This experiment expects 1 USD unit = 1 meter")

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    results = []
    for prim in Usd.PrimRange(root):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        enabled = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
        if enabled is False:
            continue
        if not prim.IsA(UsdGeom.Mesh):
            raise RuntimeError("Nonmesh collision prim needs another envelope fitting method: "
                               + str(prim.GetPath()))
        link = owner_link(prim)
        if link is None:
            raise RuntimeError("Cannot find owning rigid link for " + str(prim.GetPath()))
        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get()
        if not points:
            raise RuntimeError("No authored mesh vertices at " + str(prim.GetPath()))

        # OpenUSD uses row vectors; mesh->link equals (mesh->world)*(world->link).
        # Both are from authored USD transforms so the relative rigid offset
        # is independent of the moving articulation's live world pose.
        mesh_world = cache.GetLocalToWorldTransform(prim)
        link_world = cache.GetLocalToWorldTransform(link)
        mesh_to_link = mesh_world * link_world.GetInverse()

        vertices = []
        for point in points:
            local = mesh_to_link.Transform(
                Gf.Vec3d(float(point[0]), float(point[1]), float(point[2]))
            )
            vertices.append([float(local[0]), float(local[1]), float(local[2])])
        vertices = numpy_array(vertices)
        lower = vertices.min(axis=0)
        upper = vertices.max(axis=0)

        link_path = str(link.GetPath())
        position, orientation = runtime_pose(link_path)
        results.append({
            "prim_path": str(prim.GetPath()),
            "link_owner": link.GetName(),
            "link_prim_path": link_path,
            "vertex_count": int(len(vertices)),
            "source_geometry": "authored USD mesh vertices",
            "mesh_approximation": str(
                UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
            ) if prim.HasAPI(UsdPhysics.MeshCollisionAPI) else None,
            "local_center_m": ((lower + upper) * 0.5).tolist(),
            "local_size_m": (upper - lower).tolist(),
            "link_world_position_m": position,
            "link_world_orientation_wxyz": orientation,
        })

    if len(results) != 11 or {r["link_owner"] for r in results} != LINK_NAMES:
        raise RuntimeError(
            "Expected 11 unique Panda colliders; found "
            + repr([(r["link_owner"], r["prim_path"]) for r in results])
        )

    obstacles = []
    for name in ("Obstacle_Left", "Obstacle_Right"):
        path = ROOT + "/" + name
        cube = UsdGeom.Cube(stage.GetPrimAtPath(path))
        if not cube:
            raise RuntimeError("Missing or non-cube obstacle: " + path)
        position, rotation = runtime_pose(path)
        scales = numpy_array(XformPrim(path).get_world_scales()).reshape(-1, 3)
        if len(scales) != 1:
            raise RuntimeError("Unexpected world scale at " + path)
        sizes = np.abs(scales[0]) * float(cube.GetSizeAttr().Get())
        if np.any(sizes <= 0):
            raise RuntimeError("Invalid obstacle sizes")
        obstacles.append({
            "name": name, "position_m": position,
            "orientation_wxyz": rotation, "dimensions_m": sizes.tolist(),
        })

    arm = Articulation(ARM)
    return {
        "schema_version": "rfp.collider_bounds.v1",
        "captured_utc": datetime.now(timezone.utc).isoformat(),
        "simulation_time_s": float(timeline.get_current_time()),
        "robot_prim_path": ARM,
        "joint_names": list(arm.dof_names),
        "joint_positions": numpy_array(arm.get_dof_positions()).reshape(-1).tolist(),
        "joint_velocities": numpy_array(arm.get_dof_velocities()).reshape(-1).tolist(),
        "colliders": results,
        "obstacles": obstacles,
        "collision_status": "NOT_VALIDATED",
    }


data = capture()
folder = Path.home() / "Desktop" / "robot-flow-planner" / "outputs" / "collider_bounds"
folder.mkdir(parents=True, exist_ok=True)
dest = folder / (
    "franka_bounds_" +
    datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".json"
)
dest.write_text(json.dumps(data, indent=2), encoding="utf-8")
print("M0-F collider envelope capture:", dest)
print("Colliders:", len(data["colliders"]))
print("Obstacles:", len(data["obstacles"]))
for item in data["colliders"]:
    print(item["link_owner"], "vertices:", item["vertex_count"],
          "local box dimensions (m):", item["local_size_m"])
print("No joint commands sent. This is NOT an exact collision/safety check.")
