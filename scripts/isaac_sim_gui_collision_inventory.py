"""M0-E: inventory the ACTUAL Franka collision prims, read-only.

Run in Isaac Sim 6.2 Script Editor with /World/RFP_Franka/Arm loaded.
Prints applied collision schemas and geometry types; does not alter physics,
move joints, or claim to compute collision distances.

Saving only metadata/paths; no mesh vertices or proprietary asset bytes.
"""

import json
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import omni.usd
from pxr import Usd, UsdPhysics

ROBOT_PATH = "/World/RFP_Franka/Arm"

stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("No open USD stage")
root = stage.GetPrimAtPath(ROBOT_PATH)
if not root.IsValid():
    raise RuntimeError("Franka root is missing; load the M0-C Franka scene first")


def link_owner(prim):
    """Nearest ancestral link to aid grouping of collision shape metadata."""
    current = prim
    while current and current.IsValid():
        name = current.GetName()
        if name.startswith("panda_link") or name in (
            "panda_hand", "panda_leftfinger", "panda_rightfinger"
        ):
            return name
        current = current.GetParent()
    return None


shapes = []
for prim in Usd.PrimRange(root):
    # Not every mesh is a collider: only record prims with explicit collision APIs.
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        continue
    collision = UsdPhysics.CollisionAPI(prim)
    attr = collision.GetCollisionEnabledAttr()
    enabled = attr.Get() if attr and attr.HasAuthoredValueOpinion() else None
    schemas = list(prim.GetAppliedSchemas())
    item = {
        "prim_path": str(prim.GetPath()),
        "name": prim.GetName(),
        "type_name": prim.GetTypeName(),
        "link_owner": link_owner(prim),
        "collision_enabled_authored": enabled,
        "applied_collision_schemas": [s for s in schemas if "Collision" in s],
    }
    mesh_collision = UsdPhysics.MeshCollisionAPI(prim)
    if mesh_collision and prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        approximation = mesh_collision.GetApproximationAttr()
        item["mesh_approximation"] = approximation.Get() if approximation else None
    shapes.append(item)

report = {
    "schema_version": "rfp.collision_inventory.v1",
    "robot_prim_path": ROBOT_PATH,
    "captured_utc": datetime.now(timezone.utc).isoformat(),
    "collision_prim_count": len(shapes),
    "shape_type_counts": dict(Counter(shape["type_name"] for shape in shapes)),
    "collision_prims": shapes,
}

dest = (
    Path.home() / "Desktop" / "robot-flow-planner"
    / "outputs" / "collision_inventory"
)
dest.mkdir(parents=True, exist_ok=True)
path = dest / ("franka_collision_inventory_"
               + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
               + ".json")
path.write_text(json.dumps(report, indent=2), encoding="utf-8")
print("M0-E collision inventory saved:", path)
print("Collision prim count:", len(shapes))
print("USD geometry types:", report["shape_type_counts"])
for shape in shapes[:30]:
    print("  ", shape["link_owner"], shape["type_name"], shape["prim_path"])
if len(shapes) > 30:
    print("  ... plus", len(shapes) - 30, "additional collision prims")
if not shapes:
    print("WARNING: No explicitly applied USD CollisionAPI found. "
          "Inspect referenced asset/physics before computing clearance.")
print("This inventories COLLIDERS, not collision distances; no commands issued.")
