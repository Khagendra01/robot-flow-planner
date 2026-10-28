"""M0-G: Visual, non-colliding overlays for M0-F v2 Franka mesh envelopes.

Run this inside Isaac Sim 6.2's Window > Script Editor. The M0-C scene
must be loaded and simulation must be PLAYING when the script starts.

Loads the latest local franka_bounds_*.json (v2 only), reads each CURRENT
runtime link pose, pauses the timeline, then overlays semitransparent boxes.
The robot and obstacles are not modified; overlays have NO collision API.
To remove overlays, rerun or delete /RFP_ColliderBoundsOverlay from Stage.

Important: these are authored-mesh envelopes, NOT actual PhysX collision
shapes; positive separation cannot certify collision-free motion. Display
opacity support depends on the viewport's selected rendering mode.
"""

import json
from pathlib import Path

import numpy as np
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics
from isaacsim.core.experimental.prims import XformPrim

ROOT = "/RFP_ColliderBoundsOverlay"   # stage-root path avoids /World transforms
FRANKA_ROOT = "/World/RFP_Franka/Arm"
LINK_NAMES = {*(f"panda_link{i}" for i in range(8)),
              "panda_hand", "panda_leftfinger", "panda_rightfinger"}

path = Path.home() / "Desktop" / "robot-flow-planner" / "outputs" / "collider_bounds"
files = sorted(path.glob("franka_bounds_*.json"))
if not files:
    raise RuntimeError("No M0-F collider bounds JSON found; run the v2 exporter first")
file = files[-1]
data = json.loads(file.read_text(encoding="utf-8"))
if data.get("schema_version") != "rfp.collider_bounds.v2":
    raise RuntimeError(
        "Latest file is not v2. Re-export collider bounds after git pull. "
        "Do not visualize invalid v1 geometry."
    )

colliders = data.get("colliders")
if not isinstance(colliders, list) or len(colliders) != 11:
    raise RuntimeError("Expected 11 collider envelopes")
if {c.get("link_owner") for c in colliders} != LINK_NAMES:
    raise RuntimeError("Collider owners are missing or duplicated")

# Validate all source data before touching the USD stage.
for c in colliders:
    owner = c["link_owner"]
    owner_path = c.get("link_prim_path")
    collider_path = c.get("prim_path")
    if (not isinstance(owner_path, str)
            or owner_path != FRANKA_ROOT + "/" + owner
            or not isinstance(collider_path, str)
            or not collider_path.startswith(owner_path + "/")):
        raise RuntimeError("Invalid rigid link path in " + str(file) + ": " + owner)
    center = np.asarray(c["local_center_m"], dtype=float)
    size = np.asarray(c["local_size_m"], dtype=float)
    if (center.shape != (3,) or size.shape != (3,)
            or not np.all(np.isfinite(center)) or not np.all(np.isfinite(size))
            or np.any(size <= 0) or np.max(size) >= 2
            or np.max(np.abs(center)) >= 2):
        raise RuntimeError("Physically implausible local mesh box: " + owner)

timeline = omni.timeline.get_timeline_interface()
if not timeline.is_playing():
    raise RuntimeError("Press Play before drawing live-aligned debug boxes")
stage = omni.usd.get_context().get_stage()
if stage is None or not stage.GetPrimAtPath(FRANKA_ROOT).IsValid():
    raise RuntimeError("Franka scene is not loaded")

def as_numpy(x):
    if hasattr(x, "detach"):
        x = x.detach().cpu()
    if hasattr(x, "numpy"):
        x = x.numpy()
    a = np.asarray(x, dtype=float)
    if not np.all(np.isfinite(a)):
        raise RuntimeError("Non-finite live transform")
    return a

def quat_rotation(wxyz):
    q = np.asarray(wxyz, dtype=float)
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=float)

# Read all current link poses before pausing. Saved local envelopes move with
# the link. Using saved old world poses would NOT align with the live robot.
live = []
for c in colliders:
    p, q = XformPrim(c["link_prim_path"]).get_world_poses()
    p = as_numpy(p).reshape(-1, 3)
    q = as_numpy(q).reshape(-1, 4)
    if len(p) != 1 or len(q) != 1:
        raise RuntimeError("Unexpected number of live poses at " + c["link_prim_path"])
    rotation = quat_rotation(q[0])
    center_world = p[0] + rotation @ np.asarray(c["local_center_m"], dtype=float)
    live.append((c, center_world, q[0]))

# Pausing helps compare static boxes to the captured live state. The runtime
# poses are measured sequentially; they are not an atomic synchronized frame.
timeline.pause()

if stage.GetPrimAtPath(ROOT).IsValid():
    stage.RemovePrim(ROOT)
UsdGeom.Xform.Define(stage, ROOT)

for c, center_world, q in live:
    name = c["link_owner"]
    cube = UsdGeom.Cube.Define(stage, ROOT + "/" + name)
    cube.CreateSizeAttr(1.0)
    size = c["local_size_m"]
    transform = UsdGeom.Xformable(cube.GetPrim())
    transform.AddTranslateOp().Set(Gf.Vec3d(*map(float, center_world)))
    transform.AddOrientOp().Set(
        Gf.Quatf(float(q[0]), Gf.Vec3f(float(q[1]), float(q[2]), float(q[3])))
    )
    transform.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
    rgb = (0.1, 0.82, 0.42) if "finger" not in name else (0.2, 0.6, 1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    cube.CreateDisplayOpacityAttr([0.19])
    # Intentionally no PhysicsCollisionAPI / RigidBodyAPI. These are drawings.

print("M0-G non-colliding overlays:", ROOT)
print("Source v2 local geometry:", file.name)
print("Boxes created:", len(live))
print("Timeline paused; inspect how green (links)/blue (fingers) boxes")
print("enclose visible Franka bodies. Select", ROOT, "and press F to frame.")
print("Delete", ROOT, "in Stage to remove overlays before resuming Play.")
print("NOT exact PhysX geometry or validated collision clearance.")
