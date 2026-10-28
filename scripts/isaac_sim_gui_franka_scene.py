"""M0-C: Load a Franka Panda plus simple static box obstacles in Isaac Sim 6.2.

Run from Window > Script Editor on a NEW stage after the app has fully loaded.
This script is not a regular Python program; it uses Isaac Sim's embedded runtime.
It is an asset-backed scene; the Franka asset may require network/Nucleus access.

It deliberately DOES NOT send robot joint commands, generate trajectories, or
claim to validate collisions.
"""

import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics

import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.storage.native import get_assets_root_path

stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("Open File > New before running this script.")

ROOT = "/World/RFP_Franka"
if stage.GetPrimAtPath(ROOT).IsValid():
    raise RuntimeError(
        "The RFP_Franka scene already exists. Use File > New before rerunning "
        "(avoids changing a running articulation)."
    )

assets_root = get_assets_root_path()
if not assets_root:
    raise RuntimeError(
        "Isaac Sim could not resolve its asset root. Check asset/network access "
        "or use Create > Robots > Franka Emika Panda Arm in the GUI."
    )

UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, ROOT)
UsdPhysics.Scene.Define(stage, ROOT + "/PhysicsScene")

# NVIDIA Isaac Sim 6.2 documented Franka Panda asset.
FRANKA_USD = (
    assets_root.rstrip("/")
    + "/Isaac/Robots_Multiphysics/FrankaRobotics/FrankaPanda/franka/franka.usda"
)
try:
    robot_prim = stage_utils.add_reference_to_stage(
        usd_path=FRANKA_USD,
        path=ROOT + "/Arm",
    )
except Exception as exc:
    raise RuntimeError(
        "Could not reference the Franka asset. The initial smoke scene works "
        "without remote robot assets. Inspect Content/Console or manually choose "
        "Create > Robots > Franka Emika Panda Arm."
    ) from exc

if not robot_prim or not robot_prim.IsValid():
    raise RuntimeError("Franka asset reference did not create a valid stage prim.")


def static_box(name, center, dimensions, rgb):
    prim = UsdGeom.Cube.Define(stage, ROOT + "/" + name)
    prim.CreateSizeAttr(1.0)
    xform = UsdGeom.Xformable(prim.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*center))
    xform.AddScaleOp().Set(Gf.Vec3f(*dimensions))
    prim.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
    return prim


static_box("Ground", (0.0, 0.0, -0.05), (3.0, 3.0, 0.1), (0.35, 0.35, 0.35))
static_box("Obstacle_Left", (0.60, 0.65, 0.15), (0.22, 0.30, 0.30), (0.90, 0.45, 0.12))
static_box("Obstacle_Right", (0.60, -0.65, 0.20), (0.25, 0.25, 0.40), (0.90, 0.45, 0.12))

print("RFP Franka reference created:", ROOT + "/Arm")
print("Asset:", FRANKA_USD)
print("Static obstacle prims created. Reference loading may still be in progress.")
print("Select", ROOT, "in Stage and press F to frame.")
print("Press Play, then run scripts/isaac_sim_gui_franka_inspect.py in Script Editor.")
