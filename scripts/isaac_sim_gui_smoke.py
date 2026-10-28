"""Isaac Sim 6.2 GUI smoke test: local geometry and PhysX, no external assets.

Run ONLY in Isaac Sim's Window > Script Editor on a new, finished-loading
stage. This script is *not* for ordinary Python/pip or the training venv.

Creates an independent demo subtree that may be safely recreated by rerunning.
To inspect, select /World/RFP_Smoke in the Stage pane and press F to frame.
Press Play to see the upper blue cube fall onto the floor.
"""

import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics

stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("No open USD stage. Choose File > New first.")

ROOT = "/World/RFP_Smoke"
if stage.GetPrimAtPath(ROOT).IsValid():
    stage.RemovePrim(ROOT)

# Keep the experiment in its own local prim subtree.
UsdGeom.Xform.Define(stage, ROOT)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

physics = UsdPhysics.Scene.Define(stage, ROOT + "/PhysicsScene")
physics.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
physics.CreateGravityMagnitudeAttr(9.81)


def add_box(name, position, dimensions, rgb, dynamic=False):
    """Full dimensions and position are in meters, no network assets involved."""
    cube = UsdGeom.Cube.Define(stage, ROOT + "/" + name)
    cube.CreateSizeAttr(1.0)
    transform = UsdGeom.Xformable(cube.GetPrim())
    transform.AddTranslateOp().Set(Gf.Vec3d(*position))
    transform.AddScaleOp().Set(Gf.Vec3f(*dimensions))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if dynamic:
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(1.0)
    return cube


add_box("Ground", (0.0, 0.0, -0.05), (4.0, 4.0, 0.1), (0.35, 0.35, 0.35))
add_box("Obstacle_A", (-0.65, 0.0, 0.3), (0.35, 0.45, 0.6), (0.88, 0.48, 0.12))
add_box("Obstacle_B", (0.75, -0.2, 0.2), (0.3, 0.3, 0.4), (0.88, 0.48, 0.12))
add_box("Drop_Test", (0.0, 0.0, 1.2), (0.22, 0.22, 0.22), (0.15, 0.48, 0.9), dynamic=True)

print("RFP smoke scene created at", ROOT)
print("Select /World/RFP_Smoke in Stage, press F to frame; press Play for physics.")
print("This tests stage authoring; actual gravity/collision still needs visual verification.")
