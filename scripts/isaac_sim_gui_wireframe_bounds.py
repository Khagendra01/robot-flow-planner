"""M0-G wireframe diagnostic: replace opaque solid overlays by box edges.

Run in Isaac Sim 6.2 > Window > Script Editor after the M0-G solid overlay
already exists (simulation can be PAUSED; Play is NOT required).

Reads the 11 existing debug Cube prims' USD transforms, draws 12 edges per
box with non-colliding BasisCurves, then hides the solid box subtree.
Does NOT move the robot, start physics, or read external assets.

This is a VISUAL alignment aid, not a collision measurement or guarantee.
Delete /RFP_ColliderBoundsWireframe to remove the wireframes.
"""

from itertools import product

import omni.usd
from pxr import Gf, Usd, UsdGeom

SOLID = "/RFP_ColliderBoundsOverlay"
WIRE = "/RFP_ColliderBoundsWireframe"
NAMES = [
    *(f"panda_link{i}" for i in range(8)),
    "panda_hand",
    "panda_rightfinger",
    "panda_leftfinger",
]

stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("No open USD stage")
solid_prim = stage.GetPrimAtPath(SOLID)
if not solid_prim.IsValid():
    raise RuntimeError(
        "No solid M0-G overlay exists. First run "
        "isaac_sim_gui_visualize_collider_bounds.py while Play is active."
    )

# Compute the box vertices from its ACTUAL authored USD xformOps. We do not
# need a live physics state; the solid overlay was already frozen in place.
cache = UsdGeom.XformCache(Usd.TimeCode.Default())
corners = list(product((-0.5, 0.5), repeat=3))
edge_indices = [
    (i, j) for i, a in enumerate(corners)
    for j, b in enumerate(corners)
    if i < j and sum(abs(x - y) > 0.1 for x, y in zip(a, b)) == 1
]
if len(edge_indices) != 12:
    raise AssertionError("Expected 12 distinct cube edges")

prims = []
for name in NAMES:
    path = SOLID + "/" + name
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid() or not prim.IsA(UsdGeom.Cube):
        raise RuntimeError("Missing visual-only Cube at " + path)
    cube = UsdGeom.Cube(prim)
    size = cube.GetSizeAttr().Get()
    if size is None or abs(float(size) - 1.0) > 1e-6:
        raise RuntimeError("Unexpected visual cube side length at " + path)
    matrix = cache.GetLocalToWorldTransform(prim)
    points = [
        matrix.Transform(Gf.Vec3d(*vertex))
        for vertex in corners
    ]
    prims.append((name, points))

# Author debug lines only after all source prims have been validated.
if stage.GetPrimAtPath(WIRE).IsValid():
    stage.RemovePrim(WIRE)
UsdGeom.Xform.Define(stage, WIRE)

for name, vertices in prims:
    curves = UsdGeom.BasisCurves.Define(stage, WIRE + "/" + name)
    curves.CreateTypeAttr(UsdGeom.Tokens.linear)
    curves.CreateWrapAttr(UsdGeom.Tokens.nonperiodic)
    curves.CreateCurveVertexCountsAttr([2] * len(edge_indices))
    points = [
        Gf.Vec3f(
            float(vertices[corner_index][0]),
            float(vertices[corner_index][1]),
            float(vertices[corner_index][2]),
        )
        for pair in edge_indices
        for corner_index in pair
    ]
    curves.CreatePointsAttr(points)
    # 4 mm nominal line width. RTX rendering modes may treat widths
    # differently; this is for inspection only.
    curves.CreateWidthsAttr([0.004] * len(edge_indices))
    curves.SetWidthsInterpolation(UsdGeom.Tokens.uniform)
    color = ((0.1, 0.95, 0.4) if "finger" not in name
             else (0.1, 0.65, 1.0))
    curves.CreateDisplayColorAttr([Gf.Vec3f(*color)])

# The line root is a sibling, so hiding solid will NOT hide the wireframes.
UsdGeom.Imageable(solid_prim).MakeInvisible()
print("M0-G wireframe overlays created at", WIRE)
print("Solid boxes hidden:", SOLID)
print("Wireframe link boxes:", len(prims))
print("The robot underneath should now be visible; take screenshot.")
print("For solid boxes again: UsdGeom.Imageable(stage.GetPrimAtPath(",
      repr(SOLID), ")).MakeVisible()")
print("No physics colliders added; no commands sent.")
