# M0-F: From Franka collision meshes to conservative link-local boxes

The user supplied the M0-E collision inventory, confirming **11 enabled Mesh collision prims**: panda_link0..7, panda_hand, panda_leftfinger, panda_rightfinger. All eleven report `mesh_approximation = convexHull`.

**Important:** The inventory contains mesh prim paths and collision APIs, but not vertices, sizes, or link-local offsets. It cannot by itself give physical clearance. M0-F reads the authored mesh points directly in Isaac Sim, transforms them into the owning link coordinate system, then fits a link-aligned bounding box to each collider. These 11 boxes track actual live link poses.

## Step 1: Sync the project

In Windows Command Prompt:

~~~bat
cd %USERPROFILE%\Desktop\robot-flow-planner
git pull origin main
.venv\Scripts\activate.bat
python -m pytest -q
~~~

The project environment is for offline NumPy calculations. Do not use it to execute Isaac Sim Script Editor scripts.

## Step 2: Export full-body approximate geometry inside Isaac Sim

With the M0-C Franka scene open and **Play** active, open `Window > Script Editor`, paste:

~~~python
from pathlib import Path
path = Path.home() / "Desktop" / "robot-flow-planner" / "scripts" / "isaac_sim_gui_collider_bounds.py"
exec(compile(path.read_text(encoding="utf-8"), str(path), "exec"))
~~~

If successful:

~~~text
M0-F collider envelope capture: ...\outputs\collider_bounds\franka_bounds_....json
Colliders: 11
Obstacles: 2
panda_link0 vertices: ... local box dimensions (m): [...]
...
panda_rightfinger vertices: ... local box dimensions (m): [...]
No joint commands sent. This is NOT an exact collision/safety check.
~~~

Output includes live pose of all 11 physical bodies, local box center and full dimensions for each mesh, its vertex count, and two obstacle box poses. These JSON snapshots are stored locally and ignored by Git.

If the exporter errors, send the **first traceback**. In particular, a missing mesh point array or unexpected asset hierarchy means we should adapt to the actual USD asset rather than guess the geometry.

### How local bounds are computed

For each collider:

- Read its `UsdGeom.Mesh.points` array (authored mesh vertices).
- Convert each vertex into the owning rigid link frame using USD's authored transforms. OpenUSD uses **row-vector** matrices; the relative matrix is `meshLocalToWorld * inverse(linkLocalToWorld)`.
- Fit the min/max of all transformed points, obtaining the box's local center and dimensions.
- At snapshot time, use Isaac Sim's runtime `XformPrim.get_world_poses()` for the parent link's position and quaternion (wxyz).

This fits a link-aligned AABB enclosing the authored mesh. When rigidly transformed into world space it becomes an oriented bounding box. It includes the hand and fingers, which the earlier eight-frame snapshot omitted.

## Step 3: Run the offline proximity screen

In the project Command Prompt:

~~~bat
python scripts/analyze_collider_bounds.py
~~~

The default adds a **1 cm padding** to each side of each envelope as a tunable *experimental* margin, not a certified bound on physics cooking/error. You can choose `--padding-m 0.0` for raw geometric diagnostics.

The offline implementation uses the 15-axis separating-axis theorem (SAT) to test overlap between each link-envelope OBB and each rectangular obstacle. It reports:

- number of collider/obstacle pairs checked (11 x 2 = 22)
- number of envelope overlaps or touches
- largest projected separation axis per pair, summarized across pairs

**Do not call the reported gap Euclidean distance.** It is a signed projected margin. Positive projected separation means the two tested OBB envelopes are geometrically disjoint; an overlap is **indeterminate** for the original mesh bodies because boxes can be much bigger than their contents.

**Do not label trajectories collision-free from this result.** PhysX convex-hull cooking and contact offsets can differ from mesh-point enclosures. We also have not verified runtime frame alignment, all time steps between samples, self-collision, collision with table/ground, actual joint limits, or safe stopping behavior. A proper safety validation layer requires more than M0-F.

## Next milestone

Verify these approximate collider envelopes visually against the actual robot and compare with PhysX contact/collision queries under controlled, low-speed simulated motions. After that, implement a time-indexed *full-body* trajectory evaluator rather than assuming any endpoint-only clearance is sufficient.

Official references:

- https://docs.isaacsim.omniverse.nvidia.com/6.2.0/py/source/extensions/isaacsim.core.experimental.prims/docs/index.html
- https://openusd.org/release/api/class_usd_geom_xform_cache.html
- https://openusd.org/release/api/class_gf_matrix4d.html
