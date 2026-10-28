# M0-E: Verify body-geometry inputs before training

M0-D produced one valid, read-only Isaac Sim 6.2 Franka scene snapshot. Next we must **separate kinematic link frames from physical link collision shapes**.

## 1. Analyze the recorded frame observation (ordinary Python)

Open Command Prompt in the existing project environment:

~~~bat
cd %USERPROFILE%\Desktop\robot-flow-planner
git pull origin main
.venv\Scripts\activate.bat
python -m pytest -q
python scripts/analyze_scene_snapshot.py
~~~

Optional: `python scripts/analyze_scene_snapshot.py --json`.

This script reports:

- largest absolute **arm-joint** speed, in rad/s
- whether all 7 arm speeds are below the **0.05 rad/s data-quality heuristic** (not a safety threshold)
- nearest link **coordinate-frame origin** to a static box
- nearest **zero-radius line segment between adjacent link-frame origins** to a static box

**Do not** use positive frame/segment distances as verified robot clearance. Link origins and straight connecting lines leave out the physical width, geometry offsets, fingers, tool, and potentially other collision geometry. The script explicitly marks `valid_for_collision_label: false`.

If the arm is moving rapidly without an intended command, it may be settling under gravity or responding to authored joint drives. Verify the robot's articulation configuration and *controlled initial pose* before generating stationary demonstration labels. Do not suppress this by overwriting logged velocities.

## 2. Inventory actual robot colliders (Isaac Sim Script Editor)

With the `/World/RFP_Franka/Arm` scene loaded, paste:

~~~python
from pathlib import Path
script = Path.home() / "Desktop" / "robot-flow-planner" / "scripts" / "isaac_sim_gui_collision_inventory.py"
exec(compile(script.read_text(encoding="utf-8"), str(script), "exec"))
~~~

The read-only inspection walks the robot's referenced USD prims, finds prims with `UsdPhysics.CollisionAPI`, and records each collision shape's path, USD geometry type and any mesh approximation. It outputs a local JSON file under `outputs/collision_inventory` (ignored by Git).

The inventory does **not** extract or redistribute robot mesh vertices and does **not** measure dynamic collision clearance. When no collision prims are found, investigate the referenced asset before concluding it has no colliders.

## 3. What the next implementation must establish

Use the inventory to choose an appropriate geometric representation per link:

- Exact or conservative primitives for authored cubes/spheres/capsules.
- Approximations derived from actual collision meshes for complex link geometry, with documented fitting error and conservatism.
- Include the gripper/tool and self-collision. Confirm all transforms against the simulator while Play is active.
- Add continuous/swept collision checking and verified limits before any trajectory can be labeled collision-free.

This milestone is *scene analysis and collider discovery*, not yet a trained planner, real-time control loop, or real-world robot safety validation.

Official Isaac Sim API reference:
https://docs.isaacsim.omniverse.nvidia.com/6.2.0/py/source/extensions/isaacsim.core.experimental.prims/docs/index.html
