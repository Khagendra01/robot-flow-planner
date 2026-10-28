# One-run data collection: Franka Panda in Isaac Sim 6.2

The earlier M0-D/E/F procedure required separate Script Editor runs for joint
frames, mesh inventory, bounding boxes and an offline screen. The individual
exporters were useful for debugging USD units and transforms, but repeating
them manually is unnecessary.

**Use this one Script Editor run instead.** It works on the already loaded
M0-C Franka scene without admin privileges and writes one consolidated JSON
file for sharing and analysis.

## Run once

In Windows Command Prompt:

~~~bat
cd %USERPROFILE%\Desktop\robot-flow-planner
git pull origin main
~~~

In Isaac Sim:

1. Open the Franka scene with \`/World/RFP_Franka/Arm\`.
2. Press **Play** once; the script needs initialized runtime articulation data.
3. Open **Window > Script Editor**. Paste and Run:

~~~python
from pathlib import Path
p = Path.home() / "Desktop/robot-flow-planner/scripts/isaac_sim_gui_collect_all.py"
exec(compile(p.read_text(encoding="utf-8"), str(p), "exec"))
~~~

**That's the complete collection procedure.** No need to run any other
exporters, validators, or geometry analysis commands separately. The script
calls the verified exporters, checks the resulting data, and runs the coarse
OBB overlap screen before saving.

Expected final output:

~~~text
=== RFP ONE-RUN COLLECTION COMPLETE ===
COMBINED FILE: C:\Users\...\Desktop\robot-flow-planner\outputs\collections\franka_collection_....json
Joints: 9 (7 arm + 2 gripper)
Link frames: 8
Collision meshes and envelopes: 11
Obstacle boxes: 2
OBB comparisons: 22
Proxy boxes touching/overlapping: ...
Maximum joint-state drift between captures (mixed rad/m): ...
Peak arm-joint speed during captures (rad/s): ...
NO robot commands issued. NO collision-free label generated.
Upload just the COMBINED FILE for the next development step.
~~~

You may share only the **combined** JSON file. It embeds all underlying data.
The collector also retains the original separate files as sidecars for
reproducibility, but you do not have to upload or inspect them individually.

### Optional offline check

The collector checks its output during the same Isaac Sim run. A separate
check is not necessary, but is available in the normal Python project venv:

~~~bat
python scripts/inspect_collection.py
~~~

## Contents of one combined JSON

- \`scene_snapshot\`: 9 measured joint positions and velocities, correct units
  (7 revolute and 2 prismatic), 8 robot link-world poses, 2 obstacles.
- \`collision_inventory\`: 11 actual mesh collider prims and authored APIs.
- \`collider_bounds\`: 11 link-local boxes derived from authored mesh points,
  with corresponding live world poses; v2 only.
- \`checks\`: schema checks, inventory agreement, stationary data-quality
  heuristic, change in joint positions across sequential captures, 22 OBB
  comparisons, explicit declaration that **there are no collision-free labels**.
- \`source_files\`: original sidecar paths and capture provenance.

**Caution:** The three source samples occur sequentially in a single Script
Editor execution and are **not** guaranteed to be atomically synchronized to a
single physics frame. For moving robot/obstacle training sequences, a later
frame-stepped, synchronized collector will be required. The collector flags
notable joint movement across the measurements.

The OBB screen checks authored mesh envelopes; these are not necessarily the
same as PhysX's cooked shapes, and no swept-volume, dynamic-obstacle,
ground-contact, self-collision or safe-stop guarantee is made.

## If the run fails

Paste only the **first traceback**, not the entire Kit log. Errors about
\`Press Play\` mean the timeline was not running; errors about missing Franka
mean the M0-C scene wasn't loaded. The program never attempts to run an
elevated Python interpreter.

## Next engineering milestone

Use verified collider bounds and PhysX overlap/contact comparisons to build
an independent training label source. After calibrating a safe initial pose,
robot drives and collision query fidelity, collect **time-indexed sequences**
at fixed physics steps in large batches, not manual snapshots.
