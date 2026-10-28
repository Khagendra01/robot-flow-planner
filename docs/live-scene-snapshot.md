# M0-D: read-only live scene snapshot

## Why this is the next step

The Franka articulation has been verified in the user's Isaac Sim 6.2.0-rc.14 installation:

- 9 joint names: panda_joint1..7, panda_finger_joint1..2
- live `get_dof_positions()` output in radians
- successful in-app Script Editor execution

This stage records the **full input state** of our future learned planner, without trying to move the robot.

**Important:** PhysX/Fabric may hold current link transforms that are not written back to USD. We therefore use `isaacsim.core.experimental.prims.XformPrim.get_world_poses()` for runtime link poses, rather than relying on `UsdGeom.XformCache` while physics runs. This follows NVIDIA's latest simulation data-flow guidance.

## Steps (Windows Command Prompt + Isaac Sim GUI)

1. In an ordinary Command Prompt, pull the new code:

~~~bat
cd %USERPROFILE%\Desktop\robot-flow-planner
git pull origin main
~~~

2. In Isaac Sim, keep the `/World/RFP_Franka` scene loaded and **Play** active. Open **Window > Script Editor**, paste and run:

~~~python
from pathlib import Path
script = Path.home() / "Desktop" / "robot-flow-planner" / "scripts" / "isaac_sim_gui_scene_snapshot.py"
exec(compile(script.read_text(encoding="utf-8"), str(script), "exec"))
~~~

3. Expected console output:

~~~text
M0-D read-only snapshot saved: C:\Users\...\Desktop\robot-flow-planner\outputs\scene_snapshots\snapshot_....json
Robot joints: 9
Robot link frames: 8
Obstacles: 2
Sim time (s): ...
No joint commands issued. This is NOT a collision/safety validation.
~~~

4. Back in the original project Command Prompt (with `(.venv)` active):

~~~bat
python -m pytest -q
python scripts/validate_scene_snapshot.py
~~~

The validator checks joint arrays, 8 link frame poses, 2 static box poses/sizes, finite values, quaternion norms and unit scale. It will reject malformed data. The JSON files remain local in `outputs/scene_snapshots/` (ignored by Git).

## What the recorded values mean

- `robot.joint_positions`: 9 positions in joint order: the first 7 arm revolute joints in **radians**, the 2 prismatic finger joints in **meters**.
- `robot.joint_velocities`: 9 velocities: arm joints in **rad/s** and fingers in **m/s**.
- `robot.joint_position_units` and `robot.joint_velocity_units`: explicit per-joint units so finger translations are not confused with joint angles.
- `robot.links`: panda_link0..7 **frame** positions and orientations. A frame is NOT the complete link's occupied geometry.
- `obstacles`: two static rectangular cubes with positions, quaternion orientations and full dimensions.
- `sim_time_s`: simulation timeline time of capture.
- `wall_time_utc`: wall-clock timestamp (not a guarantee of frame synchronization).

The exporter is **one snapshot per click**, not a 100 Hz logger. It is an M0 schema test and should not be used as a live servo feedback loop. It currently assumes exact M0-C paths and stage scale 1 meter/unit.

## Troubleshooting

- `Robot missing`: open the Franka scene and check that `/World/RFP_Franka/Arm` exists.
- `Press Play`: click Play and rerun.
- `Expected panda_link0..7`: asset hierarchy may use different link names. The error lists matching prim paths; paste the error so we can adapt discovery.
- `XformPrim` / `get_world_poses` fails: send the first traceback. Runtime transform wrappers can vary across a release-candidate installation.
- `Expected obstacle missing`: use the full M0-C scene, not the smoke scene.
- `No snapshots found`: verify the Script Editor printed the saved path and that repo path matches Desktop.
- Avoid interpreting link-frame distances as collision-free clearance. We still need verified link collision geometry, self-collision, and continuous trajectory checking.

## Later integration

Once this passes, collect time-indexed sequences across controlled trajectories, introduce verified collision volumes, and convert the dataset schema to batched robot/obstacle tokens for the cross-attention planner. Never feed simulator-only future obstacle locations into policy inputs.

Official API references:

- https://docs.isaacsim.omniverse.nvidia.com/6.2.0/py/source/extensions/isaacsim.core.experimental.prims/docs/index.html
- https://docs.isaacsim.omniverse.nvidia.com/6.1.0/physics/new_physics_engine.html
