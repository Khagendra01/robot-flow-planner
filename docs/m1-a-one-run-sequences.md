# M1-A: One-run time-indexed Franka collection

## Why it was added

The first **single-run** Franka observation bundle succeeded, but its scene
and collision-bounds measurements had **identical simulator timestamps**,
despite joint speeds up to 2.336 rad/s. Consecutive synchronous Python reads
did not advance the simulation. Hence `max_joint_position_drift=0` in the
bundle is **not evidence of stationarity**.

For training sequences, we must obtain physically advancing snapshots.

## What this script does in ONE run

`scripts/isaac_sim_gui_collect_sequence.py` automatically:

1. Runs the already-working **M0 combined collector** once (joint/link snapshot,
   11 colliders and 11 link-local mesh envelopes, 2 obstacles, 22-pair screen).
2. Uses Kit's documented `await omni.kit.app.get_app().next_update_async()`
   to advance **actual simulation time**.
3. Captures **60 strictly time-increasing observations** of the nine joints,
   all 11 robot collision-body world poses, and both obstacle poses.
4. For each observation, updates all 11 proxy mesh envelopes to the current
   body transforms and checks the 22 box pairs.
5. Validates the complete time series and saves **ONE JSON file** with the
   baseline embedded under `outputs/sequences/`.

You will not need to separately run the original snapshot, inventory, bounds,
or analysis scripts. They are orchestrated automatically.

**This is a passive simulation recording**—no robot joint commands, planned
goals, physics collision oracle, collision-free training labels, or safety
certifications. It records the current scene's natural motion, including
gravity/drive settling. This is a data-pipeline milestone, not demonstrations
for the flow-matching teacher yet.

## How to run

In the same Windows Command Prompt where the repository is installed:

~~~bat
cd %USERPROFILE%\Desktop\robot-flow-planner
git pull origin main
~~~

In Isaac Sim, open the scene containing `/World/RFP_Franka/Arm`, press
**Play**, then paste into **Window > Script Editor** and press Run **once**:

~~~python
from pathlib import Path
p = Path.home() / "Desktop/robot-flow-planner/scripts/isaac_sim_gui_collect_sequence.py"
exec(compile(p.read_text(encoding="utf-8"), str(p), "exec"))
~~~

You can watch the Script Editor's Console. It reports progress at frames 15,
30, 45, and 60, then:

~~~text
=== RFP SEQUENCE COMPLETE ===
ONE FILE: ...\outputs\sequences\franka_sequence_....json
Frames: 60
Simulation elapsed (s): ...
Peak arm joint speed (rad/s): ...
Frames with any proxy overlap: ...
Upload this SINGLE sequence file.
~~~

The script is asynchronous; **the Script Editor Run button launches the
capture and returns control to Isaac's UI**. Do not press Run a second time
while it is operating. The script pauses the timeline after completion or an
error. It does **not** require Windows administrator privileges.

If the timeline is paused when run, you will receive an explicit `Press Play`
error. If no distinct simulator timestamps occur after 1,000 application
updates, the collection fails rather than silently saving repeated frames.

### Optional offline verification

Not required for collection. A separate offline validator exists in
`robot_flow_planner.sequence_bundle.validate_sequence_bundle` and runs as
part of the one-run script; it requires no Isaac Sim package.

## What to do with the result

Upload **only the one `franka_sequence_*.json` file**. It embeds the entire
baseline and 60 frames. This data helps us test whether poses change
consistently with joint states, whether the OBB screen flags possible
obstacle interactions at sampled times, and whether the time-series ingestion
pipeline is functional.

The result is **not a safety or collision label**. Exact/cooked PhysX body
comparisons, self-collision, continuous trajectories between frames, dynamics,
obstacle prediction and calibrated robot drives still require independent
verification.

## Official Isaac Sim 6.2 reference

NVIDIA demonstrates `omni.kit.app.get_app().next_update_async()` in Script
Editor simulations to advance the timeline between measurements:
https://docs.isaacsim.omniverse.nvidia.com/6.2.0/replicator_tutorials/tutorial_replicator_getting_started.html
