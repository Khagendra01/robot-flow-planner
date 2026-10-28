# M0-G: visually inspect Franka collision-envelope alignment

The first M0-F collider bounds export had a mesh-as-owning-link bug and produced
implausible multimeter extents. **Only** use the corrected
`rfp.collider_bounds.v2` outputs. The user has confirmed that the v2 offline
analyzer accepts 11 collider envelopes and checks all 22 body/obstacle pairs.
This establishes a one-time proxy screening result, **not** continuous collision
safety or fidelity to PhysX geometry.

## Before running

In your project Command Prompt:

~~~bat
cd %USERPROFILE%\Desktop\robot-flow-planner
git pull origin main
python -m pytest -q
~~~

Open the existing M0-C Franka scene in Isaac Sim and **press Play**.

Open **Window > Script Editor** and run:

~~~python
from pathlib import Path
script = Path.home() / "Desktop" / "robot-flow-planner" / "scripts" / "isaac_sim_gui_visualize_collider_bounds.py"
exec(compile(script.read_text(encoding="utf-8"), str(script), "exec"))
~~~

The script:

1. Reads the newest v2 bounds JSON from `outputs/collider_bounds/` and rejects v1.
2. Confirms 11 unique collider owners and physically plausible local geometry.
3. Reads the **current** runtime rigid-link poses with Isaac's `XformPrim`.
4. Pauses the timeline, creates 11 visual-only translucent USD cubes under
   `/RFP_ColliderBoundsOverlay` at the captured link poses.
5. Never applies PhysicsCollisionAPI or RigidBodyAPI to the cubes and never
   commands a joint.

**Note:** The runtime poses of the 11 links are collected sequentially rather
than in one atomic physics snapshot. If the robot is falling/moving rapidly,
small visual discrepancies may reflect that timing; stabilize the robot before
measuring detailed alignment.

The overlay has green boxes for the links and hand, blue boxes for fingers.
Display opacity depends on the renderer. If cubes look opaque, try hiding
individual cubes in the Stage tree to inspect the underlying mesh.
Select `/RFP_ColliderBoundsOverlay` and press **F** to frame.

## What to inspect

- Each box visibly follows the correct rigid link (no boxes several meters
  away or centered near the base incorrectly).
- All arm links, hand and both fingers are covered by their respective boxes.
- Boxes are plausibly larger than visible mesh geometry; this is expected
  because AABBs are conservative and orientation is link-aligned.
- Finger boxes are on the correct respective fingers; verify with a small
  bounded gripper action only in a later dedicated simulation experiment.
- The overlay must not make the robot react physically; these cubes are
  visualization-only.

Delete `/RFP_ColliderBoundsOverlay` in Stage to remove the cubes before
resuming Play, or reopen your saved scene.

If the script errors, send the **first traceback**. If it works, share a screenshot
of the robot with boxes visible and upload the newest v2 `franka_bounds_*.json`
to inspect actual local dimensions and offsets.

## Safe interpretation

- The prior result of **0 overlapping OBB pairs** over 22 pairs refers to **one
  sampled configuration** and two static obstacles.
- `0.357714 m` was a **separating-axis projected gap**, not Euclidean
  physical collision clearance or a safe path clearance.
- PhysX cooking, tool/finger contact, ground and self-collision, contact
  margins, uncertainty and intermediate configurations remain unverified.

After visual verification we can compare against actual physics-contact/query
results under controlled static poses. Do **not** label collision-free
trajectories on the strength of this overlay alone.

References:
- https://openusd.org/release/api/class_usd_geom_gprim.html
- https://openusd.org/release/api/class_usd_geom_cube.html
