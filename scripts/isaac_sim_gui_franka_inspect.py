"""Read Franka joint metadata/state in Isaac Sim's Script Editor (M0-C).

Prerequisites: M0-C scene loaded; press Play FIRST.
Runs read-only inspection. No joint targets or hardware commands.
"""

import omni.timeline
from isaacsim.core.experimental.prims import Articulation

path = "/World/RFP_Franka/Arm"
if not omni.timeline.get_timeline_interface().is_playing():
    raise RuntimeError("Press Play in Isaac Sim, then rerun this script.")

try:
    arm = Articulation(path)
    print("RFP Franka articulation:", path)
    print("DOF count:", arm.num_dofs)
    print("Joint names:", arm.dof_names)
    print("Joint positions:", arm.get_dof_positions())
except Exception as exc:
    raise RuntimeError(
        "The asset may still be loading or the articulation/physics may not be "
        "initialized. Verify that Franka appears in Stage and Play is active."
    ) from exc

print("Joint inspection complete (no commands were sent).")
