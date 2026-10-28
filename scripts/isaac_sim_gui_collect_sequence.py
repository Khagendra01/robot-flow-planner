"""M1-A: ONE-RUN Franka sequence collector for Isaac Sim 6.2 Script Editor.

One script invocation:
1. Collect full validated M0-D/E/F geometry baseline via collect_all.py.
2. Step the REAL Isaac timeline asynchronously across distinct time values.
3. Capture 60 chronological frames of 9 DOFs, 11 body poses, and obstacle poses.
4. At each frame, run the 22-pair conservative OBB diagnostic.
5. Save ONE JSON under outputs/sequences/ (baseline embedded).

Run from Isaac Sim's Script Editor with the M0-C Franka scene loaded and
Play active:

    from pathlib import Path
    p = Path.home() / "Desktop/robot-flow-planner/scripts/isaac_sim_gui_collect_sequence.py"
    exec(compile(p.read_text(encoding="utf-8"), str(p), "exec"))

The capture runs asynchronously so Kit can advance. Watch the Script Editor /
Console for the final "SEQUENCE COMPLETE" path. There are no joint commands,
no physical collision oracle, and NO collision-free labels.

This is a *passive* simulation rollout, not purposeful training demonstrations.
"""

import asyncio
from datetime import datetime, timezone
import json
from pathlib import Path
import runpy
import sys
import traceback

import numpy as np
import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.core.experimental.prims import Articulation, XformPrim

PROJECT = Path.home() / "Desktop" / "robot-flow-planner"
COLLECTOR = PROJECT / "scripts" / "isaac_sim_gui_collect_all.py"
SRC = PROJECT / "src"
ARM_PATH = "/World/RFP_Franka/Arm"
NUM_FRAMES = 60
MAX_UPDATES = 1000


def finite_numpy(value):
    if hasattr(value, "detach"):
        value = value.detach().cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    arr = np.asarray(value, dtype=float)
    if not np.all(np.isfinite(arr)):
        raise RuntimeError("Non-finite simulator state")
    return arr


def pose(path):
    p, q = XformPrim(path).get_world_poses()
    p = finite_numpy(p).reshape(-1, 3)
    q = finite_numpy(q).reshape(-1, 4)
    if len(p) != 1 or len(q) != 1:
        raise RuntimeError("Expected one runtime pose at " + path)
    return p[0].tolist(), q[0].tolist()


async def collect_sequence():
    stage = omni.usd.get_context().get_stage()
    if stage is None or not stage.GetPrimAtPath(ARM_PATH).IsValid():
        raise RuntimeError("Load the M0-C Franka scene before collection")
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        raise RuntimeError("Press Play before invoking the M1-A collector")
    if not COLLECTOR.is_file():
        raise RuntimeError("Missing unified collector: " + str(COLLECTOR))

    print("=== RFP ONE-RUN TIME SEQUENCE START ===")
    # Baseline exports run synchronously, while physics time is not advanced
    # between the individual captures. That limitation is reported explicitly.
    baseline_run = runpy.run_path(str(COLLECTOR), run_name="rfp_sequence_baseline")
    baseline_path = baseline_run["collection_file"]
    baseline = json.loads(Path(baseline_path).read_text(encoding="utf-8"))
    if str(SRC) not in sys.path:
        sys.path.insert(0, str(SRC))
    from robot_flow_planner.collection_bundle import validate_collection_bundle
    from robot_flow_planner.collider_bounds import analyze_collider_envelopes
    from robot_flow_planner.sequence_bundle import validate_sequence_bundle
    validate_collection_bundle(baseline)

    local_colliders = baseline["collider_bounds"]["colliders"]
    bodies = [(c["link_owner"], c["link_prim_path"]) for c in local_colliders]
    boxes = baseline["collider_bounds"]["obstacles"]
    box_names = [o["name"] for o in boxes]
    box_definitions = {o["name"]: o for o in boxes}
    robot = Articulation(ARM_PATH)

    # PhysX time advances only when the Kit event loop gets a tick. This
    # collector awaits REAL app updates, instead of making repeated synchronous
    # calls that misleadingly return identical timestamps.
    frames = []
    last_t = float(timeline.get_current_time())
    updates = 0
    first_t = None
    try:
        while len(frames) < NUM_FRAMES and updates < MAX_UPDATES:
            await omni.kit.app.get_app().next_update_async()
            updates += 1
            if not timeline.is_playing():
                raise RuntimeError("Timeline stopped during sequence capture")
            t = float(timeline.get_current_time())
            if not np.isfinite(t):
                raise RuntimeError("Non-finite simulation timestamp")
            if t < last_t - 1e-7:
                raise RuntimeError("Simulation time jumped backward during capture")
            if t <= last_t + 1e-7:
                continue  # no physics time advance yet
            last_t = t
            if first_t is None:
                first_t = t

            names = list(robot.dof_names)
            q = finite_numpy(robot.get_dof_positions()).reshape(-1).tolist()
            dq = finite_numpy(robot.get_dof_velocities()).reshape(-1).tolist()
            if len(names) != 9 or len(q) != 9 or len(dq) != 9:
                raise RuntimeError("Expected 9 Franka joints in every frame")
            frame_bodies = []
            for name, link_path in bodies:
                p, quat = pose(link_path)
                frame_bodies.append({
                    "name": name,
                    "prim_path": link_path,
                    "position_m": p,
                    "orientation_wxyz": quat,
                })

            frame_obstacles = []
            for name in box_names:
                path = "/World/RFP_Franka/" + name
                p, quat = pose(path)
                frame_obstacles.append({
                    "name": name,
                    "position_m": p,
                    "orientation_wxyz": quat,
                    "dimensions_m": box_definitions[name]["dimensions_m"],
                    # 6.2 experiment's authored box sizes are static. If
                    # obstacles rescale during play, extend this schema first.
                })

            dynamic_bounds = dict(baseline["collider_bounds"])
            dynamic_bounds["joint_positions"] = q
            dynamic_bounds["joint_velocities"] = dq
            dynamic_bounds["colliders"] = [
                {
                    **c,
                    "link_world_position_m": frame_bodies[i]["position_m"],
                    "link_world_orientation_wxyz": frame_bodies[i]["orientation_wxyz"],
                }
                for i, c in enumerate(local_colliders)
            ]
            dynamic_bounds["obstacles"] = frame_obstacles
            report = analyze_collider_envelopes(dynamic_bounds, padding_m=0.01)

            frames.append({
                "simulation_time_s": t,
                "joint_positions": q,
                "joint_velocities": dq,
                "body_poses": frame_bodies,
                "obstacles": frame_obstacles,
                "proxy_obb_pairs_tested": report["tested_pairs"],
                "proxy_overlap_pair_count": len(report["possibly_overlapping_pairs"]),
                "minimum_max_projected_gap_m": report["max_projected_gap_min_m"],
                "valid_as_collision_free_label": False,
            })
            if len(frames) % 15 == 0:
                print(f"M1-A frames: {len(frames)}/{NUM_FRAMES}, sim time: {t:.3f}s")

        if len(frames) != NUM_FRAMES:
            raise RuntimeError(
                f"Only {len(frames)}/{NUM_FRAMES} distinct physics timestamps "
                f"after {updates} Kit updates; check timeline/physics."
            )

        times = [f["simulation_time_s"] for f in frames]
        dts = np.diff(times)
        if np.any(dts <= 0):
            raise RuntimeError("Sequence timestamps are not strictly increasing")
        peak_speed = max(
            max(abs(x) for x in f["joint_velocities"][:7])
            for f in frames
        )
        sequence = {
            "schema_version": "rfp.sequence_bundle.v1",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "baseline": baseline,
            "rollout_kind": "passive_simulation_observation_no_joint_commands",
            "sampling": {
                "requested_frames": NUM_FRAMES,
                "recorded_frames": len(frames),
                "kit_updates_used": updates,
                "first_simulation_time_s": times[0],
                "last_simulation_time_s": times[-1],
                "mean_time_step_s": float(np.mean(dts)),
                "min_time_step_s": float(np.min(dts)),
                "max_time_step_s": float(np.max(dts)),
                "peak_arm_joint_speed_rad_s": float(peak_speed),
                "atomic_physics_frame_sampling_proven": False,
            },
            "frames": frames,
            "collision_status": "NOT_VALIDATED",
            "valid_as_collision_free_training_labels": False,
            "warning": (
                "This is a passive observation rollout. No commanded goal, "
                "teacher planner, cooked PhysX contact validation, self-collision "
                "or continuous-time safety check is supplied. OBB overlap is "
                "only conservative geometry screening."
            ),
        }
        validate_sequence_bundle(sequence)

        folder = PROJECT / "outputs" / "sequences"
        folder.mkdir(parents=True, exist_ok=True)
        dest = folder / (
            "franka_sequence_"
            + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
            + ".json"
        )
        partial = dest.with_suffix(".tmp")
        partial.write_text(json.dumps(sequence, indent=2), encoding="utf-8")
        partial.replace(dest)
        print("=== RFP SEQUENCE COMPLETE ===")
        print("ONE FILE:", dest)
        print("Frames:", len(frames))
        print("Simulation elapsed (s):", round(times[-1] - times[0], 4))
        print("Peak arm joint speed (rad/s):", round(peak_speed, 4))
        print("Frames with any proxy overlap:",
              sum(f["proxy_overlap_pair_count"] > 0 for f in frames))
        print("Not collision-safe training demonstrations.")
        print("Upload this SINGLE sequence file.")
    finally:
        # Leave the scene in a stable paused state, even after an exception.
        if timeline.is_playing():
            timeline.pause()


def report_task_completion(task):
    try:
        task.result()
    except Exception:
        print("=== RFP SEQUENCE COLLECTION FAILED ===")
        traceback.print_exc()


# Keep a single active collector in Script Editor's module namespace to
# avoid launching concurrent rollouts if the user presses Run twice.
import builtins
old_task = getattr(builtins, "_rfp_sequence_collection_task", None)
if old_task is not None and not old_task.done():
    raise RuntimeError(
        "An RFP sequence collection is already running. Check Console output."
    )
rfp_sequence_task = asyncio.ensure_future(collect_sequence())
builtins._rfp_sequence_collection_task = rfp_sequence_task
rfp_sequence_task.add_done_callback(report_task_completion)
print("M1-A async sequence collector scheduled; watch Console for ONE FILE.")
