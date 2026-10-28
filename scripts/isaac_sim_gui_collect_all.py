"""Collect ALL verified M0-D/E/F Franka observations in ONE Isaac Sim run.

Execute this single file using Isaac Sim 6.2's Window > Script Editor:
    from pathlib import Path
    p = Path.home() / "Desktop/robot-flow-planner/scripts/isaac_sim_gui_collect_all.py"
    exec(compile(p.read_text(encoding="utf-8"), str(p), "exec"))

PREREQUISITES:
- The M0-C scene /World/RFP_Franka/Arm exists.
- Isaac Sim's timeline is PLAYING (press Play once).
- All latest project files have been pulled from GitHub.

This orchestrates the previously VERIFIED individual exporters, not a new
unverified robot control API. It saves their sidecars as before AND a single
consolidated JSON bundle, then runs the project's offline geometry diagnostics
inside Isaac Sim (read-only NumPy). No joints are commanded or physics
colliders added. A bundle is a DATA RECORD, NEVER a collision-free label.

One run collects:
  * 9 mixed-unit joint states + 8 runtime link frames
  * the 11 authored USD collision-mesh inventory
  * 11 link-local collision mesh OBB envelopes + 11 live link poses
  * the two obstacle oriented boxes
  * source file paths, time drift, stationarity heuristic, OBB overlap screen

Snapshots are taken sequentially, not atomically on one synchronized physics
tick. If the robot moves between steps the manifest will warn about drift.
"""

from datetime import datetime, timezone
import json
from pathlib import Path
import runpy
import sys

import numpy as np
import omni.timeline
import omni.usd

ROOT = "/World/RFP_Franka"
ARM_PATH = ROOT + "/Arm"
PROJECT = Path.home() / "Desktop" / "robot-flow-planner"
SCRIPTS = PROJECT / "scripts"
SRC = PROJECT / "src"


def _execute_exporter(filename):
    script = SCRIPTS / filename
    if not script.is_file():
        raise RuntimeError(
            f"Required exporter missing: {script}. Run git pull origin main."
        )
    # Run the existing Script Editor exporter in an isolated Python namespace.
    # All three have already produced real results on this workstation.
    return runpy.run_path(str(script), run_name=f"rfp_collection_{script.stem}")


def _load_and_check_saved(output, data):
    path = Path(output).resolve()
    if not path.is_file():
        raise RuntimeError(f"Exporter did not write its JSON: {path}")
    saved = json.loads(path.read_text(encoding="utf-8"))
    if saved != data:
        raise RuntimeError(f"Exporter JSON and memory data disagree: {path}")
    return str(path)


def collect_all():
    stage = omni.usd.get_context().get_stage()
    if stage is None or not stage.GetPrimAtPath(ARM_PATH).IsValid():
        raise RuntimeError(
            "Franka scene missing. Load the M0-C scene so that "
            f"{ARM_PATH} exists before running the one-click collector."
        )
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        raise RuntimeError(
            "Press Play once, then run this collector; runtime articulation "
            "and collision transforms require the active simulation."
        )
    if not SRC.is_dir():
        raise RuntimeError("Project src directory missing: " + str(SRC))

    print("=== RFP ONE-RUN COLLECTION START ===")
    scene_run = _execute_exporter("isaac_sim_gui_scene_snapshot.py")
    inventory_run = _execute_exporter("isaac_sim_gui_collision_inventory.py")
    bounds_run = _execute_exporter("isaac_sim_gui_collider_bounds.py")

    scene = scene_run["snapshot"]
    inventory = inventory_run["report"]
    bounds = bounds_run["data"]
    sidecars = {
        "scene_snapshot": _load_and_check_saved(scene_run["path"], scene),
        "collision_inventory": _load_and_check_saved(inventory_run["path"], inventory),
        "collider_bounds": _load_and_check_saved(bounds_run["dest"], bounds),
    }

    # Reuse our independently unit-tested Python validators and SAT screen.
    if str(SRC) not in sys.path:
        sys.path.insert(0, str(SRC))
    from robot_flow_planner.scene_state import validate_scene_snapshot
    from robot_flow_planner.collider_bounds import (
        validate_collider_bounds_snapshot,
        analyze_collider_envelopes,
    )

    scene_check = validate_scene_snapshot(scene)
    bounds_check = validate_collider_bounds_snapshot(bounds)
    if inventory.get("schema_version") != "rfp.collision_inventory.v1":
        raise RuntimeError("Unexpected collision-inventory schema")
    if inventory.get("collision_prim_count") != 11:
        raise RuntimeError("Need exactly 11 inventory collision meshes")
    inv_shapes = inventory.get("collision_prims", [])
    if not isinstance(inv_shapes, list) or len(inv_shapes) != 11:
        raise RuntimeError("Collision inventory prim list incomplete")
    inv_paths = {p["prim_path"] for p in inv_shapes}
    bound_paths = {p["prim_path"] for p in bounds["colliders"]}
    if len(inv_paths) != 11 or inv_paths != bound_paths:
        raise RuntimeError("Inventoried meshes don't match exported envelopes")
    if any(p.get("collision_enabled_authored") is False for p in inv_shapes):
        raise RuntimeError("Disabled collision mesh found in inventory")
    if {p.get("name") for p in bounds["obstacles"]} != {
        p.get("name") for p in scene["obstacles"]
    }:
        raise RuntimeError("Mismatched obstacle names between observations")
    if bounds.get("robot_prim_path") != scene["robot"]["prim_path"]:
        raise RuntimeError("The two observations reference different robots")

    q0 = np.asarray(scene["robot"]["joint_positions"], dtype=float)
    q1 = np.asarray(bounds["joint_positions"], dtype=float)
    drift = float(np.max(np.abs(q1 - q0)))
    max_arm_speed = float(max(
        np.max(np.abs(scene["robot"]["joint_velocities"][:7])),
        np.max(np.abs(bounds["joint_velocities"][:7])),
    ))
    report = analyze_collider_envelopes(bounds, padding_m=0.01)

    checks = {
        "scene_schema": scene_check,
        "collider_schema": bounds_check,
        "inventory_matches_bounds": True,
        "obstacle_names_match": True,
        "max_joint_position_drift_native_units": drift,
        "max_absolute_arm_speed_rad_s": max_arm_speed,
        "stationary_by_0_05_rad_s_heuristic": bool(max_arm_speed <= 0.05),
        "pairwise_obb_screen": report,
        "valid_as_collision_free_training_label": False,
        "warning": (
            "DO NOT use this as physical or continuous-time collision ground truth. "
            "Authored-mesh OBBs are not cooked PhysX shapes; sequential snapshots "
            "can be non-atomic, and no self-collision or swept check was performed."
        ),
    }
    bundle = {
        "schema_version": "rfp.collection_bundle.v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source": "Isaac Sim Script Editor single-run collection",
        "stage_root": ROOT,
        "source_files": sidecars,
        "scene_snapshot": scene,
        "collision_inventory": inventory,
        "collider_bounds": bounds,
        "checks": checks,
    }

    folder = PROJECT / "outputs" / "collections"
    folder.mkdir(parents=True, exist_ok=True)
    destination = folder / (
        "franka_collection_"
        + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        + ".json"
    )
    # Use a temporary file then replace to avoid a half-written bundle.
    partial = destination.with_suffix(".tmp")
    partial.write_text(json.dumps(bundle, indent=2), encoding="utf-8")
    partial.replace(destination)

    print("=== RFP ONE-RUN COLLECTION COMPLETE ===")
    print("COMBINED FILE:", destination)
    print("Joints:", scene_check["joint_count"],
          "(7 arm + 2 gripper)")
    print("Link frames:", scene_check["link_frame_count"])
    print("Collision meshes and envelopes:", bounds_check["collider_count"])
    print("Obstacle boxes:", bounds_check["obstacle_count"])
    print("OBB comparisons:", report["tested_pairs"])
    print("Proxy boxes touching/overlapping:",
          len(report["possibly_overlapping_pairs"]))
    print("Maximum joint-state drift between captures (mixed rad/m):", drift)
    print("Peak arm-joint speed during captures (rad/s):", max_arm_speed)
    if max_arm_speed > 0.05 or drift > 0.01:
        print("WARNING: Robot moved during collection. This is NOT a "
              "time-synchronized stationary observation.")
    print("NO robot commands issued. NO collision-free label generated.")
    print("Upload just the COMBINED FILE for the next development step.")
    return destination


# One Script Editor Run -> all three exports + validation + one combined JSON.
collection_file = collect_all()
