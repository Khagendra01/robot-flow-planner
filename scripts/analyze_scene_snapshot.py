"""Inspect joint motion and *nonphysical* frame proxies in an M0-D snapshot.

Run with the project Python venv, NOT the Isaac Sim Script Editor.
"""

import argparse
import json
from pathlib import Path

from robot_flow_planner.scene_analysis import analyze_scene_snapshot
from robot_flow_planner.scene_state import load_scene_snapshot


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", nargs="?", type=Path,
                        help="JSON path (defaults to latest local exported snapshot)")
    parser.add_argument("--stationary-threshold", type=float, default=0.05,
                        help="Arm-joint speed data-quality threshold in rad/s (default 0.05)")
    parser.add_argument("--json", action="store_true", help="Output report as JSON")
    args = parser.parse_args()
    path = args.snapshot
    if path is None:
        directory = (Path.home() / "Desktop" / "robot-flow-planner"
                     / "outputs" / "scene_snapshots")
        candidates = sorted(directory.glob("snapshot_*.json"))
        if not candidates:
            parser.error(f"No snapshots in {directory}; provide a snapshot path")
        path = candidates[-1]

    report = analyze_scene_snapshot(
        load_scene_snapshot(path), stationary_threshold_rad_s=args.stationary_threshold
    )
    if args.json:
        print(json.dumps(report, indent=2))
        return

    motion = report["motion"]
    proxy = report["proxy_geometry"]
    print("Snapshot:", path)
    print("Simulation time: %.3f s" % report["sim_time_s"])
    print("Peak arm-joint speed: %s, %.4f rad/s" %
          (motion["peak_arm_joint"], motion["peak_arm_joint_speed_rad_s"]))
    print("Within %.3f rad/s stationary heuristic: %s" %
          (motion["stationary_threshold_rad_s"], motion["stationary_by_threshold"]))
    for label, key in [
        ("Nearest link-frame point to box", "minimum_link_frame_to_box_distance_m"),
        ("Nearest frame-to-frame segment to box", "minimum_frame_segment_to_box_distance_m"),
    ]:
        entry = proxy[key]
        print("%s: %.4f m (%s)" % (label, entry["distance_m"], entry["obstacle"]))
    print("IMPORTANT:", proxy["warning"])
    print("This report must NEVER label a robot trajectory as collision-free.")


if __name__ == "__main__":
    main()
