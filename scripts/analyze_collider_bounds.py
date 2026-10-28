"""Analyze an exported M0-F Franka collider envelope snapshot.

This is a coarse obstacle-screening tool, NOT a PhysX collision oracle or
trajectory safety validator. In particular, a separating-axis gap is NOT an
exact Euclidean clearance measurement.
"""

import argparse
import json
from pathlib import Path

from robot_flow_planner.collider_bounds import analyze_collider_envelopes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", type=Path, help="Snapshot JSON path")
    parser.add_argument("--padding-m", type=float, default=0.01,
                        help="Research proxy inflation per side in meters (default 0.01)")
    args = parser.parse_args()
    path = args.path
    if path is None:
        directory = (Path.home() / "Desktop" / "robot-flow-planner"
                     / "outputs" / "collider_bounds")
        candidates = sorted(directory.glob("franka_bounds_*.json"))
        if not candidates:
            parser.error(f"No collider snapshots found in {directory}")
        path = candidates[-1]
    data = json.loads(path.read_text(encoding="utf-8"))
    try:
        report = analyze_collider_envelopes(data, padding_m=args.padding_m)
    except ValueError as exc:
        raise SystemExit(
            f"Rejected collider bounds export: {exc}\n"
            "Run the corrected M0-F v2 exporter in Isaac Sim after git pull. "
            "Do not train on older geometry exports."
        ) from exc
    print("Source:", path)
    print("Authored collider envelopes:", len(data["colliders"]))
    print("Obstacles:", len(data["obstacles"]))
    print("OBB pairs examined:", report["tested_pairs"])
    print("Envelope pairs overlapping/touching:",
          len(report["possibly_overlapping_pairs"]))
    print("Smallest maximum separating-axis gap (m):",
          f"{report['max_projected_gap_min_m']:.6f}")
    for item in report["possibly_overlapping_pairs"][:30]:
        print("  Possibly overlapping:", item["link"], "with", item["obstacle"])
    if len(report["possibly_overlapping_pairs"]) > 30:
        print("  ... additional pairs omitted")
    print("NOT a physical collision label:", report["limitations"])


if __name__ == "__main__":
    main()
