"""Validate the latest M0-D snapshot exported by Isaac Sim.

Run with project .venv (NOT inside the Isaac Sim Script Editor):
    python scripts/validate_scene_snapshot.py
"""
from pathlib import Path

from robot_flow_planner.scene_state import load_scene_snapshot, validate_scene_snapshot


def main():
    directory = (
        Path.home() / "Desktop" / "robot-flow-planner"
        / "outputs" / "scene_snapshots"
    )
    paths = sorted(directory.glob("snapshot_*.json"))
    if not paths:
        raise SystemExit(
            f"No snapshots found at {directory}. "
            "Run isaac_sim_gui_scene_snapshot.py from the Isaac Sim Script Editor first."
        )
    latest = paths[-1]
    summary = validate_scene_snapshot(load_scene_snapshot(latest))
    print("Validated snapshot:", latest)
    for name, value in summary.items():
        print(f"{name}: {value}")
    print("Snapshot schema: PASS (not a trajectory collision check)")


if __name__ == "__main__":
    main()
