"""Optional offline check of the latest single-run Franka collection.

Run in the project .venv (NOT in Isaac Sim):
    python scripts/inspect_collection.py
"""

import argparse
from pathlib import Path

from robot_flow_planner.collection_bundle import load_collection_bundle, validate_collection_bundle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", type=Path, help="Path to collection bundle JSON")
    args = parser.parse_args()
    p = args.path
    if p is None:
        directory = Path.home() / "Desktop" / "robot-flow-planner" / "outputs" / "collections"
        matches = sorted(directory.glob("franka_collection_*.json"))
        if not matches:
            parser.error(f"No collection JSON found in {directory}")
        p = matches[-1]
    info = validate_collection_bundle(load_collection_bundle(p))
    print("Validated combined collection:", p)
    for k, v in info.items():
        print(f"{k}: {v}")
    print("DATA VALIDATION PASS. This is NOT a collision-safe trajectory label.")


if __name__ == "__main__":
    main()
