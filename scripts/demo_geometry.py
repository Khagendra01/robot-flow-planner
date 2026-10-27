"""Run the first deterministic collision example without Isaac Sim."""

import numpy as np
from robot_flow_planner import Capsule, OrientedBox, capsule_box_clearance


def main() -> None:
    box = OrientedBox(center=[0.5, 0, 0], size=[0.3, 0.4, 0.5], rotation=np.eye(3))
    for name, capsule in [
        ("near", Capsule([0, -0.5, 0], [0, 0.5, 0], 0.05)),
        ("contact", Capsule([0.5, -0.5, 0], [0.5, 0.5, 0], 0.05)),
    ]:
        clearance = capsule_box_clearance(capsule, box)
        print(f"{name:>8}: clearance={clearance:+.3f} m; collision={clearance <= 0}")


if __name__ == "__main__":
    main()
