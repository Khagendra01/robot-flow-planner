"""Robot Flow Planner: simulation-first collision-aware motion research."""

from .geometry import Capsule, OrientedBox, capsule_box_clearance, capsule_box_collision

__all__ = ["Capsule", "OrientedBox", "capsule_box_clearance", "capsule_box_collision"]
