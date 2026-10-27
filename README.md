# Robot Flow Planner

Simulation-first research into joint-conditioned, collision-aware trajectory generation for a fixed 6/7-DoF robot arm. The long-term architecture combines object-centric perception, robot geometry, cross-attention, flow matching, and independent safety validation. **This code is research software and is not approved for commanding real hardware.**

## Current implementation (M0)

- Exact Euclidean distance between a line segment and a 3D oriented box, using piecewise quadratic minimization.
- Capsule-link vs oriented-box clearance/collision with configurable margin.
- Unit tests, a deterministic demo, and an environment diagnostic script.
- No Isaac Lab scene, learned planner, continuous swept-volume checks, controller, or camera integration yet.

The geometry primitives are CPU-only and intentionally independent of simulator/PyTorch versions. A single primitive collision query is **not** a guarantee that a robot trajectory is collision-free.

## Run the M0 checks on Windows PowerShell

~~~powershell
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e ".[dev]"
python -m pytest -q
python scripts/demo_geometry.py
python scripts/check_environment.py
~~~

For Linux, activate with source .venv/bin/activate. Python 3.12 aligns with Isaac Lab 3.0, although this standalone geometry package can run on Python 3.10+.

## Planned modules

1. **M0:** 7-DoF Franka scene + robot kinematics, collision geometry and baseline unit tests.
2. **M1:** Isaac Lab 3.0 / Isaac Sim 6.2 integration, configurable boxes, and geometric teacher trajectories.
3. **M2:** Structured scene-state dataset, deterministic behavior cloning, independent evaluation.
4. **M3:** Link-to-obstacle cross-attention and conditional flow-matching candidate trajectories.
5. **M4:** Time-aware dynamic obstacle prediction, receding-horizon replanning, continuously checked clearance and fail-safe behavior.
6. **M5:** Calibrated TPP camera fusion + SAM tracking, comparison to privileged sim states.
7. **M6:** Hardware integration only after validated safety review and protective stopping architecture.

See [architecture and compatibility notes](docs/architecture.md).

## Versioning strategy

Keep the geometry core in a lightweight Python environment. Use a **separate environment** for Isaac Lab 3.0 / Isaac Sim 6.2 (Python 3.12) and, if needed, a separate Linux environment or process for cuRobo CUDA extension builds. Do not blindly install an older cuRobo requirements file into the Isaac Lab environment.

- Isaac Lab 3.0: https://docs.nvidia.com/isaaclab/3.0/isaac-lab/installation
- cuRobo: https://curobo.org/get_started/1_install_instructions.html

## Research/evaluation principles

Track goal success, collisions, minimum clearance, runtime latency p50/p95/p99, dynamics feasibility, stopping ability and failures on held-out scenes. Train only from information available at inference time; simulator future trajectories are for supervision/evaluation, not as policy inputs. Preserve failed/blocked episodes for training stop/reject behavior.
