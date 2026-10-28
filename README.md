# Robot Flow Planner

Simulation-first research into joint-conditioned, collision-aware trajectory generation for a fixed 6/7-DoF robot arm. The long-term architecture combines object-centric perception, robot geometry, cross-attention, flow matching, and independent safety validation. **This code is research software and is not approved for commanding real hardware.**

## Current implementation (M0)

- Exact Euclidean distance between a line segment and a 3D oriented box, using piecewise quadratic minimization.
- Capsule-link vs oriented-box clearance/collision with configurable margin.
- Unit tests, a deterministic demo, and an environment diagnostic script.
- No Isaac Lab scene, learned planner, continuous swept-volume checks, controller, or camera integration yet.

The geometry primitives are CPU-only and intentionally independent of simulator/PyTorch versions. A single primitive collision query is **not** a guarantee that a robot trajectory is collision-free.

## Run M0 on Windows (Command Prompt / cmd.exe)

Use **activate.bat in Command Prompt**, not Activate.ps1 (which belongs to PowerShell). The `(.venv)` prefix should appear in the prompt; `python` must resolve inside `.venv`, not the global Python installation.

~~~bat
cd %USERPROFILE%\Desktop\robot-flow-planner
py -3.12 -m venv .venv
.venv\Scripts\activate.bat
where python
python -c "import sys; print(sys.executable, 'venv active:', sys.prefix != sys.base_prefix)"
python -m pip install -e ".[dev]"
python -m pytest -q
python scripts/demo_geometry.py
python scripts/check_environment.py
~~~

For **PowerShell**, use `.\.venv\Scripts\Activate.ps1` instead of `activate.bat`. For Linux, use `source .venv/bin/activate`. Python 3.12 is preferred to match Isaac Lab 3.0.

## Enable CUDA PyTorch for the RTX 5090

`nvidia-smi` reports the *driver-supported CUDA version*, not the version of CUDA available in your Python package. A version ending in `+cpu` has no GPU support. Inside the **activated** project environment, install NVIDIA Isaac Lab 3.0's pinned CUDA 13.0 versions:

~~~bat
python -m pip install torch==2.11.0 torchvision==0.26.0 --index-url https://download.pytorch.org/whl/cu130
python scripts/check_environment.py
python scripts/verify_cuda.py
~~~

`verify_cuda.py` runs an actual GPU matrix multiplication, not just a driver check. The official wheel index includes Windows Python 3.12 builds. A separate system-wide CUDA Toolkit is not necessary for ordinary prebuilt PyTorch wheels. If packages install under `AppData\Local\Programs\Python\Python312\Lib\site-packages`, the intended venv is likely **not** active.

**Keep Isaac Lab/Isaac Sim in a separate environment** to avoid dependency conflicts. Isaac Sim 6.2 requires at least 32 GB of system RAM and 50 GB of SSD storage; confirm local availability before installing it.

See NVIDIA's [Isaac Lab 3.0 installation instructions](https://docs.nvidia.com/isaaclab/3.0/isaac-lab/installation).

## Planned modules

1. **M0:** 7-DoF Franka scene + robot kinematics, collision geometry and baseline unit tests.
2. **M1:** Isaac Lab 3.0 / Isaac Sim 6.2 integration, configurable boxes, and geometric teacher trajectories.
3. **M2:** Structured scene-state dataset, deterministic behavior cloning, independent evaluation.
4. **M3:** Link-to-obstacle cross-attention and conditional flow-matching candidate trajectories.
5. **M4:** Time-aware dynamic obstacle prediction, receding-horizon replanning, continuously checked clearance and fail-safe behavior.
6. **M5:** Calibrated TPP camera fusion + SAM tracking, comparison to privileged sim states.
7. **M6:** Hardware integration only after validated safety review and protective stopping architecture.

See [architecture and compatibility notes](docs/architecture.md).

Windows workstation checkpoint, GitHub DNS troubleshooting, and the separate Isaac Lab 3.0 install route: [Windows next steps](docs/windows-next-steps.md).

If Isaac Sim has opened but Windows admin prompts or read-only asset icons are confusing, run the asset-free [GUI/physics smoke test](docs/isaac-sim-gui-smoke-test.md) with [this Script Editor example](scripts/isaac_sim_gui_smoke.py).

Once the falling-cube physics test is confirmed, follow the [Franka Panda GUI scene and joint-inspection guide](docs/franka-gui-test.md).

After the Franka reports 9 DOFs, use the [M0-D scene snapshot instructions](docs/live-scene-snapshot.md) and [read-only Script Editor exporter](scripts/isaac_sim_gui_scene_snapshot.py) to save joint states, 8 live link frames, and 2 obstacle boxes. The [offline validator](scripts/validate_scene_snapshot.py) checks schema and units; snapshots remain local in ignored `outputs/`.

After snapshot validation, run [M0-E scene analysis and collision-prim discovery](docs/m0-e-scene-analysis.md) using [offline frame diagnostics](scripts/analyze_scene_snapshot.py) and the [Isaac Sim collision inventory](scripts/isaac_sim_gui_collision_inventory.py). Frame distances are NOT robot-body collision clearances.

With the 11 collider meshes identified, [M0-F mesh-envelope export](docs/m0-f-collider-bounds.md) derives per-link boxes from actual USD vertices with [this Script Editor exporter](scripts/isaac_sim_gui_collider_bounds.py), then [screens overlaps offline](scripts/analyze_collider_bounds.py). Bounding-box screening is **not** a trajectory-safety certificate.

**M0-F correction:** The first real export exposed a mesh-as-link transform bug (multi-meter link sizes). The fixed exporter writes schema `rfp.collider_bounds.v2`, and the analyzer rejects v1. See the correction notes in [M0-F instructions](docs/m0-f-collider-bounds.md); re-export before using any bounds.

**Preferred workflow: ONE RUN.** With the Franka scene loaded and Play active, execute the [single-run collector](scripts/isaac_sim_gui_collect_all.py) once in Isaac Sim's Script Editor. It automatically collects joint/link data, collision inventory, real mesh envelopes, and an offline-style proxy overlap screen into **one JSON file**. See [one-run instructions](docs/one-run-collection.md). The older scripts remain available for debugging individual failures only.

**M1-A (time sequence):** The original bundle validated one simulation timestamp, so it cannot measure trajectory motion. For 60 actual time-advancing observations **in a single Script Editor run**, use [M1-A one-run sequence collector](scripts/isaac_sim_gui_collect_sequence.py) and its [guide](docs/m1-a-one-run-sequences.md). The script automatically gathers a fresh full baseline and embeds all 60 frames in one JSON. No joint commands or physical collision labels.

**Performance pivot:** The manually operated Isaac Sim GUI / serial NumPy geometry loop was a bring-up workflow, not the scalable planner. The new [batched Torch OBB module](src/robot_flow_planner/torch_geometry.py) and [one-command CPU/GPU benchmark](scripts/benchmark_torch_geometry.py) measure actual RTX acceleration. See [headless vectorized M1-B plan](docs/performance-reset.md). No GPU or Isaac Lab speedup claimed until measured.

**M1-A verified on workstation:** 60 distinct 60 Hz observations (0.983 s of simulation), 11 robot-body poses and two static obstacle boxes per frame; 1,320 OBB proxy tests, no flagged proxy overlap. The robot moved substantially, but this was a passive rollout, NOT a goal-conditioned or physically collision-labeled demonstration. See [verified M1-A results](docs/m1-a-verified-results.md).

With v2 bounds passing offline checks, use the [M0-G Isaac Sim overlay guide](docs/m0-g-visualize-colliders.md) and [visualization script](scripts/isaac_sim_gui_visualize_collider_bounds.py) to inspect whether 11 link-aligned visual boxes enclose the actual robot; this does not validate PhysX collision margins.

## Versioning strategy

Keep the geometry core in a lightweight Python environment. Use a **separate environment** for Isaac Lab 3.0 / Isaac Sim 6.2 (Python 3.12) and, if needed, a separate Linux environment or process for cuRobo CUDA extension builds. Do not blindly install an older cuRobo requirements file into the Isaac Lab environment.

- Isaac Lab 3.0: https://docs.nvidia.com/isaaclab/3.0/isaac-lab/installation
- cuRobo: https://curobo.org/get_started/1_install_instructions.html

## Research/evaluation principles

Track goal success, collisions, minimum clearance, runtime latency p50/p95/p99, dynamics feasibility, stopping ability and failures on held-out scenes. Train only from information available at inference time; simulator future trajectories are for supervision/evaluation, not as policy inputs. Preserve failed/blocked episodes for training stop/reject behavior.
