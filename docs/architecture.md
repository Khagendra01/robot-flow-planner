# Architecture, hardware and milestones (2026-10-27)

## Available hardware (user-provided snapshot)

- Windows NT build 26100 (Windows 11 family; the console prints 10.0).
- NVIDIA RTX 5090, 32,607 MiB VRAM, WDDM driver 581.95.
- nvidia-smi reports CUDA 13.0 as the driver-supported API level; this **does not** establish that a CUDA toolkit or CUDA-enabled PyTorch is installed.
- CPU model, system RAM, disk space, installed Python and local Isaac Sim status are **unknown**. Run python scripts/check_environment.py and check those before installing the large simulator stack.

## Compatibility decision

Isaac Lab **3.0** is the target, with optional Isaac Sim **6.2** and Python **3.12**. Current NVIDIA documentation lists Windows 11 as supported and the default PyTorch CUDA 13 stack's minimum Windows driver as 580.88. The user's 581.95 driver clears the published minimum, though this is not an end-to-end compatibility test.

The cuRobo public install guide lists Windows as experimental. Treat it as an optional teacher process, not a hard dependency of the initial geometry or simulator code. If CUDA extension builds fail on native Windows, try a separately managed Ubuntu machine or supported Linux environment; WSL2 compatibility must be verified for the specific graphics/simulator workflow before promising it will run there.

Official references:
- https://docs.nvidia.com/isaaclab/3.0/isaac-lab/installation
- https://curobo.org/get_started/1_install_instructions.html

## Data contract (planned; not yet implemented)

- Robot state: q [7], dq [7], link poses, timestamp and robot definition/version.
- Goal: end-effector position [3], quaternion [4], tolerances.
- Obstacles: OBB position [N,3], rotation [N,3,3], size [N,3], velocity [N,3], uncertainty and timestamp.
- Trajectories: joint_positions [H,7] with per-waypoint timestamps, feasibility label, clearance, teacher metadata.
- Perception extensions: camera intrinsics/extrinsics, depth, instance masks, tracking IDs.
- Do not leak simulator-only future obstacle positions into learned-policy observations.

## Model (planned; not yet implemented)

Structured robot-link tokens cross-attend to time-indexed obstacle tokens, conditioned on robot joint state and desired goal. A deterministic behavior-cloning baseline precedes a conditional flow-matching trajectory decoder. Sample several candidate joint trajectories, then independently validate them. SAM serves perception; optional Glimmer-based distillation may serve high-level task constraints, not the motor-control loop.

## Safety boundary

The M0 capsule-to-OBB distance is a **single-configuration geometry primitive** only. A real 7-DoF system additionally needs verified robot URDF/USD link geometry, forward kinematics, adjacent-link exemptions, self-collision, a swept-volume or conservative continuous-time trajectory validator, tracked uncertainty, joint limits, velocity/acceleration/torque and stopping-distance constraints, watchdogs, and independent protective stop. A finite waypoint sample is insufficient for declaring collision-free motion.

No real-robot commands should be enabled until hardware-specific controls and a hazard assessment are in place.
