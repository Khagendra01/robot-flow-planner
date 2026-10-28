# Performance reset: replace GUI loops with batched GPU simulation

## What was slow and why

The first Isaac Sim milestones used GUI Script Editor, per-prim USD traversal,
serial snapshot files, Python loops over 22 body/obstacle OBB pairs per frame,
and screenshot/upload feedback after almost every script. Most of those
operations are **not GPU-accelerated**. An RTX 5090 does not make Python/JSON,
USD asset authoring, network downloads or sequential human-in-the-loop steps
run thousands of times faster.

The real 60-frame dataset provides **0.983333 s of simulated time** and
60 distinct timestamps. The separate combined baseline is embedded, so
there is no reason to run individual exporters again. The available bundle's
creation timestamps are consistent with a few seconds of wall-clock work,
but they do not provide a controlled physics-only frame-rate benchmark.
Do **not** use "60Hz recorded timestamps" as a measured real-time FPS.

## Immediate GPU baseline: ONE terminal command

In the existing project venv (the one with torch 2.11.0+cu130):

~~~bat
cd %USERPROFILE%\Desktop\robot-flow-planner
git pull origin main
.venv\Scripts\activate.bat
python scripts/benchmark_torch_geometry.py
~~~

This benchmarks the EXACT 15-axis oriented-box broadphase, now written with
batched PyTorch tensor math:

- A fair same-input-size CPU/GPU comparison of 4,096 pairs.
- 65,536 more OBB pairs in one batched CUDA call.
- Explicit \`torch.cuda.synchronize()\` before/after each measured GPU call.
- A check that CPU and GPU produce matching results.

Tiny batches can be *slower* on the GPU due to kernel-launch and Python
dispatch overhead. The large batch is important. Results depend on device
state, kernel scheduling, GPU clocks and input size. This is a **synthetic
geometry microbenchmark**, not Isaac Lab simulation throughput, collision
ground truth, or end-to-end model training speed. Do not quote it as "robot
trajectories per second."

## M1-B proper headless benchmark (next development target)

Use **Isaac Lab 3.0** as a downstream task package, not the editor/GUI
collector as the main pipeline. The v3.0.0 release supports different backends,
including \`newton_mjwarp\` (kit-less/warp native) and \`isaacsim_physx\`
(Isaac Sim PhysX). Backend performance and collision semantics must be measured
separately; do not assume they yield identical labels.

1. **First physics benchmark:** deterministic single Franka environment with
   explicit 7-DoF initial state, joint limits, goal, and a static obstacle.
   Verify joint drive behavior and contacts.
2. **Vectorize:** start with 64 simulated environments then scale 256, 512,
   and higher only if GPU/host memory and physics step time allow.
3. **Dynamic obstacles:** each environment has independently seeded moving
   rectangular obstacles; record both observed state and time-aware obstacle
   motion ground truth *without leaking future ground truth into policy
   features*.
4. **Teacher/data labels:** sample goal-reaching joint trajectories with a
   geometric planner; check all 11 robot collider bodies, self collisions,
   ground, joint limits, and time-indexed obstacle collision. Record collision
   oracle provenance and sampling/uncertainty. Swept checks or validated
   conservative stepping must cover between-sample motion.
5. **GPU batching:** move broadphase, data transforms, losses and eventual
   flow-matching training tensors onto the GPU; avoid Python loops over
   environments/bodies/timesteps.
6. **Benchmarks:** report simulator steps/second, scenes/second, verified
   trajectories/second, GPU utilization, peak VRAM, and rejected/colliding
   cases. Profile slowest kernels before increasing batch size.

**Windows note:** Isaac Lab's documented multi-GPU/multi-node launcher relies
on Linux + NCCL. Your single RTX 5090 works for *single-GPU vectorized*
environments without multi-GPU launcher support.

## Performance targets are measurements, not promises

No claimed "1000 environments" or "millions of trajectories per hour" until
we have actually run the headless benchmark. Collision-heavy articulated
scenes can be CPU-bound, memory-bound, or limited by physics accuracy even
with abundant GPU compute.

## Relevant upstream references

- Isaac Lab 3.0 release: https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0
- Isaac Sim 6.2 standalone/headless workflow: https://docs.isaacsim.omniverse.nvidia.com/6.2.0/introduction/workflows.html
- Isaac Lab 3.0 multi-GPU limitation: https://isaac-sim.github.io/IsaacLab/release/3.0.0/source/features/multi_gpu.html
