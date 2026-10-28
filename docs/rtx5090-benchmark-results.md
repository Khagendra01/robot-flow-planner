# RTX 5090 geometry benchmark — measured, 2026-10-28 UTC

## Verified workstation numbers

User ran the existing project virtual environment explicitly:

```bat
.venv\Scripts\python.exe scripts\benchmark_torch_geometry.py
```

The exact console output reported:

| Item | Measured |
|---|---:|
| PyTorch | 2.11.0+cu130 |
| GPU | NVIDIA GeForce RTX 5090 |
| CPU median, 4096 OBB pairs | 1.983 ms |
| GPU median, same 4096 pairs | 0.360 ms |
| Measured CPU/GPU same-size ratio | 5.50× |
| GPU median, 65536 pairs | 0.728 ms |
| Large GPU throughput | 90,083,849 SAT box pairs/s |
| Maximum CPU/GPU output difference | 3.5762787e-07 |

The user first ran the script via unactivated global Python with PyTorch
2.14.1+cpu and no CUDA. Running through the **project's existing GPU-enabled
venv** corrected that without any reinstall.

## Interpretation

These are **synthetic, batched 15-axis separating-axis-theorem calculations**.
They are not 90 million full robot collisions, 90 million PhysX calls, or
90 million goal-reaching robot trajectories per second. The kernel checks
transformed boxes, not exact PhysX cooked convex hull geometry. The 5.50×
comparison uses exactly 4096 pairs on each device; large-batch GPU throughput
should be reported separately because the workload differs.

CPU results, GPU results and elapsed times are from the user's actual output.
We have not measured Isaac Lab Franka physics throughput on this Windows
workstation yet. Optimization should now focus on vectorized simulated
environments, real motion planning demonstrations, physical collision
validation, and dataset throughput, not further micro-optimizing OBB SAT.

## Next one-command real-world bottleneck measurement

The next benchmark is **Isaac Lab 3.0's native headless runtime benchmark on
the Franka reaching task**:

```bat
cd %USERPROFILE%\Desktop\robot-flow-planner
git pull origin main
.venv\Scripts\python.exe scripts\benchmark_headless_franka.py
```

The launcher looks for the existing `Desktop\isaaclab-env\Scripts\isaaclab.exe`
or `Desktop\IsaacLab\isaaclab.bat`. It does **not** install or modify
packages. If neither launch path is installed, it exits with a specific
dependency report rather than silently timing a CPU substitute. The default
benchmarks 64 Franka reaching environments, with 30 warm-up and 200 measured
environment steps. Scale later by running:

```bat
.venv\Scripts\python.exe scripts\benchmark_headless_franka.py --envs 64 256
```

Each requested environment count runs the *same* upstream Isaac Lab benchmark
in a fresh process. Results and logs go to
`outputs/benchmarks/headless_franka_*/` and are not committed.

**Limitations:** Isaac Lab benchmark runtime uses random actions; it does not
provide a custom moving-obstacle task or verified collision-free teacher
trajectories. Actual physics stepping (and not GPU box SAT throughput) is
what matters for scaling trajectory data generation.

Official Isaac Lab 3.0 benchmark docs:
https://isaac-sim.github.io/IsaacLab/v3.0.0/source/developer-tools/benchmarking/run_benchmarks.html
