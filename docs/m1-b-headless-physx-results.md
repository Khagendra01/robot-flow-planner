# M1-B: Measured headless PhysX runtime and startup bottleneck

Single exploratory run on 2026-10-28 UTC; not a repeated performance study.

Isaac Lab 3.0.0, Isaac Sim 6.2.0-rc.14, Windows 11, Intel Core Ultra 9
285K, RTX 5090 (31.84 GB), PyTorch 2.11.0+cu130. Task
Isaac-Reach-Franka, PhysX, no rendering, seed 42, 64 environments,
30 warm-up iterations, 200 measured iterations.

| Metric | Measured |
|---|---:|
| Status | Complete, exit code 0 |
| Environment steps sampled | 12,800 = 64 × 200 |
| Runtime-only mean aggregate FPS | 3,757.896 environment steps/s |
| Host-return environment-step FPS | 3,758.554 environment steps/s |
| Runtime measurement wall time | 3.406 s |
| Mean / peak iteration time | 17.031 / 215.086 ms |
| Peak reported collection FPS | 5,201.349 environment steps/s |
| Mean GPU utilization | 37.75% (4 samples) |
| GPU memory | 11.35 GB mean, 11.36 GB peak |
| Mean process RAM | 9.21 GB |
| Scene initialization time | 291.281 s |
| Entire launch / benchmark wall time | 311.369 s |
| Effective measured steps / full process second | ~41.11 steps/s |
| Scene creation share of process time | ~93.55% |

The last two calculated ratios include **cold startup** and are distinct
from Isaac Lab's runtime-only collection FPS. Do not quote 3,758 as
end-to-end trajectory data generation, collision labels or training
throughput; the benchmark uses random actions.

## Where the time went

The log reports basic scene initialization at 01:38:40 UTC. At 01:43:21,
roughly 280 seconds later, asset handling reports three Franka payloads
(physics.usda, geometries.usd, materials.usda) loaded from a Windows
Temp-based local cache of remote S3 assets. That cache handler warns:
each copy is checked against the server before reuse. After that message,
the task logs ~3.949 seconds of scene setup.

Likely bottleneck: **remote asset resolution/cache validation** rather
than GPU math. The available log does not pinpoint which network call
accounts for each second, so this remains a grounded hypothesis. The
numerous nonfatal RTX prim-definition and USD instancer warnings should be
monitored if workloads change. They did not prevent benchmark completion.

The physics frame recorder's ~2.19 ms and environment step ~17.03 ms
refer to different measurement boundaries; they are not simply additive.

## Immediate decisions

1. **Keep the scene running:** use one persistent headless worker with
   thousands of seeded episode resets instead of launching a new Isaac
   process for each short trial. A 3.4-second measurement following
   5 minutes of startup is inherently an inefficient workflow.
2. **Investigate asset cache latency:** upstream documentation recommends
   enabling Hub Workstation Cache for repeated slow AWS-hosted asset
   loads. Do not delete the current cache or blindly rewrite USD paths.
   A local asset override requires validation of referenced USD payloads.
3. **Run longer benchmarks only after fixing setup:** 64 environments
   is a baseline; 256 and 512 require independent physics/contact and
   resource validation, not just unused VRAM.
4. **Build meaningful data:** deterministic joint-goal episodes, moving
   obstacle states, actual PhysX contact/collision evaluation, self and
   ground collision, goal validity and continuous-time safeguards.
   Record oracle provenance and valid trajectories per second.

## References

Official Isaac Lab 3.0 benchmark methodology:
https://isaac-sim.github.io/IsaacLab/v3.0.0/source/developer-tools/benchmarking/run_benchmarks.html

Asset caching guidance:
https://docs.nvidia.com/isaaclab/3.0/isaac-lab/installation

Next project milestone: persistent, seeded, headless M1-C
Franka simulation/teacher benchmark; not another manual snapshot exporter.
