# M1-A: Verified 60-frame sequence result

Observed from an actual Isaac Sim 6.2 Franka Panda run, captured 2026-10-28 UTC.

## Empirical result

The single-run collector successfully produced `rfp.sequence_bundle.v1` containing its whole M0 scene/geometry baseline and **60 actual time-advancing frames**:

| Metric | Observed |
|---|---:|
| Sampled simulation time | 0.833333 to 1.816667 s |
| Consecutive time step | 0.016667 s (60 Hz) |
| Elapsed time, first to last frame | 0.983333 s |
| Joint values per frame | 9 (7 revolute, 2 prismatic) |
| Robot-body poses per frame | 11 |
| Obstacles per frame | 2 |
| Total proxy OBB pair tests | 1,320 (22 per frame) |
| Frames with any proxy overlap | 0 / 60 |
| Minimum projected SAT gap over frames | ~0.3561 m |
| Maximum observed arm-joint speed | ~2.6100 rad/s |

Joint motion, first to last sampled pose:

- Panda joint 4: **-1.59455 to -2.79204 rad**, a change of **-1.19749 rad**.
- Panda joint 6: **2.18192 to 3.02967 rad**, a change of **+0.84775 rad**.
- Panda hand frame traveled approximately **0.488 m**; each finger origin approximately **0.503 m**.
- Both obstacles' world positions were unchanged across all sampled frames.

The baseline is embedded in the sequence JSON; the separate baseline file need **not** be submitted or generated manually when using the one-run collector.

## What this proves

- Isaac Sim can expose 9 joint values and all 11 collider-owning body transforms per sample.
- Asynchronous Kit updates advance simulator time, unlike repeated synchronous reads.
- The per-frame obstacle / authored-mesh box screening is consistent enough to run at every recorded step without an exception.
- One Script Editor invocation can produce a validated multi-frame sequence in one JSON.

## What this does NOT prove

**No collision-free labels were produced.** The 22 pairwise tests per frame use approximate link-aligned OBBs enclosing the authored collision mesh points; the reported projected gap is not exact Euclidean clearance from a cooked PhysX shape. Frame interval motion, ground/self collisions, contact offsets, gripper contacts and continuous swept volumes remain unvalidated.

This rollout was **passive**: no goal, collision-free teacher motion, or deliberately varied initial state was provided. The two obstacle boxes were **static**. The resulting data is useful for the ingestion and kinematics/transform consistency pipeline but cannot train a general obstacle-avoiding joint trajectory generator by itself. Also, the baseline collection's repeated 0.816667 s timestamp remains a single synchronous instant; only the 60 later samples establish time advance. `atomic_physics_frame_sampling_proven: false` remains accurate.

## Next milestone (M1-B)

Build a **single-run simulation benchmark** rather than additional hand-triggered snapshots:

1. Reset robot to a deliberate joint state, and establish joint-limit / initial-state consistency; verify behavior in simulation.
2. Generate reproducible goals and obstacle configurations with seeds. Add moving box obstacles with explicit time-indexed ground truth.
3. Use a geometric teacher or independently verified PhysX collision-query layer for contact/collision labeling; do not conflate our OBB proxy with ground truth.
4. Collect complete time-stamped trajectories at fixed physics steps, including action/goal context, collision outcomes, and train/validation splits.
5. Preserve a separate final trajectory validator before planning results are considered usable.

A deep-learning flow model, visual segmentation and real-world robot execution are **not yet implemented**. The next work should prioritize a valid teacher/benchmark and reproducible automated simulation data.
