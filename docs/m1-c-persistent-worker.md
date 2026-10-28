# M1-C: one persistent headless Franka worker

The M1-B native Isaac Lab 3.0 runtime benchmark on this workstation ran
64 Franka environments at ~3,758 **runtime-only** environment steps/s but
spent ~291 s creating the scene. A log showed a ~280 s gap during remote
asset resolution/cached Franka USD validation.

To avoid paying that initialization cost for every little experiment,
M1-C creates the built-in reach task **once** and then performs many
seeded reset/rollout blocks in that SAME simulator.

## The one command

In Windows Command Prompt, from project root:

~~~bat
cd %USERPROFILE%\Desktop\robot-flow-planner
git pull origin main
.venv\Scripts\python.exe scripts\run_persistent_franka.py
~~~

The script automatically switches to the existing
Desktop/isaaclab-env/Scripts/python.exe with no reinstall. It uses
the Isaac Lab v3.0 native app launcher, no GUI visualization, no
Isaac Sim Script Editor, and no extra per-episode process launches.
If that environment doesn't exist, the launcher reports the missing
path clearly. It does not touch Python packages.

By default, it runs:

- 64 parallel Franka reaching environments (built-in task)
- 20 consecutive **blocks** of 200 steps (up to 256,000 environment steps)
- seeded vectorized action sampling; one bulk reset between blocks
- one Isaac Lab environment creation and one simulation process
- per-block reward summary, termination/truncation counts, and environment
  steps/second; combined summary at end

A **block is not a single episode**: the Isaac Lab task can auto-reset
individual environments within a block. An aggregate reward is not goal
success or collision-free ground truth.

Progress will print as each block completes. The full initial cold
asset-resolution delay may still occur **once**. Reusing a scene amortizes
that overhead; it does not fix the underlying remote-cache delay.

One output JSON:

~~~text
outputs/persistent_franka/persistent_franka_*.json
~~~

It saves each block incrementally, so partial observations survive if
later steps fail. Upload only that final JSON (or the latest file plus
the first traceback if there is an error).

## Why this is not yet a trajectory training dataset

This is intentionally **random-action/goal-reaching smoke data**.
It is NOT a teacher planner. It does not introduce moving obstacles or
provide independent cooked PhysX collision/contact oracle labels, swept
collision checks, task success labels or full time-series storage.
Do not use it to train a safe obstacle-avoiding robot.

We first need to prove that scene setup is paid only once and that
physics stepping/reset loop works reliably in the installed Isaac
Lab 3.0.0 release. Then implement a custom downstream Isaac Lab task
which exposes explicitly controlled obstacle trajectories and
ground-truth collision outcomes. Only then build a valid trajectory
teacher and write replayable, self-contained demonstration episodes.

## Next performance engineering actions

- Diagnose remote asset cache checks using upstream Isaac Lab guidance;
  avoid deleting locally cached USD files.
- Make realistic environment counts adjustable (64/128/256) and
  profile steady-state runtime before scaling.
- Add a metric for reset time, simulator step time and total worker
  wall time. Cold startup must remain separately accounted for.
- Add robot/obstacle collision queries with explicit PhysX oracle
  provenance, including full-body/self-ground checks and between-step
  coverage.
- Add seeded goal-conditioned trajectory planning and replay,
  then batched tensor storage for flow-matching training.

Upstream Isaac Lab 3.0 entry point reference:
https://raw.githubusercontent.com/isaac-sim/IsaacLab/v3.0.0/source/isaaclab_rl/isaaclab_rl/entrypoints/simple_agents.py

Upstream environment launcher usage:
https://isaac-sim.github.io/IsaacLab/release/3.0.0/source/how-to/register_rl_env_gym.html
