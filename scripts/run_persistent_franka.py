"""M1-C: ONE COMMAND, long-lived headless Isaac Lab 3.0 Franka worker.

From the project root, with any terminal Python interpreter:

    .venv\Scripts\python.exe scripts\run_persistent_franka.py

Optional:
    .venv\Scripts\python.exe scripts\run_persistent_franka.py --envs 64 --blocks 20 --steps-per-block 200 --seed 42

Automatically runs with the user's EXISTING desktop/isaaclab-env Python.
NO installs, NO GUI, NO per-block process launches, NO robot safety labels.

Initial exercise: vectorized built-in Isaac-Reach-Franka task, seeded resets
and synthetic random actions. "Block" means a fixed number of environment
steps and can include AUTO-RESETS, not one completed goal-reaching episode.
This is a persistent *pipeline throughput* test and is not yet a trajectory
teacher, obstacle avoidance benchmark, or valid training demonstration.

Set warp.config.enable_backward=False BEFORE importing Isaac Lab (upstream
Isaac Lab's own sample does this to reduce cold kernel compilation).
"""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys


PROJECT = Path(__file__).resolve().parents[1]
ISAAC_PYTHON = Path.home() / "Desktop" / "isaaclab-env" / "Scripts" / "python.exe"
TASK = "Isaac-Reach-Franka"


def worker_command(current_executable: str, arguments: list[str]) -> list[str] | None:
    """Return single re-exec command only when not already in Isaac Lab venv."""
    target = ISAAC_PYTHON.resolve()
    if not target.is_file():
        raise FileNotFoundError(
            f"Isaac Lab Python missing at {target}. This script intentionally "
            "does not install or modify your Python environments."
        )
    if Path(current_executable).resolve() == target:
        return None
    return [str(target), str(Path(__file__).resolve()), *arguments]


def _start_worker_if_needed() -> None:
    cmd = worker_command(sys.executable, sys.argv[1:])
    if cmd is None:
        return
    print("Launching persistent Isaac Lab worker (one process, one scene)...", flush=True)
    print("Python:", cmd[0], flush=True)
    # Avoid shell=True and avoid reinstalling or modifying dependencies.
    completed = subprocess.run(cmd, cwd=str(PROJECT), env=os.environ.copy())
    raise SystemExit(completed.returncode)


def collect_block_stats(env, steps: int, rng, torch):
    """Run a fixed-length block of random actions with no per-frame CPU sync."""
    import time

    total_reward = torch.zeros((), device=env.unwrapped.device)
    terminations = torch.zeros((), device=env.unwrapped.device, dtype=torch.int64)
    timeouts = torch.zeros((), device=env.unwrapped.device, dtype=torch.int64)

    # The reach task has a Box action space with batch dimension [num_envs,D].
    if not hasattr(env.action_space, "shape") or env.action_space.shape is None:
        raise RuntimeError("Expected a single-agent Box action space")
    action_shape = tuple(env.action_space.shape)
    if len(action_shape) != 2 or action_shape[0] != env.unwrapped.num_envs:
        raise RuntimeError(f"Unexpected action shape {action_shape}")

    device = env.unwrapped.device
    if str(device).startswith("cuda"):
        torch.cuda.synchronize(device)
    started = time.perf_counter()
    with torch.inference_mode():
        for _ in range(steps):
            # Use the device RNG seeded at the START of each block.
            # Avoid a Python per-environment action loop.
            action = 2.0 * torch.rand(action_shape, device=device, generator=rng) - 1.0
            _, rewards, terminated, truncated, _ = env.step(action)
            total_reward += rewards.sum()
            terminations += terminated.sum()
            timeouts += truncated.sum()
    if str(device).startswith("cuda"):
        torch.cuda.synchronize(device)
    elapsed = time.perf_counter() - started
    return {
        "steps": int(steps),
        "environment_steps": int(steps * env.unwrapped.num_envs),
        "wall_seconds": elapsed,
        "environment_steps_per_second": steps * env.unwrapped.num_envs / elapsed,
        "mean_reward_per_environment_step": (total_reward / (steps * env.unwrapped.num_envs)).item(),
        "termination_events": int(terminations.item()),
        "truncation_events": int(timeouts.item()),
    }


def _run_isaac_lab() -> int:
    # Set warp compile policy exactly as upstream Isaac Lab sample before
    # importing modules that may define Warp kernels.
    import warp as wp
    wp.config.enable_backward = False

    import argparse
    from datetime import datetime, timezone
    import json
    import time

    import gymnasium as gym
    import torch
    from isaaclab.app import add_launcher_args, launch_simulation
    from isaaclab.utils import validate
    import isaaclab_tasks  # noqa: F401 - registers built-in reaching task
    from isaaclab_tasks.utils import resolve_task_config, setup_preset_cli

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", default=TASK, help="Registered Isaac Lab reach task")
    parser.add_argument("--envs", type=int, default=64)
    parser.add_argument("--blocks", type=int, default=20,
                        help="Reset and run this many blocks in ONE simulator")
    parser.add_argument("--steps-per-block", type=int, default=200)
    parser.add_argument("--seed", type=int, default=42)
    add_launcher_args(parser)
    parser.set_defaults(device=None, visualizer=["none"])
    requested = list(sys.argv[1:])
    if not any(value.startswith("physics=") for value in requested):
        requested.append("physics=isaacsim_physx")
    args, hydra_args = setup_preset_cli(parser, requested)
    sys.argv = [sys.argv[0], *hydra_args]
    if args.envs < 1 or args.blocks < 1 or args.steps_per_block < 1:
        parser.error("--envs, --blocks and --steps-per-block must be positive")
    if args.task != TASK:
        parser.error("M1-C currently validates only Isaac-Reach-Franka")
    torch.manual_seed(args.seed)

    # This is the official Isaac Lab v3 task-resolution path; it selects the
    # PhysX variant using the CLI selector physics=isaacsim_physx if specified.
    cfg, _ = resolve_task_config(args.task, "")
    cfg.scene.num_envs = args.envs
    args.device = cfg.sim.device
    validate(cfg)
    print("=== M1-C ONE-PROCESS FRANKA START ===", flush=True)
    print(f"Task: {args.task}; envs: {args.envs}; blocks: {args.blocks}; "
          f"steps/block: {args.steps_per_block}; device: {cfg.sim.device}", flush=True)

    created_at = datetime.now(timezone.utc)
    output_dir = PROJECT / "outputs" / "persistent_franka"
    output_dir.mkdir(parents=True, exist_ok=True)
    dest = output_dir / f"persistent_franka_{created_at.strftime('%Y%m%dT%H%M%S%fZ')}.json"
    manifest = {
        "schema_version": "rfp.persistent_franka.v1",
        "created_utc": created_at.isoformat(),
        "task": args.task,
        "num_envs": args.envs,
        "blocks_requested": args.blocks,
        "steps_per_block": args.steps_per_block,
        "seed": args.seed,
        "device": str(cfg.sim.device),
        "rollout_type": "seeded_random_actions_not_teacher_trajectories",
        "physics_collision_ground_truth": False,
        "valid_as_collision_free_training_data": False,
        "one_persistent_environment_created": True,
        "blocks": [],
    }

    def flush():
        temporary = dest.with_suffix(".tmp")
        temporary.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        temporary.replace(dest)

    with launch_simulation(cfg, args):
        env = gym.make(args.task, cfg=cfg)
        try:
            if int(env.unwrapped.num_envs) != args.envs:
                raise RuntimeError("Isaac Lab environment count did not match request")
            # One scene is created here, before the block loop; reset()
            # should not recreate USD scene assets.
            env.reset(seed=args.seed)
            for block in range(args.blocks):
                block_seed = args.seed + block
                # A block boundary is a repeatable experimental reset, not an
                # episode boundary guaranteed by the reach task's done logic.
                env.reset(seed=block_seed)
                device = env.unwrapped.device
                generator = torch.Generator(device=device).manual_seed(block_seed)
                metrics = collect_block_stats(env, args.steps_per_block, generator, torch)
                manifest["blocks"].append({"block": block, "seed": block_seed, **metrics})
                flush()
                print(f"Block {block+1}/{args.blocks}: "
                      f"{metrics['environment_steps_per_second']:,.0f} env-steps/s; "
                      f"mean reward={metrics['mean_reward_per_environment_step']:.4f}; "
                      f"term/trunc={metrics['termination_events']}/{metrics['truncation_events']}",
                      flush=True)
        finally:
            env.close()

    total_steps = sum(b["environment_steps"] for b in manifest["blocks"])
    total_runtime = sum(b["wall_seconds"] for b in manifest["blocks"])
    manifest["summary"] = {
        "measured_environment_steps": total_steps,
        "total_block_runtime_seconds_excluding_startup": total_runtime,
        "aggregate_environment_steps_per_second_excluding_startup":
            (total_steps / total_runtime if total_runtime else None),
        "blocks_completed": len(manifest["blocks"]),
        "note": "No exact PhysX collision labels, no obstacle-avoiding teacher; "
                "these are randomly controlled reaching observations.",
    }
    flush()
    print("=== M1-C PERSISTENT FRANKA COMPLETE ===", flush=True)
    print("ONE FILE:", dest, flush=True)
    print("Env steps:", total_steps, "in", len(manifest["blocks"]),
          "blocks with only ONE scene creation", flush=True)
    print("Rate excluding cold startup:",
          f"{manifest['summary']['aggregate_environment_steps_per_second_excluding_startup']:,.0f} steps/s",
          flush=True)
    return 0


def main() -> int:
    _start_worker_if_needed()
    return _run_isaac_lab()


if __name__ == "__main__":
    raise SystemExit(main())
