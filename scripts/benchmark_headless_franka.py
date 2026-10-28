"""One-command REAL Isaac Lab 3.0 Franka headless simulation benchmark.

Run from the project directory with any Python 3.12 interpreter:
    .venv\Scripts\python.exe scripts\benchmark_headless_franka.py

This is a pretraining PHYSICS/ENVIRONMENT benchmark, not a teacher planner
and not an exact collision-oracle test. It uses the existing separate
Desktop/isaaclab-env installation or an existing Desktop/IsaacLab checkout.

Does not pip install, edit any scene, or download assets. On missing Isaac Lab
3.0 CLI it prints precise next steps rather than secretly using a CPU stub.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

PROJECT = Path(__file__).resolve().parents[1]
DESKTOP = Path.home() / "Desktop"
DEFAULT_ENV = DESKTOP / "isaaclab-env"
DEFAULT_SOURCE = DESKTOP / "IsaacLab"
TASK = "Isaac-Reach-Franka-v0"


def cli_candidates(python_env: Path, checkout: Path) -> list[list[str]]:
    """Use a v3 CLI executable, or an existing v3 source checkout wrapper."""
    return [
        [str(python_env / "Scripts" / "isaaclab.exe")],
        [str(python_env / "Scripts" / "isaaclab.bat")],
        [str(python_env / "Scripts" / "isaaclab.cmd")],
        [str(checkout / "isaaclab.bat")],
    ]


def find_cli(python_env: Path, checkout: Path) -> list[str] | None:
    for parts in cli_candidates(python_env, checkout):
        if Path(parts[0]).is_file():
            return parts
    # Do not silently use a global 'isaaclab' which may target the wrong
    # Python/CUDA or an old Isaac Lab version.
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--envs", nargs="+", type=int, default=[64],
                        help="Environment counts to benchmark, e.g. --envs 64 256")
    parser.add_argument("--warmup-steps", type=int, default=30)
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--physics", choices=("isaacsim_physx", "newton_mjwarp"),
                        default="isaacsim_physx")
    parser.add_argument("--isaaclab-env", type=Path, default=DEFAULT_ENV,
                        help="Existing IsaacLab/IsaacSim Python env")
    parser.add_argument("--isaaclab-checkout", type=Path, default=DEFAULT_SOURCE,
                        help="Existing Isaac Lab v3 source checkout")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print the commands, do not start Isaac Lab")
    args = parser.parse_args()
    if any(n < 1 for n in args.envs):
        parser.error("Each environment count must be positive")
    if args.steps < 1 or args.warmup_steps < 0:
        parser.error("Invalid step counts")
    cli = find_cli(args.isaaclab_env, args.isaaclab_checkout)
    if cli is None:
        print("M1-B BLOCKED: Isaac Lab 3.0 'isaaclab' CLI was not found.")
        print("Checked:", *[parts[0] for parts in
              cli_candidates(args.isaaclab_env, args.isaaclab_checkout)], sep="\n  ")
        print("Your existing Isaac Sim GUI installation can still work without")
        print("the separate Isaac Lab 3.0 benchmark CLI being installed.")
        print("Do NOT reinstall PyTorch or CUDA. Verify the installed Isaac Lab")
        print("release and launcher before modifying either environment.")
        print("Reference: https://isaac-sim.github.io/IsaacLab/v3.0.0/")
        return 2

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    root = PROJECT / "outputs" / "benchmarks" / ("headless_franka_" + stamp)
    if not args.dry_run:
        root.mkdir(parents=True, exist_ok=True)

    manifest = {
        "schema_version": "rfp.physics_benchmark.v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "tool": "Isaac Lab 3.0 built-in benchmark runtime",
        "task": TASK,
        "physics": args.physics,
        "cuda_geometry_benchmark": "different workload; results not comparable",
        "warmup_steps": args.warmup_steps,
        "measured_steps": args.steps,
        "isaaclab_launcher": str(cli[0]),
        "measurements": [],
        "physx_collision_labels_verified": False,
        "trajectory_teacher_present": False,
    }
    for n in args.envs:
        subdir = root / f"envs_{n}"
        # Isaac Lab 3.0 CLI documents this exact syntax (NOT Isaac Lab 2.x
        # --headless and not direct scripts/reinforcement_learning invocations).
        cmd = [
            *cli, "benchmark", "runtime",
            "--task", TASK,
            "--num_envs", str(n),
            "--warmup_steps", str(args.warmup_steps),
            "--num_steps", str(args.steps),
            "--seed", str(args.seed),
            "--visualizer", "none",
            "--benchmark_formatter", "schema,summary",
            "--output_path", str(subdir),
            f"physics={args.physics}",
        ]
        print(f"=== Headless Franka: {n} environments, {args.physics} ===", flush=True)
        print("Command:", subprocess.list2cmdline(cmd), flush=True)
        if args.dry_run:
            continue

        # Explicit launch from existing checkout when using its .bat, but
        # do not alter the user's installed Python or package environment.
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        start = time.perf_counter()
        log_path = root / f"envs_{n}.log"
        # Windows subprocess .bat wrappers require cmd.exe. list2cmdline
        # quoting is performed by subprocess. No shell=True.
        with log_path.open("w", encoding="utf-8", errors="replace") as log:
            process = subprocess.Popen(
                cmd, cwd=str(args.isaaclab_checkout if cli[0] == str(
                    args.isaaclab_checkout / "isaaclab.bat") else PROJECT),
                env=env, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True,
                errors="replace", bufsize=1,
            )
            assert process.stdout is not None
            for line in process.stdout:
                log.write(line)
                # Print all errors and summary lines, but avoid flooding the
                # terminal with all Kit startup warnings.
                if any(term in line.lower() for term in (
                    "error", "traceback", "summary report", "collection fps",
                    "environment step", "total fps", "workflow_name",
                    "completed", "failed",
                )):
                    print(line.rstrip(), flush=True)
            code = process.wait()
        elapsed = time.perf_counter() - start
        manifest["measurements"].append({
            "num_envs": n,
            "status": "completed" if code == 0 else "failed",
            "return_code": code,
            "launch_and_benchmark_wall_seconds": elapsed,
            "log": str(log_path),
            "benchmark_output_dir": str(subdir),
        })
        # Write after every run, even if one test fails.
        path = root / "manifest.json"
        path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        print(f"Run {n}: exit code {code}; log saved to {log_path}", flush=True)
        if code:
            print("Benchmark failed. Check the earliest error in the log.")
            print("No measured PhysX throughput claimed for this run.")
            return code
    print("Benchmarks saved under:", root)
    print("Report the built-in Isaac Lab Collection FPS and Step FPS,")
    print("which are distinct from synthetic CUDA OBB-pairs-per-second.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
