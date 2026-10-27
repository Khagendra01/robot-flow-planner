# Windows workstation: RTX 5090 and Isaac Lab setup

_Last updated 2026-10-27. All commands assume **Command Prompt** (cmd.exe), not PowerShell, unless explicitly noted._

## Verified local checkpoint

Reported by workstation (not remotely reproduced):

- Windows 11 build 26100, Python 3.12.10.
- RTX 5090 (32 GB), driver 581.95, CUDA compute capability 12.0.
- Approximately 64 GB of system memory.
- `torch 2.11.0+cu130`; `torch.cuda.is_available()` was `True`; a 512×512 CUDA matrix multiplication succeeded.
- M0 geometry tests: 12/12 passing.

This validates **CUDA-enabled PyTorch**, not Isaac Sim/Isaac Lab, CUDA extension compilation, or end-to-end motion planning.

## Recover from GitHub DNS failure

If `git pull origin main` reports `Could not resolve host: github.com`, do not reclone: the existing local repository is not corrupted. Run:

~~~bat
nslookup github.com
ipconfig /flushdns
git pull origin main
~~~

If DNS still fails, check connectivity, VPN/proxy, firewall, and whether another network works. If DNS succeeds but Git fails, inspect:

~~~bat
git config --show-origin --get-regexp "http\..*proxy"
~~~

A nonzero exit code from that last command can simply mean no proxy is configured. Do not unset a corporate proxy unless your network administrator instructs you to.

After networking works:

~~~bat
cd %USERPROFILE%\Desktop\robot-flow-planner
git pull origin main
.venv\Scripts\activate.bat
python scripts/verify_cuda.py
python -m pytest -q
~~~

The `verify_cuda.py` file was added to the remote repository after the original clone. If `git pull` failed, the local copy may not exist. The previously successful inline GPU matrix multiplication is already a valid preliminary CUDA smoke test.

## Isaac Lab / Isaac Sim in a SEPARATE environment

Isaac Lab 3.0 officially supports Windows 11 and Python 3.12; Isaac Sim 6.2 is an optional backend. Your CUDA-capable workstation meets the published **baseline** RAM/VRAM and driver criteria, but performance and all dependencies still require validation. Rendering, cameras, and large batches can consume significant additional resources.

**Wait until DNS/network package downloads are reliable before attempting the large installation.** The current `robot-flow-planner\.venv` works; keep it intact. NVIDIA's current recommended package workflow uses `uv` and exact dependency overrides for Isaac Sim compatibility.

For a separate **cmd.exe** environment, one possible install route is:

~~~bat
cd %USERPROFILE%\Desktop
py -3.12 -m venv isaaclab-env
isaaclab-env\Scripts\activate.bat
python -m pip install --upgrade pip uv
uv pip install "isaaclab[isaacsim]==3.0.0" --overrides "https://raw.githubusercontent.com/isaac-sim/IsaacLab/v3.0.0/tools/wheel_builder/uv-overrides.txt" --extra-index-url https://pypi.nvidia.com --torch-backend cu130 --index-strategy unsafe-best-match
isaacsim
~~~

The above `uv` install is from the official released-package method, not yet tested on this particular workstation. The first simulator launch may download extensions and request acceptance of NVIDIA's Omniverse EULA. Enable Windows long paths before installation, as directed by NVIDIA, if your system does not already support them. If the install fails, collect the complete error before switching toolchains or downgrading dependencies.

**Do not** copy wheel selections from older Isaac Lab / Isaac Sim 5.x setup guides into this environment. Do not install experimental cuRobo Windows CUDA extensions into the Isaac Lab environment until the simulator works.

## First simulator exit gates

1. `isaacsim` launches successfully (or an official Newton/Isaac Lab minimal example runs, if choosing that backend).
2. Load a 7-DoF Franka model, retrieve joint states, and exercise a reversible, bounded test motion.
3. Spawn a static oriented box; verify model-vs-physics collision behavior.
4. Create random box scenes with deterministic seeds and save robot/world-state observations.
5. Keep low-level execution and any physical robot disabled: the M0 geometry module alone is not a robot trajectory safety system.

Official documentation: https://docs.nvidia.com/isaaclab/3.0/isaac-lab/installation
