"""Print local versions. No NVIDIA toolkit or optional dependencies required."""

import platform
import subprocess
import sys


def main() -> None:
    print(f"OS: {platform.platform()}")
    print(f"Python: {sys.version.split()[0]}")
    print(f"Interpreter: {sys.executable}")
    in_venv = sys.prefix != sys.base_prefix
    print(f"Virtual environment active: {in_venv}")
    if not in_venv:
        print("WARNING: Running outside a virtual environment; packages may be installed globally.")
    try:
        p = subprocess.run(
            ["nvidia-smi", "--query-gpu=name,driver_version,memory.total,memory.free,compute_cap", "--format=csv,noheader"],
            capture_output=True, text=True, timeout=15, check=True,
        )
        print("GPU (name, driver, VRAM total/free, capability):", p.stdout.strip())
    except (OSError, subprocess.SubprocessError) as exc:
        print("nvidia-smi not available:", exc)
    try:
        import torch
    except ImportError:
        print("PyTorch: not installed (fine for M0)")
    else:
        print(f"PyTorch: {torch.__version__}, CUDA build: {torch.version.cuda}")
        print(f"CUDA available: {torch.cuda.is_available()}")
        if torch.version.cuda is None:
            print("NOTE: This is a CPU-only PyTorch build. See README.md for the CUDA 13.0 wheel.")
        if torch.cuda.is_available():
            print(f"Device: {torch.cuda.get_device_name(0)}")
            print(f"Compute capability: {torch.cuda.get_device_capability(0)}")
            try:
                print("Small CUDA allocation:", float(torch.ones(1, device="cuda")[0]))
            except Exception as exc:
                print("CUDA allocation FAILED:", exc)
                raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
