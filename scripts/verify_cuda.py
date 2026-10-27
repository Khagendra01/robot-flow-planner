"""Check that the current Python interpreter can actually compute on a CUDA GPU.

Run after installing a CUDA-enabled PyTorch wheel in the intended virtual env.
This is not a full Isaac Lab or simulator compatibility test.
"""

import sys


def main() -> None:
    try:
        import torch
    except ImportError as exc:
        raise SystemExit(
            "PyTorch is not installed in this interpreter. "
            "Activate the correct virtual environment and install a CUDA-enabled build."
        ) from exc

    print("Python executable:", sys.executable)
    print("Virtual environment:", sys.prefix != sys.base_prefix)
    print("PyTorch:", torch.__version__)
    print("PyTorch CUDA build:", torch.version.cuda)
    print("CUDA available:", torch.cuda.is_available())

    if not torch.cuda.is_available():
        raise SystemExit(
            "CUDA is unavailable. If the version ends in '+cpu', install a CUDA wheel "
            "inside the intended environment; see README.md. "
            "Do not assume nvidia-smi's CUDA Version means PyTorch is GPU enabled."
        )

    print("GPU:", torch.cuda.get_device_name(0))
    print("Compute capability:", torch.cuda.get_device_capability(0))
    try:
        a = torch.randn((512, 512), device="cuda", dtype=torch.float32)
        b = a @ a
        torch.cuda.synchronize()
        print("GPU matmul:", tuple(b.shape), "finite:", bool(torch.isfinite(b).all()))
        if not bool(torch.isfinite(b).all()):
            raise SystemExit("GPU matmul produced non-finite values.")
    except RuntimeError as exc:
        raise SystemExit("GPU matmul failed: " + str(exc)) from exc
    print("CUDA smoke test: PASS")


if __name__ == "__main__":
    main()
