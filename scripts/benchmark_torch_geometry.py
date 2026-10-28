"""M1-B benchmark: batch 15-axis robot-obstacle OBB checks on CUDA.

This microbenchmark intentionally DOES NOT run PhysX/Isaac Sim, train a neural
network, or prove collision safety. It measures the geometry broadphase
performance of our Torch kernel, with GPU timing synchronized correctly.

Run in your EXISTING robot-flow-planner .venv:
    python scripts/benchmark_torch_geometry.py

Optional:
    python scripts/benchmark_torch_geometry.py --pairs 131072 --cpu-pairs 4096

Synthetic input represents randomized OBB pairs (not motion trajectories).
For meaningful scaling compare CPU and GPU for the SAME small pair batch, then
run the large CUDA batch. Overheads mean small GPU jobs can be slower than CPU.
"""

import argparse
import time

import torch

from robot_flow_planner.torch_geometry import (
    batched_obb_separating_gaps,
    quaternion_wxyz_to_rotation,
)


def make_pairs(n: int, device: torch.device, seed: int):
    # Generate in CPU memory for reproducibility, then transfer ONCE.
    generator = torch.Generator(device="cpu").manual_seed(seed)
    center_a = torch.randn(n, 3, generator=generator) * 0.5
    center_b = center_a + torch.randn(n, 3, generator=generator) * 0.45
    rot_a = quaternion_wxyz_to_rotation(
        torch.randn(n, 4, generator=generator)
    )
    rot_b = quaternion_wxyz_to_rotation(
        torch.randn(n, 4, generator=generator)
    )
    size_a = 0.03 + torch.rand(n, 3, generator=generator) * 0.28
    size_b = 0.03 + torch.rand(n, 3, generator=generator) * 0.28
    return tuple(t.to(device) for t in (center_a, rot_a, size_a,
                                        center_b, rot_b, size_b))


def bench(pairs, repeats: int):
    device = pairs[0].device
    for _ in range(2):
        out = batched_obb_separating_gaps(*pairs)
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    timings = []
    for _ in range(repeats):
        if device.type == "cuda":
            torch.cuda.synchronize(device)
        start = time.perf_counter()
        out = batched_obb_separating_gaps(*pairs)
        if device.type == "cuda":
            torch.cuda.synchronize(device)
        timings.append(time.perf_counter() - start)
    assert out.shape == (len(pairs[0]),)
    if not torch.isfinite(out).all().item():
        raise RuntimeError("Non-finite SAT results in the benchmark")
    return sorted(timings)[len(timings) // 2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pairs", type=int, default=65536,
                        help="Large CUDA batch (default: 65,536 OBB pairs)")
    parser.add_argument("--cpu-pairs", type=int, default=4096,
                        help="Comparable CPU/GPU batch (default: 4,096 pairs)")
    parser.add_argument("--repeats", type=int, default=5)
    args = parser.parse_args()
    if args.pairs < 1 or args.cpu_pairs < 1 or args.repeats < 1:
        parser.error("Batch sizes and repeat count must be positive")

    cpu = torch.device("cpu")
    reference = make_pairs(args.cpu_pairs, cpu, seed=2026)
    cpu_time = bench(reference, args.repeats)
    print("Torch:", torch.__version__)
    print("CPU batch:", args.cpu_pairs, "pairs")
    print(f"CPU median: {1000*cpu_time:.3f} ms | "
          f"{args.cpu_pairs/cpu_time:,.0f} pairs/s")

    if not torch.cuda.is_available():
        print("CUDA unavailable. GPU benchmark skipped.")
        print("This is NOT an Isaac Sim or PhysX performance result.")
        return

    cuda = torch.device("cuda:0")
    print("CUDA device:", torch.cuda.get_device_name(0))
    matching = tuple(t.to(cuda) for t in reference)
    gpu_small = bench(matching, args.repeats)
    with torch.no_grad():
        cpu_result = batched_obb_separating_gaps(*reference).cpu()
        cuda_result = batched_obb_separating_gaps(*matching).cpu()
    absolute_error = float((cpu_result - cuda_result).abs().max())
    if not torch.allclose(cpu_result, cuda_result, atol=1e-5, rtol=1e-5):
        raise RuntimeError("GPU/CPU SAT results disagree")
    print(f"GPU same-size median: {1000*gpu_small:.3f} ms | "
          f"{args.cpu_pairs/gpu_small:,.0f} pairs/s")
    print(f"Same-size CPU/GPU ratio: {cpu_time/gpu_small:.2f}x "
          "(>1 means CUDA faster)")

    large = make_pairs(args.pairs, cuda, seed=2027)
    gpu_large = bench(large, args.repeats)
    print(f"GPU large batch: {args.pairs:,} pairs")
    print(f"GPU large median: {1000*gpu_large:.3f} ms | "
          f"{args.pairs/gpu_large:,.0f} pairs/s")
    print("Largest abs CPU/GPU difference (same-size inputs):",
          f"{absolute_error:.8g}")
    print("PASS: batched SAT computed on CUDA.")
    print("LIMIT: synthetic boxes, no physics stepping, no cooked PhysX "
          "shape validation, no true collision-free path labels.")


if __name__ == "__main__":
    main()
