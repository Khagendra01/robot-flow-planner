"""GPU/CPU batched oriented-box separating-axis broadphase.

Requires PyTorch; imported only when explicitly used, so the NumPy-only
project core and its CI remain lightweight.

This is the batched equivalent of the project OBB 15-axis SAT diagnostic.
The returned MAX signed projected separation is NOT exact surface distance,
penetration depth, or a PhysX collision result. No continuous-time check.

Shape contract: centers [N,3], rotations [N,3,3] (local -> world),
full edge lengths [N,3]. Rotation matrices MUST be orthonormal; callers
validate rigid poses before entering the high-throughput kernel.
"""

from __future__ import annotations

import torch


def batched_obb_separating_gaps(
    center_a: torch.Tensor,
    rotation_a: torch.Tensor,
    size_a: torch.Tensor,
    center_b: torch.Tensor,
    rotation_b: torch.Tensor,
    size_b: torch.Tensor,
) -> torch.Tensor:
    """Per-pair signed SAT projected gap, shape [N], all on input device.

    Positive: the two supplied OBBs are disjoint on at least one axis.
    Nonpositive: the OBBs touch or overlap (within FP roundoff).

    Uses 3 A axes, 3 B axes and 9 nonparallel cross-product axes.
    Inputs must be finite real-valued tensors with positive sizes and rigid
    orthonormal rotation matrices. The benchmark provides such inputs.
    """
    data = (center_a, rotation_a, size_a, center_b, rotation_b, size_b)
    if not all(isinstance(x, torch.Tensor) for x in data):
        raise TypeError("All OBB inputs must be PyTorch tensors")
    n = center_a.shape[0] if center_a.ndim == 2 else -1
    if n <= 0:
        raise ValueError("center_a must be a nonempty [N,3] tensor")
    if (center_a.shape != (n, 3) or size_a.shape != (n, 3)
            or center_b.shape != (n, 3) or size_b.shape != (n, 3)
            or rotation_a.shape != (n, 3, 3)
            or rotation_b.shape != (n, 3, 3)):
        raise ValueError("Expected centers/sizes [N,3] and rotations [N,3,3]")
    device, dtype = center_a.device, center_a.dtype
    if dtype not in (torch.float32, torch.float64):
        raise TypeError("Use float32 or float64 inputs")
    if any(x.device != device or x.dtype != dtype for x in data):
        raise ValueError("All tensors must share device and floating dtype")

    # Each ROW of transposed R is a local coordinate axis in world space.
    a_axes = rotation_a.transpose(-1, -2)  # [N,3,3]
    b_axes = rotation_b.transpose(-1, -2)
    cross = torch.cross(
        a_axes[:, :, None, :].expand(-1, -1, 3, -1),
        b_axes[:, None, :, :].expand(-1, 3, -1, -1),
        dim=-1,
    ).reshape(n, 9, 3)
    axes = torch.cat((a_axes, b_axes, cross), dim=1)  # [N,15,3]
    magnitudes = torch.linalg.vector_norm(axes, dim=-1, keepdim=True)
    is_axis = magnitudes[..., 0] > 1e-8
    axes = axes / magnitudes.clamp_min(1e-8)

    difference = center_b - center_a
    projections = torch.abs(torch.sum(axes * difference[:, None, :], dim=-1))
    projected_a = torch.matmul(axes, rotation_a).abs()  # [N,15,3]
    projected_b = torch.matmul(axes, rotation_b).abs()
    extent_a = 0.5 * torch.sum(projected_a * size_a[:, None, :], dim=-1)
    extent_b = 0.5 * torch.sum(projected_b * size_b[:, None, :], dim=-1)
    gaps = projections - extent_a - extent_b
    # Parallel zero-norm cross axes are not SAT tests.
    gaps = gaps.masked_fill(~is_axis, float("-inf"))
    return gaps.max(dim=1).values


def quaternion_wxyz_to_rotation(q: torch.Tensor) -> torch.Tensor:
    """Batch convert normalized quaternions [N,4] to rigid [N,3,3] matrices."""
    if not isinstance(q, torch.Tensor) or q.ndim != 2 or q.shape[1] != 4:
        raise ValueError("Expected quaternion tensor [N,4]")
    q = torch.nn.functional.normalize(q, dim=-1)
    w, x, y, z = q.unbind(-1)
    row0 = torch.stack((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)), -1)
    row1 = torch.stack((2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)), -1)
    row2 = torch.stack((2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)), -1)
    return torch.stack((row0, row1, row2), dim=-2)
