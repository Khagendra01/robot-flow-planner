"""Batched Torch OBB SAT matches the existing NumPy reference implementation."""

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from robot_flow_planner.geometry import OrientedBox
from robot_flow_planner.collider_bounds import obb_separating_gap
from robot_flow_planner.torch_geometry import (
    batched_obb_separating_gaps,
    quaternion_wxyz_to_rotation,
)


def test_aligned_separated_and_overlapping_pairs():
    a = torch.tensor([[0, 0, 0], [0, 0, 0]], dtype=torch.float64)
    b = torch.tensor([[3, 0, 0], [0.5, 0, 0]], dtype=torch.float64)
    r = torch.eye(3, dtype=torch.float64).repeat(2, 1, 1)
    sizes = torch.full((2, 3), 2.0, dtype=torch.float64)
    result = batched_obb_separating_gaps(a, r, sizes, b, r, sizes).tolist()
    assert result[0] == pytest.approx(1.0)
    assert result[1] < 0


def test_random_pairs_match_numpy_sat():
    rng = np.random.default_rng(1024)
    n = 48
    a = rng.normal(size=(n, 3))
    b = rng.normal(size=(n, 3))
    size_a = rng.uniform(0.03, 0.4, size=(n, 3))
    size_b = rng.uniform(0.03, 0.4, size=(n, 3))
    qa = torch.tensor(rng.normal(size=(n, 4)), dtype=torch.float64)
    qb = torch.tensor(rng.normal(size=(n, 4)), dtype=torch.float64)
    ra = quaternion_wxyz_to_rotation(qa)
    rb = quaternion_wxyz_to_rotation(qb)
    gaps = batched_obb_separating_gaps(
        torch.tensor(a, dtype=torch.float64), ra,
        torch.tensor(size_a, dtype=torch.float64),
        torch.tensor(b, dtype=torch.float64), rb,
        torch.tensor(size_b, dtype=torch.float64),
    ).numpy()
    expected = [
        obb_separating_gap(OrientedBox(a[i], size_a[i], ra[i].numpy()),
                           OrientedBox(b[i], size_b[i], rb[i].numpy()))
        for i in range(n)
    ]
    assert gaps == pytest.approx(expected, abs=1e-10)


def test_quaternion_wxyz_z_rotation():
    q = torch.tensor([[np.sqrt(0.5), 0, 0, np.sqrt(0.5)]], dtype=torch.float64)
    rotation = quaternion_wxyz_to_rotation(q)
    assert (rotation[0] @ torch.tensor([1., 0., 0.], dtype=torch.float64)).tolist() \
           == pytest.approx([0., 1., 0.])


def test_reject_wrong_batch_shape():
    with pytest.raises(ValueError, match="centers/sizes"):
        batched_obb_separating_gaps(
            torch.zeros((2, 3)), torch.eye(3).repeat(2, 1, 1),
            torch.zeros((2, 3)), torch.zeros((3, 3)),
            torch.eye(3).repeat(3, 1, 1), torch.zeros((3, 3)),
        )


def test_float64_and_float32_close():
    torch.manual_seed(0)
    n = 16
    a, b = torch.randn(n, 3), torch.randn(n, 3)
    ra = quaternion_wxyz_to_rotation(torch.randn(n, 4))
    rb = quaternion_wxyz_to_rotation(torch.randn(n, 4))
    sa, sb = torch.rand(n, 3) + 0.1, torch.rand(n, 3) + 0.1
    result32 = batched_obb_separating_gaps(a, ra, sa, b, rb, sb)
    result64 = batched_obb_separating_gaps(
        a.double(), ra.double(), sa.double(),
        b.double(), rb.double(), sb.double(),
    )
    assert result32.numpy() == pytest.approx(result64.numpy(), abs=1e-5)
