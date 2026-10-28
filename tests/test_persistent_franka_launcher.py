"""Unit-test the persistent worker's no-install interpreter selection.

Does not launch Isaac Sim or import Isaac Lab. Test subprocess isolation
before running any expensive robotics simulation.
"""

import runpy
from pathlib import Path

RUNNER = Path(__file__).resolve().parents[1] / "scripts" / "run_persistent_franka.py"


def load():
    return runpy.run_path(str(RUNNER), run_name="rfp_persistent_test")


def test_reexec_when_running_project_venv(tmp_path):
    m = load()
    target = tmp_path / "isaaclab-env" / "Scripts" / "python.exe"
    target.parent.mkdir(parents=True)
    target.write_text("mock executable")
    fn = m["worker_command"]
    fn.__globals__["ISAAC_PYTHON"] = target
    command = fn(str(tmp_path / "other" / "python.exe"), ["--envs", "64"])
    assert command[0] == str(target.resolve())
    assert command[1].endswith("run_persistent_franka.py")
    assert command[-2:] == ["--envs", "64"]


def test_no_nested_process_when_already_isaaclab_python(tmp_path):
    m = load()
    target = tmp_path / "isaaclab-env" / "Scripts" / "python.exe"
    target.parent.mkdir(parents=True)
    target.write_text("mock executable")
    fn = m["worker_command"]
    fn.__globals__["ISAAC_PYTHON"] = target
    assert fn(str(target), ["--blocks", "20"]) is None


def test_missing_isaaclab_python_is_clear_error(tmp_path):
    m = load()
    fn = m["worker_command"]
    fn.__globals__["ISAAC_PYTHON"] = tmp_path / "does_not_exist.exe"
    try:
        fn(str(tmp_path / "project" / "python.exe"), [])
    except FileNotFoundError as exc:
        assert "does not install" in str(exc)
    else:
        raise AssertionError("Expected clear missing-interpreter error")
