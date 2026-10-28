"""No-Isaac dependency tests for the one-command native Franka benchmark runner."""

from pathlib import Path
import runpy
import sys

RUNNER = Path(__file__).resolve().parents[1] / "scripts" / "benchmark_headless_franka.py"


def load():
    # run_name != "__main__" prevents ever starting a real Isaac Lab process.
    return runpy.run_path(str(RUNNER), run_name="rfp_test_module")


def test_missing_isaaclab_launcher_detected(tmp_path):
    api = load()
    env = tmp_path / "env"
    source = tmp_path / "IsaacLab"
    assert api["find_cli"](env, source) is None


def test_venv_entrypoint_preferred_to_source_checkout(tmp_path):
    api = load()
    env = tmp_path / "env"
    source = tmp_path / "IsaacLab"
    (env / "Scripts").mkdir(parents=True)
    source.mkdir()
    (env / "Scripts" / "isaaclab.exe").write_text("stub")
    (source / "isaaclab.bat").write_text("stub")
    assert api["find_cli"](env, source) == [
        str(env / "Scripts" / "isaaclab.exe")
    ]


def test_dry_run_never_starts_physics(tmp_path, monkeypatch, capsys):
    api = load()
    env = tmp_path / "env"
    source = tmp_path / "IsaacLab"
    (env / "Scripts").mkdir(parents=True)
    (env / "Scripts" / "isaaclab.exe").write_text("stub")
    monkeypatch.setattr(sys, "argv", [
        str(RUNNER), "--dry-run",
        "--isaaclab-env", str(env),
        "--isaaclab-checkout", str(source),
        "--envs", "64", "256",
    ])
    assert api["main"]() == 0
    out = capsys.readouterr().out
    assert "Isaac-Reach-Franka-v0" in out
    assert "64 environments" in out
    assert "256 environments" in out
    assert "--visualizer none" in out
