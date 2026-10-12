"""Calibration temporarily owns the object area and restores its original model."""
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "as_sim/scripts"))
import handeye_target_server as target


@pytest.mark.parametrize("spawn_fails", [False, True])
def test_dataset_is_restored_after_calibration_or_failed_board_spawn(monkeypatch, spawn_fails):
    commands, removed = [], []
    values = {"world_name": "scanner_env", "dataset_model_file": "/tmp/original-model.sdf",
              **{"dataset_" + axis: value for axis, value in zip(("x", "y", "z", "R", "P", "Y"),
                                                                (0.65, 0.02, 0.09, 0.1, 0.2, 0.3))}}
    node = SimpleNamespace(visible=False, dataset_removed=False, remove=removed.append,
                           get_parameter=lambda name: SimpleNamespace(value=values[name]))

    def run(command, **kwargs):
        commands.append(command)
        if spawn_fails and "spawn_charuco.py" in command:
            raise RuntimeError("board creation failed")

    monkeypatch.setattr(target.subprocess, "run", run)
    monkeypatch.setattr(target, "get_package_prefix", lambda *_: "/sim")
    response = SimpleNamespace(success=False, message="")
    target.HandeyeTarget.show(node, None, response)
    assert response.success is not spawn_fails
    assert removed == ["dataset_object"]
    if not spawn_fails:
        target.HandeyeTarget.show(node, None, response)
        assert len(commands) == 1  # Duplicate setup cannot create another board.
    target.HandeyeTarget.hide(node, None, response)
    assert response.success
    assert not node.visible and not node.dataset_removed
    restore = commands[-1]
    assert restore[restore.index("-file") + 1] == "/tmp/original-model.sdf"
    assert restore[restore.index("-z") + 1] == "0.09"
    assert restore[restore.index("-Y") + 1] == "0.3"
    assert "charuco_calibration_board" in removed if not spawn_fails else len(removed) == 1
