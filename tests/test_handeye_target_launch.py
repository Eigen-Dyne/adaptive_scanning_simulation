"""The target server must receive the model from the object-spawn timer itself."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, TimerAction


@pytest.mark.parametrize("gui_enabled", [True, False])
def test_spawn_passes_restoration_model_without_shared_timer_configuration(monkeypatch, gui_enabled):
    simulation = Path(__file__).resolve().parents[1] / "as_sim"
    spec = importlib.util.spec_from_file_location("gazebo_spawn", simulation / "launch/gazebo_spawn.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original_share = module.get_package_share_directory
    monkeypatch.setattr(module, "get_package_share_directory",
                        lambda name: str(simulation) if name == "as_sim" else original_share(name))
    nodes, callbacks = [], []
    original_node, original_exit = module.Node, module.OnProcessExit

    def node(**kwargs):
        result = original_node(**kwargs)
        if kwargs.get("executable") == "handeye_target_server.py":
            nodes.append((result, kwargs["parameters"][0]))
        return result

    def on_exit(**kwargs):
        callbacks.append(kwargs["on_exit"])
        return original_exit(**kwargs)

    monkeypatch.setattr(module, "Node", node)
    monkeypatch.setattr(module, "OnProcessExit", on_exit)
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(object="1.stl", obj_place_on_floor="false",
                                         obj_z="0.321", obj_Y="0.42",
                                         gui=str(gui_enabled).lower())
    timer = next(action for action in description.entities if isinstance(action, TimerAction)
                 and float(action.period) == 6.5)
    actions = timer.actions[0].execute(context)
    assert not nodes  # No part insertion while the GUI is still initializing.
    if gui_enabled:
        with pytest.raises(RuntimeError, match="GUI did not publish"):
            callbacks[-1](SimpleNamespace(returncode=124), context)
        actions = callbacks[-1](SimpleNamespace(returncode=0), context)
    actions[0].execute(context)
    server, parameters = nodes[-1]
    model = Path(parameters["dataset_model_file"])
    try:
        assert model.is_file()
        assert "Dataset/1.stl" in model.read_text()
        assert parameters["dataset_z"] == 0.321
        assert parameters["dataset_Y"] == 0.42
        assert "handeye_dataset_sdf" not in context.launch_configurations
        # The server starts after successful creation, preventing late object
        # creation from undoing the board server's removal.
        success = SimpleNamespace(returncode=0)
        assert len(callbacks[0](success, context)) == 1
        assert callbacks[0](SimpleNamespace(returncode=1), context) == []
        assert callbacks[-1](success, context) == [server]
        with pytest.raises(RuntimeError, match="Dataset spawn failed"):
            callbacks[-1](SimpleNamespace(returncode=1), LaunchContext())
    finally:
        model.unlink()
