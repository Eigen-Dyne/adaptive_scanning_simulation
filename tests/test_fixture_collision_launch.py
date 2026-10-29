"""Gazebo must use the same selected collision fixture as the MoveIt container."""
import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


@pytest.mark.parametrize("launch_name", ["gazebo.launch.py", "gazebo_spawn.launch.py"])
@pytest.mark.parametrize("fixture", ["creaform_700", "zeiss", None])
def test_fixture_environment_reaches_spawned_collision_model(monkeypatch, launch_name, fixture):
    if fixture is None:
        monkeypatch.delenv("ROBOT_FIXTURE", raising=False)
    else:
        monkeypatch.setenv("ROBOT_FIXTURE", fixture)
    source = Path(__file__).resolve().parents[1] / "as_sim/launch" / launch_name
    spec = importlib.util.spec_from_file_location("fixture_launch", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    commands = []
    original_command = module.Command
    def command(*args, **kwargs):
        result = original_command(*args, **kwargs)
        commands.append(result)
        return result
    monkeypatch.setattr(module, "Command", command)
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations["fixture_type"] == (fixture or "config")
    model = ET.fromstring(commands[0].perform(context))
    collision = model.find("link[@name='scanner_fixture_bottom_link']/collision/geometry")
    if fixture in ("creaform_700", "zeiss"):
        assert collision.find("box") is None
        mesh = collision.find("mesh")
        assert mesh is not None
        expected = ("creaform_700_collision.stl" if fixture == "creaform_700"
                    else "scanner_assembled_fixture.stl")
        assert mesh.get("filename").endswith(expected)
        assert Path(mesh.get("filename").removeprefix("file://")).is_file()
    if launch_name == "gazebo.launch.py":
        included = next(a for a in description.entities if isinstance(a, IncludeLaunchDescription))
        arguments = dict(included.launch_arguments)
        assert arguments["fixture_type"].perform(context) == (fixture or "config")


def test_explicit_fixture_override_takes_precedence(monkeypatch):
    monkeypatch.setenv("ROBOT_FIXTURE", "zeiss")
    source = Path(__file__).resolve().parents[1] / "as_sim/launch/gazebo_spawn.launch.py"
    spec = importlib.util.spec_from_file_location("fixture_override", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations["fixture_type"] = "creaform_700"
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations["fixture_type"] == "creaform_700"
