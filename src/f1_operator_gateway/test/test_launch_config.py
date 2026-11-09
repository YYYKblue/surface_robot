"""Resolve the App's real launch tree without starting ROS nodes or hardware."""
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from ament_index_python import packages
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
    PopEnvironment, PopLaunchConfigurations, PushEnvironment,
    PushLaunchConfigurations, RegisterEventHandler, SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node


SOURCE_PACKAGES = Path(__file__).resolve().parents[2]
CONTROLLERS = {
    "pool_controller": ("rectangle_config", "f1_pool_cleaning", "pool_cleaning.yaml"),
    "l_controller": ("l_config", "f1_l_cleaning", "l_cleaning.yaml"),
}


@pytest.fixture
def inspect_launch(monkeypatch, tmp_path):
    # Use this checkout for every included launch/YAML, even with a copied install.
    original_share = packages.get_package_share_directory
    monkeypatch.setattr(packages, "get_package_share_directory", lambda name: (
        str(SOURCE_PACKAGES / name) if (SOURCE_PACKAGES / name).is_dir()
        else original_share(name)))
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    from f1_pool_cleaning import map_profile
    monkeypatch.setattr(map_profile, "profile", lambda _: SimpleNamespace(
        rectangle=object(), route=object(), errors={}))

    def inspect(managed=True, **overrides):
        context = LaunchContext()
        context.launch_configurations.update(
            mode="localization", map=str(tmp_path / "unused.pbstream"),
            start_localization="false", start_bridge="false", start_gateway="false",
            **overrides)
        results = {}

        def visit(actions):
            for action in actions:
                if isinstance(action, LaunchDescription):
                    visit(action.entities)
                elif isinstance(action, Node):
                    name = perform_substitutions(
                        context, normalize_to_list_of_substitutions(action.node_executable))
                    if name not in CONTROLLERS:
                        continue
                    # Expand exactly the files passed to ROS, including launch_ros's
                    # rejection of a nonexistent/empty path. Never call Node.execute.
                    action._perform_substitutions(context)
                    files, effective = [], {}
                    for filename, is_file in action._Node__expanded_parameter_arguments:
                        assert is_file
                        path = Path(filename)
                        files.append(path.resolve())
                        document = yaml.safe_load(path.read_text())
                        for key in ("/**", name, "/" + name):
                            effective.update(document.get(key, {}).get("ros__parameters", {}))
                        if path.name.startswith("launch_params_"):
                            path.unlink()
                    results[name] = (files, effective)
                elif isinstance(action, RegisterEventHandler):
                    continue  # No processes are started, so exit events are irrelevant.
                elif isinstance(action, (
                    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                    OpaqueFunction, SetLaunchConfiguration, PushLaunchConfigurations,
                    PopLaunchConfigurations, PushEnvironment, PopEnvironment,
                )):
                    visit(action.execute(context) or [])
                else:
                    raise AssertionError(f"Unexpected launch action: {type(action).__name__}")

        filename = "operator_mode.launch.py" if managed else "operator_bringup.launch.py"
        description = PythonLaunchDescriptionSource(str(
            SOURCE_PACKAGES / "f1_operator_gateway" / "launch" / filename
        )).get_launch_description(context)
        visit([description])
        assert set(results) == set(CONTROLLERS)
        return results

    return inspect


@pytest.mark.parametrize("managed", [True, False])
def test_default_controller_yaml_is_loaded_through_launch_tree(inspect_launch, managed):
    for name, (files, parameters) in inspect_launch(managed).items():
        _, package, filename = CONTROLLERS[name]
        assert (SOURCE_PACKAGES / package / "config" / filename).resolve() in files
        assert parameters["use_map_tf"] is True
        assert parameters["require_localization_ready"] is True
        assert parameters["localization_status_topic"] == "/f1_localization/localization_status"


@pytest.mark.parametrize("managed", [True, False])
def test_custom_controller_yaml_is_preserved(inspect_launch, tmp_path, managed):
    overrides = {}
    for name, (argument, _, _) in CONTROLLERS.items():
        path = tmp_path / (name + ".yaml")
        path.write_text(yaml.safe_dump({name: {"ros__parameters": dict(
            use_map_tf=True, require_localization_ready=True, brush_percent=42.0,
        )}}))
        overrides[argument] = str(path)
    for name, (files, parameters) in inspect_launch(managed, **overrides).items():
        argument, _, _ = CONTROLLERS[name]
        assert Path(overrides[argument]) in files
        assert parameters["brush_percent"] == 42.0
        assert parameters["use_map_tf"] is True
        assert parameters["require_localization_ready"] is True


@pytest.mark.parametrize("argument", ["l_config", "rectangle_config"])
@pytest.mark.parametrize("kind", ["missing", "directory"])
def test_invalid_explicit_controller_yaml_stops_launch(inspect_launch, tmp_path, argument, kind):
    path = tmp_path / "missing.yaml" if kind == "missing" else tmp_path
    with pytest.raises((ValueError, RuntimeError), match=argument):
        inspect_launch(**{argument: str(path)})
