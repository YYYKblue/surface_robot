"""Evaluate the installed launch without executing Cartographer or hardware."""
import importlib.util
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


def launch_module():
    share = Path(get_package_share_directory('f1_localization'))
    spec = importlib.util.spec_from_file_location('localization_launch', share / 'launch/localization.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('prefix', ['', 'l_cleaning'])
def test_standalone_launch_owns_only_localization(reference_map, monkeypatch, prefix):
    module = launch_module()
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(map=str(reference_map), cleaning_prefix=prefix,
                                         localization_config='/field/localization.yaml')
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    nodes = module.nodes(context)
    assert [n['package'] for n in nodes] == ['tf2_ros', 'cartographer_ros', 'cartographer_ros', 'f1_localization']
    cartographer, manager = nodes[1], nodes[-1]
    assert '-start_trajectory_with_default_topics=false' in cartographer['arguments']
    assert manager['executable'] == 'localization_manager'
    params = manager['parameters'][1]
    assert params['map'] == str(reference_map)
    assert (Path(params['configuration_directory']) / params['configuration_basename']).is_file()
    assert manager['parameters'][-1] == '/field/localization.yaml'
    assert manager['name'] == 'pool_localization_manager'  # preserve YAML selectors
    if prefix:
        assert manager['remappings'] == [
            (f'/pool_cleaning/{name}', f'/l_cleaning/{name}')
            for name in ('localization_status', 'localization_ready', 'set_initial_pose', 'relocalize')]
    else:
        assert manager['remappings'] == []
    context.launch_configurations['publish_lidar_tf'] = 'false'
    assert len(module.nodes(context)) == 3


def test_missing_reference_state_fails_before_nodes_are_created():
    module = launch_module()
    context = LaunchContext()
    context.launch_configurations.update(map='', load_state_filename='')
    with pytest.raises(ValueError, match='set map'):
        module.nodes(context)
