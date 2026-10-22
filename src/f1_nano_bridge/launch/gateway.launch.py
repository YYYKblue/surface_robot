from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bind_address', default_value='127.0.0.1'),
        DeclareLaunchArgument('allow_fifth_thruster', default_value='false'),
        Node(package='f1_nano_bridge', executable='f1_ui_gateway', name='f1_ui_gateway',
             output='screen', parameters=[{'bind_address': LaunchConfiguration('bind_address'),
                 'allow_fifth_thruster': ParameterValue(LaunchConfiguration('allow_fifth_thruster'), value_type=bool)}]),
    ])
