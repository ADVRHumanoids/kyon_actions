from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', description='Controller gains and joint modes YAML'),
        DeclareLaunchArgument('mpc_config', default_value=PathJoinSubstitution([
            FindPackageShare('kyon_actions'), 'config', 'replay_model.yaml']),
            description='Replay horizon and fixed-joint model YAML'),
        DeclareLaunchArgument('rate', default_value='500'),
        DeclareLaunchArgument('command_timeout', default_value='0.5'),
        Node(
            package='kyon_controller', executable='controller', output='screen',
            parameters=[{
                'config': LaunchConfiguration('config'),
                'mpc_config': LaunchConfiguration('mpc_config'),
                'rate': ParameterValue(LaunchConfiguration('rate'), value_type=int),
                'use_sim_time': False,
                'command_timeout': ParameterValue(LaunchConfiguration('command_timeout'), value_type=float),
            }],
            remappings=[
                ('robot_description', '/xbotcore/robot_description'),
                ('robot_description_semantic', '/xbotcore/robot_description_semantic'),
            ],
        ),
    ])
