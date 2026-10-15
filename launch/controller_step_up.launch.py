from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', description='Controller gains and joint modes YAML'),
        DeclareLaunchArgument('mpc_config', description='Replay horizon and fixed-joint model YAML'),
        DeclareLaunchArgument('rate', default_value='500'),
        Node(
            package='kyon_controller', executable='controller', output='screen',
            parameters=[{
                'config': LaunchConfiguration('config'),
                'mpc_config': LaunchConfiguration('mpc_config'),
                'rate': ParameterValue(LaunchConfiguration('rate'), value_type=int),
                'use_sim_time': False,
            }],
            remappings=[
                ('robot_description', '/xbotcore/robot_description'),
                ('robot_description_semantic', '/xbotcore/robot_description_semantic'),
            ],
        ),
    ])
