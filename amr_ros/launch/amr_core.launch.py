import sys
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch.conditions import UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    default_core_params = os.path.join(get_package_share_directory('amr_ros'), 'config', 'parameters.yaml')
    params_file = LaunchConfiguration('params_file')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_core_params,
        description='Parameter file passed to amr_core.',
    )

    core = Node(
        package='amr_core',
        executable='amr_core',
        name='amr_core',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        declare_params_file,
        core,
    ])

