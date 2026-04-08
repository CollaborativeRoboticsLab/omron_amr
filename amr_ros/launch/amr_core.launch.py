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

    core_params = os.path.join(get_package_share_directory('amr_ros'), 'config', 'parameters.yaml')
    core = Node(
        package='amr_core',
        executable='amr_core',
        name='amr_core',
        output='screen',
        parameters=[core_params],
    )

    return LaunchDescription([
        core,
    ])

