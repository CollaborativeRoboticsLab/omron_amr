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

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    robot_description_override = LaunchConfiguration('robot_description_override')

    declare_robot_description_override = DeclareLaunchArgument(
        'robot_description_override',
        default_value='false',
        description='URDF/Xacro file path inside the package'
    )

    robot_description = {'robot_description': load_file('amr_description', 'urdf/LD250.urdf')}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='log',
        parameters=[robot_description],
        condition=UnlessCondition(robot_description_override)
    )

    core_params = os.path.join(get_package_share_directory('amr_ros'), 'config', 'parameters.yaml')
    core = Node(
        package='amr_core',
        executable='amr_core',
        name='amr_core',
        output='screen',
        parameters=[core_params],
    )

    return LaunchDescription([
        declare_robot_description_override,
        robot_state_publisher,
        core,
    ])

