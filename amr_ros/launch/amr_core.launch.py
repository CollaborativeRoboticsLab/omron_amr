from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None


def generate_launch_description():
    robot_description = LaunchConfiguration('robot_description')
    
    # Launch arg: robot_description as raw XML string
    declare_robot_description = DeclareLaunchArgument(
        'robot_description',
        default_value=load_file('amr_description', 'urdf/LD250.urdf') or '',
        description='URDF XML for robot_state_publisher'
    )

    core_parms = os.path.join(get_package_share_directory('amr_ros'), 'config', 'parameters.yaml')

    core = Node(
        package='amr_core',
        executable='amr_core',
        name='amr_core',
        output='screen',
        parameters=[core_parms],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='log',
        parameters=[{'robot_description': robot_description}],
    )

    ld = LaunchDescription()
    ld.add_action(declare_robot_description)
    ld.add_action(core)
    ld.add_action(robot_state_publisher)

    return ld

