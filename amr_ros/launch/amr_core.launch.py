from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    core_parms = os.path.join(get_package_share_directory('amr_ros'), 'config', 'parameters.yaml')

    robot_description_config = load_file('amr_description', 'urdf/LD250.urdf')
    robot_description = {'robot_description' : robot_description_config}

    core = Node(
        package='amr_core',
        executable='amr_core',
        name='amr_core',
        output='screen',
        parameters=[core_parms],
    )

    # Publish Robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='log',
        parameters=[robot_description],
    )
        
    map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'pose']
    )

    ld = LaunchDescription()

    ld.add_action(core)
    ld.add_action(robot_state_publisher)
    ld.add_action(map_node)

    return ld

