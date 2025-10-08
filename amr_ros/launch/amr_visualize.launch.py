import os
import sys
import yaml
import json
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
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
    robot_description_config = load_file('amr_description', 'urdf/AMR_Platform.urdf')
    robot_description = {'robot_description' : robot_description_config}

    # RViz
    rviz_config_file = get_package_share_directory('amr_ros') + "/rviz/rviz_amr_platform.rviz"
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
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

    return LaunchDescription([
        robot_state_publisher, 
        rviz_node,
        map_node
    ])

