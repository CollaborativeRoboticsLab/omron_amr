import os
import sys
import yaml
import json
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
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
    rviz_config_file = get_package_share_directory('amr_ros') + "/rviz/rviz_ld250.rviz"
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=LaunchConfiguration('namespace'),
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='log',
        arguments=['-d', rviz_config_file],
        )
    
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        rviz_node,
    ])

