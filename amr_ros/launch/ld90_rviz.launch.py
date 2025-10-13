import os
import sys
import yaml
import json
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # RViz
    rviz_config_file = get_package_share_directory('amr_ros') + "/rviz/rviz_ld90.rviz"
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        )

    return LaunchDescription([
        rviz_node
        ])

