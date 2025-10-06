from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

core_parms = os.path.join(get_package_share_directory('amr_ros'), 'config', 'parameters.yaml')

def generate_launch_description():

    core = Node(
        package='amr_core_cpp',
        executable='amr_core_cpp',
        name='amr_core_cpp',
        output='screen',
        parameters=[core_parms],
    )

    ld = LaunchDescription()

    ld.add_action(core)

    return ld

