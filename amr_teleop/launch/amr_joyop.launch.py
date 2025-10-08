import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    teleop_config = os.path.join( get_package_share_directory('amr_joyop'), 'config', 'teleop.yaml')

    joystickNode=Node(
            package='joy',
            namespace='',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.3, 'autorepeat_rate': 20.0}],
            arguments=['--ros-args', '--log-level', 'info']
    )

    teleopJoystickNode=Node(
            package='teleop_twist_joy',
            namespace='',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[teleop_config],
            remappings=[('/cmd_vel', 'amr/cmd_vel')],
            arguments=['--ros-args', '--log-level', 'info']
    )

    ld.add_action(joystickNode)
    ld.add_action(teleopJoystickNode)
    
    return ld