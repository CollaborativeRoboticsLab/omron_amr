import sys
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _create_nodes(context):

    params_file = LaunchConfiguration('params_file').perform(context)
    extra_params_file = LaunchConfiguration('extra_params_file').perform(context)

    parameter_files = [params_file]
    if extra_params_file:
        parameter_files.append(extra_params_file)

    core = Node(
        package='amr_core',
        executable='amr_core',
        name='amr_core',
        output='screen',
        parameters=parameter_files,
    )

    return [core]

def generate_launch_description():

    default_core_params = os.path.join(get_package_share_directory('amr_ros'), 'config', 'parameters.yaml')
    params_file = LaunchConfiguration('params_file')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_core_params,
        description='Parameter file passed to amr_core.',
    )

    declare_extra_params_file = DeclareLaunchArgument(
        'extra_params_file',
        default_value='',
        description='Optional extra parameter file layered on top of params_file.',
    )

    return LaunchDescription([
        declare_params_file,
        declare_extra_params_file,
        OpaqueFunction(function=_create_nodes),
    ])

