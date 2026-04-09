import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	package_dir = get_package_share_directory('amr_nav2')
	rviz_config_file = LaunchConfiguration('rviz_config')

	declare_rviz_config_cmd = DeclareLaunchArgument(
		'rviz_config',
		default_value=os.path.join(package_dir, 'rviz', 'nav2.rviz'),
		description='Full path to the RViz config for Nav2 mapping and localization.')

	rviz_node = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2_nav2',
		output='screen',
		arguments=['-d', rviz_config_file],
	)

	return LaunchDescription([
		declare_rviz_config_cmd,
		rviz_node,
	])
