from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	rviz = LaunchConfiguration('rviz')
	robot_description_override = LaunchConfiguration('robot_description_override')

	xacro_file = PathJoinSubstitution([FindPackageShare('amr_description'), 'xacro', 'ld90_robot.urdf.xacro'])
	
	robot_description = {
		'robot_description': ParameterValue(Command(['xacro', ' ', xacro_file]), value_type=str)
	}

	declare_rviz = DeclareLaunchArgument(
		'rviz',
		default_value='false',
		description='Launch RViz with the LD90 configuration.',
	)
	
	declare_robot_description_override = DeclareLaunchArgument(
		'robot_description_override',
		default_value='false',
		description='Disable loading the ld90 robot description, a mother launch can override this to load a different robot description',
  	)

	core_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(PathJoinSubstitution([
			FindPackageShare('amr_ros'),
			'launch',
			'amr_core.launch.py',
		]))
	)

	robot_state_publisher = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher',
		output='screen',
		parameters=[robot_description],
        condition=UnlessCondition(robot_description_override)
	)

	rviz_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(PathJoinSubstitution([
			FindPackageShare('amr_ros'),
			'launch',
			'ld90_rviz.launch.py',
		])),
		condition=IfCondition(rviz),
	)

	return LaunchDescription([
		declare_rviz,
        declare_robot_description_override,
		core_launch,
		robot_state_publisher,
		rviz_launch,
	])
