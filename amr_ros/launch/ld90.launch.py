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
	use_nav2 = LaunchConfiguration('use_nav2')
	use_slam = LaunchConfiguration('use_slam')
	use_localization = LaunchConfiguration('use_localization')
	nav2_params_file = LaunchConfiguration('nav2_params_file')
	map_yaml_file = LaunchConfiguration('map')

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

	declare_use_nav2 = DeclareLaunchArgument(
		'use_nav2',
		default_value='false',
		description='Launch the Nav2 stack for the LD90 platform.',
	)

	declare_use_slam = DeclareLaunchArgument(
		'use_slam',
		default_value='false',
		description='Launch SLAM Toolbox inside the Nav2 child launch.',
	)

	declare_use_localization = DeclareLaunchArgument(
		'use_localization',
		default_value='false',
		description='Launch AMCL localization inside the Nav2 child launch.',
	)

	declare_nav2_params_file = DeclareLaunchArgument(
		'nav2_params_file',
		default_value=PathJoinSubstitution([FindPackageShare('amr_nav2'), 'config', 'ld90_nav2.yaml']),
		description='Nav2 parameter file to pass to the LD90 navigation launch.',
	)

	declare_map = DeclareLaunchArgument(
		'map',
		default_value='',
		description='Map yaml filename inside amr_nav2/maps used when Nav2 localization is enabled.',
	)

	core_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(PathJoinSubstitution([
			FindPackageShare('amr_ros'),
			'launch',
			'amr_core.launch.py',
		])),
		launch_arguments={
			'params_file': PathJoinSubstitution([FindPackageShare('amr_ros'), 'config', 'ld90_parameters.yaml']),
		}.items(),
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

	nav2_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(PathJoinSubstitution([
			FindPackageShare('amr_nav2'),
			'launch',
			'ld90.launch.py',
		])),
		condition=IfCondition(use_nav2),
		launch_arguments={
			'params_file': nav2_params_file,
			'use_slam': use_slam,
			'use_localization': use_localization,
			'map': map_yaml_file,
		}.items(),
	)

	return LaunchDescription([
		declare_rviz,
        declare_robot_description_override,
		declare_use_nav2,
		declare_use_slam,
		declare_use_localization,
		declare_nav2_params_file,
		declare_map,
		core_launch,
		robot_state_publisher,
		nav2_launch,
		rviz_launch,
	])
