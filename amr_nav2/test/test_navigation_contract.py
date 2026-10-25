import ast
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch' / 'nav2.launch.py'


def navigation_parameters():
    return yaml.safe_load((PACKAGE / 'config' / 'ld250_nav2.yaml').read_text())


def launch_calls(name):
    tree = ast.parse(LAUNCH.read_text())
    return [call for call in ast.walk(tree)
            if isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
            and call.func.id == name]


def keywords(call):
    return {keyword.arg: keyword.value for keyword in call.keywords}


def command_remappings(call):
    remappings = keywords(call)['remappings']
    if isinstance(remappings, ast.BinOp) and isinstance(remappings.op, ast.Add):
        return dict(ast.literal_eval(remappings.right))
    return {}


def test_controller_and_behaviors_share_the_smoother_in_both_launch_modes():
    for launch_type in ('Node', 'ComposableNode'):
        for package in ('nav2_controller', 'nav2_behaviors', 'nav2_velocity_smoother'):
            nodes = [call for call in launch_calls(launch_type)
                     if ast.literal_eval(keywords(call)['package']) == package]
            assert len(nodes) == 1
            remappings = command_remappings(nodes[0])
            assert remappings['cmd_vel'] == 'cmd_vel_nav'
            if package == 'nav2_velocity_smoother':
                assert remappings['cmd_vel_smoothed'] == 'cmd_vel'
            assert all(not topic.startswith('/') for topic in remappings.values())


def test_default_navigation_parameter_file_exists():
    declarations = [call for call in launch_calls('DeclareLaunchArgument')
                    if ast.literal_eval(call.args[0]) == 'params_file']
    assert len(declarations) == 1
    default = keywords(declarations[0])['default_value']
    assert isinstance(default, ast.Call)
    suffix = [ast.literal_eval(argument) for argument in default.args[1:]]
    assert (PACKAGE.joinpath(*suffix)).is_file()


def test_command_types_and_open_loop_feedback_are_explicit():
    params = navigation_parameters()
    for node in ('controller_server', 'behavior_server', 'velocity_smoother'):
        assert params[node]['ros__parameters']['enable_stamped_cmd_vel'] is False
    assert params['velocity_smoother']['ros__parameters']['feedback'] == 'OPEN_LOOP'


def test_controller_and_smoother_limits_fit_both_driver_configurations():
    params = navigation_parameters()
    controller = params['controller_server']['ros__parameters']
    planner = controller['FollowPath']
    smoother = params['velocity_smoother']['ros__parameters']
    assert planner['min_vel_x'] >= smoother['min_velocity'][0]
    assert planner['max_vel_x'] <= smoother['max_velocity'][0]
    assert planner['max_vel_theta'] <= smoother['max_velocity'][2]
    assert planner['acc_lim_x'] == smoother['max_accel'][0]
    assert planner['decel_lim_x'] == smoother['max_decel'][0]
    assert planner['acc_lim_theta'] == smoother['max_accel'][2]
    assert planner['decel_lim_theta'] == smoother['max_decel'][2]
    assert smoother['min_velocity'][1] == smoother['max_velocity'][1] == 0.0
    for filename in ('parameters.yaml', 'ld250_parameters.yaml'):
        hardware = yaml.safe_load((PACKAGE.parent / 'amr_ros' / 'config' / filename).read_text())
        driver = hardware['amr_core']['ros__parameters']['driver']
        assert smoother['max_velocity'][0] <= driver['max_linear_speed'] / 1000.0
        assert smoother['min_velocity'][0] >= driver['min_linear_speed'] / 1000.0
        assert smoother['max_velocity'][2] <= math.radians(driver['max_angular_speed'])
        assert smoother['min_velocity'][2] >= math.radians(driver['min_angular_speed'])
        assert controller['controller_frequency'] == smoother['smoothing_frequency'] == driver['send_frequency']
        assert 1.0 / smoother['smoothing_frequency'] < driver['cmd_vel_timeout_sec']
        assert driver['cmd_vel_topic'] == '/cmd_vel'
        assert driver['unsafe_drive'] is True


def test_navigation_odometry_topics_agree_for_the_default_unnamespaced_launch():
    params = navigation_parameters()
    assert params['bt_navigator']['ros__parameters']['odom_topic'] == 'odom'
    assert params['velocity_smoother']['ros__parameters']['odom_topic'] == 'odom'
    for filename in ('parameters.yaml', 'ld250_parameters.yaml'):
        hardware = yaml.safe_load((PACKAGE.parent / 'amr_ros' / 'config' / filename).read_text())
        assert hardware['amr_core']['ros__parameters']['driver']['odom_topic'] == '/odom'


def test_installed_smoother_publishes_reliable_twist_on_isolated_localhost(monkeypatch):
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import Twist
    from lifecycle_msgs.srv import ChangeState
    import rclpy
    from rclpy.qos import ReliabilityPolicy

    monkeypatch.setenv('ROS_DOMAIN_ID', str(190 + os.getpid() % 20))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    namespace = '/offline_nav_contract'
    executable = Path(get_package_prefix('nav2_velocity_smoother')) / 'lib/nav2_velocity_smoother/velocity_smoother'
    command = [str(executable), '--ros-args', '-r', f'__ns:={namespace}',
               '-r', 'cmd_vel:=cmd_vel_nav', '-r', 'cmd_vel_smoothed:=cmd_vel']
    parameters = navigation_parameters()['velocity_smoother']['ros__parameters']
    parameters['use_sim_time'] = False
    for name, value in parameters.items():
        command.extend(['-p', f'{name}:={json.dumps(value)}'])
    with tempfile.TemporaryFile(mode='w+t') as output:
        process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT)
        node = None
        try:
            rclpy.init()
            node = rclpy.create_node('navigation_contract_test', namespace=namespace)
            state = node.create_client(ChangeState, 'velocity_smoother/change_state')
            assert state.wait_for_service(timeout_sec=5.0)
            for transition in (1, 3):
                request = ChangeState.Request()
                request.transition.id = transition
                future = state.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
                assert future.done() and future.result().success
            received = []
            subscription = node.create_subscription(Twist, 'cmd_vel', received.append, 1)
            publisher = node.create_publisher(Twist, 'cmd_vel_nav', 1)
            target = Twist()
            target.linear.x = 0.4
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                publisher.publish(target)
                rclpy.spin_once(node, timeout_sec=0.05)
                if received and math.isclose(received[-1].linear.x, 0.4, abs_tol=1e-6):
                    break
            assert received and math.isclose(received[-1].linear.x, 0.4, abs_tol=1e-6)
            inputs = node.get_subscriptions_info_by_topic(f'{namespace}/cmd_vel_nav')
            outputs = node.get_publishers_info_by_topic(f'{namespace}/cmd_vel')
            assert inputs and outputs
            for endpoint in inputs + outputs:
                assert endpoint.topic_type == 'geometry_msgs/msg/Twist'
                assert endpoint.qos_profile.reliability == ReliabilityPolicy.RELIABLE
            assert subscription is not None
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5.0)