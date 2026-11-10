import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import pytest
from nav_msgs.msg import OccupancyGrid


path = Path(__file__).resolve().parents[1] / 'scripts/pad_slam_map.py'
spec = importlib.util.spec_from_file_location('pad_slam_map', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_padding_preserves_observations_and_leaves_border_unknown():
    message = OccupancyGrid()
    message.header.frame_id = 'map'
    message.info.resolution = 0.5
    message.info.width = 2
    message.info.height = 2
    message.info.origin.position.x = 0.4
    message.info.origin.orientation.w = 1.0
    message.data = [0, 100, -1, 25]

    result = module.pad_map(message, 1.0)

    assert result.info.width == result.info.height == 6
    assert result.info.origin.position.x == pytest.approx(-0.6)
    assert result.info.origin.position.y == pytest.approx(-1.0)
    assert result.header == message.header
    assert list(result.data) == [-1] * 14 + [0, 100] + [-1] * 4 + [-1, 25] + [-1] * 14
    assert list(message.data) == [0, 100, -1, 25]


def test_padding_respects_rotated_map_origin():
    message = OccupancyGrid()
    message.info.resolution = 1.0
    message.info.width = message.info.height = 1
    message.info.origin.orientation.z = math.sin(math.pi / 4)
    message.info.origin.orientation.w = math.cos(math.pi / 4)
    message.data = [100]
    result = module.pad_map(message, 1.0)
    assert result.info.origin.position.x == pytest.approx(1.0)
    assert result.info.origin.position.y == pytest.approx(-1.0)
    assert list(result.data) == [-1, -1, -1, -1, 100, -1, -1, -1, -1]


def test_invalid_map_is_rejected():
    message = OccupancyGrid()
    with pytest.raises(ValueError):
        module.pad_map(message, 1.0)


@pytest.mark.parametrize('namespace', ['', 'offline_amr'])
def test_slam_launch_publishes_padded_map_in_isolated_domain(namespace):
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import MapMetaData
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    domain = 130 + os.getpid() % 30
    environment = os.environ.copy()
    environment.update({'ROS_DOMAIN_ID': str(domain), 'ROS_LOCALHOST_ONLY': '1'})
    command = ['ros2', 'launch', 'amr_nav2', 'slam.launch.py', 'use_sim_time:=false']
    if namespace:
        command.append(f'namespace:={namespace}')
    context = Context()
    node = None
    executor = None
    with tempfile.TemporaryFile(mode='w+t') as output:
        process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                   env=environment, start_new_session=True)
        try:
            rclpy.init(context=context, domain_id=domain)
            node = rclpy.create_node('offline_slam_test', context=context, namespace=namespace)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            raw, padded, metadata = [], [], []
            subscriptions = [
                node.create_subscription(OccupancyGrid, 'slam_map_raw', raw.append, qos),
                node.create_subscription(OccupancyGrid, 'map', padded.append, qos),
                node.create_subscription(MapMetaData, 'map_metadata', metadata.append, qos),
            ]
            tf_publisher = node.create_publisher(TFMessage, 'tf', 10)
            scan_publisher = node.create_publisher(LaserScan, 'scan', 10)
            deadline = time.monotonic() + 25.0
            while time.monotonic() < deadline and not (raw and padded and metadata):
                if process.poll() is not None:
                    output.seek(0)
                    pytest.fail(f'SLAM launch exited early:\n{output.read()}')
                stamp = node.get_clock().now().to_msg()
                transforms = []
                for parent, child, offset in (('odom', 'base_link', 0.0),
                                              ('base_link', 'laser_frame', 0.45)):
                    transform = TransformStamped()
                    transform.header.stamp = stamp
                    transform.header.frame_id = parent
                    transform.child_frame_id = child
                    transform.transform.translation.x = offset
                    transform.transform.rotation.w = 1.0
                    transforms.append(transform)
                tf_publisher.publish(TFMessage(transforms=transforms))
                scan = LaserScan()
                scan.header.stamp = stamp
                scan.header.frame_id = 'laser_frame'
                scan.angle_min = -0.5
                scan.angle_max = 0.5
                scan.angle_increment = 0.01
                scan.range_min = 0.02
                scan.range_max = 8.0
                scan.ranges = [3.0] * 101
                scan_publisher.publish(scan)
                executor.spin_once(timeout_sec=0.1)
            assert raw and padded and metadata
            assert raw[-1].info.origin.position.x > 0.0
            assert padded[-1].info.origin.position.x < 0.0
            assert padded[-1].info.width > raw[-1].info.width
            assert padded[-1] == module.pad_map(raw[-1], 1.0)
            assert metadata[-1] == padded[-1].info
            assert subscriptions
        finally:
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
            if context.ok():
                context.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5.0)