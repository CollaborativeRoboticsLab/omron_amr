#!/usr/bin/env python3

import copy
import math
from array import array

import rclpy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def pad_map(message, margin):
    resolution = message.info.resolution
    width = message.info.width
    height = message.info.height
    if not math.isfinite(resolution) or resolution <= 0 or not math.isfinite(margin) or margin < 0:
        raise ValueError('Invalid map resolution or margin')
    if width == 0 or height == 0 or len(message.data) != width * height:
        raise ValueError('Invalid map dimensions or data')

    cells = math.ceil(margin / resolution)
    padded_width = width + 2 * cells
    padded_height = height + 2 * cells
    result = copy.deepcopy(message)
    result.info.width = padded_width
    result.info.height = padded_height
    orientation = message.info.origin.orientation
    yaw = math.atan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y),
                     1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z))
    distance = cells * resolution
    result.info.origin.position.x -= distance * (math.cos(yaw) - math.sin(yaw))
    result.info.origin.position.y -= distance * (math.sin(yaw) + math.cos(yaw))
    data = array('b', [-1]) * (padded_width * padded_height)
    for row in range(height):
        source = row * width
        destination = (row + cells) * padded_width + cells
        data[destination:destination + width] = message.data[source:source + width]
    result.data = data
    return result


class SlamMapPadding(Node):
    def __init__(self):
        super().__init__('slam_map_padding')
        self.declare_parameter('margin', 1.0)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(OccupancyGrid, 'map', qos)
        self.metadata_publisher = self.create_publisher(MapMetaData, 'map_metadata', qos)
        self.subscription = self.create_subscription(OccupancyGrid, 'slam_map_raw', self.on_map, qos)

    def on_map(self, message):
        try:
            padded = pad_map(message, self.get_parameter('margin').value)
            self.publisher.publish(padded)
            self.metadata_publisher.publish(padded.info)
        except ValueError as error:
            self.get_logger().error(f'Cannot pad SLAM map: {error}')


def main():
    rclpy.init()
    node = SlamMapPadding()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()