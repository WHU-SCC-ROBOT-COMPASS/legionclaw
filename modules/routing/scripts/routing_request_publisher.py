#!/usr/bin/env python3
"""
RoutingRequest消息发送器
读取routing_request.txt文件，模拟发送ROS2 RoutingRequest消息
"""

import argparse
import math
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from ros2_interface.msg import Header, KeyPoint, RoutingRequest


class RoutingRequestPublisher(Node):
    """RoutingRequest消息发布节点"""

    def __init__(self, file_path: str, topic: str = '/legion_bridge/RoutingRequest'):
        super().__init__('routing_request_publisher')

        self.declare_parameter('file_path', file_path)
        self.declare_parameter('topic', topic)

        self.file_path = self.get_parameter('file_path').value
        self.topic = self.get_parameter('topic').value
        self.count = 0
        self.max_count = 5

        self.get_logger().info(f'Reading waypoints from: {self.file_path}')
        self.get_logger().info(f'Publishing to topic: {self.topic}')

        qos = QoSProfile(depth=10)
        self.publisher = self.create_publisher(RoutingRequest, self.topic, qos)

        self.routing_request = self.build_routing_request()
        self.timer = self.create_timer(0.5, self.timer_callback)

    def parse_waypoints(self) -> list:
        """解析waypoint文件"""
        waypoints = []
        path = Path(self.file_path)

        if not path.exists():
            self.get_logger().error(f'File not found: {self.file_path}')
            return waypoints

        with open(path, 'r') as f:
            lines = f.readlines()

        if len(lines) < 2:
            self.get_logger().warn('File is empty or has no data lines')
            return waypoints

        header = lines[0].strip().split()
        self.get_logger().info(f'Header: {header}')

        for line in lines[1:]:
            parts = line.strip().split()
            if len(parts) < 5:
                self.get_logger().warn(f'Skipping invalid line: {line}')
                continue

            waypoint = {
                'id': int(parts[0]),
                'latitude': float(parts[1]),
                'longitude': float(parts[2]),
                'heading': float(parts[3]),
                'name': parts[4] if len(parts) > 4 else ''
            }
            waypoints.append(waypoint)
            self.get_logger().info(
                f"Parsed waypoint: id={waypoint['id']}, lat={waypoint['latitude']}, "
                f"lon={waypoint['longitude']}, heading={waypoint['heading']}, name={waypoint['name']}"
            )

        return waypoints

    def build_routing_request(self) -> RoutingRequest:
        """构建RoutingRequest消息"""
        msg = RoutingRequest()

        # header = Header()
        # header.stamp = Time(sec=0, nsec=0)
        # header.frame_id = 'routing_request'
        # msg.header = header

        msg.request_source = 'routing_request_publisher'

        waypoints = self.parse_waypoints()
        if not waypoints:
            self.get_logger().warn('No valid waypoints parsed')
            return msg

        msg.request_type = 1  # 正常路由
        msg.num_of_kp = len(waypoints)

        for wp in waypoints:
            kp = KeyPoint()
            kp.id = wp['id']
            kp.latitude = wp['latitude']
            kp.longitude = wp['longitude']
            kp.ele = 0.0
            kp.heading = math.radians(wp['heading'])
            kp.name = wp['name']
            msg.key_point_list.append(kp)

        self.get_logger().info(
            f'Built RoutingRequest with {msg.num_of_kp} waypoints, '
            f'type={msg.request_type}, source={msg.request_source}'
        )

        return msg

    def timer_callback(self):
        """定时发布消息"""
        if self.count < self.max_count:
            self.publisher.publish(self.routing_request)
            self.count += 1
            self.get_logger().debug(f'Published RoutingRequest message ({self.count}/{self.max_count})')
        else:
            raise KeyboardInterrupt


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Publish RoutingRequest messages from file'
    )
    parser.add_argument(
        'file_path',
        nargs='?',
        default='modules/routing/routing_request.txt',
        help='Path to waypoint file (default: modules/routing/routing_request.txt)'
    )
    parser.add_argument(
        '-t', '--topic',
        default='/legion_bridge/RoutingRequest',
        help='ROS2 topic to publish to (default: /legion_bridge/RoutingRequest)'
    )

    cmd_args = sys.argv[1:] if args is None else args
    parsed_args = parser.parse_args(cmd_args)

    rclpy.init(args=sys.argv)
    node = RoutingRequestPublisher(
        file_path=parsed_args.file_path,
        topic=parsed_args.topic
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
