#!/usr/bin/env python3
import math
import os
import csv

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RouteRecorder(Node):
    def __init__(self):
        super().__init__('route_recorder')

        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter(
            'route_file',
            '/home/unitree/go2_fastlio_ws/src/'
            'go2_fastlio_patrol/routes/route_demo.csv',
        )
        self.declare_parameter('min_distance', 0.4)
        self.declare_parameter('default_speed', 0.20)
        self.declare_parameter('startup_enable_file', '')

        self.odom_topic = self.get_parameter('odom_topic').value
        self.route_file = self.get_parameter('route_file').value
        self.min_distance = float(self.get_parameter('min_distance').value)
        self.default_speed = float(self.get_parameter('default_speed').value)
        self.startup_enable_file = str(
            self.get_parameter('startup_enable_file').value
        ).strip()
        self.recording_enabled = not self.startup_enable_file

        os.makedirs(os.path.dirname(self.route_file), exist_ok=True)

        self.file = open(self.route_file, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['id', 'x', 'y', 'yaw', 'v'])
        self.file.flush()

        self.point_id = 0
        self.last_x = None
        self.last_y = None

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info('route_recorder started')
        self.get_logger().info(f'odom_topic: {self.odom_topic}')
        self.get_logger().info(f'route_file: {self.route_file}')
        self.get_logger().info(f'min_distance: {self.min_distance}')
        if self.startup_enable_file:
            self.get_logger().info(
                'ROUTE_RECORDER_GATE_ACTIVE '
                f'file={self.startup_enable_file}'
            )

    def odom_callback(self, msg):
        if (
            self.startup_enable_file
            and not os.path.isfile(self.startup_enable_file)
        ):
            return
        if not self.recording_enabled:
            self.recording_enabled = True
            self.point_id = 0
            self.last_x = None
            self.last_y = None
            self.get_logger().info(
                'ROUTE_RECORDER_GATE_RELEASED '
                f'file={self.startup_enable_file}'
            )

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        should_save = False

        if self.last_x is None:
            should_save = True
        else:
            dist = math.hypot(x - self.last_x, y - self.last_y)
            if dist >= self.min_distance:
                should_save = True

        if should_save:
            self.writer.writerow([
                self.point_id,
                f'{x:.6f}',
                f'{y:.6f}',
                f'{yaw:.6f}',
                f'{self.default_speed:.3f}'
            ])
            self.file.flush()

            self.get_logger().info(
                f'saved point {self.point_id}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}'
            )

            self.point_id += 1
            self.last_x = x
            self.last_y = y

    def destroy_node(self):
        try:
            self.file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RouteRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('route_recorder stopped by Ctrl+C')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
