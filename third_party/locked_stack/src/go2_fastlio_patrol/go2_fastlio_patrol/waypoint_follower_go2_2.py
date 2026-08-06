#!/usr/bin/env python3
import csv
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from go2_fastlio_patrol.controller_core import (
    lookahead_index,
    nearest_global,
    nearest_window,
    target_command,
)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('cmd_topic', '/patrol_cmd')
        self.declare_parameter(
            'route_file',
            '/home/unitree/go2_fastlio_ws/src/'
            'go2_fastlio_patrol/routes/route_demo.csv',
        )

        self.declare_parameter('v_base', 0.25)
        self.declare_parameter('max_vx', 0.25)
        self.declare_parameter('k_yaw', 0.6)
        self.declare_parameter('max_yaw_rate', 0.30)

        self.declare_parameter('lookahead_distance', 0.8)
        self.declare_parameter('reach_distance', 0.4)
        self.declare_parameter('goal_distance', 0.5)

        self.declare_parameter('loop_mode', 'once')   # once / pingpong
        self.declare_parameter('search_window', 6)

        self.declare_parameter('turn_in_place_angle', 1.0)
        self.declare_parameter('slow_down_angle', 0.5)

        self.declare_parameter('stuck_time', 3.0)
        self.declare_parameter('relocalize_distance', 1.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.route_file = self.get_parameter('route_file').value

        self.v_base = float(self.get_parameter('v_base').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)

        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.reach_distance = float(self.get_parameter('reach_distance').value)
        self.goal_distance = float(self.get_parameter('goal_distance').value)

        self.loop_mode = self.get_parameter('loop_mode').value
        self.search_window = int(self.get_parameter('search_window').value)

        self.turn_in_place_angle = float(self.get_parameter('turn_in_place_angle').value)
        self.slow_down_angle = float(self.get_parameter('slow_down_angle').value)

        self.stuck_time = float(self.get_parameter('stuck_time').value)
        self.relocalize_distance = float(self.get_parameter('relocalize_distance').value)

        self.route = self.load_route(self.route_file)

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.direction = 1
        self.nearest_index = 0
        self.target_index = 0
        self.finished = False
        self.initialized = False

        self.last_progress_index = 0
        self.last_progress_time = time.time()
        self.last_log_time = 0.0

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('enhanced waypoint_follower started')
        self.get_logger().info(f'route_file: {self.route_file}')
        self.get_logger().info(f'route points: {len(self.route)}')
        self.get_logger().info(f'loop_mode: {self.loop_mode}')
        self.get_logger().info(f'lookahead_distance: {self.lookahead_distance}')
        self.get_logger().info(f'reach_distance: {self.reach_distance}')

    def load_route(self, path):
        route = []
        with open(path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                route.append({
                    'x': float(row['x']),
                    'y': float(row['y']),
                    'yaw': float(row['yaw']),
                    'v': float(row['v'])
                })

        if len(route) < 2:
            raise RuntimeError('route file must contain at least 2 points')

        return route

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        if not self.initialized:
            self.nearest_index = self.find_nearest_global()
            self.target_index = self.nearest_index
            self.last_progress_index = self.nearest_index
            self.last_progress_time = time.time()
            self.initialized = True

            d0 = self.distance_to_index(0)
            self.get_logger().info(
                f'init nearest_index={self.nearest_index}, '
                f'distance_to_start={d0:.3f}'
            )

    def distance_to_index(self, i):
        p = self.route[i]
        return math.hypot(p['x'] - self.current_x, p['y'] - self.current_y)

    def find_nearest_global(self):
        return nearest_global(
            self.route,
            self.current_x,
            self.current_y,
        )

    def find_nearest_window(self):
        return nearest_window(
            self.route,
            self.current_x,
            self.current_y,
            self.nearest_index,
            self.search_window,
        )

    def update_nearest_index(self):
        new_i, new_d = self.find_nearest_window()

        # 正向时索引只允许大体前进，不轻易倒退
        if self.direction > 0:
            if new_i >= self.nearest_index:
                self.nearest_index = min(new_i, self.nearest_index + 1)
        else:
            if new_i <= self.nearest_index:
                self.nearest_index = max(new_i, self.nearest_index - 1)

        # 如果偏离路线太远，允许全局重定位一次
        if new_d > self.relocalize_distance:
            global_i = self.find_nearest_global()
            self.get_logger().warn(
                f'far from local route: d={new_d:.2f}, relocalize '
                f'nearest_index {self.nearest_index} -> {global_i}'
            )
            self.nearest_index = global_i

        # 记录进度，给卡住检测使用
        if self.nearest_index != self.last_progress_index:
            self.last_progress_index = self.nearest_index
            self.last_progress_time = time.time()

    def compute_lookahead_index(self):
        return lookahead_index(
            self.route,
            self.nearest_index,
            self.direction,
            self.lookahead_distance,
        )

    def publish_stop(self):
        cmd = Twist()
        self.pub.publish(cmd)

    def handle_goal(self):
        n = len(self.route)

        if self.direction > 0:
            goal_i = n - 1
            if self.nearest_index < max(1, n - 2):
                return False
        else:
            goal_i = 0
            if self.nearest_index > min(n - 2, 1):
                return False

        dist_goal = self.distance_to_index(goal_i)

        if dist_goal > self.goal_distance:
            return False

        if self.loop_mode == 'pingpong':
            if self.direction > 0:
                self.direction = -1
                self.nearest_index = n - 1
                self.get_logger().info('reach end, switch to backward')
            else:
                self.direction = 1
                self.nearest_index = 0
                self.get_logger().info('reach start, switch to forward')

            self.last_progress_index = self.nearest_index
            self.last_progress_time = time.time()
            return False

        self.get_logger().info('goal reached, stop')
        self.finished = True
        self.publish_stop()
        return True

    def control_loop(self):
        if self.current_x is None:
            self.publish_stop()
            return

        if self.finished:
            self.publish_stop()
            return

        self.update_nearest_index()

        if self.handle_goal():
            return

        self.target_index = self.compute_lookahead_index()

        target = self.route[self.target_index]
        command = target_command(
            target,
            self.current_x,
            self.current_y,
            self.current_yaw,
            self.v_base,
            self.max_vx,
            self.k_yaw,
            self.max_yaw_rate,
            self.turn_in_place_angle,
            self.slow_down_angle,
        )
        dist_to_target = command['distance']
        alpha = command['alpha']
        vx = command['vx']
        yaw_rate = command['yaw_rate']

        # 卡住检测：长时间没有路线索引进展时，重新找最近点
        now = time.time()
        if now - self.last_progress_time > self.stuck_time:
            old_i = self.nearest_index
            self.nearest_index = self.find_nearest_global()
            self.last_progress_index = self.nearest_index
            self.last_progress_time = now
            self.get_logger().warn(
                f'stuck recovery: nearest_index {old_i} -> {self.nearest_index}'
            )

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = 0.0
        cmd.angular.z = float(yaw_rate)
        self.pub.publish(cmd)

        if now - self.last_log_time > 1.0:
            self.last_log_time = now
            self.get_logger().info(
                f'nearest={self.nearest_index}, target={self.target_index}, '
                f'dist={dist_to_target:.2f}, alpha={alpha:.2f}, '
                f'vx={vx:.2f}, yaw_rate={yaw_rate:.2f}, dir={self.direction}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('waypoint_follower stopped by Ctrl+C')
        node.publish_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
