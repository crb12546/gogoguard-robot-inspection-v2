#!/usr/bin/env python3
import json
import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String
from unitree_api.msg import Request

from .safety_core import (
    limit_planar_command,
    localization_gate,
    pointcloud_xyz_layout,
    point_in_lateral_motion_roi,
    runtime_authorization_gate,
    safety_status_payload,
    stream_receive_age,
)


SPORT_API_ID_MOVE = 1008


def timing_summary(values):
    valid = sorted(
        float(value)
        for value in values
        if math.isfinite(float(value))
    )
    if not valid:
        return 0.0, 0.0, 0.0
    p95_index = max(0, int(math.ceil(len(valid) * 0.95)) - 1)
    return (
        sum(valid) / len(valid),
        valid[p95_index],
        valid[-1],
    )


class UnitreeSafeCmdNode(Node):
    def __init__(self):
        super().__init__('unitree_safe_cmd_node')

        self.declare_parameter('cmd_topic', '/patrol_cmd')
        self.declare_parameter('pointcloud_topic', '/cloud_registered_body')
        self.declare_parameter('expected_cloud_frame', 'base_link')
        self.declare_parameter('sport_request_topic', '/api/sport/request')
        self.declare_parameter('output_cmd_topic', '')
        self.declare_parameter('safety_status_topic', '/go2/safety/status')

        self.declare_parameter('max_vx', 0.5)
        self.declare_parameter('max_vy', 0.15)
        self.declare_parameter('max_yaw_rate', 0.45)
        self.declare_parameter('publish_rate', 40.0)
        self.declare_parameter('cmd_timeout', 0.20)
        self.declare_parameter('cloud_timeout', 0.20)
        self.declare_parameter('require_obstacle_gate', True)
        self.declare_parameter('require_localization', True)
        self.declare_parameter('localization_status_topic', '/localization/status')
        self.declare_parameter('localization_timeout', 0.20)
        self.declare_parameter('expected_map_version', '')
        self.declare_parameter('require_runtime_authorization', True)
        self.declare_parameter(
            'runtime_authorization_topic',
            '/go2/runtime/motion_authorized',
        )
        self.declare_parameter('runtime_authorization_timeout', 0.20)

        self.declare_parameter('roi_x_min', 0.35)
        self.declare_parameter('roi_x_max', 1.20)
        self.declare_parameter('roi_y_min', -0.45)
        self.declare_parameter('roi_y_max', 0.45)
        self.declare_parameter('roi_z_min', 0.25)
        self.declare_parameter('roi_z_max', 0.90)

        self.declare_parameter('lateral_cmd_deadband', 0.02)
        self.declare_parameter('lateral_roi_x_min', 0.10)
        self.declare_parameter('lateral_roi_x_max', 1.00)
        self.declare_parameter('lateral_roi_inner_y', 0.30)
        self.declare_parameter('lateral_roi_outer_y', 0.65)
        self.declare_parameter('lateral_min_stop_points', 12)
        self.declare_parameter('diagnostic_corridor_x_min', 0.35)
        self.declare_parameter('diagnostic_corridor_x_max', 2.50)
        self.declare_parameter('diagnostic_corridor_inner_y', 0.30)
        self.declare_parameter('diagnostic_corridor_outer_y', 1.20)

        self.declare_parameter('stop_distance', 0.70)
        self.declare_parameter('resume_distance', 0.95)
        self.declare_parameter('min_stop_points', 12)
        self.declare_parameter('stop_frames', 1)
        self.declare_parameter('clear_frames', 5)
        self.declare_parameter('point_skip', 2)
        self.declare_parameter('max_cloud_process_rate', 20.0)
        self.declare_parameter('min_valid_cloud_points', 50)
        self.declare_parameter('max_future_cloud_stamp_s', 0.50)

        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.expected_cloud_frame = str(
            self.get_parameter('expected_cloud_frame').value
        ).strip()
        if not self.expected_cloud_frame:
            raise RuntimeError('expected_cloud_frame is required')
        self.sport_request_topic = self.get_parameter('sport_request_topic').value
        self.output_cmd_topic = str(self.get_parameter('output_cmd_topic').value)
        self.safety_status_topic = str(
            self.get_parameter('safety_status_topic').value
        ).strip()
        if not self.safety_status_topic:
            raise RuntimeError('safety_status_topic is required')

        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.cloud_timeout = float(self.get_parameter('cloud_timeout').value)
        self.require_obstacle_gate = bool(
            self.get_parameter('require_obstacle_gate').value
        )
        self.require_localization = bool(
            self.get_parameter('require_localization').value
        )
        self.localization_status_topic = str(
            self.get_parameter('localization_status_topic').value
        ).strip()
        self.localization_timeout = float(
            self.get_parameter('localization_timeout').value
        )
        self.expected_map_version = str(
            self.get_parameter('expected_map_version').value
        ).strip()
        self.require_runtime_authorization = bool(
            self.get_parameter('require_runtime_authorization').value
        )
        self.runtime_authorization_topic = str(
            self.get_parameter('runtime_authorization_topic').value
        ).strip()
        self.runtime_authorization_timeout = float(
            self.get_parameter('runtime_authorization_timeout').value
        )
        if self.require_localization and not self.expected_map_version:
            raise RuntimeError(
                'EXPECTED_MAP_VERSION_REQUIRED_WHEN_LOCALIZATION_GATE_ENABLED'
            )

        self.roi_x_min = float(self.get_parameter('roi_x_min').value)
        self.roi_x_max = float(self.get_parameter('roi_x_max').value)
        self.roi_y_min = float(self.get_parameter('roi_y_min').value)
        self.roi_y_max = float(self.get_parameter('roi_y_max').value)
        self.roi_z_min = float(self.get_parameter('roi_z_min').value)
        self.roi_z_max = float(self.get_parameter('roi_z_max').value)

        self.lateral_cmd_deadband = float(self.get_parameter('lateral_cmd_deadband').value)
        self.lateral_roi_x_min = float(self.get_parameter('lateral_roi_x_min').value)
        self.lateral_roi_x_max = float(self.get_parameter('lateral_roi_x_max').value)
        self.lateral_roi_inner_y = float(self.get_parameter('lateral_roi_inner_y').value)
        self.lateral_roi_outer_y = float(self.get_parameter('lateral_roi_outer_y').value)
        self.lateral_min_stop_points = int(self.get_parameter('lateral_min_stop_points').value)
        self.diagnostic_corridor_x_min = float(
            self.get_parameter('diagnostic_corridor_x_min').value
        )
        self.diagnostic_corridor_x_max = float(
            self.get_parameter('diagnostic_corridor_x_max').value
        )
        self.diagnostic_corridor_inner_y = abs(float(
            self.get_parameter('diagnostic_corridor_inner_y').value
        ))
        self.diagnostic_corridor_outer_y = abs(float(
            self.get_parameter('diagnostic_corridor_outer_y').value
        ))
        if (
            self.diagnostic_corridor_x_max <= self.diagnostic_corridor_x_min
            or self.diagnostic_corridor_outer_y
            <= self.diagnostic_corridor_inner_y
        ):
            raise RuntimeError('diagnostic corridor bounds are invalid')

        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.resume_distance = float(self.get_parameter('resume_distance').value)
        self.min_stop_points = int(self.get_parameter('min_stop_points').value)
        self.stop_frames = int(self.get_parameter('stop_frames').value)
        self.clear_frames = int(self.get_parameter('clear_frames').value)
        self.point_skip = max(1, int(self.get_parameter('point_skip').value))
        self.max_cloud_process_rate = float(self.get_parameter('max_cloud_process_rate').value)
        self.min_valid_cloud_points = int(
            self.get_parameter('min_valid_cloud_points').value
        )
        self.max_future_cloud_stamp_s = float(
            self.get_parameter('max_future_cloud_stamp_s').value
        )
        if self.publish_rate <= 0.0:
            raise RuntimeError('publish_rate must be positive')
        safety_timeouts = [self.cmd_timeout]
        if self.require_obstacle_gate:
            safety_timeouts.append(self.cloud_timeout)
        if self.require_localization:
            safety_timeouts.append(self.localization_timeout)
        if self.require_runtime_authorization:
            safety_timeouts.append(self.runtime_authorization_timeout)
        if min(safety_timeouts) <= 0.0 or max(safety_timeouts) > 0.20:
            raise RuntimeError(
                'safety stream timeouts must be in (0, 0.20] seconds'
            )
        if self.max_cloud_process_rate <= 0.0:
            raise RuntimeError('max_cloud_process_rate must be positive')
        if self.require_obstacle_gate and self.max_cloud_process_rate * self.cloud_timeout < 2.0:
            raise RuntimeError(
                'cloud process rate must provide at least two checks per timeout'
            )
        if self.min_valid_cloud_points <= 0:
            raise RuntimeError('min_valid_cloud_points must be positive')
        if not 0.05 <= self.max_future_cloud_stamp_s <= 1.0:
            raise RuntimeError('max_future_cloud_stamp_s must be in [0.05, 1.0]')
        if self.resume_distance < self.stop_distance:
            raise RuntimeError('resume_distance must not be below stop_distance')

        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_yaw_rate = 0.0
        self.last_cmd_time = 0.0
        self.last_log_time = 0.0
        self.last_cloud_time = 0.0
        self.last_cloud_stamp_age = float('inf')
        self.last_cloud_process_time = 0.0
        self.last_cloud_validity_reason = 'cloud_missing'
        self.last_valid_cloud_point_count = 0
        self.last_localization_status = None
        self.last_localization_time = 0.0
        self.last_localization_reason = 'localization_status_missing'
        self.runtime_authorized = False
        self.last_runtime_authorization_time = 0.0
        self.timing_period = 1.0 / self.publish_rate
        self.timing_last_timer_mono = 0.0
        self.timing_last_cmd_mono = 0.0
        self.timing_last_cloud_mono = 0.0
        self.timing_last_report_mono = time.monotonic()
        self.timing_last_alert_mono = 0.0
        self.timing_timer_gaps_ms = []
        self.timing_timer_compute_ms = []
        self.timing_cmd_gaps_ms = []
        self.timing_cmd_callback_ms = []
        self.timing_cloud_gaps_ms = []
        self.timing_cloud_callback_ms = []
        self.timing_publish_ms = []
        self.timing_cmd_ages_ms = []
        self.timing_cloud_ages_ms = []
        self.timing_cmd_count = 0
        self.timing_output_count = 0
        self.timing_override_count = 0
        self.timing_deadline_misses = 0

        self.obstacle_stop = False
        self.stop_frame_count = 0
        self.clear_frame_count = 0

        self.last_stop_count = 0
        self.last_roi_count = 0
        self.last_nearest_x = float('inf')
        self.last_lateral_count = 0
        self.last_nearest_lateral_y = float('inf')
        self.last_left_corridor_count = 0
        self.last_right_corridor_count = 0
        self.last_left_corridor_nearest_x = float('inf')
        self.last_right_corridor_nearest_x = float('inf')

        self.pub = None
        self.twist_pub = None
        if self.output_cmd_topic:
            self.twist_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        else:
            self.pub = self.create_publisher(Request, self.sport_request_topic, 10)
        self.safety_status_pub = self.create_publisher(
            String,
            self.safety_status_topic,
            10,
        )
        self.create_subscription(Twist, self.cmd_topic, self.cmd_callback, 10)
        self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )
        if self.require_localization:
            self.create_subscription(
                String,
                self.localization_status_topic,
                self.localization_callback,
                10,
            )
        if self.require_runtime_authorization:
            self.create_subscription(
                Bool,
                self.runtime_authorization_topic,
                self.runtime_authorization_callback,
                10,
            )
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            f'unitree_safe_cmd_node ACTIVE: obstacle -> Move(0,0,0), '
            f'resume after {self.clear_frames} clear frames; lateral swept ROI enabled'
        )
        self.get_logger().info(
            'cloud safety freshness uses local receive time; '
            'message header stamp is diagnostic only, '
            f'receive_timeout={self.cloud_timeout:.2f}s'
        )
        if self.twist_pub is not None:
            self.get_logger().info(f'output mode: Twist -> {self.output_cmd_topic}')
        else:
            self.get_logger().info(
                'output mode: unitree_api Request -> '
                f'{self.sport_request_topic}'
            )
        if self.require_localization:
            self.get_logger().info(
                'localization safety gate enabled: '
                f'map_version={self.expected_map_version} '
                f'timeout={self.localization_timeout:.2f}s'
            )
        if self.require_runtime_authorization:
            self.get_logger().info(
                'runtime motion authorization enabled: '
                f'topic={self.runtime_authorization_topic} '
                f'timeout={self.runtime_authorization_timeout:.2f}s'
            )

    def localization_callback(self, msg):
        receive_time = time.time()
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            self.last_localization_status = None
            self.last_localization_time = receive_time
            self.last_localization_reason = 'localization_status_json'
            return
        self.last_localization_status = payload
        self.last_localization_time = receive_time
        _, self.last_localization_reason = localization_gate(
            payload,
            self.expected_map_version,
            0.0,
            self.localization_timeout,
        )

    def runtime_authorization_callback(self, msg):
        self.runtime_authorized = msg.data is True
        self.last_runtime_authorization_time = time.time()

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def cmd_callback(self, msg):
        callback_start = time.monotonic()
        if self.timing_last_cmd_mono > 0.0:
            self.timing_cmd_gaps_ms.append(
                (callback_start - self.timing_last_cmd_mono) * 1000.0
            )
        self.timing_last_cmd_mono = callback_start
        try:
            (
                self.last_vx,
                self.last_vy,
                self.last_yaw_rate,
            ) = limit_planar_command(
                msg.linear.x,
                msg.linear.y,
                msg.angular.z,
                self.max_vx,
                self.max_vy,
                self.max_yaw_rate,
            )
            self.last_cmd_time = time.time()
            self.timing_cmd_count += 1
        finally:
            self.timing_cmd_callback_ms.append(
                (time.monotonic() - callback_start) * 1000.0
            )

    def make_move_request(self, vx, vy, yaw_rate):
        req = Request()
        req.header.identity.id = 9100
        req.header.identity.api_id = SPORT_API_ID_MOVE
        req.header.lease.id = 0
        req.header.policy.priority = 0
        req.header.policy.noreply = False
        req.parameter = json.dumps({
            'x': float(vx),
            'y': float(vy),
            'z': float(yaw_rate)
        })
        req.binary = []
        return req

    def publish_move(self, vx, vy, yaw_rate):
        publish_start = time.monotonic()
        if self.twist_pub is not None:
            cmd = Twist()
            cmd.linear.x = float(vx)
            cmd.linear.y = float(vy)
            cmd.angular.z = float(yaw_rate)
            self.twist_pub.publish(cmd)
        else:
            if self.pub is None:
                raise RuntimeError('no command output publisher is configured')
            self.pub.publish(self.make_move_request(vx, vy, yaw_rate))
        self.timing_publish_ms.append(
            (time.monotonic() - publish_start) * 1000.0
        )
        self.timing_output_count += 1

    def get_xyz_layout(self, msg):
        fields = {
            field.name: (field.offset, field.datatype)
            for field in msg.fields
            if field.name in ('x', 'y', 'z')
        }
        return pointcloud_xyz_layout(
            fields,
            msg.point_step,
            msg.row_step,
            msg.width,
            msg.height,
            len(msg.data),
            PointField.FLOAT32,
        )

    def in_roi(self, x, y, z):
        return (
            self.roi_x_min <= x <= self.roi_x_max
            and self.roi_y_min <= y <= self.roi_y_max
            and self.roi_z_min <= z <= self.roi_z_max
        )

    def cloud_callback(self, msg):
        callback_start = time.monotonic()
        if self.timing_last_cloud_mono > 0.0:
            self.timing_cloud_gaps_ms.append(
                (callback_start - self.timing_last_cloud_mono) * 1000.0
            )
        self.timing_last_cloud_mono = callback_start
        try:
            self.process_cloud_message(msg)
        finally:
            self.timing_cloud_callback_ms.append(
                (time.monotonic() - callback_start) * 1000.0
            )

    def process_cloud_message(self, msg):
        if msg.header.frame_id != self.expected_cloud_frame:
            self.last_cloud_validity_reason = 'cloud_frame_mismatch'
            self.get_logger().error(
                'rejecting safety cloud frame %s; expected %s'
                % (msg.header.frame_id, self.expected_cloud_frame)
            )
            return
        now = time.time()
        min_period = 1.0 / max(self.max_cloud_process_rate, 1.0)
        if now - self.last_cloud_process_time < min_period:
            return
        self.last_cloud_process_time = now

        stamp_time = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )
        if stamp_time > now + self.max_future_cloud_stamp_s:
            # A future cloud makes the odom -> map TF chain impossible. Do not
            # refresh the receive watchdog merely because structurally valid
            # bytes arrived; fail closed until the common sensor epoch is sane.
            self.last_cloud_validity_reason = 'sensor_clock_future'
            self.last_valid_cloud_point_count = 0
            self.last_cloud_stamp_age = float('inf')
            return

        offsets, layout_reason = self.get_xyz_layout(msg)
        if offsets is None:
            self.last_cloud_validity_reason = layout_reason
            self.last_valid_cloud_point_count = 0
            self.get_logger().warn(
                'rejecting invalid PointCloud2 safety input: %s' % layout_reason
            )
            return

        endian = '>' if msg.is_bigendian else '<'
        data = msg.data
        point_step = msg.point_step
        roi_count = 0
        stop_count = 0
        nearest_x = float('inf')
        lateral_count = 0
        nearest_lateral_y = float('inf')
        left_corridor_count = 0
        right_corridor_count = 0
        left_corridor_nearest_x = float('inf')
        right_corridor_nearest_x = float('inf')
        valid_point_count = 0

        flat_index = 0
        for row in range(msg.height):
            row_base = row * msg.row_step
            for column in range(msg.width):
                should_sample = flat_index % self.point_skip == 0
                flat_index += 1
                if not should_sample:
                    continue
                base = row_base + column * point_step
                try:
                    x = struct.unpack_from(endian + 'f', data, base + offsets['x'])[0]
                    y = struct.unpack_from(endian + 'f', data, base + offsets['y'])[0]
                    z = struct.unpack_from(endian + 'f', data, base + offsets['z'])[0]
                except struct.error:
                    self.last_cloud_validity_reason = 'cloud_data_truncated'
                    self.last_valid_cloud_point_count = 0
                    return

                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue
                valid_point_count += 1

                if self.in_roi(x, y, z):
                    roi_count += 1
                    nearest_x = min(nearest_x, x)
                    active_stop_distance = (
                        self.resume_distance
                        if self.obstacle_stop
                        else self.stop_distance
                    )
                    if x <= active_stop_distance:
                        stop_count += 1

                if point_in_lateral_motion_roi(
                    x,
                    y,
                    z,
                    self.last_vy,
                    cmd_deadband=self.lateral_cmd_deadband,
                    x_min=self.lateral_roi_x_min,
                    x_max=self.lateral_roi_x_max,
                    inner_y=self.lateral_roi_inner_y,
                    outer_y=self.lateral_roi_outer_y,
                    z_min=self.roi_z_min,
                    z_max=self.roi_z_max,
                ):
                    lateral_count += 1
                    nearest_lateral_y = min(nearest_lateral_y, abs(y))

                if (
                    self.diagnostic_corridor_x_min
                    <= x
                    <= self.diagnostic_corridor_x_max
                    and self.roi_z_min <= z <= self.roi_z_max
                ):
                    if (
                        self.diagnostic_corridor_inner_y
                        <= y
                        <= self.diagnostic_corridor_outer_y
                    ):
                        left_corridor_count += 1
                        left_corridor_nearest_x = min(
                            left_corridor_nearest_x, x
                        )
                    elif (
                        -self.diagnostic_corridor_outer_y
                        <= y
                        <= -self.diagnostic_corridor_inner_y
                    ):
                        right_corridor_count += 1
                        right_corridor_nearest_x = min(
                            right_corridor_nearest_x, x
                        )

        self.last_valid_cloud_point_count = valid_point_count
        if valid_point_count < self.min_valid_cloud_points:
            self.last_cloud_validity_reason = 'cloud_insufficient_valid_points'
            return

        # Only a structurally valid cloud with enough finite XYZ samples may
        # refresh the motion-safety watchdog.  Receipt of malformed/all-NaN
        # messages must therefore become a cloud timeout and a zero command.
        self.last_cloud_time = now
        self.last_cloud_validity_reason = 'cloud_valid'
        if stamp_time <= 0.0:
            stamp_time = now
        self.last_cloud_stamp_age = max(0.0, now - stamp_time)

        self.last_roi_count = roi_count
        self.last_stop_count = stop_count
        self.last_nearest_x = nearest_x
        self.last_lateral_count = lateral_count
        self.last_nearest_lateral_y = nearest_lateral_y
        self.last_left_corridor_count = left_corridor_count
        self.last_right_corridor_count = right_corridor_count
        self.last_left_corridor_nearest_x = left_corridor_nearest_x
        self.last_right_corridor_nearest_x = right_corridor_nearest_x

        unsafe = (
            stop_count >= self.min_stop_points
            or lateral_count >= self.lateral_min_stop_points
        )

        if unsafe:
            self.stop_frame_count += 1
            self.clear_frame_count = 0
        else:
            self.clear_frame_count += 1
            self.stop_frame_count = 0

        if not self.obstacle_stop and self.stop_frame_count >= self.stop_frames:
            self.obstacle_stop = True
            nearest_text = 'inf' if nearest_x == float('inf') else f'{nearest_x:.2f}'
            self.get_logger().warn(
                f'OBSTACLE DETECTED: stop_count={stop_count}, roi_count={roi_count}, '
                f'lateral_count={lateral_count}, nearest_x={nearest_text}'
            )

        # 关键：连续 clear_frames 帧没有危险点，才恢复
        if self.obstacle_stop and self.clear_frame_count >= self.clear_frames:
            self.obstacle_stop = False
            self.get_logger().info(
                f'obstacle cleared after {self.clear_frame_count} clear frames, resume move'
            )

    def timer_callback(self):
        loop_start = time.monotonic()
        if self.timing_last_timer_mono > 0.0:
            gap_ms = (
                loop_start - self.timing_last_timer_mono
            ) * 1000.0
            self.timing_timer_gaps_ms.append(gap_ms)
            if gap_ms > self.timing_period * 1500.0:
                self.timing_deadline_misses += 1
        self.timing_last_timer_mono = loop_start
        try:
            self.publish_safe_cycle()
        finally:
            self.timing_timer_compute_ms.append(
                (time.monotonic() - loop_start) * 1000.0
            )
            self.maybe_log_timing(time.monotonic())

    def publish_safe_cycle(self):
        now = time.time()
        command_age = stream_receive_age(now, self.last_cmd_time)
        cloud_receive_age = stream_receive_age(
            now,
            self.last_cloud_time,
        )
        localization_receive_age = stream_receive_age(
            now,
            self.last_localization_time,
        )
        localization_ready = True
        localization_reason = 'localization_not_required'
        if self.require_localization:
            localization_ready, localization_reason = localization_gate(
                self.last_localization_status,
                self.expected_map_version,
                localization_receive_age,
                self.localization_timeout,
            )
            self.last_localization_reason = localization_reason
        authorization_ready = True
        authorization_reason = 'runtime_authorization_not_required'
        authorization_receive_age = stream_receive_age(
            now,
            self.last_runtime_authorization_time,
        )
        if self.require_runtime_authorization:
            authorization_ready, authorization_reason = runtime_authorization_gate(
                self.runtime_authorized,
                authorization_receive_age,
                self.runtime_authorization_timeout,
            )

        out_vx = self.last_vx
        out_vy = self.last_vy
        out_yaw_rate = self.last_yaw_rate
        reason = 'normal'

        if not authorization_ready:
            out_vx, out_vy, out_yaw_rate = (0.0, 0.0, 0.0)
            reason = authorization_reason

        elif not localization_ready:
            out_vx, out_vy, out_yaw_rate = (0.0, 0.0, 0.0)
            reason = localization_reason

        elif command_age > self.cmd_timeout:
            out_vx, out_vy, out_yaw_rate = (0.0, 0.0, 0.0)
            reason = 'cmd_timeout'

        elif self.require_obstacle_gate and cloud_receive_age > self.cloud_timeout:
            out_vx, out_vy, out_yaw_rate = (0.0, 0.0, 0.0)
            reason = 'cloud_timeout'

        elif self.require_obstacle_gate and self.obstacle_stop:
            out_vx, out_vy, out_yaw_rate = (0.0, 0.0, 0.0)
            reason = 'obstacle'

        if math.isfinite(command_age):
            self.timing_cmd_ages_ms.append(command_age * 1000.0)
        if math.isfinite(cloud_receive_age):
            self.timing_cloud_ages_ms.append(
                cloud_receive_age * 1000.0
            )
        if reason != 'normal':
            self.timing_override_count += 1
        self.publish_move(out_vx, out_vy, out_yaw_rate)
        status_message = String()
        status_message.data = json.dumps(
            safety_status_payload(
                timestamp=now,
                expected_map_version=self.expected_map_version,
                localization_status=self.last_localization_status,
                localization_ready=localization_ready,
                authorization_ready=authorization_ready,
                command_age=command_age,
                command_timeout=self.cmd_timeout,
                cloud_age=cloud_receive_age,
                cloud_timeout=self.cloud_timeout,
                obstacle_stop=self.obstacle_stop,
                obstacle_distance=self.last_nearest_x,
                stop_reason=reason,
                cloud_validity_reason=self.last_cloud_validity_reason,
                cloud_valid_point_count=self.last_valid_cloud_point_count,
                localization_age=localization_receive_age,
                localization_timeout=self.localization_timeout,
                authorization_age=authorization_receive_age,
                authorization_timeout=self.runtime_authorization_timeout,
                obstacle_stop_point_count=self.last_stop_count,
                obstacle_roi_point_count=self.last_roi_count,
                lateral_stop_point_count=self.last_lateral_count,
                lateral_obstacle_distance=self.last_nearest_lateral_y,
                left_corridor_point_count=self.last_left_corridor_count,
                right_corridor_point_count=self.last_right_corridor_count,
                left_corridor_nearest_x=self.last_left_corridor_nearest_x,
                right_corridor_nearest_x=self.last_right_corridor_nearest_x,
            ),
            ensure_ascii=False,
            separators=(',', ':'),
        )
        self.safety_status_pub.publish(status_message)

        if now - self.last_log_time > 0.5:
            self.last_log_time = now
            nearest_text = (
                'inf'
                if self.last_nearest_x == float('inf')
                else f'{self.last_nearest_x:.2f}'
            )

            if reason == 'normal':
                self.get_logger().info(
                    f'Move x={out_vx:.3f}, y={out_vy:.3f}, z={out_yaw_rate:.3f}, '
                    f'stop_count={self.last_stop_count}, roi_count={self.last_roi_count}, '
                    f'lateral_count={self.last_lateral_count}, '
                    f'nearest_x={nearest_text}, clear_frames={self.clear_frame_count}, '
                    f'cloud_receive_age={cloud_receive_age:.3f}s, '
                    f'cloud_stamp_age={self.last_cloud_stamp_age:.3f}s'
                    f', localization_age={localization_receive_age:.3f}s'
                )
            else:
                self.get_logger().warn(
                    f'SAFE OVERRIDE {reason}: Move x=0.000, y=0.000, z=0.000, '
                    f'raw_x={self.last_vx:.3f}, raw_y={self.last_vy:.3f}, '
                    f'raw_z={self.last_yaw_rate:.3f}, '
                    f'stop_count={self.last_stop_count}, roi_count={self.last_roi_count}, '
                    f'lateral_count={self.last_lateral_count}, '
                    f'nearest_x={nearest_text}, clear_frames={self.clear_frame_count}, '
                    f'cloud_receive_age={cloud_receive_age:.3f}s, '
                    f'cloud_stamp_age={self.last_cloud_stamp_age:.3f}s'
                    f', localization_age={localization_receive_age:.3f}s'
                )

    def maybe_log_timing(self, now_mono):
        if now_mono - self.timing_last_report_mono < 1.0:
            return

        timer_avg, timer_p95, timer_max = timing_summary(
            self.timing_timer_gaps_ms
        )
        compute_avg, compute_p95, compute_max = timing_summary(
            self.timing_timer_compute_ms
        )
        cmd_gap_avg, cmd_gap_p95, cmd_gap_max = timing_summary(
            self.timing_cmd_gaps_ms
        )
        cmd_cb_avg, cmd_cb_p95, cmd_cb_max = timing_summary(
            self.timing_cmd_callback_ms
        )
        cloud_gap_avg, cloud_gap_p95, cloud_gap_max = timing_summary(
            self.timing_cloud_gaps_ms
        )
        cloud_cb_avg, cloud_cb_p95, cloud_cb_max = timing_summary(
            self.timing_cloud_callback_ms
        )
        publish_avg, publish_p95, publish_max = timing_summary(
            self.timing_publish_ms
        )
        cmd_age_avg, cmd_age_p95, cmd_age_max = timing_summary(
            self.timing_cmd_ages_ms
        )
        cloud_age_avg, cloud_age_p95, cloud_age_max = timing_summary(
            self.timing_cloud_ages_ms
        )
        self.get_logger().info(
            'TIMING_SAFE '
            f'period_ms={self.timing_period * 1000.0:.1f} '
            f'timer_gap_ms={timer_avg:.3f}/{timer_p95:.3f}/'
            f'{timer_max:.3f} '
            f'compute_ms={compute_avg:.3f}/{compute_p95:.3f}/'
            f'{compute_max:.3f} '
            f'cmd_gap_ms={cmd_gap_avg:.3f}/{cmd_gap_p95:.3f}/'
            f'{cmd_gap_max:.3f} '
            f'cmd_callback_ms={cmd_cb_avg:.3f}/{cmd_cb_p95:.3f}/'
            f'{cmd_cb_max:.3f} '
            f'cmd_age_ms={cmd_age_avg:.3f}/{cmd_age_p95:.3f}/'
            f'{cmd_age_max:.3f} '
            f'cloud_gap_ms={cloud_gap_avg:.3f}/{cloud_gap_p95:.3f}/'
            f'{cloud_gap_max:.3f} '
            f'cloud_callback_ms={cloud_cb_avg:.3f}/{cloud_cb_p95:.3f}/'
            f'{cloud_cb_max:.3f} '
            f'cloud_age_ms={cloud_age_avg:.3f}/{cloud_age_p95:.3f}/'
            f'{cloud_age_max:.3f} '
            f'ros_publish_ms={publish_avg:.3f}/{publish_p95:.3f}/'
            f'{publish_max:.3f} '
            f'deadline_miss={self.timing_deadline_misses} '
            f'cmd_count={self.timing_cmd_count} '
            f'output_count={self.timing_output_count} '
            f'override_count={self.timing_override_count}'
        )

        alert = (
            timer_max > self.timing_period * 1500.0
            or compute_max > self.timing_period * 500.0
            or cmd_gap_max > self.cmd_timeout * 500.0
            or cloud_cb_max > self.timing_period * 1000.0
        )
        if alert and now_mono - self.timing_last_alert_mono >= 1.0:
            self.timing_last_alert_mono = now_mono
            self.get_logger().warn(
                'TIMING_ALERT_SAFE '
                f'timer_gap_max_ms={timer_max:.3f} '
                f'compute_max_ms={compute_max:.3f} '
                f'cmd_gap_max_ms={cmd_gap_max:.3f} '
                f'cmd_age_max_ms={cmd_age_max:.3f} '
                f'cloud_callback_max_ms={cloud_cb_max:.3f} '
                f'cloud_age_max_ms={cloud_age_max:.3f}'
            )

        self.timing_timer_gaps_ms.clear()
        self.timing_timer_compute_ms.clear()
        self.timing_cmd_gaps_ms.clear()
        self.timing_cmd_callback_ms.clear()
        self.timing_cloud_gaps_ms.clear()
        self.timing_cloud_callback_ms.clear()
        self.timing_publish_ms.clear()
        self.timing_cmd_ages_ms.clear()
        self.timing_cloud_ages_ms.clear()
        self.timing_cmd_count = 0
        self.timing_output_count = 0
        self.timing_override_count = 0
        self.timing_deadline_misses = 0
        self.timing_last_report_mono = now_mono

    def destroy_node(self):
        if rclpy.ok():
            try:
                self.publish_move(0.0, 0.0, 0.0)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnitreeSafeCmdNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info('Ctrl+C, publish Move(0,0,0)')
            node.publish_move(0.0, 0.0, 0.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
