"""Publish a navigation-only cloud with rigid robot self returns removed."""

from __future__ import annotations

from .obstacle_filter_core import SelfExclusionBox, filtered_xyz_points


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    class ObstacleCloudFilter(Node):
        def __init__(self) -> None:
            super().__init__("navigation_obstacle_cloud_filter")
            self.declare_parameter("input_topic", "/navigation/cloud_body")
            self.declare_parameter("output_topic", "/navigation/cloud_obstacles")
            self.declare_parameter("expected_frame", "base_link")
            self.declare_parameter("self_x_min", -0.35)
            self.declare_parameter("self_x_max", 0.42)
            self.declare_parameter("self_y_min", -0.22)
            self.declare_parameter("self_y_max", 0.22)
            self.declare_parameter("self_z_min", -0.10)
            self.declare_parameter("self_z_max", 1.05)

            self.expected_frame = str(
                self.get_parameter("expected_frame").value
            ).strip()
            if not self.expected_frame:
                raise RuntimeError("expected_frame is required")
            self.self_box = SelfExclusionBox(
                x_min=float(self.get_parameter("self_x_min").value),
                x_max=float(self.get_parameter("self_x_max").value),
                y_min=float(self.get_parameter("self_y_min").value),
                y_max=float(self.get_parameter("self_y_max").value),
                z_min=float(self.get_parameter("self_z_min").value),
                z_max=float(self.get_parameter("self_z_max").value),
            )
            self.self_box.validate()
            output_topic = str(self.get_parameter("output_topic").value).strip()
            input_topic = str(self.get_parameter("input_topic").value).strip()
            if not input_topic or not output_topic or input_topic == output_topic:
                raise RuntimeError("obstacle cloud topics must be distinct and non-empty")
            self.publisher = self.create_publisher(
                PointCloud2, output_topic, qos_profile_sensor_data
            )
            self.subscription = self.create_subscription(
                PointCloud2,
                input_topic,
                self.cloud_callback,
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                "navigation obstacle cloud active: %s -> %s; rigid self box "
                "x=[%.2f, %.2f] y=[%.2f, %.2f] z=[%.2f, %.2f]"
                % (
                    input_topic,
                    output_topic,
                    self.self_box.x_min,
                    self.self_box.x_max,
                    self.self_box.y_min,
                    self.self_box.y_max,
                    self.self_box.z_min,
                    self.self_box.z_max,
                )
            )

        def cloud_callback(self, message: PointCloud2) -> None:
            if message.header.frame_id != self.expected_frame:
                self.get_logger().error(
                    "rejecting obstacle cloud frame %s; expected %s"
                    % (message.header.frame_id, self.expected_frame)
                )
                return
            try:
                values = point_cloud2.read_points(
                    message,
                    field_names=("x", "y", "z"),
                    skip_nans=True,
                )
                filtered = list(
                    filtered_xyz_points(values, self_box=self.self_box)
                )
                output = point_cloud2.create_cloud_xyz32(message.header, filtered)
            except Exception as exc:
                self.get_logger().error("obstacle cloud filtering failed: %s" % exc)
                return
            self.publisher.publish(output)

    rclpy.init(args=None)
    node = ObstacleCloudFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
