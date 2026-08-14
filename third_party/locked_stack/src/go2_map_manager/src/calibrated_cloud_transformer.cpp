// Copyright 2026 GoGoGuard

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class CalibratedCloudTransformer : public rclcpp::Node
{
public:
  CalibratedCloudTransformer()
  : Node("calibrated_cloud_transformer")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/navigation/cloud_lidar");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/navigation/cloud_body");
    source_frame_ = declare_parameter<std::string>("source_frame", "lidar_link");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");

    if (source_frame_.empty() || target_frame_.empty() || source_frame_ == target_frame_) {
      throw std::runtime_error("source_frame and target_frame must be distinct and non-empty");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CalibratedCloudTransformer::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "calibrated cloud path: %s [%s] -> %s [%s]",
      input_topic_.c_str(), source_frame_.c_str(), output_topic_.c_str(),
      target_frame_.c_str());
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (message->header.frame_id != source_frame_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "rejecting cloud with frame_id '%s'; expected '%s'",
        message->header.frame_id.c_str(), source_frame_.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      // The mount transform is static.  TimePointZero also makes startup
      // deterministic when the first cloud arrives immediately after TF.
      transform = tf_buffer_->lookupTransform(
        target_frame_, source_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "validated mount TF unavailable; body cloud withheld: %s", error.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> source;
    pcl::fromROSMsg(*message, source);

    const auto & translation = transform.transform.translation;
    const auto & rotation = transform.transform.rotation;
    Eigen::Quaternionf quaternion(
      static_cast<float>(rotation.w),
      static_cast<float>(rotation.x),
      static_cast<float>(rotation.y),
      static_cast<float>(rotation.z));
    const float norm = quaternion.norm();
    if (!std::isfinite(norm) || norm < 1.0e-6F) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "mount TF contains an invalid quaternion; body cloud withheld");
      return;
    }
    quaternion.normalize();

    Eigen::Affine3f affine = Eigen::Affine3f::Identity();
    affine.translation() <<
      static_cast<float>(translation.x),
      static_cast<float>(translation.y),
      static_cast<float>(translation.z);
    affine.linear() = quaternion.toRotationMatrix();

    pcl::PointCloud<pcl::PointXYZI> target;
    pcl::transformPointCloud(source, target, affine);
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(target, output);
    output.header = message->header;
    output.header.frame_id = target_frame_;
    publisher_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string source_frame_;
  std::string target_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CalibratedCloudTransformer>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("calibrated_cloud_transformer"), "%s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
