// Copyright 2026 GoGoGuard

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "go2_map_manager/localization_core.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using MapIndex = pcl::KdTreeFLANN<pcl::PointXYZ>;
using Registration = small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ>;
using go2_map_manager::LocalizationState;
using go2_map_manager::LocalizationStateMachine;
using go2_map_manager::InitialEvidencePolicy;
using go2_map_manager::InitialPoseSeed;
using go2_map_manager::RegistrationAssessment;
using go2_map_manager::RegistrationMetrics;
using go2_map_manager::RegistrationThresholds;

double degrees(double radians)
{
  return radians * 180.0 / kPi;
}

double normalized_angle(double radians)
{
  return std::atan2(std::sin(radians), std::cos(radians));
}

double yaw_from_rotation(const Eigen::Matrix3d & rotation)
{
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

Eigen::Isometry3d pose_to_isometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond rotation(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (!std::isfinite(rotation.norm()) || rotation.norm() < 1.0e-9) {
    throw std::runtime_error("odometry contains an invalid quaternion");
  }
  rotation.normalize();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = rotation.toRotationMatrix();
  transform.translation() << pose.position.x, pose.position.y, pose.position.z;
  return transform;
}

geometry_msgs::msg::Quaternion quaternion_message(const Eigen::Matrix3d & rotation)
{
  const Eigen::Quaterniond quaternion(rotation);
  geometry_msgs::msg::Quaternion output;
  output.x = quaternion.x();
  output.y = quaternion.y();
  output.z = quaternion.z();
  output.w = quaternion.w();
  return output;
}

Eigen::Isometry3d yaw_transform(double x, double y, double z, double yaw)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() << x, y, z;
  transform.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return transform;
}

double rotation_distance(const Eigen::Isometry3d & first, const Eigen::Isometry3d & second)
{
  Eigen::Quaterniond delta(first.rotation().transpose() * second.rotation());
  delta.normalize();
  return std::abs(Eigen::AngleAxisd(delta).angle());
}

Eigen::Isometry3d blend_transform(
  const Eigen::Isometry3d & current,
  const Eigen::Isometry3d & candidate,
  double alpha,
  double max_translation_step,
  double max_rotation_step)
{
  const Eigen::Vector3d translation_delta = candidate.translation() - current.translation();
  const double translation_norm = translation_delta.norm();
  double translation_fraction = std::clamp(alpha, 0.0, 1.0);
  if (translation_norm > 1.0e-9) {
    translation_fraction = std::min(
      translation_fraction, max_translation_step / translation_norm);
  }

  const Eigen::Quaterniond current_rotation(current.rotation());
  const Eigen::Quaterniond candidate_rotation(candidate.rotation());
  const double angle = rotation_distance(current, candidate);
  double rotation_fraction = std::clamp(alpha, 0.0, 1.0);
  if (angle > 1.0e-9) {
    rotation_fraction = std::min(rotation_fraction, max_rotation_step / angle);
  }

  Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
  output.translation() = current.translation() + translation_fraction * translation_delta;
  output.linear() =
    current_rotation.slerp(rotation_fraction, candidate_rotation).toRotationMatrix();
  return output;
}

Cloud::Ptr filter_and_downsample(
  const Cloud::ConstPtr & input,
  double min_z,
  double max_z,
  double max_range,
  double leaf)
{
  auto finite = Cloud::Ptr(new Cloud());
  finite->reserve(input->size());
  const double max_range_squared = max_range * max_range;
  for (const auto & point : input->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    if (point.z < min_z || point.z > max_z) {
      continue;
    }
    if (max_range > 0.0 &&
      static_cast<double>(point.x) * point.x +
      static_cast<double>(point.y) * point.y > max_range_squared)
    {
      continue;
    }
    finite->push_back(point);
  }

  auto output = Cloud::Ptr(new Cloud());
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
  voxel.setInputCloud(finite);
  voxel.filter(*output);
  return output;
}

struct RegistrationRun
{
  Eigen::Isometry3d map_from_base = Eigen::Isometry3d::Identity();
  RegistrationMetrics metrics;
  double ranking_score = std::numeric_limits<double>::infinity();
};

RegistrationRun run_registration(
  Registration & registration,
  const Cloud::Ptr & scan,
  const Eigen::Isometry3d & initial_map_from_base,
  double correspondence_distance,
  double input_age_s,
  bool allow_unconverged_ranking = false,
  double minimum_ranking_inlier_ratio = 0.0)
{
  auto aligned = Cloud::Ptr(new Cloud());
  registration.align(*aligned, initial_map_from_base.matrix().cast<float>());
  const auto & result = registration.getRegistrationResult();

  RegistrationRun run;
  run.map_from_base = result.T_target_source;
  run.metrics.converged = result.converged;
  run.metrics.input_points = scan->size();
  run.metrics.inlier_ratio = scan->empty() ? 0.0 :
    static_cast<double>(result.num_inliers) / static_cast<double>(scan->size());
  run.metrics.fitness_mse = registration.getFitnessScore(correspondence_distance);
  run.metrics.input_age_s = input_age_s;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(result.H);
  if (solver.info() == Eigen::Success) {
    const auto eigenvalues = solver.eigenvalues();
    run.metrics.hessian_min_eigenvalue = std::max(0.0, eigenvalues.minCoeff());
    const double maximum = eigenvalues.maxCoeff();
    run.metrics.hessian_condition = run.metrics.hessian_min_eigenvalue > 0.0 ?
      maximum / run.metrics.hessian_min_eigenvalue : std::numeric_limits<double>::max();
  } else {
    run.metrics.hessian_min_eigenvalue = 0.0;
    run.metrics.hessian_condition = std::numeric_limits<double>::max();
  }
  if ((result.converged || allow_unconverged_ranking) &&
    std::isfinite(run.metrics.fitness_mse) &&
    run.metrics.inlier_ratio >= minimum_ranking_inlier_ratio)
  {
    run.ranking_score = run.metrics.fitness_mse +
      0.25 * (1.0 - std::clamp(run.metrics.inlier_ratio, 0.0, 1.0));
  }
  return run;
}

bool materially_different(
  const RegistrationRun & first,
  const RegistrationRun & second,
  double translation_m,
  double rotation_deg)
{
  return (first.map_from_base.translation() - second.map_from_base.translation()).norm() >=
         translation_m ||
         degrees(rotation_distance(first.map_from_base, second.map_from_base)) >= rotation_deg;
}

std::string json_number(double value)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream output;
  output << std::setprecision(8) << value;
  return output.str();
}

}  // namespace

class ContinuousMapLocalizer : public rclcpp::Node
{
public:
  ContinuousMapLocalizer()
  : Node("continuous_map_localizer")
  {
    map_file_ = declare_parameter<std::string>("map_file", "");
    map_version_ = declare_parameter<std::string>("map_version", "");
    quality_profile_id_ = declare_parameter<std::string>("quality.profile_id", "");
    registration_type_ = declare_parameter<std::string>("registration.type", "VGICP");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/navigation/cloud_body");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_odom_base_tf_ = declare_parameter<bool>("publish_odom_base_tf", false);
    update_period_s_ = declare_parameter<double>("update_period_s", 0.4);
    status_publish_period_s_ = declare_parameter<double>("status_publish_period_s", 0.1);
    max_odom_cloud_skew_s_ = declare_parameter<double>("max_odom_cloud_skew_s", 0.08);
    min_scan_z_ = declare_parameter<double>("min_scan_z", -1.5);
    max_scan_z_ = declare_parameter<double>("max_scan_z", 2.5);
    max_scan_range_ = declare_parameter<double>("max_scan_range", 35.0);
    tracking_scan_leaf_ = declare_parameter<double>("tracking_scan_leaf", 0.25);
    coarse_scan_leaf_ = declare_parameter<double>("coarse_scan_leaf", 0.65);
    tracking_map_leaf_ = declare_parameter<double>("tracking_map_leaf", 0.30);
    coarse_map_leaf_ = declare_parameter<double>("coarse_map_leaf", 0.80);
    registration_threads_ = declare_parameter<int>("registration_threads", 4);
    local_map_radius_m_ = declare_parameter<double>("local_map.radius_m", 45.0);
    local_map_refresh_distance_m_ = declare_parameter<double>(
      "local_map.refresh_distance_m", 5.0);
    initial_x_ = declare_parameter<double>("initialization.center_x", 0.0);
    initial_y_ = declare_parameter<double>("initialization.center_y", 0.0);
    initial_z_ = declare_parameter<double>("initialization.center_z", 0.0);
    initial_xy_radius_ = declare_parameter<double>("initialization.xy_radius", 5.0);
    initial_xy_step_ = declare_parameter<double>("initialization.xy_step", 2.5);
    initial_yaw_center_deg_ = declare_parameter<double>(
      "initialization.yaw_center_deg", 0.0);
    initial_yaw_radius_deg_ = declare_parameter<double>(
      "initialization.yaw_radius_deg", 60.0);
    initial_yaw_step_deg_ = declare_parameter<double>("initialization.yaw_step_deg", 30.0);
    initial_result_zone_margin_m_ = declare_parameter<double>(
      "initialization.result_zone_margin_m", 0.25);
    initial_result_yaw_margin_deg_ = declare_parameter<double>(
      "initialization.result_yaw_margin_deg", 5.0);
    recovery_xy_radius_ = declare_parameter<double>("recovery.xy_radius", 4.0);
    recovery_yaw_radius_deg_ = declare_parameter<double>("recovery.yaw_radius_deg", 45.0);
    initial_accumulation_window_s_ = declare_parameter<double>(
      "initialization.accumulation_window_s", 2.5);
    initial_min_duration_s_ = declare_parameter<double>(
      "initialization.min_accumulation_duration_s", 2.0);
    initial_accumulation_leaf_ = declare_parameter<double>(
      "initialization.accumulation_leaf", 0.18);
    initial_min_samples_ = declare_parameter<int>("initialization.min_scans", 8);
    initial_min_points_ = declare_parameter<int>("initialization.min_accumulated_points", 500);
    initial_max_samples_ = declare_parameter<int>("initialization.max_scans", 40);
    coarse_correspondence_m_ = declare_parameter<double>(
      "initialization.max_correspondence_m", 3.0);
    screening_iterations_ = declare_parameter<int>("initialization.screening_iterations", 5);
    screening_min_inlier_ratio_ = declare_parameter<double>(
      "initialization.screening_min_inlier_ratio", 0.05);
    coarse_iterations_ = declare_parameter<int>("initialization.max_iterations", 15);
    max_refine_candidates_ = declare_parameter<int>(
      "initialization.max_refine_candidates", 8);
    ambiguity_ratio_ = declare_parameter<double>("initialization.ambiguity_ratio", 1.12);
    ambiguity_translation_m_ = declare_parameter<double>(
      "initialization.ambiguity_translation_m", 1.0);
    ambiguity_rotation_deg_ = declare_parameter<double>(
      "initialization.ambiguity_rotation_deg", 20.0);
    tracking_correspondence_m_ = declare_parameter<double>(
      "tracking.max_correspondence_m", 1.2);
    tracking_iterations_ = declare_parameter<int>("tracking.max_iterations", 30);
    smoothing_alpha_ = declare_parameter<double>("tracking.smoothing_alpha", 0.35);
    max_translation_step_m_ = declare_parameter<double>(
      "tracking.max_translation_step_m", 0.15);
    max_rotation_step_deg_ = declare_parameter<double>(
      "tracking.max_rotation_step_deg", 2.0);
    usable_timeout_s_ = declare_parameter<double>("tracking.usable_timeout_s", 1.0);
    lost_timeout_s_ = declare_parameter<double>("tracking.lost_timeout_s", 2.5);
    thresholds_.min_input_points = static_cast<std::size_t>(declare_parameter<int>(
        "quality.min_input_points", 250));
    thresholds_.min_inlier_ratio = declare_parameter<double>("quality.min_inlier_ratio", 0.20);
    thresholds_.max_fitness_mse = declare_parameter<double>("quality.max_fitness_mse", 0.20);
    thresholds_.min_hessian_eigenvalue = declare_parameter<double>(
      "quality.min_hessian_eigenvalue", 1.0e-6);
    thresholds_.max_hessian_condition = declare_parameter<double>(
      "quality.max_hessian_condition", 1.0e8);
    thresholds_.max_correction_translation_m = declare_parameter<double>(
      "quality.max_correction_translation_m", 0.75);
    thresholds_.max_correction_rotation_deg = declare_parameter<double>(
      "quality.max_correction_rotation_deg", 15.0);
    thresholds_.max_input_age_s = declare_parameter<double>("quality.max_input_age_s", 0.35);

    validate_parameters();
    initial_evidence_policy_.min_samples = static_cast<std::size_t>(initial_min_samples_);
    initial_evidence_policy_.min_points = static_cast<std::size_t>(initial_min_points_);
    initial_evidence_policy_.min_duration_s = initial_min_duration_s_;
    load_map();
    if (!refresh_local_maps(Eigen::Vector3d(initial_x_, initial_y_, initial_z_), true)) {
      throw std::runtime_error("initialization zone has insufficient local map geometry");
    }
    configure_registration(
      screening_registration_, coarse_map_, coarse_correspondence_m_,
      screening_iterations_, std::max(0.8, coarse_map_leaf_ * 1.5));
    configure_registration(
      coarse_registration_, coarse_map_, coarse_correspondence_m_,
      coarse_iterations_, std::max(0.5, coarse_map_leaf_));
    configure_registration(
      tracking_registration_, tracking_map_, tracking_correspondence_m_,
      tracking_iterations_, std::max(0.5, tracking_map_leaf_ * 2.0));
    registrations_configured_ = true;

    state_machine_.begin_search();
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    status_publisher_ = create_publisher<std_msgs::msg::String>("/localization/status", 10);
    usable_publisher_ = create_publisher<std_msgs::msg::Bool>("/localization/usable", 10);
    confidence_publisher_ =
      create_publisher<std_msgs::msg::Float32>("/localization/confidence", 10);
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose", 10);
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/localization/reset",
      std::bind(
        &ContinuousMapLocalizer::reset_callback, this,
        std::placeholders::_1, std::placeholders::_2));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ContinuousMapLocalizer::odom_callback, this, std::placeholders::_1));
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ContinuousMapLocalizer::cloud_callback, this, std::placeholders::_1));
    tf_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&ContinuousMapLocalizer::publish_transform, this));
    watchdog_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(status_publish_period_s_)),
      std::bind(&ContinuousMapLocalizer::watchdog_tick, this));

    RCLCPP_INFO(
      get_logger(),
      "continuous localization ready: map=%s version=%s quality=%s registration=%s cloud=%s",
      map_file_.c_str(), map_version_.c_str(), quality_profile_id_.c_str(),
      registration_type_.c_str(), cloud_topic_.c_str());
    publish_status();
  }

private:
  struct OdomSample
  {
    double stamp = 0.0;
    Eigen::Isometry3d odom_from_base = Eigen::Isometry3d::Identity();
  };

  struct InitialScanSample
  {
    double stamp = 0.0;
    Eigen::Isometry3d odom_from_base = Eigen::Isometry3d::Identity();
    Cloud::Ptr cloud_base;
  };

  void validate_parameters()
  {
    const auto near = [](double value, double expected) {
        return std::abs(value - expected) <=
               1.0e-9 * std::max(1.0, std::abs(expected));
      };
    if (map_file_.empty() || !std::filesystem::is_regular_file(map_file_)) {
      throw std::runtime_error("map_file must name an existing PCD file");
    }
    if (map_version_.empty()) {
      throw std::runtime_error("map_version is required; unversioned maps are forbidden");
    }
    if (quality_profile_id_ != "go2-vgicp-orin-v1") {
      throw std::runtime_error("unsupported or missing sealed localization quality profile");
    }
    if (registration_type_ != "VGICP") {
      throw std::runtime_error("production localization registration.type must be VGICP");
    }
    if (!near(local_map_radius_m_, 45.0) ||
      !near(local_map_refresh_distance_m_, 5.0) || !near(max_scan_range_, 35.0) ||
      !near(tracking_scan_leaf_, 0.25) || !near(coarse_scan_leaf_, 0.65) ||
      !near(tracking_map_leaf_, 0.30) || !near(coarse_map_leaf_, 0.80) ||
      registration_threads_ != 4 || !near(coarse_correspondence_m_, 3.0) ||
      screening_iterations_ != 5 || !near(screening_min_inlier_ratio_, 0.05) ||
      coarse_iterations_ != 15 || !near(tracking_correspondence_m_, 1.2) ||
      tracking_iterations_ != 30 || thresholds_.min_input_points != 250 ||
      !near(thresholds_.min_inlier_ratio, 0.20) ||
      !near(thresholds_.max_fitness_mse, 0.20) ||
      !near(thresholds_.min_hessian_eigenvalue, 1.0e-6) ||
      !near(thresholds_.max_hessian_condition, 1.0e8) ||
      !near(thresholds_.max_correction_translation_m, 0.75) ||
      !near(thresholds_.max_correction_rotation_deg, 15.0) ||
      !near(thresholds_.max_input_age_s, 0.35))
    {
      throw std::runtime_error(
              "ROS parameters do not match sealed quality profile go2-vgicp-orin-v1");
    }
    if (map_frame_ != "map" || odom_frame_ != "odom" || base_frame_ != "base_link") {
      throw std::runtime_error("frame contract must remain map -> odom -> base_link");
    }
    if (update_period_s_ < 0.2 || update_period_s_ > 0.5 ||
      status_publish_period_s_ < 0.05 || status_publish_period_s_ > 0.1 ||
      initial_xy_radius_ < 0.5 || initial_xy_radius_ > 6.0 ||
      initial_xy_step_ <= 0.0 || initial_xy_step_ > initial_xy_radius_ ||
      !std::isfinite(initial_yaw_center_deg_) || initial_yaw_radius_deg_ < 5.0 ||
      initial_yaw_radius_deg_ > 90.0 || initial_yaw_step_deg_ <= 0.0 ||
      initial_yaw_step_deg_ > initial_yaw_radius_deg_ ||
      initial_result_zone_margin_m_ < 0.0 || initial_result_zone_margin_m_ > 1.0 ||
      initial_result_yaw_margin_deg_ < 0.0 || initial_result_yaw_margin_deg_ > 30.0 ||
      recovery_xy_radius_ < 1.0 || recovery_xy_radius_ > 6.0 ||
      recovery_yaw_radius_deg_ < initial_yaw_step_deg_ || recovery_yaw_radius_deg_ > 90.0)
    {
      throw std::runtime_error("invalid localization timing/search parameters");
    }
    std::size_t initial_seed_count = 0;
    for (double dx = -initial_xy_radius_; dx <= initial_xy_radius_ + 1.0e-6;
      dx += initial_xy_step_)
    {
      for (double dy = -initial_xy_radius_; dy <= initial_xy_radius_ + 1.0e-6;
        dy += initial_xy_step_)
      {
        if (dx * dx + dy * dy > initial_xy_radius_ * initial_xy_radius_ + 1.0e-6) {
          continue;
        }
        for (double yaw = -initial_yaw_radius_deg_;
          yaw <= initial_yaw_radius_deg_ + 1.0e-6;
          yaw += initial_yaw_step_deg_)
        {
          ++initial_seed_count;
        }
      }
    }
    if (initial_seed_count == 0 || initial_seed_count > 2000) {
      throw std::runtime_error("initial localization search exceeds the sealed compute budget");
    }
    std::size_t recovery_seed_count = 0;
    for (double dx = -recovery_xy_radius_; dx <= recovery_xy_radius_ + 1.0e-6;
      dx += initial_xy_step_)
    {
      for (double dy = -recovery_xy_radius_; dy <= recovery_xy_radius_ + 1.0e-6;
        dy += initial_xy_step_)
      {
        if (dx * dx + dy * dy > recovery_xy_radius_ * recovery_xy_radius_ + 1.0e-6) {
          continue;
        }
        for (double yaw = -recovery_yaw_radius_deg_;
          yaw <= recovery_yaw_radius_deg_ + 1.0e-6;
          yaw += initial_yaw_step_deg_)
        {
          ++recovery_seed_count;
        }
      }
    }
    if (recovery_seed_count == 0 || recovery_seed_count > 2000) {
      throw std::runtime_error("LOST recovery search exceeds the sealed compute budget");
    }
    if (max_odom_cloud_skew_s_ <= 0.0 || max_odom_cloud_skew_s_ > 0.20 ||
      min_scan_z_ >= max_scan_z_ || max_scan_range_ <= 0.0 ||
      tracking_scan_leaf_ <= 0.0 || coarse_scan_leaf_ <= 0.0 ||
      tracking_map_leaf_ <= 0.0 || coarse_map_leaf_ <= 0.0 ||
      registration_threads_ < 1 || registration_threads_ > 4 ||
      local_map_refresh_distance_m_ < 1.0 || local_map_refresh_distance_m_ > 10.0 ||
      local_map_radius_m_ < max_scan_range_ +
      std::max(initial_xy_radius_, recovery_xy_radius_) + coarse_correspondence_m_ ||
      local_map_radius_m_ > 100.0)
    {
      throw std::runtime_error("invalid localization input/compute parameters");
    }
    if (initial_accumulation_window_s_ < initial_min_duration_s_ ||
      initial_min_duration_s_ <= 0.0 || initial_accumulation_leaf_ <= 0.0 ||
      initial_min_samples_ < 2 || initial_max_samples_ < initial_min_samples_ ||
      initial_min_points_ < 1 || screening_iterations_ < 1 || coarse_iterations_ < 1 ||
      screening_min_inlier_ratio_ <= 0.0 ||
      screening_min_inlier_ratio_ > thresholds_.min_inlier_ratio ||
      max_refine_candidates_ < 2 || max_refine_candidates_ > 64)
    {
      throw std::runtime_error("invalid initialization evidence/search budget");
    }
    if (usable_timeout_s_ <= 0.0 || lost_timeout_s_ <= usable_timeout_s_) {
      throw std::runtime_error("tracking lost timeout must exceed usable timeout");
    }
    if (tracking_correspondence_m_ <= 0.0 || tracking_iterations_ < 1 ||
      tracking_iterations_ > 50 || smoothing_alpha_ <= 0.0 || smoothing_alpha_ > 1.0 ||
      max_translation_step_m_ <= 0.0 || max_translation_step_m_ > 0.25 ||
      max_rotation_step_deg_ <= 0.0 || max_rotation_step_deg_ > 5.0)
    {
      throw std::runtime_error("invalid continuous localization correction budget");
    }
  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    // Reset always fails safe: no transform is published and the motion gate
    // remains false until a completely new initial search reaches TRACKING.
    has_transform_ = false;
    has_trusted_transform_ = false;
    map_from_odom_ = Eigen::Isometry3d::Identity();
    last_map_from_base_ = Eigen::Isometry3d::Identity();
    odometry_.clear();
    initial_scans_.clear();
    last_update_time_ = std::chrono::steady_clock::time_point{};
    last_accepted_time_ = std::chrono::steady_clock::time_point{};
    last_metrics_ = RegistrationMetrics{};
    last_assessment_ = RegistrationAssessment{};
    last_assessment_.reason = "controlled_reset";
    recovery_search_active_ = false;
    recovery_center_map_from_base_ = Eigen::Isometry3d::Identity();
    local_map_center_valid_ = false;
    state_machine_.begin_search();
    publish_status();
    response->success = true;
    response->message =
      "localization reset; motion remains blocked until fresh initial search succeeds";
    RCLCPP_WARN(get_logger(), "controlled localization reset requested");
  }

  void load_map()
  {
    auto raw = Cloud::Ptr(new Cloud());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_file_, *raw) != 0 || raw->empty()) {
      throw std::runtime_error("cannot load localization map: " + map_file_);
    }
    full_tracking_map_ = filter_and_downsample(
      raw, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
      0.0, tracking_map_leaf_);
    full_coarse_map_ = filter_and_downsample(
      raw, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
      0.0, coarse_map_leaf_);
    if (full_tracking_map_->size() < 500 || full_coarse_map_->size() < 100) {
      throw std::runtime_error("localization map is too sparse after filtering");
    }
    tracking_map_index_.setInputCloud(full_tracking_map_);
    coarse_map_index_.setInputCloud(full_coarse_map_);
  }

  static Cloud::Ptr indexed_cloud(
    const Cloud::ConstPtr & source,
    const std::vector<int> & indices)
  {
    auto output = Cloud::Ptr(new Cloud());
    output->reserve(indices.size());
    for (const int index : indices) {
      output->push_back(source->points.at(static_cast<std::size_t>(index)));
    }
    output->width = static_cast<std::uint32_t>(output->size());
    output->height = 1;
    output->is_dense = source->is_dense;
    return output;
  }

  bool refresh_local_maps(const Eigen::Vector3d & center, bool force)
  {
    if (local_map_center_valid_ && !force &&
      (center.head<2>() - local_map_center_.head<2>()).norm() <
      local_map_refresh_distance_m_)
    {
      return true;
    }

    const auto started = std::chrono::steady_clock::now();
    pcl::PointXYZ query;
    query.x = static_cast<float>(center.x());
    query.y = static_cast<float>(center.y());
    query.z = static_cast<float>(center.z());
    std::vector<int> tracking_indices;
    std::vector<int> coarse_indices;
    std::vector<float> squared_distances;
    tracking_map_index_.radiusSearch(
      query, local_map_radius_m_, tracking_indices, squared_distances);
    squared_distances.clear();
    coarse_map_index_.radiusSearch(
      query, local_map_radius_m_, coarse_indices, squared_distances);

    if (tracking_indices.size() < 500 || coarse_indices.size() < 100) {
      RCLCPP_ERROR(
        get_logger(),
        "LOCAL_MAP_SPARSE center=(%.3f,%.3f,%.3f) radius=%.2f tracking=%zu coarse=%zu",
        center.x(), center.y(), center.z(), local_map_radius_m_,
        tracking_indices.size(), coarse_indices.size());
      return false;
    }

    auto next_tracking = indexed_cloud(full_tracking_map_, tracking_indices);
    auto next_coarse = indexed_cloud(full_coarse_map_, coarse_indices);
    tracking_map_ = std::move(next_tracking);
    coarse_map_ = std::move(next_coarse);
    if (registrations_configured_) {
      screening_registration_.setInputTarget(coarse_map_);
      coarse_registration_.setInputTarget(coarse_map_);
      tracking_registration_.setInputTarget(tracking_map_);
    }
    local_map_center_ = center;
    local_map_center_valid_ = true;
    last_local_map_refresh_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    maximum_local_map_refresh_ms_ = std::max(
      maximum_local_map_refresh_ms_, last_local_map_refresh_ms_);
    ++local_map_refresh_count_;
    RCLCPP_INFO(
      get_logger(),
      "LOCAL_MAP_REFRESH center=(%.3f,%.3f,%.3f) radius=%.2f tracking=%zu coarse=%zu ms=%.3f",
      center.x(), center.y(), center.z(), local_map_radius_m_, tracking_map_->size(),
      coarse_map_->size(), last_local_map_refresh_ms_);
    return true;
  }

  void configure_registration(
    Registration & registration,
    const Cloud::Ptr & target,
    double correspondence_distance,
    int iterations,
    double voxel_resolution)
  {
    registration.setNumThreads(registration_threads_);
    registration.setCorrespondenceRandomness(20);
    registration.setMaxCorrespondenceDistance(correspondence_distance);
    registration.setMaximumIterations(iterations);
    registration.setTransformationEpsilon(1.0e-3);
    registration.setRotationEpsilon(0.1 * kPi / 180.0);
    registration.setVoxelResolution(voxel_resolution);
    registration.setRegistrationType(registration_type_);
    registration.setInputTarget(target);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (message->header.frame_id != odom_frame_ || message->child_frame_id != base_frame_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "rejecting odometry frames %s -> %s; expected odom -> base_link",
        message->header.frame_id.c_str(), message->child_frame_id.c_str());
      return;
    }
    try {
      OdomSample sample;
      sample.stamp = stamp_seconds(message->header.stamp);
      sample.odom_from_base = pose_to_isometry(message->pose.pose);
      odometry_.push_back(sample);
      while (odometry_.size() > 300) {
        odometry_.pop_front();
      }
      if (publish_odom_base_tf_) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header = message->header;
        transform.header.frame_id = odom_frame_;
        transform.child_frame_id = base_frame_;
        transform.transform.translation.x = sample.odom_from_base.translation().x();
        transform.transform.translation.y = sample.odom_from_base.translation().y();
        transform.transform.translation.z = sample.odom_from_base.translation().z();
        transform.transform.rotation = quaternion_message(sample.odom_from_base.rotation());
        tf_broadcaster_->sendTransform(transform);
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
    }
  }

  bool matching_odometry(double cloud_stamp, OdomSample & output) const
  {
    if (odometry_.empty()) {
      return false;
    }
    const auto closest = std::min_element(
      odometry_.begin(), odometry_.end(),
      [cloud_stamp](const OdomSample & first, const OdomSample & second) {
        return std::abs(first.stamp - cloud_stamp) < std::abs(second.stamp - cloud_stamp);
      });
    if (std::abs(closest->stamp - cloud_stamp) > max_odom_cloud_skew_s_) {
      return false;
    }
    output = *closest;
    return true;
  }

  void append_initial_scan(
    const Cloud::Ptr & raw,
    const OdomSample & odom,
    double cloud_stamp)
  {
    if (!initial_scans_.empty() && cloud_stamp <= initial_scans_.back().stamp) {
      initial_scans_.clear();
    }
    InitialScanSample sample;
    sample.stamp = cloud_stamp;
    sample.odom_from_base = odom.odom_from_base;
    sample.cloud_base = filter_and_downsample(
      raw, min_scan_z_, max_scan_z_, max_scan_range_, initial_accumulation_leaf_);
    initial_scans_.push_back(sample);
    while (!initial_scans_.empty() &&
      (cloud_stamp - initial_scans_.front().stamp > initial_accumulation_window_s_ ||
      static_cast<int>(initial_scans_.size()) > initial_max_samples_))
    {
      initial_scans_.pop_front();
    }
  }

  go2_map_manager::InitialEvidenceAssessment initial_evidence() const
  {
    std::size_t point_count = 0;
    for (const auto & sample : initial_scans_) {
      point_count += sample.cloud_base->size();
    }
    const double duration = initial_scans_.size() < 2 ? 0.0 :
      initial_scans_.back().stamp - initial_scans_.front().stamp;
    return go2_map_manager::assess_initial_evidence(
      initial_scans_.size(), point_count, duration, initial_evidence_policy_);
  }

  Cloud::Ptr accumulated_initial_scan(const OdomSample & current_odom) const
  {
    auto accumulated = Cloud::Ptr(new Cloud());
    std::size_t point_count = 0;
    for (const auto & sample : initial_scans_) {
      point_count += sample.cloud_base->size();
    }
    accumulated->reserve(point_count);
    const Eigen::Isometry3d current_base_from_odom =
      current_odom.odom_from_base.inverse();
    for (const auto & sample : initial_scans_) {
      Cloud transformed;
      const Eigen::Isometry3d current_base_from_sample_base =
        current_base_from_odom * sample.odom_from_base;
      pcl::transformPointCloud(
        *sample.cloud_base, transformed,
        current_base_from_sample_base.matrix().cast<float>());
      *accumulated += transformed;
    }
    return accumulated;
  }

  void begin_recovery_search(const OdomSample * current_odom)
  {
    if (!has_trusted_transform_) {
      recovery_search_active_ = false;
      has_transform_ = false;
      initial_scans_.clear();
      state_machine_.begin_search();
      RCLCPP_WARN(
        get_logger(),
        "LOCALIZATION_STARTUP_RETRY no trusted map pose exists; startup zone retained");
      return;
    }
    // A patrol-time LOST must never send the search back to the physical
    // startup zone.  Continue propagating the last trusted map->odom with the
    // live FAST-LIO odometry, then search only around that predicted pose.
    if (current_odom != nullptr) {
      recovery_center_map_from_base_ = map_from_odom_ * current_odom->odom_from_base;
    } else if (!odometry_.empty()) {
      recovery_center_map_from_base_ = map_from_odom_ * odometry_.back().odom_from_base;
    } else {
      recovery_center_map_from_base_ = last_map_from_base_;
    }
    recovery_search_active_ = true;
    has_transform_ = false;
    initial_scans_.clear();
    state_machine_.mark_lost();
    RCLCPP_WARN(
      get_logger(),
      "LOCALIZATION_RECOVERY center=(%.3f,%.3f,%.3f) yaw_deg=%.2f radius=%.2f",
      recovery_center_map_from_base_.translation().x(),
      recovery_center_map_from_base_.translation().y(),
      recovery_center_map_from_base_.translation().z(),
      degrees(yaw_from_rotation(recovery_center_map_from_base_.rotation())),
      recovery_xy_radius_);
  }

  double search_center_x() const
  {
    return recovery_search_active_ ? recovery_center_map_from_base_.translation().x() : initial_x_;
  }

  double search_center_y() const
  {
    return recovery_search_active_ ? recovery_center_map_from_base_.translation().y() : initial_y_;
  }

  double search_center_z() const
  {
    return recovery_search_active_ ? recovery_center_map_from_base_.translation().z() : initial_z_;
  }

  double search_center_yaw_deg() const
  {
    return recovery_search_active_ ?
      degrees(yaw_from_rotation(recovery_center_map_from_base_.rotation())) :
      initial_yaw_center_deg_;
  }

  double search_xy_radius() const
  {
    return recovery_search_active_ ? recovery_xy_radius_ : initial_xy_radius_;
  }

  double search_yaw_radius_deg() const
  {
    return recovery_search_active_ ? recovery_yaw_radius_deg_ : initial_yaw_radius_deg_;
  }

  void record_registration_timing(
    const std::chrono::steady_clock::time_point & started,
    bool tracking_deadline_applies)
  {
    last_registration_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    maximum_registration_ms_ = std::max(maximum_registration_ms_, last_registration_ms_);
    ++registration_count_;
    last_registration_mode_ = tracking_deadline_applies ? "TRACKING" :
      (recovery_search_active_ ? "RECOVERY" : "STARTUP");
    if (tracking_deadline_applies && last_registration_ms_ > update_period_s_ * 1000.0) {
      ++registration_deadline_miss_count_;
      RCLCPP_WARN(
        get_logger(), "LOCALIZATION_CPU registration_ms=%.3f deadline_ms=%.3f misses=%zu",
        last_registration_ms_, update_period_s_ * 1000.0,
        registration_deadline_miss_count_);
    }
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (message->header.frame_id != base_frame_) {
      publish_rejection("wrong_cloud_frame");
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "rejecting cloud frame %s; expected base_link", message->header.frame_id.c_str());
      return;
    }
    const double cloud_stamp = stamp_seconds(message->header.stamp);
    OdomSample odom;
    if (!matching_odometry(cloud_stamp, odom)) {
      publish_rejection("missing_time_aligned_odometry");
      return;
    }
    Cloud::Ptr raw(new Cloud());
    pcl::fromROSMsg(*message, *raw);
    const double input_age_s = std::max(0.0, now().seconds() - cloud_stamp);

    if (state_machine_.state() == LocalizationState::kLost) {
      initial_scans_.clear();
      state_machine_.begin_search();
    }
    if (!has_transform_) {
      append_initial_scan(raw, odom, cloud_stamp);
    }

    const auto steady_now = std::chrono::steady_clock::now();
    if (last_update_time_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(steady_now - last_update_time_).count() < update_period_s_)
    {
      return;
    }
    last_update_time_ = steady_now;

    const bool tracking_update = has_transform_;
    const auto registration_started = std::chrono::steady_clock::now();
    const Eigen::Vector3d local_map_center = has_transform_ ?
      (map_from_odom_ * odom.odom_from_base).translation() :
      Eigen::Vector3d(search_center_x(), search_center_y(), search_center_z());
    if (!refresh_local_maps(local_map_center, false)) {
      last_assessment_.accepted = false;
      last_assessment_.confidence = 0.0;
      last_assessment_.reason = "local_map_too_sparse";
      if (state_machine_.observe(false) == LocalizationState::kLost) {
        begin_recovery_search(&odom);
      }
      record_registration_timing(registration_started, tracking_update);
      publish_status();
      return;
    }
    RegistrationRun run;
    bool ambiguous = false;
    if (!has_transform_) {
      const auto evidence = initial_evidence();
      if (!evidence.ready) {
        last_assessment_.accepted = false;
        last_assessment_.confidence = 0.0;
        last_assessment_.reason = evidence.reason;
        publish_status();
        return;
      }
      run = initial_search(
        accumulated_initial_scan(odom), input_age_s, ambiguous);
      if (ambiguous) {
        last_metrics_ = run.metrics;
        record_registration_timing(registration_started, tracking_update);
        publish_rejection("ambiguous_initialization");
        return;
      }
    } else {
      auto scan = filter_and_downsample(
        raw, min_scan_z_, max_scan_z_, max_scan_range_, tracking_scan_leaf_);
      tracking_registration_.setInputSource(scan);
      const Eigen::Isometry3d initial_map_from_base = map_from_odom_ * odom.odom_from_base;
      run = run_registration(
        tracking_registration_, scan, initial_map_from_base,
        tracking_correspondence_m_, input_age_s);
      const Eigen::Isometry3d candidate_map_from_odom =
        run.map_from_base * odom.odom_from_base.inverse();
      run.metrics.correction_translation_m =
        (candidate_map_from_odom.translation() - map_from_odom_.translation()).norm();
      run.metrics.correction_rotation_deg = degrees(
        rotation_distance(map_from_odom_, candidate_map_from_odom));
    }

    const bool enforce_jump_limit = has_transform_;
    const RegistrationAssessment assessment = go2_map_manager::assess_registration(
      run.metrics, thresholds_, enforce_jump_limit);
    last_metrics_ = run.metrics;
    last_assessment_ = assessment;

    if (!has_transform_ && assessment.accepted &&
      !initial_result_within_declared_zone(run.map_from_base))
    {
      last_assessment_.accepted = false;
      last_assessment_.confidence = 0.0;
      last_assessment_.reason = "initial_result_outside_declared_zone";
      if (state_machine_.observe(false) == LocalizationState::kLost) {
        begin_recovery_search(&odom);
      }
      record_registration_timing(registration_started, tracking_update);
      publish_status();
      return;
    }

    if (!assessment.accepted) {
      if (state_machine_.observe(false) == LocalizationState::kLost) {
        begin_recovery_search(&odom);
      }
      record_registration_timing(registration_started, tracking_update);
      publish_status();
      return;
    }

    Eigen::Isometry3d candidate_map_from_odom = run.map_from_base * odom.odom_from_base.inverse();
    if (has_transform_) {
      map_from_odom_ = blend_transform(
        map_from_odom_, candidate_map_from_odom, smoothing_alpha_,
        max_translation_step_m_, max_rotation_step_deg_ * kPi / 180.0);
    } else {
      map_from_odom_ = candidate_map_from_odom;
      has_transform_ = true;
      has_trusted_transform_ = true;
      initial_scans_.clear();
    }
    last_accepted_time_ = steady_now;
    last_map_from_base_ = map_from_odom_ * odom.odom_from_base;
    state_machine_.observe(true);
    record_registration_timing(registration_started, tracking_update);
    if (state_machine_.state() == LocalizationState::kTracking) {
      recovery_search_active_ = false;
    }
    publish_pose(message->header.stamp);
    publish_status();
  }

  RegistrationRun initial_search(
    const Cloud::Ptr & raw,
    double input_age_s,
    bool & ambiguous)
  {
    auto scan = filter_and_downsample(
      raw, min_scan_z_, max_scan_z_, max_scan_range_, coarse_scan_leaf_);
    std::vector<RegistrationRun> screening_candidates;
    if (scan->size() < thresholds_.min_input_points) {
      RegistrationRun empty;
      empty.metrics.input_points = scan->size();
      empty.metrics.input_age_s = input_age_s;
      return empty;
    }
    // RegistrationPCL preprocesses the source at setInputSource(). Reuse that
    // work for the whole pose grid instead of rebuilding covariance data for
    // every candidate.
    screening_registration_.setInputSource(scan);
    const double center_x = search_center_x();
    const double center_y = search_center_y();
    const double center_z = search_center_z();
    const double center_yaw_deg = search_center_yaw_deg();
    const double xy_radius = search_xy_radius();
    const double yaw_radius_deg = search_yaw_radius_deg();
    for (double dx = -xy_radius; dx <= xy_radius + 1.0e-6;
      dx += initial_xy_step_)
    {
      for (double dy = -xy_radius; dy <= xy_radius + 1.0e-6;
        dy += initial_xy_step_)
      {
        if (dx * dx + dy * dy > xy_radius * xy_radius + 1.0e-6) {
          continue;
        }
        for (double yaw_offset_deg = -yaw_radius_deg;
          yaw_offset_deg <= yaw_radius_deg + 1.0e-6;
          yaw_offset_deg += initial_yaw_step_deg_)
        {
          const InitialPoseSeed seed = go2_map_manager::initial_pose_seed(
            center_x, center_y, center_yaw_deg * kPi / 180.0,
            dx, dy, yaw_offset_deg * kPi / 180.0);
          const Eigen::Isometry3d guess_map_from_base = yaw_transform(
            seed.map_x, seed.map_y, center_z, seed.map_yaw_rad);
          screening_candidates.push_back(
            run_registration(
              screening_registration_, scan, guess_map_from_base,
              coarse_correspondence_m_, input_age_s, true,
              screening_min_inlier_ratio_));
        }
      }
    }
    if (screening_candidates.empty()) {
      RegistrationRun empty;
      empty.metrics.input_points = scan->size();
      empty.metrics.input_age_s = input_age_s;
      return empty;
    }
    std::sort(
      screening_candidates.begin(), screening_candidates.end(),
      [](const RegistrationRun & first, const RegistrationRun & second) {
        return first.ranking_score < second.ranking_score;
      });
    if (!std::isfinite(screening_candidates.front().ranking_score)) {
      return screening_candidates.front();
    }

    std::vector<RegistrationRun> refine_seeds;
    for (const auto & candidate : screening_candidates) {
      if (!std::isfinite(candidate.ranking_score)) {
        break;
      }
      const bool duplicates_existing = std::any_of(
        refine_seeds.begin(), refine_seeds.end(),
        [&](const RegistrationRun & existing) {
          return !materially_different(
            existing, candidate,
            ambiguity_translation_m_, ambiguity_rotation_deg_);
        });
      if (!duplicates_existing) {
        refine_seeds.push_back(candidate);
      }
      if (static_cast<int>(refine_seeds.size()) >= max_refine_candidates_) {
        break;
      }
    }
    if (refine_seeds.empty()) {
      return screening_candidates.front();
    }
    coarse_registration_.setInputSource(scan);
    std::vector<RegistrationRun> candidates;
    candidates.reserve(refine_seeds.size());
    for (const auto & seed : refine_seeds) {
      candidates.push_back(
        run_registration(
          coarse_registration_, scan, seed.map_from_base,
          coarse_correspondence_m_, input_age_s));
    }
    std::sort(
      candidates.begin(), candidates.end(),
      [](const RegistrationRun & first, const RegistrationRun & second) {
        return first.ranking_score < second.ranking_score;
      });
    const RegistrationRun & coarse_best = candidates.front();
    if (!std::isfinite(coarse_best.ranking_score)) {
      return coarse_best;
    }
    for (std::size_t index = 1; index < candidates.size(); ++index) {
      if (!materially_different(
          coarse_best, candidates[index], ambiguity_translation_m_, ambiguity_rotation_deg_))
      {
        continue;
      }
      ambiguous = candidates[index].ranking_score <=
        coarse_best.ranking_score * ambiguity_ratio_;
      break;
    }
    if (ambiguous) {
      return coarse_best;
    }
    auto fine_scan = filter_and_downsample(
      raw, min_scan_z_, max_scan_z_, max_scan_range_, tracking_scan_leaf_);
    tracking_registration_.setInputSource(fine_scan);
    return run_registration(
      tracking_registration_, fine_scan, coarse_best.map_from_base,
      tracking_correspondence_m_, input_age_s);
  }

  bool initial_result_within_declared_zone(
    const Eigen::Isometry3d & map_from_base) const
  {
    const double dx = map_from_base.translation().x() - search_center_x();
    const double dy = map_from_base.translation().y() - search_center_y();
    const double distance = std::hypot(dx, dy);
    const double yaw_error_deg = degrees(
      std::abs(
        normalized_angle(
          yaw_from_rotation(map_from_base.rotation()) -
          search_center_yaw_deg() * kPi / 180.0)));
    return distance <= search_xy_radius() + initial_result_zone_margin_m_ &&
           yaw_error_deg <= search_yaw_radius_deg() + initial_result_yaw_margin_deg_;
  }

  void publish_rejection(const std::string & reason)
  {
    last_assessment_.accepted = false;
    last_assessment_.confidence = 0.0;
    last_assessment_.reason = reason;
    if (state_machine_.observe(false) == LocalizationState::kLost) {
      begin_recovery_search(nullptr);
    }
    publish_status();
  }

  bool localization_usable() const
  {
    if (!has_transform_ || state_machine_.state() != LocalizationState::kTracking ||
      last_accepted_time_.time_since_epoch().count() == 0)
    {
      return false;
    }
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_accepted_time_).count() <= usable_timeout_s_;
  }

  void watchdog_tick()
  {
    if (has_transform_ && last_accepted_time_.time_since_epoch().count() != 0) {
      const double age_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_accepted_time_).count();
      if (age_s > lost_timeout_s_) {
        begin_recovery_search(nullptr);
        last_assessment_.accepted = false;
        last_assessment_.confidence = 0.0;
        last_assessment_.reason = "localization_input_timeout";
      } else if (age_s > usable_timeout_s_) {
        last_assessment_.accepted = false;
        last_assessment_.confidence = 0.0;
        last_assessment_.reason = "localization_input_stale";
      }
    }
    publish_status();
  }

  void publish_status()
  {
    const bool usable = localization_usable();
    const double accepted_age_s =
      last_accepted_time_.time_since_epoch().count() == 0 ?
      std::numeric_limits<double>::quiet_NaN() :
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_accepted_time_).count();
    std_msgs::msg::Bool usable_message;
    usable_message.data = usable;
    usable_publisher_->publish(usable_message);
    std_msgs::msg::Float32 confidence_message;
    confidence_message.data = static_cast<float>(last_assessment_.confidence);
    confidence_publisher_->publish(confidence_message);

    std_msgs::msg::String status;
    std::ostringstream json;
    json << "{\"schemaVersion\":1"
         << ",\"state\":\"" << go2_map_manager::to_string(state_machine_.state()) << "\""
         << ",\"usable\":" << (usable ? "true" : "false")
         << ",\"mapVersion\":\"" << map_version_ << "\""
         << ",\"qualityProfileId\":\"" << quality_profile_id_ << "\""
         << ",\"registrationType\":\"" << registration_type_ << "\""
         << ",\"reason\":\"" << last_assessment_.reason << "\""
         << ",\"confidence\":" << json_number(last_assessment_.confidence)
         << ",\"inputPoints\":" << last_metrics_.input_points
         << ",\"inlierRatio\":" << json_number(last_metrics_.inlier_ratio)
         << ",\"fitnessMse\":" << json_number(last_metrics_.fitness_mse)
         << ",\"hessianCondition\":" << json_number(last_metrics_.hessian_condition)
         << ",\"correctionTranslationM\":" <<
      json_number(last_metrics_.correction_translation_m)
         << ",\"correctionRotationDeg\":" <<
      json_number(last_metrics_.correction_rotation_deg)
         << ",\"initialScanCount\":" << initial_scans_.size()
         << ",\"searchMode\":\"" << (recovery_search_active_ ? "RECOVERY" : "STARTUP") << "\""
         << ",\"searchCenterX\":" << json_number(search_center_x())
         << ",\"searchCenterY\":" << json_number(search_center_y())
         << ",\"searchRadiusM\":" << json_number(search_xy_radius())
         << ",\"registrationMs\":" << json_number(last_registration_ms_)
         << ",\"registrationMode\":\"" << last_registration_mode_ << "\""
         << ",\"maximumRegistrationMs\":" << json_number(maximum_registration_ms_)
         << ",\"registrationCount\":" << registration_count_
         << ",\"registrationDeadlineMissCount\":" << registration_deadline_miss_count_
         << ",\"localMapRadiusM\":" << json_number(local_map_radius_m_)
         << ",\"localMapCenterX\":" <<
      json_number(local_map_center_valid_ ? local_map_center_.x() :
      std::numeric_limits<double>::quiet_NaN())
         << ",\"localMapCenterY\":" <<
      json_number(local_map_center_valid_ ? local_map_center_.y() :
      std::numeric_limits<double>::quiet_NaN())
         << ",\"localMapTrackingPoints\":" <<
      (tracking_map_ ? tracking_map_->size() : 0)
         << ",\"localMapCoarsePoints\":" << (coarse_map_ ? coarse_map_->size() : 0)
         << ",\"localMapRefreshMs\":" << json_number(last_local_map_refresh_ms_)
         << ",\"maximumLocalMapRefreshMs\":" <<
      json_number(maximum_local_map_refresh_ms_)
         << ",\"localMapRefreshCount\":" << local_map_refresh_count_
         << ",\"lastAcceptedAgeS\":" << json_number(accepted_age_s)
         << "}";
    status.data = json.str();
    status_publisher_->publish(status);
  }

  void publish_pose(const builtin_interfaces::msg::Time & stamp)
  {
    if (!has_transform_) {
      return;
    }
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.pose.position.x = last_map_from_base_.translation().x();
    pose.pose.pose.position.y = last_map_from_base_.translation().y();
    pose.pose.pose.position.z = last_map_from_base_.translation().z();
    pose.pose.pose.orientation = quaternion_message(last_map_from_base_.rotation());
    const double variance = std::max(1.0e-4, last_metrics_.fitness_mse);
    pose.pose.covariance[0] = variance;
    pose.pose.covariance[7] = variance;
    pose.pose.covariance[14] = variance * 2.0;
    pose.pose.covariance[21] = variance;
    pose.pose.covariance[28] = variance;
    pose.pose.covariance[35] = variance;
    pose_publisher_->publish(pose);
  }

  void publish_transform()
  {
    if (!localization_usable()) {
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = map_from_odom_.translation().x();
    transform.transform.translation.y = map_from_odom_.translation().y();
    transform.transform.translation.z = map_from_odom_.translation().z();
    transform.transform.rotation = quaternion_message(map_from_odom_.rotation());
    tf_broadcaster_->sendTransform(transform);
  }

  std::string map_file_;
  std::string map_version_;
  std::string quality_profile_id_;
  std::string registration_type_ = "VGICP";
  std::string cloud_topic_;
  std::string odom_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_odom_base_tf_ = false;
  double update_period_s_ = 0.4;
  double status_publish_period_s_ = 0.1;
  double max_odom_cloud_skew_s_ = 0.08;
  double min_scan_z_ = -1.5;
  double max_scan_z_ = 2.5;
  double max_scan_range_ = 35.0;
  double tracking_scan_leaf_ = 0.25;
  double coarse_scan_leaf_ = 0.65;
  double tracking_map_leaf_ = 0.30;
  double coarse_map_leaf_ = 0.80;
  int registration_threads_ = 4;
  double local_map_radius_m_ = 45.0;
  double local_map_refresh_distance_m_ = 5.0;
  double initial_x_ = 0.0;
  double initial_y_ = 0.0;
  double initial_z_ = 0.0;
  double initial_xy_radius_ = 5.0;
  double initial_xy_step_ = 2.5;
  double initial_yaw_center_deg_ = 0.0;
  double initial_yaw_radius_deg_ = 60.0;
  double initial_yaw_step_deg_ = 30.0;
  double initial_result_zone_margin_m_ = 0.25;
  double initial_result_yaw_margin_deg_ = 5.0;
  double recovery_xy_radius_ = 4.0;
  double recovery_yaw_radius_deg_ = 45.0;
  double initial_accumulation_window_s_ = 2.5;
  double initial_min_duration_s_ = 2.0;
  double initial_accumulation_leaf_ = 0.18;
  int initial_min_samples_ = 8;
  int initial_min_points_ = 500;
  int initial_max_samples_ = 40;
  double coarse_correspondence_m_ = 3.0;
  int screening_iterations_ = 5;
  double screening_min_inlier_ratio_ = 0.05;
  int coarse_iterations_ = 15;
  int max_refine_candidates_ = 8;
  double ambiguity_ratio_ = 1.12;
  double ambiguity_translation_m_ = 1.0;
  double ambiguity_rotation_deg_ = 20.0;
  double tracking_correspondence_m_ = 1.2;
  int tracking_iterations_ = 30;
  double smoothing_alpha_ = 0.35;
  double max_translation_step_m_ = 0.15;
  double max_rotation_step_deg_ = 2.0;
  double usable_timeout_s_ = 1.0;
  double lost_timeout_s_ = 2.5;

  Cloud::Ptr full_tracking_map_;
  Cloud::Ptr full_coarse_map_;
  Cloud::Ptr tracking_map_;
  Cloud::Ptr coarse_map_;
  MapIndex tracking_map_index_;
  MapIndex coarse_map_index_;
  Registration screening_registration_;
  Registration coarse_registration_;
  Registration tracking_registration_;
  RegistrationThresholds thresholds_;
  LocalizationStateMachine state_machine_;
  RegistrationMetrics last_metrics_;
  RegistrationAssessment last_assessment_;
  InitialEvidencePolicy initial_evidence_policy_;
  std::deque<OdomSample> odometry_;
  std::deque<InitialScanSample> initial_scans_;
  bool has_transform_ = false;
  bool has_trusted_transform_ = false;
  bool recovery_search_active_ = false;
  bool registrations_configured_ = false;
  bool local_map_center_valid_ = false;
  Eigen::Isometry3d map_from_odom_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d last_map_from_base_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d recovery_center_map_from_base_ = Eigen::Isometry3d::Identity();
  Eigen::Vector3d local_map_center_ = Eigen::Vector3d::Zero();
  double last_registration_ms_ = 0.0;
  std::string last_registration_mode_ = "NONE";
  double maximum_registration_ms_ = 0.0;
  std::size_t registration_count_ = 0;
  std::size_t registration_deadline_miss_count_ = 0;
  double last_local_map_refresh_ms_ = 0.0;
  double maximum_local_map_refresh_ms_ = 0.0;
  std::size_t local_map_refresh_count_ = 0;
  std::chrono::steady_clock::time_point last_update_time_;
  std::chrono::steady_clock::time_point last_accepted_time_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr usable_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr confidence_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ContinuousMapLocalizer>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("continuous_map_localizer"), "%s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
