// Copyright 2026 GoGoGuard

#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <small_gicp/pcl/pcl_registration.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{

constexpr double kPi = 3.14159265358979323846;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Vgicp = small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ>;

struct BenchmarkResult
{
  std::string algorithm;
  bool converged = false;
  int iterations = -1;
  double elapsed_ms = 0.0;
  double fitness_mse = std::numeric_limits<double>::infinity();
  double inlier_ratio = 0.0;
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
};

double finite_number(const char * value, const char * label)
{
  std::size_t consumed = 0;
  const double parsed = std::stod(value, &consumed);
  if (consumed != std::string(value).size() || !std::isfinite(parsed)) {
    throw std::runtime_error(std::string(label) + " must be finite");
  }
  return parsed;
}

Cloud::Ptr load_and_filter(const std::string & path, double leaf)
{
  auto raw = Cloud::Ptr(new Cloud());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *raw) != 0 || raw->empty()) {
    throw std::runtime_error("cannot load PCD: " + path);
  }
  auto output = Cloud::Ptr(new Cloud());
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
  voxel.setInputCloud(raw);
  voxel.filter(*output);
  if (output->size() < 100) {
    throw std::runtime_error("PCD is too sparse after filtering: " + path);
  }
  return output;
}

Eigen::Matrix4d initial_transform(double x, double y, double z, double yaw_deg)
{
  Eigen::Matrix4d output = Eigen::Matrix4d::Identity();
  output.block<3, 3>(0, 0) = Eigen::AngleAxisd(
    yaw_deg * kPi / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  output.block<3, 1>(0, 3) << x, y, z;
  return output;
}

double inlier_ratio(
  const Cloud::ConstPtr & target,
  const Cloud::ConstPtr & source,
  const Eigen::Matrix4d & transform,
  double maximum_distance)
{
  Cloud aligned;
  pcl::transformPointCloud(*source, aligned, transform.cast<float>());
  pcl::KdTreeFLANN<pcl::PointXYZ> index;
  index.setInputCloud(target);
  const double maximum_squared = maximum_distance * maximum_distance;
  std::vector<int> neighbor(1);
  std::vector<float> squared_distance(1);
  std::size_t inliers = 0;
  for (const auto & point : aligned) {
    if (index.nearestKSearch(point, 1, neighbor, squared_distance) == 1 &&
      squared_distance.front() <= maximum_squared)
    {
      ++inliers;
    }
  }
  return aligned.empty() ? 0.0 :
    static_cast<double>(inliers) / static_cast<double>(aligned.size());
}

BenchmarkResult run_ndt(
  const Cloud::Ptr & target,
  const Cloud::Ptr & source,
  const Eigen::Matrix4d & initial)
{
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> registration;
  registration.setInputTarget(target);
  registration.setInputSource(source);
  registration.setResolution(1.0);
  registration.setStepSize(0.1);
  registration.setTransformationEpsilon(0.01);
  registration.setMaximumIterations(35);
  Cloud aligned;
  const auto started = std::chrono::steady_clock::now();
  registration.align(aligned, initial.cast<float>());

  BenchmarkResult result;
  result.algorithm = "NDT";
  result.elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  result.converged = registration.hasConverged();
  result.transform = registration.getFinalTransformation().cast<double>();
  result.fitness_mse = registration.getFitnessScore(2.0);
  result.inlier_ratio = inlier_ratio(target, source, result.transform, 2.0);
  return result;
}

BenchmarkResult run_gicp(
  const Cloud::Ptr & target,
  const Cloud::Ptr & source,
  const Eigen::Matrix4d & initial)
{
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> registration;
  registration.setInputTarget(target);
  registration.setInputSource(source);
  registration.setMaxCorrespondenceDistance(1.2);
  registration.setMaximumIterations(30);
  registration.setTransformationEpsilon(1.0e-3);
  registration.setRotationEpsilon(0.1 * kPi / 180.0);
  registration.setCorrespondenceRandomness(20);
  Cloud aligned;
  const auto started = std::chrono::steady_clock::now();
  registration.align(aligned, initial.cast<float>());

  BenchmarkResult result;
  result.algorithm = "GICP";
  result.elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  result.converged = registration.hasConverged();
  result.transform = registration.getFinalTransformation().cast<double>();
  result.fitness_mse = registration.getFitnessScore(1.2);
  result.inlier_ratio = inlier_ratio(target, source, result.transform, 1.2);
  return result;
}

BenchmarkResult run_vgicp(
  const Cloud::Ptr & target,
  const Cloud::Ptr & source,
  const Eigen::Matrix4d & initial,
  int threads)
{
  Vgicp registration;
  registration.setNumThreads(threads);
  registration.setCorrespondenceRandomness(20);
  registration.setMaxCorrespondenceDistance(1.2);
  registration.setMaximumIterations(30);
  registration.setTransformationEpsilon(1.0e-3);
  registration.setRotationEpsilon(0.1 * kPi / 180.0);
  registration.setVoxelResolution(0.6);
  registration.setRegistrationType("VGICP");
  registration.setInputTarget(target);
  registration.setInputSource(source);
  Cloud aligned;
  const auto started = std::chrono::steady_clock::now();
  registration.align(aligned, initial.cast<float>());
  const auto & registration_result = registration.getRegistrationResult();

  BenchmarkResult result;
  result.algorithm = "VGICP";
  result.elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  result.converged = registration_result.converged;
  result.iterations = static_cast<int>(registration_result.iterations);
  result.transform = registration_result.T_target_source.matrix();
  result.fitness_mse = registration.getFitnessScore(1.2);
  result.inlier_ratio = source->empty() ? 0.0 :
    static_cast<double>(registration_result.num_inliers) /
    static_cast<double>(source->size());
  return result;
}

double yaw_deg(const Eigen::Matrix4d & transform)
{
  return std::atan2(transform(1, 0), transform(0, 0)) * 180.0 / kPi;
}

std::string json_number(double value)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream output;
  output << std::setprecision(10) << value;
  return output.str();
}

void print_result(
  const BenchmarkResult & result,
  const Eigen::Matrix4d & initial,
  std::size_t target_points,
  std::size_t source_points)
{
  const Eigen::Vector3d translation = result.transform.block<3, 1>(0, 3);
  const double delta_translation =
    (translation - initial.block<3, 1>(0, 3)).norm();
  const double delta_yaw = std::abs(std::remainder(
    yaw_deg(result.transform) - yaw_deg(initial), 360.0));
  const bool accepted = result.converged && std::isfinite(result.fitness_mse) &&
    result.fitness_mse <= 0.20 && result.inlier_ratio >= 0.20;
  std::cout << std::setprecision(10)
            << "{\"schema\":\"go2.localization_registration_benchmark.v1\""
            << ",\"algorithm\":\"" << result.algorithm << "\""
            << ",\"converged\":" << (result.converged ? "true" : "false")
            << ",\"acceptedByTrackingGate\":" << (accepted ? "true" : "false")
            << ",\"targetPoints\":" << target_points
            << ",\"sourcePoints\":" << source_points
            << ",\"iterations\":";
  if (result.iterations >= 0) {
    std::cout << result.iterations;
  } else {
    std::cout << "null";
  }
  std::cout
            << ",\"elapsedMs\":" << json_number(result.elapsed_ms)
            << ",\"fitnessMse\":" << json_number(result.fitness_mse)
            << ",\"inlierRatio\":" << json_number(result.inlier_ratio)
            << ",\"deltaFromInitialM\":" << json_number(delta_translation)
            << ",\"deltaFromInitialYawDeg\":" << json_number(delta_yaw)
            << ",\"result\":{\"x\":" << json_number(translation.x())
            << ",\"y\":" << json_number(translation.y())
            << ",\"z\":" << json_number(translation.z())
            << ",\"yawDeg\":" << json_number(yaw_deg(result.transform)) << "}}\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 8 && argc != 9) {
    std::cerr << "usage: localization_registration_benchmark TARGET_PCD SOURCE_PCD "
              << "X Y Z YAW_DEG THREADS [REPEAT]\n";
    return 2;
  }
  try {
    const Eigen::Matrix4d initial = initial_transform(
      finite_number(argv[3], "x"), finite_number(argv[4], "y"),
      finite_number(argv[5], "z"), finite_number(argv[6], "yaw_deg"));
    const int threads = std::stoi(argv[7]);
    const int repeat = argc == 9 ? std::stoi(argv[8]) : 1;
    if (threads < 1 || threads > 4 || repeat < 1 || repeat > 20) {
      throw std::runtime_error("threads must be 1..4 and repeat must be 1..20");
    }
    const auto target = load_and_filter(argv[1], 0.30);
    const auto source = load_and_filter(argv[2], 0.25);
    for (int index = 0; index < repeat; ++index) {
      print_result(run_ndt(target, source, initial), initial, target->size(), source->size());
      print_result(run_gicp(target, source, initial), initial, target->size(), source->size());
      print_result(
        run_vgicp(target, source, initial, threads), initial, target->size(), source->size());
    }
  } catch (const std::exception & error) {
    std::cerr << "localization benchmark failed: " << error.what() << "\n";
    return 2;
  }
  return 0;
}
