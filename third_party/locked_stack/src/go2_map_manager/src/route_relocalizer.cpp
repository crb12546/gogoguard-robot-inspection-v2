#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{

constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double a)
{
    while (a > kPi)
    {
        a -= 2.0 * kPi;
    }
    while (a < -kPi)
    {
        a += 2.0 * kPi;
    }
    return a;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
{
    return std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double yaw_from_matrix(const Eigen::Matrix4f &m)
{
    return std::atan2(static_cast<double>(m(1, 0)), static_cast<double>(m(0, 0)));
}

double transform_xy_distance(const Eigen::Matrix4f &a, const Eigen::Matrix4f &b)
{
    const double dx = static_cast<double>(a(0, 3) - b(0, 3));
    const double dy = static_cast<double>(a(1, 3) - b(1, 3));
    return std::hypot(dx, dy);
}

bool transforms_materially_different(
    const Eigen::Matrix4f &a,
    const Eigen::Matrix4f &b,
    double min_translation,
    double min_yaw_deg)
{
    return transform_xy_distance(a, b) >= min_translation ||
           std::abs(normalize_angle(yaw_from_matrix(a) - yaw_from_matrix(b))) >= min_yaw_deg * kPi / 180.0;
}

Eigen::Matrix4f make_yaw_xy_transform(double yaw, double x, double y)
{
    Eigen::Matrix4f t = Eigen::Matrix4f::Identity();
    const float c = static_cast<float>(std::cos(yaw));
    const float s = static_cast<float>(std::sin(yaw));
    t(0, 0) = c;
    t(0, 1) = -s;
    t(1, 0) = s;
    t(1, 1) = c;
    t(0, 3) = static_cast<float>(x);
    t(1, 3) = static_cast<float>(y);
    return t;
}

std::vector<std::string> split_csv_line(const std::string &line)
{
    std::vector<std::string> out;
    std::string cell;
    std::stringstream ss(line);
    while (std::getline(ss, cell, ','))
    {
        out.push_back(cell);
    }
    return out;
}

int find_col(const std::vector<std::string> &header, const std::string &name)
{
    for (size_t i = 0; i < header.size(); ++i)
    {
        if (header[i] == name)
        {
            return static_cast<int>(i);
        }
    }
    return -1;
}

bool parse_double(const std::string &text, double &value)
{
    try
    {
        size_t used = 0;
        value = std::stod(text, &used);
        return used > 0;
    }
    catch (...)
    {
        return false;
    }
}

std::string format_double(double value)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(6) << value;
    return os.str();
}

struct RouteStart
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

bool read_route_start(const std::string &route_file, RouteStart &start, std::string &error)
{
    std::ifstream in(route_file);
    if (!in.is_open())
    {
        error = "cannot open route file: " + route_file;
        return false;
    }

    std::string header_line;
    if (!std::getline(in, header_line))
    {
        error = "route csv is empty: " + route_file;
        return false;
    }
    const auto header = split_csv_line(header_line);
    const int ix = find_col(header, "x");
    const int iy = find_col(header, "y");
    const int iyaw = find_col(header, "yaw");
    if (ix < 0 || iy < 0 || iyaw < 0)
    {
        error = "route csv must contain x,y,yaw columns: " + route_file;
        return false;
    }

    std::string row_line;
    while (std::getline(in, row_line))
    {
        if (row_line.empty())
        {
            continue;
        }
        const auto row = split_csv_line(row_line);
        if (static_cast<int>(row.size()) <= std::max({ix, iy, iyaw}))
        {
            continue;
        }
        if (!parse_double(row[ix], start.x) ||
            !parse_double(row[iy], start.y) ||
            !parse_double(row[iyaw], start.yaw))
        {
            error = "invalid first route waypoint numeric values";
            return false;
        }
        return true;
    }

    error = "route csv has no waypoint rows: " + route_file;
    return false;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
    double min_z,
    double max_z,
    double max_range,
    bool use_center,
    double center_x,
    double center_y,
    double voxel_leaf,
    int max_points)
{
    auto filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    filtered->reserve(input->size());

    const double max_range_sq = max_range > 0.0 ? max_range * max_range : 0.0;
    for (const auto &p : input->points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        {
            continue;
        }
        if (p.z < min_z || p.z > max_z)
        {
            continue;
        }
        if (max_range > 0.0 && use_center)
        {
            const double dx = static_cast<double>(p.x) - center_x;
            const double dy = static_cast<double>(p.y) - center_y;
            if (dx * dx + dy * dy > max_range_sq)
            {
                continue;
            }
        }
        filtered->push_back(p);
    }

    if (voxel_leaf > 0.0 && !filtered->empty())
    {
        auto downsampled = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(filtered);
        const float leaf = static_cast<float>(voxel_leaf);
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.filter(*downsampled);
        filtered = downsampled;
    }

    if (max_points > 0 && static_cast<int>(filtered->size()) > max_points)
    {
        auto limited = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        limited->reserve(static_cast<size_t>(max_points));
        const double stride = static_cast<double>(filtered->size()) / static_cast<double>(max_points);
        for (int i = 0; i < max_points; ++i)
        {
            const size_t idx = std::min(
                filtered->size() - 1,
                static_cast<size_t>(std::floor(static_cast<double>(i) * stride)));
            limited->push_back(filtered->points[idx]);
        }
        filtered = limited;
    }

    filtered->width = static_cast<uint32_t>(filtered->size());
    filtered->height = 1;
    filtered->is_dense = true;
    return filtered;
}

bool transform_route_csv(
    const std::string &in_file,
    const std::string &out_file,
    const Eigen::Matrix4f &t_new_from_old,
    std::string &error)
{
    std::ifstream in(in_file);
    if (!in.is_open())
    {
        error = "cannot open route file: " + in_file;
        return false;
    }

    std::filesystem::create_directories(std::filesystem::path(out_file).parent_path());
    const std::string tmp_file = out_file + ".tmp";
    std::ofstream out(tmp_file);
    if (!out.is_open())
    {
        error = "cannot open output route file: " + tmp_file;
        return false;
    }

    std::string header_line;
    if (!std::getline(in, header_line))
    {
        error = "route csv is empty: " + in_file;
        return false;
    }
    const auto header = split_csv_line(header_line);
    const int ix = find_col(header, "x");
    const int iy = find_col(header, "y");
    const int iz = find_col(header, "z");
    const int iyaw = find_col(header, "yaw");
    if (ix < 0 || iy < 0 || iyaw < 0)
    {
        error = "route csv must contain x,y,yaw columns: " + in_file;
        return false;
    }

    out << header_line << "\n";
    const double yaw_delta = yaw_from_matrix(t_new_from_old);
    int rows = 0;
    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty())
        {
            continue;
        }
        auto row = split_csv_line(line);
        const int need = std::max({ix, iy, iz, iyaw});
        if (static_cast<int>(row.size()) <= need)
        {
            continue;
        }

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double yaw = 0.0;
        if (!parse_double(row[ix], x) ||
            !parse_double(row[iy], y) ||
            !parse_double(row[iyaw], yaw))
        {
            error = "invalid route waypoint numeric values";
            return false;
        }
        if (iz >= 0)
        {
            (void)parse_double(row[iz], z);
        }

        Eigen::Vector4f p_old(
            static_cast<float>(x),
            static_cast<float>(y),
            static_cast<float>(z),
            1.0f);
        const Eigen::Vector4f p_new = t_new_from_old * p_old;

        row[ix] = format_double(p_new.x());
        row[iy] = format_double(p_new.y());
        if (iz >= 0)
        {
            row[iz] = format_double(p_new.z());
        }
        row[iyaw] = format_double(normalize_angle(yaw + yaw_delta));

        for (size_t i = 0; i < row.size(); ++i)
        {
            if (i > 0)
            {
                out << ",";
            }
            out << row[i];
        }
        out << "\n";
        ++rows;
    }

    out.close();
    if (rows < 2)
    {
        error = "transformed route has fewer than 2 rows";
        return false;
    }
    std::filesystem::rename(tmp_file, out_file);
    return true;
}

void write_meta_json(
    const std::string &out_route_file,
    const std::string &route_file,
    const std::string &map_file,
    const Eigen::Matrix4f &t_old_from_current,
    double fitness,
    double alternate_fitness,
    int scan_points,
    int map_points,
    int cloud_frames,
    double capture_translation,
    double capture_yaw,
    bool start_anchor_applied,
    double start_anchor_residual,
    double start_anchor_yaw_error)
{
    const std::string meta_file = out_route_file + ".relocalize.json";
    std::ofstream meta(meta_file);
    if (!meta.is_open())
    {
        return;
    }

    meta << "{\n";
    meta << "  \"schema\": 2,\n";
    meta << "  \"sourceRoute\": \"" << route_file << "\",\n";
    meta << "  \"mapFile\": \"" << map_file << "\",\n";
    meta << "  \"outputRoute\": \"" << out_route_file << "\",\n";
    meta << "  \"fitness\": " << std::fixed << std::setprecision(6) << fitness << ",\n";
    if (std::isfinite(alternate_fitness))
    {
        meta << "  \"alternateFitness\": " << std::fixed << std::setprecision(6) << alternate_fitness << ",\n";
    }
    meta << "  \"scanPoints\": " << scan_points << ",\n";
    meta << "  \"mapPoints\": " << map_points << ",\n";
    meta << "  \"cloudFrames\": " << cloud_frames << ",\n";
    meta << "  \"captureTranslation\": " << std::fixed << std::setprecision(6) << capture_translation << ",\n";
    meta << "  \"captureYawDeg\": " << std::fixed << std::setprecision(3) << capture_yaw * 180.0 / kPi << ",\n";
        meta << "  \"startAnchorApplied\": " << (start_anchor_applied ? "true" : "false") << ",\n";
        meta << "  \"startAnchorResidual\": " << std::fixed << std::setprecision(6) << start_anchor_residual << ",\n";
        meta << "  \"startAnchorYawErrorDeg\": " << std::fixed << std::setprecision(3)
            << start_anchor_yaw_error * 180.0 / kPi << ",\n";
    meta << "  \"transformOldFromCurrent\": [";
    for (int r = 0; r < 4; ++r)
    {
        for (int c = 0; c < 4; ++c)
        {
            if (r != 0 || c != 0)
            {
                meta << ", ";
            }
            meta << std::fixed << std::setprecision(8) << t_old_from_current(r, c);
        }
    }
    meta << "]\n";
    meta << "}\n";
}

struct IcpResult
{
    bool ok = false;
    double fitness = std::numeric_limits<double>::infinity();
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
};

IcpResult run_icp(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
    const Eigen::Matrix4f &guess,
    double max_corr,
    int iterations,
    double trans_eps,
    double fit_eps)
{
    IcpResult result;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaxCorrespondenceDistance(max_corr);
    icp.setMaximumIterations(iterations);
    icp.setTransformationEpsilon(trans_eps);
    icp.setEuclideanFitnessEpsilon(fit_eps);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned, guess);

    result.ok = icp.hasConverged();
    result.fitness = icp.getFitnessScore(max_corr);
    result.transform = icp.getFinalTransformation();
    return result;
}

class RelocalizerNode : public rclcpp::Node
{
public:
    RelocalizerNode()
        : Node("route_relocalizer")
    {
        route_file = declare_parameter<std::string>("route_file", "");
        map_file = declare_parameter<std::string>("map_file", "");
        out_route_file = declare_parameter<std::string>("out_route_file", "/tmp/go2_route_runtime/relocalized.csv");
        cloud_topic = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
        cloud_in_body_frame = declare_parameter<bool>("cloud_in_body_frame", false);
        odom_topic = declare_parameter<std::string>("odom_topic", "/Odometry");
        collect_seconds = declare_parameter<double>("collect_seconds", 3.0);
        min_points = declare_parameter<int>("min_points", 200);
        min_z = declare_parameter<double>("min_z", -1.5);
        max_z = declare_parameter<double>("max_z", 2.5);
        max_scan_range = declare_parameter<double>("max_scan_range", 35.0);
        map_voxel_leaf = declare_parameter<double>("map_voxel_leaf", 0.35);
        scan_voxel_leaf = declare_parameter<double>("scan_voxel_leaf", 0.25);
        coarse_voxel_leaf = declare_parameter<double>("coarse_voxel_leaf", 0.60);
        coarse_max_points = declare_parameter<int>("coarse_max_points", 2500);
        yaw_search_deg = declare_parameter<double>("yaw_search_deg", 180.0);
        yaw_step_deg = declare_parameter<double>("yaw_step_deg", 30.0);
        xy_search_radius = declare_parameter<double>("xy_search_radius", 5.0);
        xy_search_step = declare_parameter<double>("xy_search_step", 2.0);
        max_correspondence_distance = declare_parameter<double>("max_correspondence_distance", 1.5);
        coarse_max_correspondence_distance = declare_parameter<double>("coarse_max_correspondence_distance", 2.5);
        icp_max_iterations = declare_parameter<int>("icp_max_iterations", 60);
        coarse_icp_iterations = declare_parameter<int>("coarse_icp_iterations", 25);
        fitness_threshold = declare_parameter<double>("fitness_threshold", 0.12);
        min_cloud_frames = declare_parameter<int>("min_cloud_frames", 8);
        max_capture_translation = declare_parameter<double>("max_capture_translation", 0.10);
        max_capture_yaw_deg = declare_parameter<double>("max_capture_yaw_deg", 5.0);
        ambiguity_fitness_ratio = declare_parameter<double>("ambiguity_fitness_ratio", 1.20);
        ambiguity_min_translation = declare_parameter<double>("ambiguity_min_translation", 1.00);
        ambiguity_min_yaw_deg = declare_parameter<double>("ambiguity_min_yaw_deg", 20.0);
        anchor_route_start = declare_parameter<bool>("anchor_route_start", false);
        max_start_anchor_residual = declare_parameter<double>("max_start_anchor_residual", 0.0);
        max_start_anchor_yaw_deg = declare_parameter<double>("max_start_anchor_yaw_deg", 0.0);

        scan_raw.reset(new pcl::PointCloud<pcl::PointXYZ>());

        odom_sub = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic,
            20,
            std::bind(&RelocalizerNode::odom_callback, this, std::placeholders::_1));
        cloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic,
            10,
            std::bind(&RelocalizerNode::cloud_callback, this, std::placeholders::_1));
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto &p = msg->pose.pose.position;
        current_x = p.x;
        current_y = p.y;
        current_z = p.z;
        current_yaw = yaw_from_quaternion(msg->pose.pose.orientation);
        const auto &q = msg->pose.pose.orientation;
        current_orientation = Eigen::Quaternionf(q.w, q.x, q.y, q.z).normalized();
        have_odom = true;
        if (have_first_cloud && !capture_pose_set)
        {
            capture_start_x = current_x;
            capture_start_y = current_y;
            capture_start_yaw = current_yaw;
            capture_pose_set = true;
        }
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        if (cloud.empty())
        {
            return;
        }
        if (cloud_in_body_frame && !have_odom)
        {
            return;
        }

        for (const auto &p : cloud.points)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            {
                continue;
            }
            if (cloud_in_body_frame)
            {
                const Eigen::Vector3f point_in_body(p.x, p.y, p.z);
                const Eigen::Vector3f point_in_odom = current_orientation * point_in_body +
                    Eigen::Vector3f(
                        static_cast<float>(current_x),
                        static_cast<float>(current_y),
                        static_cast<float>(current_z));
                scan_raw->push_back(pcl::PointXYZ(point_in_odom.x(), point_in_odom.y(), point_in_odom.z()));
            }
            else
            {
                scan_raw->push_back(p);
            }
        }
        ++cloud_frames;
        if (!have_first_cloud)
        {
            have_first_cloud = true;
            first_cloud_time = std::chrono::steady_clock::now();
            if (have_odom)
            {
                capture_start_x = current_x;
                capture_start_y = current_y;
                capture_start_yaw = current_yaw;
                capture_pose_set = true;
            }
        }
    }

    std::string route_file;
    std::string map_file;
    std::string out_route_file;
    std::string cloud_topic;
    bool cloud_in_body_frame = false;
    std::string odom_topic;
    double collect_seconds = 3.0;
    int min_points = 200;
    double min_z = -1.5;
    double max_z = 2.5;
    double max_scan_range = 35.0;
    double map_voxel_leaf = 0.35;
    double scan_voxel_leaf = 0.25;
    double coarse_voxel_leaf = 0.60;
    int coarse_max_points = 2500;
    double yaw_search_deg = 180.0;
    double yaw_step_deg = 30.0;
    double xy_search_radius = 5.0;
    double xy_search_step = 2.0;
    double max_correspondence_distance = 1.5;
    double coarse_max_correspondence_distance = 2.5;
    int icp_max_iterations = 60;
    int coarse_icp_iterations = 25;
    double fitness_threshold = 0.12;
    int min_cloud_frames = 8;
    double max_capture_translation = 0.10;
    double max_capture_yaw_deg = 5.0;
    double ambiguity_fitness_ratio = 1.20;
    double ambiguity_min_translation = 1.00;
    double ambiguity_min_yaw_deg = 20.0;
    bool anchor_route_start = false;
    double max_start_anchor_residual = 0.0;
    double max_start_anchor_yaw_deg = 0.0;

    bool have_odom = false;
    bool have_first_cloud = false;
    int cloud_frames = 0;
    double current_x = 0.0;
    double current_y = 0.0;
    double current_z = 0.0;
    double current_yaw = 0.0;
    Eigen::Quaternionf current_orientation = Eigen::Quaternionf::Identity();
    bool capture_pose_set = false;
    double capture_start_x = 0.0;
    double capture_start_y = 0.0;
    double capture_start_yaw = 0.0;
    std::chrono::steady_clock::time_point first_cloud_time;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_raw;

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
};

}  // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RelocalizerNode>();

    if (node->route_file.empty() || node->map_file.empty())
    {
        std::cerr << "RELOCALIZE_FAILED missing route_file or map_file" << std::endl;
        rclcpp::shutdown();
        return 2;
    }

    RouteStart route_start;
    std::string error;
    if (!read_route_start(node->route_file, route_start, error))
    {
        std::cerr << "RELOCALIZE_FAILED " << error << std::endl;
        rclcpp::shutdown();
        return 2;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_raw(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(node->map_file, *map_raw) != 0 || map_raw->empty())
    {
        std::cerr << "RELOCALIZE_FAILED cannot load map pcd: " << node->map_file << std::endl;
        rclcpp::shutdown();
        return 2;
    }

    const auto wait_start = std::chrono::steady_clock::now();
    const auto hard_deadline = wait_start + std::chrono::seconds(static_cast<int>(std::ceil(node->collect_seconds + 10.0)));
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if (node->have_first_cloud)
        {
            const auto elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - node->first_cloud_time).count();
            if (elapsed >= node->collect_seconds)
            {
                break;
            }
        }
        if (std::chrono::steady_clock::now() >= hard_deadline)
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!node->have_odom)
    {
        std::cerr << "RELOCALIZE_FAILED no odometry received on " << node->odom_topic << std::endl;
        rclcpp::shutdown();
        return 3;
    }
    if (static_cast<int>(node->scan_raw->size()) < node->min_points)
    {
        std::cerr << "RELOCALIZE_FAILED too few scan points raw=" << node->scan_raw->size()
                  << " frames=" << node->cloud_frames << std::endl;
        rclcpp::shutdown();
        return 3;
    }
    if (node->cloud_frames < node->min_cloud_frames)
    {
        std::cerr << "RELOCALIZE_FAILED too few cloud frames=" << node->cloud_frames
                  << " minimum=" << node->min_cloud_frames << std::endl;
        rclcpp::shutdown();
        return 3;
    }
    if (!node->capture_pose_set)
    {
        std::cerr << "RELOCALIZE_FAILED missing odometry at capture start" << std::endl;
        rclcpp::shutdown();
        return 3;
    }

    const double capture_translation = std::hypot(
        node->current_x - node->capture_start_x,
        node->current_y - node->capture_start_y);
    const double capture_yaw = std::abs(normalize_angle(node->current_yaw - node->capture_start_yaw));
    if (capture_translation > node->max_capture_translation ||
        capture_yaw > node->max_capture_yaw_deg * kPi / 180.0)
    {
        std::cerr << "RELOCALIZE_FAILED robot moved during capture"
                  << " translation=" << std::fixed << std::setprecision(3) << capture_translation
                  << "/" << node->max_capture_translation
                  << " yaw_deg=" << capture_yaw * 180.0 / kPi
                  << "/" << node->max_capture_yaw_deg << std::endl;
        rclcpp::shutdown();
        return 3;
    }

    auto map_fine = filter_cloud(
        map_raw,
        node->min_z,
        node->max_z,
        0.0,
        false,
        0.0,
        0.0,
        node->map_voxel_leaf,
        0);
    auto scan_fine = filter_cloud(
        node->scan_raw,
        node->min_z,
        node->max_z,
        node->max_scan_range,
        true,
        node->current_x,
        node->current_y,
        node->scan_voxel_leaf,
        0);
    auto map_coarse = filter_cloud(
        map_raw,
        node->min_z,
        node->max_z,
        0.0,
        false,
        0.0,
        0.0,
        node->coarse_voxel_leaf,
        0);
    auto scan_coarse = filter_cloud(
        node->scan_raw,
        node->min_z,
        node->max_z,
        node->max_scan_range,
        true,
        node->current_x,
        node->current_y,
        node->coarse_voxel_leaf,
        node->coarse_max_points);

    if (static_cast<int>(scan_fine->size()) < node->min_points ||
        static_cast<int>(map_fine->size()) < node->min_points ||
        static_cast<int>(scan_coarse->size()) < node->min_points ||
        static_cast<int>(map_coarse->size()) < node->min_points)
    {
        std::cerr << "RELOCALIZE_FAILED insufficient filtered points"
                  << " scan=" << scan_fine->size()
                  << " map=" << map_fine->size()
                  << " scan_coarse=" << scan_coarse->size()
                  << " map_coarse=" << map_coarse->size()
                  << std::endl;
        rclcpp::shutdown();
        return 3;
    }

    const double base_yaw = route_start.yaw - node->current_yaw;
    IcpResult best;
    std::vector<IcpResult> coarse_candidates;
    int tested = 0;

    const double yaw_step = std::max(1.0, std::abs(node->yaw_step_deg));
    const double xy_step = std::max(0.2, std::abs(node->xy_search_step));
    for (double yaw_deg = -std::abs(node->yaw_search_deg);
         yaw_deg <= std::abs(node->yaw_search_deg) + 1e-6;
         yaw_deg += yaw_step)
    {
        const double yaw = base_yaw + yaw_deg * kPi / 180.0;
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        const double rotated_current_x = c * node->current_x - s * node->current_y;
        const double rotated_current_y = s * node->current_x + c * node->current_y;

        for (double dx = -std::abs(node->xy_search_radius);
             dx <= std::abs(node->xy_search_radius) + 1e-6;
             dx += xy_step)
        {
            for (double dy = -std::abs(node->xy_search_radius);
                 dy <= std::abs(node->xy_search_radius) + 1e-6;
                 dy += xy_step)
            {
                const double tx = route_start.x + dx - rotated_current_x;
                const double ty = route_start.y + dy - rotated_current_y;
                const Eigen::Matrix4f guess = make_yaw_xy_transform(yaw, tx, ty);
                const auto result = run_icp(
                    scan_coarse,
                    map_coarse,
                    guess,
                    node->coarse_max_correspondence_distance,
                    node->coarse_icp_iterations,
                    1e-6,
                    1e-5);
                ++tested;
                if (result.ok && std::isfinite(result.fitness))
                {
                    coarse_candidates.push_back(result);
                    if (result.fitness < best.fitness)
                    {
                        best = result;
                    }
                }
            }
        }
    }

    if (!best.ok || !std::isfinite(best.fitness))
    {
        std::cerr << "RELOCALIZE_FAILED icp did not converge tested=" << tested << std::endl;
        rclcpp::shutdown();
        return 4;
    }

    std::sort(
        coarse_candidates.begin(),
        coarse_candidates.end(),
        [](const IcpResult &a, const IcpResult &b) { return a.fitness < b.fitness; });

    IcpResult alternate;
    for (const auto &candidate : coarse_candidates)
    {
        if (transforms_materially_different(
                candidate.transform,
                best.transform,
                node->ambiguity_min_translation,
                node->ambiguity_min_yaw_deg))
        {
            alternate = candidate;
            break;
        }
    }

    auto refined = run_icp(
        scan_fine,
        map_fine,
        best.transform,
        node->max_correspondence_distance,
        node->icp_max_iterations,
        1e-8,
        1e-6);
    if (refined.ok && refined.fitness < best.fitness)
    {
        best = refined;
    }

    if (alternate.ok)
    {
        const auto refined_alternate = run_icp(
            scan_fine,
            map_fine,
            alternate.transform,
            node->max_correspondence_distance,
            node->icp_max_iterations,
            1e-8,
            1e-6);
        if (refined_alternate.ok && std::isfinite(refined_alternate.fitness))
        {
            alternate = refined_alternate;
        }
        if (alternate.fitness < best.fitness)
        {
            std::swap(best, alternate);
        }
    }

    if (best.fitness > node->fitness_threshold)
    {
        std::cerr << "RELOCALIZE_FAILED fitness=" << std::fixed << std::setprecision(4) << best.fitness
                  << " threshold=" << node->fitness_threshold
                  << " tested=" << tested
                  << " scan=" << scan_fine->size()
                  << " map=" << map_fine->size()
                  << std::endl;
        rclcpp::shutdown();
        return 4;
    }
    if (alternate.ok &&
        transforms_materially_different(
            alternate.transform,
            best.transform,
            node->ambiguity_min_translation,
            node->ambiguity_min_yaw_deg) &&
        alternate.fitness <= best.fitness * node->ambiguity_fitness_ratio)
    {
        std::cerr << "RELOCALIZE_FAILED ambiguous pose"
                  << " best_fitness=" << std::fixed << std::setprecision(4) << best.fitness
                  << " alternate_fitness=" << alternate.fitness
                  << " ratio_limit=" << node->ambiguity_fitness_ratio
                  << " separation_m=" << transform_xy_distance(best.transform, alternate.transform)
                  << " yaw_separation_deg="
                  << std::abs(normalize_angle(yaw_from_matrix(best.transform) - yaw_from_matrix(alternate.transform))) * 180.0 / kPi
                  << std::endl;
        rclcpp::shutdown();
        return 4;
    }

    Eigen::Matrix4f t_old_from_current = best.transform;
    bool start_anchor_applied = false;
    double start_anchor_residual = 0.0;
    double start_anchor_yaw_error = 0.0;
    if (node->anchor_route_start)
    {
        const Eigen::Vector4f current_pose(
            static_cast<float>(node->current_x),
            static_cast<float>(node->current_y),
            0.0f,
            1.0f);
        const Eigen::Vector4f mapped_current = t_old_from_current * current_pose;
        start_anchor_residual = std::hypot(
            static_cast<double>(mapped_current.x()) - route_start.x,
            static_cast<double>(mapped_current.y()) - route_start.y);
        start_anchor_yaw_error = std::abs(normalize_angle(
            yaw_from_matrix(t_old_from_current) + node->current_yaw - route_start.yaw));
        const double max_anchor_yaw = node->max_start_anchor_yaw_deg * kPi / 180.0;
        if (node->max_start_anchor_residual <= 0.0 ||
            node->max_start_anchor_yaw_deg <= 0.0 ||
            start_anchor_residual > node->max_start_anchor_residual ||
            start_anchor_yaw_error > max_anchor_yaw)
        {
            std::cerr << "RELOCALIZE_FAILED start anchor residual=" << std::fixed << std::setprecision(3)
                      << start_anchor_residual << "/" << node->max_start_anchor_residual
                      << " yaw_error_deg=" << start_anchor_yaw_error * 180.0 / kPi
                      << "/" << node->max_start_anchor_yaw_deg << std::endl;
            rclcpp::shutdown();
            return 4;
        }
        t_old_from_current(0, 3) += static_cast<float>(route_start.x - mapped_current.x());
        t_old_from_current(1, 3) += static_cast<float>(route_start.y - mapped_current.y());
        start_anchor_applied = true;
    }
    const Eigen::Matrix4f t_current_from_old = t_old_from_current.inverse();
    if (!transform_route_csv(node->route_file, node->out_route_file, t_current_from_old, error))
    {
        std::cerr << "RELOCALIZE_FAILED " << error << std::endl;
        rclcpp::shutdown();
        return 2;
    }

    write_meta_json(
        node->out_route_file,
        node->route_file,
        node->map_file,
        t_old_from_current,
        best.fitness,
        alternate.ok ? alternate.fitness : std::numeric_limits<double>::infinity(),
        static_cast<int>(scan_fine->size()),
        static_cast<int>(map_fine->size()),
        node->cloud_frames,
        capture_translation,
        capture_yaw,
        start_anchor_applied,
        start_anchor_residual,
        start_anchor_yaw_error);

    std::cout << "RELOCALIZE_OK"
              << " route=" << node->route_file
              << " map=" << node->map_file
              << " out=" << node->out_route_file
              << " fitness=" << std::fixed << std::setprecision(4) << best.fitness
              << " tested=" << tested
              << " scan=" << scan_fine->size()
              << " map=" << map_fine->size()
              << " frames=" << node->cloud_frames
              << " capture_translation=" << capture_translation
              << " capture_yaw_deg=" << capture_yaw * 180.0 / kPi
              << " yaw_old_from_current=" << yaw_from_matrix(t_old_from_current)
              << " x_old_from_current=" << t_old_from_current(0, 3)
              << " y_old_from_current=" << t_old_from_current(1, 3)
              << " start_anchor=" << (start_anchor_applied ? "true" : "false")
              << " start_anchor_residual=" << start_anchor_residual
              << std::endl;

    rclcpp::shutdown();
    return 0;
}
