#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <fstream>
#include <iomanip>
#include <filesystem>

class SubmapBuilder : public rclcpp::Node
{
public:
    SubmapBuilder()
        : Node("submap_builder")
    {
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered",
            10,
            std::bind(&SubmapBuilder::cloudCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry",
            100,
            std::bind(&SubmapBuilder::odomCallback, this, std::placeholders::_1));

        current_submap_.reset(new pcl::PointCloud<pcl::PointXYZI>());

        save_directory_ = std::string(std::getenv("HOME")) + "/maps";

        std::filesystem::create_directories(save_directory_);

        traj_file_.open(save_directory_ + "/trajectory.txt");

        if (!traj_file_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory file.");
        }

        current_pose_ = Eigen::Matrix4f::Identity();

        last_save_position_ = Eigen::Vector3f::Zero();

        submap_index_ = 0;

        first_pose_received_ = false;

        save_distance_threshold_ = 10.0;

        RCLCPP_INFO(this->get_logger(), "Submap Builder Started.");
    }

    ~SubmapBuilder()
    {
        saveCurrentSubmap();

        if (traj_file_.is_open())
        {
            traj_file_.close();
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr current_submap_;

    Eigen::Matrix4f current_pose_;

    Eigen::Vector3f current_position_;
    Eigen::Vector3f last_save_position_;

    bool first_pose_received_;

    std::ofstream traj_file_;

    std::string save_directory_;

    int submap_index_;

    double save_distance_threshold_;

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto &p = msg->pose.pose.position;
        const auto &q = msg->pose.pose.orientation;

        Eigen::Quaternionf quat(q.w, q.x, q.y, q.z);

        Eigen::Matrix3f rotation = quat.toRotationMatrix();

        current_pose_.setIdentity();

        current_pose_.block<3, 3>(0, 0) = rotation;
        current_pose_(0, 3) = p.x;
        current_pose_(1, 3) = p.y;
        current_pose_(2, 3) = p.z;

        current_position_ = Eigen::Vector3f(p.x, p.y, p.z);

        if (!first_pose_received_)
        {
            last_save_position_ = current_position_;
            first_pose_received_ = true;
        }

        if (traj_file_.is_open())
        {
            traj_file_ << std::fixed << std::setprecision(6)
                       << msg->header.stamp.sec << "."
                       << msg->header.stamp.nanosec << " "
                       << p.x << " "
                       << p.y << " "
                       << p.z << " "
                       << q.x << " "
                       << q.y << " "
                       << q.z << " "
                       << q.w << std::endl;
        }
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!first_pose_received_)
        {
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*cloud, *transformed_cloud, current_pose_);

        *current_submap_ += *transformed_cloud;

        double distance = (current_position_ - last_save_position_).norm();

        if (distance > save_distance_threshold_)
        {
            saveCurrentSubmap();
            current_submap_->clear();
            last_save_position_ = current_position_;
        }
    }

    void saveCurrentSubmap()
    {
        if (current_submap_->empty())
        {
            return;
        }

        std::stringstream ss;
        ss << save_directory_ << "/submap_"
           << std::setfill('0') << std::setw(4) << submap_index_
           << ".pcd";

        pcl::io::savePCDFileBinary(ss.str(), *current_submap_);

        RCLCPP_INFO(this->get_logger(), "Saved submap: %s | points: %zu",
                    ss.str().c_str(), current_submap_->size());

        submap_index_++;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SubmapBuilder>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
