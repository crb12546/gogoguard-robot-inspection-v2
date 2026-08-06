#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <numeric>
#include <string>
#include <unistd.h>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "go2_cmd_vel_bridge/command_safety.hpp"

namespace
{
constexpr uint32_t kCmdPacketMagic = 0x4732434dU;  // "G2CM"
constexpr uint16_t kCmdPacketVersion = 2;

#pragma pack(push, 1)
struct CmdPacket
{
  uint32_t magic;
  uint16_t version;
  uint16_t packet_size;
  uint64_t sequence;
  uint64_t send_steady_ns;
  uint64_t send_system_ns;
  float vx;
  float vy;
  float vyaw;
};
#pragma pack(pop)

struct Summary
{
  Summary(double average_in = 0.0, double p95_in = 0.0, double maximum_in = 0.0)
  : average(average_in), p95(p95_in), maximum(maximum_in) {}

  double average;
  double p95;
  double maximum;
};

Summary summarize(std::vector<double> values)
{
  if (values.empty()) return Summary();
  std::sort(values.begin(), values.end());
  const auto p95_index = std::min(
    values.size() - 1,
    static_cast<size_t>(std::ceil(values.size() * 0.95)) - 1);
  const double total = std::accumulate(values.begin(), values.end(), 0.0);
  return Summary(total / values.size(), values[p95_index], values.back());
}

uint64_t steady_now_ns()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

uint64_t system_now_ns()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
}
}  // namespace

class CmdVelUdpSender : public rclcpp::Node
{
public:
  CmdVelUdpSender() : Node("cmd_vel_udp_sender")
  {
    this->declare_parameter<std::string>("target_ip", "127.0.0.1");
    this->declare_parameter<int>("target_port", 5005);
    this->declare_parameter<double>("max_vx", 0.3);
    this->declare_parameter<double>("max_vy", 0.10);
    this->declare_parameter<double>("max_vyaw", 0.5);
    // ROS and Unitree SportClient both use +Y to the robot's left on this
    // robot. Keep the conversion parameter explicit for safe rollback/tests.
    this->declare_parameter<double>("unitree_vy_sign", 1.0);

    target_ip_ = this->get_parameter("target_ip").as_string();
    target_port_ = this->get_parameter("target_port").as_int();
    max_vx_ = this->get_parameter("max_vx").as_double();
    max_vy_ = std::fabs(this->get_parameter("max_vy").as_double());
    max_vyaw_ = this->get_parameter("max_vyaw").as_double();
    unitree_vy_sign_ =
      this->get_parameter("unitree_vy_sign").as_double() < 0.0 ? -1.0 : 1.0;

    sock_ = socket(AF_INET, SOCK_DGRAM, 0);

    std::memset(&addr_, 0, sizeof(addr_));
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(target_port_);
    inet_pton(AF_INET, target_ip_.c_str(), &addr_.sin_addr);

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CmdVelUdpSender::callback, this, std::placeholders::_1));
    report_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&CmdVelUdpSender::report_timing, this));

    RCLCPP_INFO(
      this->get_logger(),
      "UDP sender to %s:%d, packet_v=%u, max_vy=%.3f, unitree_vy_sign=%+.0f",
      target_ip_.c_str(), target_port_, kCmdPacketVersion, max_vy_,
      unitree_vy_sign_);
  }

  ~CmdVelUdpSender()
  {
    if (sock_ >= 0) close(sock_);
  }

private:
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const uint64_t callback_start_ns = steady_now_ns();
    if (last_callback_ns_ > 0) {
      callback_gaps_ms_.push_back(
        static_cast<double>(callback_start_ns - last_callback_ns_) / 1e6);
    }
    last_callback_ns_ = callback_start_ns;

    CmdPacket pkt{};
    pkt.magic = kCmdPacketMagic;
    pkt.version = kCmdPacketVersion;
    pkt.packet_size = sizeof(CmdPacket);
    pkt.sequence = ++sequence_;
    const auto command = go2_cmd_vel_bridge::sanitize_planar_command(
      msg->linear.x, msg->linear.y, msg->angular.z,
      max_vx_, max_vy_, max_vyaw_);
    pkt.vx = command.vx;
    pkt.vy = static_cast<float>(unitree_vy_sign_ * command.vy);
    pkt.vyaw = command.vyaw;
    if (!command.valid) {
      ++invalid_commands_;
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "non-finite or invalid cmd_vel rejected; sending immediate zero command");
    }
    pkt.send_steady_ns = steady_now_ns();
    pkt.send_system_ns = system_now_ns();

    const auto send_start = std::chrono::steady_clock::now();
    const ssize_t sent = sendto(
      sock_, &pkt, sizeof(pkt), 0,
      reinterpret_cast<sockaddr *>(&addr_), sizeof(addr_));
    const auto send_end = std::chrono::steady_clock::now();
    send_call_ms_.push_back(
      std::chrono::duration<double, std::milli>(
        send_end - send_start).count());
    callback_compute_ms_.push_back(
      static_cast<double>(steady_now_ns() - callback_start_ns) / 1e6);

    if (sent != static_cast<ssize_t>(sizeof(pkt))) {
      ++send_errors_;
      last_errno_ = errno;
    }
    ++callback_count_;
    last_vx_ = pkt.vx;
    last_vy_ = pkt.vy;
    last_vyaw_ = pkt.vyaw;
  }

  void report_timing()
  {
    const auto gap = summarize(callback_gaps_ms_);
    const auto callback = summarize(callback_compute_ms_);
    const auto send = summarize(send_call_ms_);
    const double cmd_age_ms = last_callback_ns_ > 0
      ? static_cast<double>(steady_now_ns() - last_callback_ns_) / 1e6
      : -1.0;

    RCLCPP_INFO(
      this->get_logger(),
      "TIMING_SENDER ros_gap_ms=%.3f/%.3f/%.3f "
      "callback_ms=%.3f/%.3f/%.3f sendto_ms=%.3f/%.3f/%.3f "
      "cmd_age_ms=%.3f count=%llu errors=%llu invalid=%llu errno=%d seq=%llu "
      "cmd=(%.3f,%.3f,%.3f)",
      gap.average, gap.p95, gap.maximum,
      callback.average, callback.p95, callback.maximum,
      send.average, send.p95, send.maximum,
      cmd_age_ms,
      static_cast<unsigned long long>(callback_count_),
      static_cast<unsigned long long>(send_errors_),
      static_cast<unsigned long long>(invalid_commands_),
      last_errno_,
      static_cast<unsigned long long>(sequence_),
      last_vx_, last_vy_, last_vyaw_);

    if (
      gap.maximum > 100.0 || callback.maximum > 25.0 ||
      send.maximum > 10.0 || send_errors_ > 0 || invalid_commands_ > 0)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "TIMING_ALERT_SENDER ros_gap_max_ms=%.3f callback_max_ms=%.3f "
        "sendto_max_ms=%.3f errors=%llu invalid=%llu errno=%d",
        gap.maximum, callback.maximum, send.maximum,
        static_cast<unsigned long long>(send_errors_),
        static_cast<unsigned long long>(invalid_commands_), last_errno_);
    }

    callback_gaps_ms_.clear();
    callback_compute_ms_.clear();
    send_call_ms_.clear();
    callback_count_ = 0;
    send_errors_ = 0;
    invalid_commands_ = 0;
    last_errno_ = 0;
  }

  int sock_{-1};
  sockaddr_in addr_{};
  std::string target_ip_;
  int target_port_{5005};
  double max_vx_{0.3};
  double max_vy_{0.10};
  double max_vyaw_{0.5};
  double unitree_vy_sign_{1.0};
  uint64_t sequence_{0};
  uint64_t last_callback_ns_{0};
  uint64_t callback_count_{0};
  uint64_t send_errors_{0};
  uint64_t invalid_commands_{0};
  int last_errno_{0};
  float last_vx_{0.0f};
  float last_vy_{0.0f};
  float last_vyaw_{0.0f};
  std::vector<double> callback_gaps_ms_;
  std::vector<double> callback_compute_ms_;
  std::vector<double> send_call_ms_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelUdpSender>());
  rclcpp::shutdown();
  return 0;
}
