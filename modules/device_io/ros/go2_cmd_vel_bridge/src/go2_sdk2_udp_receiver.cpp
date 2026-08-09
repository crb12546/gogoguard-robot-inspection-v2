#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#include "go2_cmd_vel_bridge/command_watchdog.hpp"
#include "go2_cmd_vel_bridge/command_safety.hpp"

using namespace unitree::robot;

namespace
{
volatile std::sig_atomic_t g_running = 1;

void stop_signal_handler(int)
{
  g_running = 0;
}

constexpr uint32_t kCmdPacketMagic = 0x4732434dU;  // "G2CM"
constexpr uint16_t kCmdPacketVersion = 2;
// Device I/O owns only the final validity/watchdog envelope. Navigation owns
// the target cruise and acceleration profile inside this control headroom.
constexpr double kMaxForwardMps = 0.90;
constexpr double kMaxLateralMps = 0.20;
constexpr double kMaxYawRateRps = 0.60;

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

class StructuredEventLog
{
public:
  StructuredEventLog()
  {
    const char *configured = std::getenv("GO2_SDK_EVENT_LOG");
    if (configured == nullptr || *configured == '\0') return;
    required_ = true;
    path_ = configured;
    try {
      const std::filesystem::path path(path_);
      if (!path.is_absolute() ||
          !std::filesystem::is_directory(path.parent_path()))
      {
        error_ = "event log parent is unavailable";
        return;
      }
      constexpr uintmax_t kRotateBytes = 64ULL * 1024ULL * 1024ULL;
      if (std::filesystem::exists(path) &&
          std::filesystem::file_size(path) >= kRotateBytes)
      {
        const std::filesystem::path previous(path.string() + ".1");
        std::error_code ignored;
        std::filesystem::remove(previous, ignored);
        std::filesystem::rename(path, previous);
      }
      stream_.open(path_, std::ios::out | std::ios::app);
      if (!stream_.is_open()) error_ = "event log cannot be opened";
    } catch (const std::exception & exc) {
      error_ = exc.what();
    }
  }

  bool ready() const {return !required_ || stream_.is_open();}
  const std::string & error() const {return error_;}

  void Write(const std::string & event, const std::string & fields = "")
  {
    if (!stream_.is_open()) return;
    stream_ << "{\"schema\":\"go2.sdk_motion_event.v1\","
            << "\"wallTimeNs\":" << system_now_ns() << ","
            << "\"monotonicNs\":" << steady_now_ns() << ","
            << "\"event\":\"" << event << "\"";
    if (!fields.empty()) stream_ << "," << fields;
    stream_ << "}" << std::endl;
  }

private:
  bool required_{false};
  std::string path_;
  std::string error_;
  std::ofstream stream_;
};

uint64_t command_timeout_ns()
{
  constexpr long kDefaultTimeoutMs = 250;
  constexpr long kMinimumTimeoutMs = 100;
  constexpr long kMaximumTimeoutMs = 2000;
  const char *value = std::getenv("GO2_SDK_CMD_TIMEOUT_MS");
  if (value == nullptr || *value == '\0') {
    return static_cast<uint64_t>(kDefaultTimeoutMs) * 1000000ULL;
  }
  char *end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  const long timeout_ms =
    end != value && *end == '\0'
    ? std::max(kMinimumTimeoutMs, std::min(kMaximumTimeoutMs, parsed))
    : kDefaultTimeoutMs;
  return static_cast<uint64_t>(timeout_ms) * 1000000ULL;
}
}  // namespace

int main(int argc, char **argv)
{
  std::string net_interface = "eth0";
  int port = 5005;
  bool prepare_posture = false;

  if (argc >= 2) net_interface = argv[1];
  if (argc >= 3) port = std::stoi(argv[2]);
  if (argc >= 4) {
    if (std::string(argv[3]) != "prepare-posture") {
      std::cerr << "unsupported third argument; expected prepare-posture"
                << std::endl;
      return 7;
    }
    prepare_posture = true;
  }
  if (argc > 4) {
    std::cerr << "too many arguments" << std::endl;
    return 7;
  }

  StructuredEventLog event_log;
  if (!event_log.ready()) {
    std::cerr << "SDK structured event log unavailable: "
              << event_log.error() << std::endl;
    return 10;
  }

  ChannelFactory::Instance()->Init(0, net_interface);

  unitree::robot::go2::SportClient sport_client;
  sport_client.SetTimeout(10.0f);
  sport_client.Init();

  std::cout << "SDK command limits: max_vx=0.900 max_vy=0.200 "
            << "max_vyaw=0.600" << std::endl;

  // The active-map runtime is deliberately restarted when a new immutable
  // release is activated.  A software generation switch must never move the
  // physical robot, so posture preparation is opt-in at the process boundary.
  // The legacy patrol launcher passes the explicit token after its human
  // controlled startup gate; the warm target runtime does not.
  std::cout << "SDK posture preparation="
            << (prepare_posture ? "explicitly_enabled" : "disabled")
            << std::endl;
  if (prepare_posture) {
    const int32_t stand_up_ret = sport_client.StandUp();
    std::cout << "StandUp ret=" << stand_up_ret << std::endl;
    if (stand_up_ret != 0) {
      std::cerr << "SDK2 receiver startup failed: StandUp ret="
                << stand_up_ret << std::endl;
      return 4;
    }
    sleep(2);
    const int32_t balance_stand_ret = sport_client.BalanceStand();
    std::cout << "BalanceStand ret=" << balance_stand_ret << std::endl;
    if (balance_stand_ret != 0) {
      std::cerr << "SDK2 receiver startup failed: BalanceStand ret="
                << balance_stand_ret << std::endl;
      return 5;
    }
    sleep(1);
  }

  // StopMove is a Sport-mode command.  A dog that is lying down or has just
  // booted can reject it even though the SDK channel is healthy, so explicit
  // posture preparation must finish first.  We still require StopMove to
  // succeed before opening the UDP command boundary: a previous process may
  // have died while a non-zero Move was active, and readiness must prove a
  // zero-motion baseline rather than silently ignoring that state.
  const int32_t startup_stop_ret = sport_client.StopMove();
  event_log.Write(
    "startup_stop", "\"returnCode\":" + std::to_string(startup_stop_ret));
  std::cout << "STARTUP_STOP StopMove ret=" << startup_stop_ret << std::endl;
  if (startup_stop_ret != 0) {
    std::cerr << "SDK2 receiver cannot establish a zero-motion baseline"
              << std::endl;
    return 8;
  }

  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    std::cerr << "socket failed errno=" << errno << std::endl;
    return 2;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port);

  if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    std::cerr << "bind failed port=" << port << " errno=" << errno
              << std::endl;
    close(sock);
    return 3;
  }

  std::cout << "SDK2 receiver started on interface: " << net_interface
            << std::endl;
  std::cout << "UDP listen port: " << port
            << ", packet_v=" << kCmdPacketVersion << std::endl;

  const uint64_t watchdog_timeout_ns = command_timeout_ns();
  go2_cmd_vel_bridge::CommandWatchdog command_watchdog(
    watchdog_timeout_ns);
  std::cout << "SDK command watchdog timeout_ms="
            << watchdog_timeout_ns / 1000000ULL
            << " policy=arm_after_first_valid_packet_stop_once_per_outage"
            << std::endl;

  // Wake well before the watchdog deadline even if the upstream chain stops.
  timeval receive_timeout{};
  receive_timeout.tv_sec = 0;
  receive_timeout.tv_usec = 50000;
  if (setsockopt(
    sock, SOL_SOCKET, SO_RCVTIMEO,
    &receive_timeout, sizeof(receive_timeout)) != 0)
  {
    std::cerr << "setsockopt SO_RCVTIMEO failed errno=" << errno
              << std::endl;
    close(sock);
    return 6;
  }

  CmdPacket pkt{};
  uint64_t last_receive_ns = 0;
  uint64_t last_sequence = 0;
  uint64_t report_started_ns = steady_now_ns();
  uint64_t last_alert_ns = 0;
  uint64_t receive_count = 0;
  uint64_t sequence_gaps = 0;
  uint64_t invalid_packets = 0;
  uint64_t watchdog_stops = 0;
  float last_vx = 0.0f;
  float last_vy = 0.0f;
  float last_vyaw = 0.0f;
  std::vector<double> receive_gaps_ms;
  std::vector<double> udp_transit_ms;
  std::vector<double> move_call_ms;
  std::vector<double> source_to_move_done_ms;

  auto report_timing = [&](uint64_t now_ns) {
    const auto receive_gap = summarize(receive_gaps_ms);
    const auto transit = summarize(udp_transit_ms);
    const auto move = summarize(move_call_ms);
    const auto total = summarize(source_to_move_done_ms);
    const double cmd_age_ms = last_receive_ns > 0
      ? static_cast<double>(now_ns - last_receive_ns) / 1e6
      : -1.0;

    std::cout << std::fixed << std::setprecision(3)
              << "TIMING_RECEIVER "
              << "udp_gap_ms=" << receive_gap.average << "/"
              << receive_gap.p95 << "/" << receive_gap.maximum << " "
              << "udp_transit_ms=" << transit.average << "/"
              << transit.p95 << "/" << transit.maximum << " "
              << "move_call_ms=" << move.average << "/"
              << move.p95 << "/" << move.maximum << " "
              << "sender_to_move_done_ms=" << total.average << "/"
              << total.p95 << "/" << total.maximum << " "
              << "cmd_age_ms=" << cmd_age_ms << " "
              << "count=" << receive_count << " "
              << "seq_gaps=" << sequence_gaps << " "
              << "invalid=" << invalid_packets << " "
              << "last_seq=" << last_sequence << " "
              << "cmd=(" << last_vx << "," << last_vy << ","
              << last_vyaw << ")" << std::endl;

    std::ostringstream timing_fields;
    timing_fields << std::fixed << std::setprecision(3)
                  << "\"udpGapP95Ms\":" << receive_gap.p95 << ","
                  << "\"udpGapMaxMs\":" << receive_gap.maximum << ","
                  << "\"udpTransitP95Ms\":" << transit.p95 << ","
                  << "\"moveCallP95Ms\":" << move.p95 << ","
                  << "\"sourceToMoveP95Ms\":" << total.p95 << ","
                  << "\"commandAgeMs\":" << cmd_age_ms << ","
                  << "\"sequenceGaps\":" << sequence_gaps << ","
                  << "\"invalidPackets\":" << invalid_packets;
    event_log.Write("timing", timing_fields.str());

    const bool alert =
      receive_gap.maximum > 100.0 || transit.maximum > 20.0 ||
      move.maximum > 25.0 || total.maximum > 50.0 ||
      sequence_gaps > 0 || invalid_packets > 0;
    if (alert && now_ns - last_alert_ns >= 1000000000ULL) {
      last_alert_ns = now_ns;
      std::cerr << std::fixed << std::setprecision(3)
                << "TIMING_ALERT_RECEIVER "
                << "udp_gap_max_ms=" << receive_gap.maximum << " "
                << "udp_transit_max_ms=" << transit.maximum << " "
                << "move_call_max_ms=" << move.maximum << " "
                << "sender_to_move_done_max_ms=" << total.maximum << " "
                << "seq_gaps=" << sequence_gaps << " "
                << "invalid=" << invalid_packets << std::endl;
    }

    receive_gaps_ms.clear();
    udp_transit_ms.clear();
    move_call_ms.clear();
    source_to_move_done_ms.clear();
    receive_count = 0;
    sequence_gaps = 0;
    invalid_packets = 0;
    report_started_ns = now_ns;
  };

  std::signal(SIGINT, stop_signal_handler);
  std::signal(SIGTERM, stop_signal_handler);

  while (g_running != 0)
  {
    const ssize_t n = recv(sock, &pkt, sizeof(pkt), 0);
    const uint64_t receive_ns = steady_now_ns();
    if (n == static_cast<ssize_t>(sizeof(pkt)) &&
        pkt.magic == kCmdPacketMagic &&
        pkt.version == kCmdPacketVersion &&
        pkt.packet_size == sizeof(CmdPacket))
    {
      const auto command = go2_cmd_vel_bridge::sanitize_planar_command(
        pkt.vx, pkt.vy, pkt.vyaw,
        kMaxForwardMps, kMaxLateralMps, kMaxYawRateRps);
      if (!command.valid) {
        ++invalid_packets;
        last_vx = 0.0F;
        last_vy = 0.0F;
        last_vyaw = 0.0F;
        const int32_t stop_ret = sport_client.StopMove();
        event_log.Write(
          "invalid_command_stop",
          "\"returnCode\":" + std::to_string(stop_ret));
        std::cerr << "INVALID_CMD_PACKET_STOP non-finite velocity StopMove ret="
                  << stop_ret << std::endl;
        continue;
      }
      if (last_receive_ns > 0) {
        receive_gaps_ms.push_back(
          static_cast<double>(receive_ns - last_receive_ns) / 1e6);
      }
      if (pkt.send_steady_ns <= receive_ns) {
        udp_transit_ms.push_back(
          static_cast<double>(receive_ns - pkt.send_steady_ns) / 1e6);
      }
      if (last_sequence > 0 && pkt.sequence > last_sequence + 1) {
        sequence_gaps += pkt.sequence - last_sequence - 1;
      }

      last_receive_ns = receive_ns;
      last_sequence = pkt.sequence;
      last_vx = command.vx;
      last_vy = command.vy;
      last_vyaw = command.vyaw;

      const bool recovered_from_timeout =
        command_watchdog.OnValidCommand(receive_ns);
      if (recovered_from_timeout) {
        event_log.Write(
          "watchdog_recovered",
          "\"sequence\":" + std::to_string(pkt.sequence));
        std::cout << "CMD_WATCHDOG_RECOVERED sequence=" << pkt.sequence
                  << std::endl;
      }

      const uint64_t move_start_ns = steady_now_ns();
      sport_client.Move(last_vx, last_vy, last_vyaw);
      const uint64_t move_done_ns = steady_now_ns();
      move_call_ms.push_back(
        static_cast<double>(move_done_ns - move_start_ns) / 1e6);
      if (pkt.send_steady_ns <= move_done_ns) {
        source_to_move_done_ms.push_back(
          static_cast<double>(move_done_ns - pkt.send_steady_ns) / 1e6);
      }
      ++receive_count;
    }
    else if (n >= 0) {
      ++invalid_packets;
    }
    else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      std::cerr << "recv failed errno=" << errno << std::endl;
    }

    if (command_watchdog.ShouldStop(receive_ns)) {
      const int32_t stop_ret = sport_client.StopMove();
      ++watchdog_stops;
      last_vx = 0.0f;
      last_vy = 0.0f;
      last_vyaw = 0.0f;
      const double age_ms = static_cast<double>(
        receive_ns - command_watchdog.last_command_ns()) / 1e6;
      std::ostringstream watchdog_fields;
      watchdog_fields << std::fixed << std::setprecision(3)
                      << "\"ageMs\":" << age_ms << ","
                      << "\"timeoutMs\":"
                      << command_watchdog.timeout_ns() / 1e6 << ","
                      << "\"returnCode\":" << stop_ret << ","
                      << "\"totalStops\":" << watchdog_stops;
      event_log.Write("watchdog_stop", watchdog_fields.str());
      std::cerr << std::fixed << std::setprecision(3)
                << "CMD_WATCHDOG_STOP age_ms=" << age_ms
                << " timeout_ms="
                << command_watchdog.timeout_ns() / 1e6
                << " StopMove ret=" << stop_ret
                << " total=" << watchdog_stops << std::endl;
    }

    if (receive_ns - report_started_ns >= 1000000000ULL) {
      report_timing(receive_ns);
    }
  }

  const int32_t shutdown_stop_ret = sport_client.StopMove();
  event_log.Write(
    "shutdown_stop",
    "\"returnCode\":" + std::to_string(shutdown_stop_ret));
  std::cout << "SHUTDOWN_STOP StopMove ret=" << shutdown_stop_ret << std::endl;
  close(sock);
  return shutdown_stop_ret == 0 ? 0 : 9;
}
