#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

using namespace unitree::robot;

namespace
{
void usage(const char *prog)
{
  std::cout
    << "Usage:\n"
    << "  " << prog << " --iface eth1 stop\n"
    << "  " << prog << " --iface eth1 balance\n"
    << "  " << prog << " --iface eth1 move_probe [--vx 0.05] [--vyaw 0.0] [--duration 0.6]\n\n"
    << "Safety limits for move_probe: |vx| <= 0.10 m/s, |vyaw| <= 0.20 rad/s, duration <= 1.0 s.\n"
    << "No action is executed unless one of stop/balance/move_probe is provided.\n";
}

float parse_float(const std::string &value, const std::string &name)
{
  char *end = nullptr;
  const float result = std::strtof(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0')
  {
    throw std::runtime_error("invalid " + name + ": " + value);
  }
  return result;
}
}  // namespace

int main(int argc, char **argv)
{
  const char *default_iface = std::getenv("GO2_SDK_IF");
  std::string iface = (default_iface && *default_iface) ? default_iface : "eth1";
  std::string action;
  float vx = 0.05f;
  float vyaw = 0.0f;
  float duration = 0.6f;

  try
  {
    for (int i = 1; i < argc; ++i)
    {
      const std::string arg = argv[i];
      if (arg == "--help" || arg == "-h")
      {
        usage(argv[0]);
        return 0;
      }
      if (arg == "--iface" && i + 1 < argc)
      {
        iface = argv[++i];
      }
      else if (arg == "--vx" && i + 1 < argc)
      {
        vx = parse_float(argv[++i], "vx");
      }
      else if (arg == "--vyaw" && i + 1 < argc)
      {
        vyaw = parse_float(argv[++i], "vyaw");
      }
      else if (arg == "--duration" && i + 1 < argc)
      {
        duration = parse_float(argv[++i], "duration");
      }
      else if (action.empty())
      {
        action = arg;
      }
      else
      {
        std::cerr << "Unexpected argument: " << arg << std::endl;
        usage(argv[0]);
        return 2;
      }
    }

    if (action.empty())
    {
      usage(argv[0]);
      return 2;
    }
    if (action != "stop" && action != "balance" && action != "move_probe")
    {
      std::cerr << "Unknown action: " << action << std::endl;
      usage(argv[0]);
      return 2;
    }

    vx = std::max(-0.10f, std::min(0.10f, vx));
    vyaw = std::max(-0.20f, std::min(0.20f, vyaw));
    duration = std::max(0.1f, std::min(1.0f, duration));

    std::cout << "Initializing SDK2 on interface " << iface << std::endl;
    ChannelFactory::Instance()->Init(0, iface);

    unitree::robot::go2::SportClient sport_client;
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    if (action == "stop")
    {
      std::cout << "Sending StopMove" << std::endl;
      sport_client.StopMove();
      return 0;
    }

    if (action == "balance")
    {
      std::cout << "Sending StandUp, then BalanceStand" << std::endl;
      sport_client.StandUp();
      std::this_thread::sleep_for(std::chrono::seconds(2));
      sport_client.BalanceStand();
      return 0;
    }

    std::cout << "Sending BalanceStand before short Move probe" << std::endl;
    sport_client.BalanceStand();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Move probe: vx=" << vx << " vyaw=" << vyaw << " duration=" << duration << "s" << std::endl;
    const auto start = std::chrono::steady_clock::now();
    const auto limit = std::chrono::duration<float>(duration);
    while (std::chrono::steady_clock::now() - start < limit)
    {
      sport_client.Move(vx, 0.0f, vyaw);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "Sending StopMove" << std::endl;
    for (int i = 0; i < 5; ++i)
    {
      sport_client.StopMove();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
  }
  catch (const std::exception &e)
  {
    std::cerr << "motion probe failed: " << e.what() << std::endl;
    return 1;
  }
}
