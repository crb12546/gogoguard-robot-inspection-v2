#include <iostream>
#include <stdexcept>
#include <string>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/vui/vui_client.hpp>

int main(int argc, char** argv) {
  if (argc != 4 || std::string(argv[2]) != "set-volume") {
    std::cerr << "usage: go2_vui_control INTERFACE set-volume 0..10\n";
    return 64;
  }

  int requested = -1;
  try {
    requested = std::stoi(argv[3]);
  } catch (const std::exception&) {
    std::cerr << "volume must be an integer in 0..10\n";
    return 64;
  }
  if (requested < 0 || requested > 10) {
    std::cerr << "volume must be in 0..10\n";
    return 64;
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  unitree::robot::go2::VuiClient client;
  client.SetTimeout(3.0f);
  client.Init();
  const int set_code = client.SetVolume(requested);
  int observed = -1;
  const int get_code = client.GetVolume(observed);
  std::cout << "{\"service\":\"vui\",\"action\":\"set-volume\""
            << ",\"requested\":" << requested
            << ",\"setCode\":" << set_code
            << ",\"getCode\":" << get_code
            << ",\"observed\":" << observed << "}\n";
  return set_code == 0 && get_code == 0 && observed == requested ? 0 : 2;
}
