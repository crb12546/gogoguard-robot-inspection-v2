#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

namespace {

constexpr std::size_t kMaxPcmBytes = 96000;
constexpr const char* kAppName = "gogoguard";

int parse_int(const char* text, int minimum, int maximum, const char* name) {
  try {
    std::size_t consumed = 0;
    const int value = std::stoi(text, &consumed);
    if (text[consumed] != '\0' || value < minimum || value > maximum) {
      throw std::out_of_range(name);
    }
    return value;
  } catch (const std::exception&) {
    std::cerr << name << " must be between " << minimum << " and " << maximum
              << std::endl;
    std::exit(2);
  }
}

int finish(const std::string& action, int32_t code) {
  if (code != 0) {
    std::cerr << "VOICE_COMMAND_FAILED action=" << action << " code=" << code
              << std::endl;
    return 3;
  }
  std::cout << "VOICE_COMMAND_OK action=" << action << std::endl;
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: go2_voice_control INTERFACE "
                 "say SPEAKER_ID TEXT|get-volume|set-volume VALUE|stop|"
                 "play-pcm STREAM_ID"
              << std::endl;
    return 2;
  }

  const std::string interface = argv[1];
  const std::string command = argv[2];
  unitree::robot::ChannelFactory::Instance()->Init(0, interface);
  unitree::robot::g1::AudioClient client;
  client.Init();
  client.SetTimeout(3.0f);

  if (command == "say") {
    if (argc != 5) {
      std::cerr << "say requires SPEAKER_ID and TEXT" << std::endl;
      return 2;
    }
    const int speaker_id = parse_int(argv[3], 0, 16, "speaker id");
    const std::string text = argv[4];
    if (text.empty() || text.size() > 2000) {
      std::cerr << "text is empty or too long" << std::endl;
      return 2;
    }
    return finish(command, client.TtsMaker(text, speaker_id));
  }

  if (command == "get-volume") {
    if (argc != 3) {
      return 2;
    }
    uint8_t volume = 0;
    const int32_t code = client.GetVolume(volume);
    if (code == 0) {
      std::cout << "VOLUME=" << static_cast<int>(volume) << std::endl;
    }
    return finish(command, code);
  }

  if (command == "set-volume") {
    if (argc != 4) {
      return 2;
    }
    const int volume = parse_int(argv[3], 0, 100, "volume");
    return finish(command, client.SetVolume(static_cast<uint8_t>(volume)));
  }

  if (command == "stop") {
    if (argc != 3) {
      return 2;
    }
    return finish(command, client.PlayStop(kAppName));
  }

  if (command == "play-pcm") {
    if (argc != 4) {
      return 2;
    }
    const std::string stream_id = argv[3];
    if (stream_id.empty() || stream_id.size() > 128) {
      std::cerr << "invalid stream id" << std::endl;
      return 2;
    }
    std::vector<uint8_t> pcm(
        std::istreambuf_iterator<char>(std::cin), std::istreambuf_iterator<char>());
    if (pcm.empty() || pcm.size() > kMaxPcmBytes || pcm.size() % 2 != 0) {
      std::cerr << "PCM must contain 2-96000 even bytes" << std::endl;
      return 2;
    }
    return finish(command, client.PlayStream(kAppName, stream_id, pcm));
  }

  std::cerr << "unsupported voice command" << std::endl;
  return 2;
}
