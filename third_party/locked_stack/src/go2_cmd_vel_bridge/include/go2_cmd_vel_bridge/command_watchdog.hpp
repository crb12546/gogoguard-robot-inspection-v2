#ifndef GO2_CMD_VEL_BRIDGE__COMMAND_WATCHDOG_HPP_
#define GO2_CMD_VEL_BRIDGE__COMMAND_WATCHDOG_HPP_

#include <cstdint>

namespace go2_cmd_vel_bridge
{

// Small, dependency-free state machine used by the SDK receiver.  It arms
// only after the first valid command, trips once per outage, and rearms when a
// later valid command arrives.  Keeping this independent of ROS and Unitree's
// SDK makes the safety transition testable on a development computer.
class CommandWatchdog
{
public:
  explicit CommandWatchdog(uint64_t timeout_ns)
  : timeout_ns_(timeout_ns > 0 ? timeout_ns : 1)
  {
  }

  // Return true when this command recovers a previously tripped watchdog.
  bool OnValidCommand(uint64_t now_ns)
  {
    const bool recovered = stopped_;
    armed_ = true;
    stopped_ = false;
    last_command_ns_ = now_ns;
    return recovered;
  }

  // Return true exactly once when the current outage crosses the timeout.
  bool ShouldStop(uint64_t now_ns)
  {
    if (!armed_ || stopped_ || now_ns < last_command_ns_) {
      return false;
    }
    if (now_ns - last_command_ns_ <= timeout_ns_) {
      return false;
    }
    stopped_ = true;
    return true;
  }

  bool armed() const {return armed_;}
  bool stopped() const {return stopped_;}
  uint64_t timeout_ns() const {return timeout_ns_;}
  uint64_t last_command_ns() const {return last_command_ns_;}

private:
  uint64_t timeout_ns_{1};
  uint64_t last_command_ns_{0};
  bool armed_{false};
  bool stopped_{false};
};

}  // namespace go2_cmd_vel_bridge

#endif  // GO2_CMD_VEL_BRIDGE__COMMAND_WATCHDOG_HPP_
