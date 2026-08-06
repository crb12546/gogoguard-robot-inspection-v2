#ifndef GO2_CMD_VEL_BRIDGE__COMMAND_SAFETY_HPP_
#define GO2_CMD_VEL_BRIDGE__COMMAND_SAFETY_HPP_

#include <algorithm>
#include <cmath>

namespace go2_cmd_vel_bridge
{

struct SafePlanarCommand
{
  float vx{0.0F};
  float vy{0.0F};
  float vyaw{0.0F};
  bool valid{false};
};

inline SafePlanarCommand sanitize_planar_command(
  double vx,
  double vy,
  double vyaw,
  double max_vx,
  double max_vy,
  double max_vyaw)
{
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vyaw) ||
    !std::isfinite(max_vx) || !std::isfinite(max_vy) ||
    !std::isfinite(max_vyaw) || max_vx < 0.0 || max_vy < 0.0 ||
    max_vyaw < 0.0)
  {
    return {};
  }
  SafePlanarCommand output;
  output.vx = static_cast<float>(std::max(-max_vx, std::min(max_vx, vx)));
  output.vy = static_cast<float>(std::max(-max_vy, std::min(max_vy, vy)));
  output.vyaw = static_cast<float>(
    std::max(-max_vyaw, std::min(max_vyaw, vyaw)));
  output.valid = true;
  return output;
}

}  // namespace go2_cmd_vel_bridge

#endif  // GO2_CMD_VEL_BRIDGE__COMMAND_SAFETY_HPP_
