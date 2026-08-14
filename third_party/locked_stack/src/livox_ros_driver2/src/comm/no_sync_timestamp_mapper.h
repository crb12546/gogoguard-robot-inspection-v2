//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//

#ifndef LIVOX_DRIVER_NO_SYNC_TIMESTAMP_MAPPER_H_
#define LIVOX_DRIVER_NO_SYNC_TIMESTAMP_MAPPER_H_

#include <algorithm>
#include <cstdint>
#include <mutex>

namespace livox_ros {

// A MID-360/MID-360S packet with timestamp type 0 carries the lidar uptime in
// nanoseconds. Preserve that stable device clock and add one ROS epoch offset.
// Using the host arrival time for every packet makes delayed packets appear to
// contain delayed measurements and can break frame boundaries.
class NoSyncTimestampMapper {
 public:
  uint64_t Map(uint64_t device_time_ns, uint64_t host_epoch_ns) {
    std::lock_guard<std::mutex> lock(mutex_);

    const bool device_restarted =
        initialized_ && device_time_ns < last_device_time_ns_ &&
        last_device_time_ns_ - device_time_ns > kDeviceResetThresholdNs;
    bool host_clock_stepped = false;
    if (initialized_) {
      // systemd-timesyncd may correct CLOCK_REALTIME after the driver has
      // already seen its first packet.  Compare host and device deltas so the
      // common LiDAR/IMU epoch follows that correction instead of remaining
      // permanently in the past or future.  Small packet reordering and
      // ordinary receive jitter stay below this deliberately coarse bound.
      if (host_epoch_ns + kHostClockStepThresholdNs < last_host_epoch_ns_) {
        host_clock_stepped = true;
      } else if (host_epoch_ns >= last_host_epoch_ns_ &&
                 device_time_ns >= last_device_time_ns_) {
        const uint64_t host_delta_ns = host_epoch_ns - last_host_epoch_ns_;
        const uint64_t device_delta_ns = device_time_ns - last_device_time_ns_;
        const uint64_t delta_error_ns =
            host_delta_ns >= device_delta_ns
                ? host_delta_ns - device_delta_ns
                : device_delta_ns - host_delta_ns;
        host_clock_stepped = delta_error_ns > kHostClockStepThresholdNs;
      }
    }
    if (!initialized_ || device_restarted || host_clock_stepped) {
      initialized_ = true;
      device_anchor_ns_ = device_time_ns;
      host_anchor_ns_ = host_epoch_ns;
      last_device_time_ns_ = device_time_ns;
      last_host_epoch_ns_ = host_epoch_ns;
      return host_epoch_ns;
    }

    last_device_time_ns_ = std::max(last_device_time_ns_, device_time_ns);
    last_host_epoch_ns_ = host_epoch_ns;
    if (device_time_ns >= device_anchor_ns_) {
      return host_anchor_ns_ + (device_time_ns - device_anchor_ns_);
    }

    // A small cross-stream reordering between IMU and point packets is valid.
    const uint64_t rollback_ns = device_anchor_ns_ - device_time_ns;
    return rollback_ns < host_anchor_ns_ ? host_anchor_ns_ - rollback_ns : 0;
  }

 private:
  static constexpr uint64_t kDeviceResetThresholdNs = 1000000000ULL;
  static constexpr uint64_t kHostClockStepThresholdNs = 1000000000ULL;

  std::mutex mutex_;
  bool initialized_ = false;
  uint64_t device_anchor_ns_ = 0;
  uint64_t host_anchor_ns_ = 0;
  uint64_t last_device_time_ns_ = 0;
  uint64_t last_host_epoch_ns_ = 0;
};

}  // namespace livox_ros

#endif  // LIVOX_DRIVER_NO_SYNC_TIMESTAMP_MAPPER_H_
