#include <cassert>
#include <cstdint>

#include "comm/no_sync_timestamp_mapper.h"

int main() {
  constexpr uint64_t kSecond = 1000000000ULL;
  livox_ros::NoSyncTimestampMapper mapper;

  assert(mapper.Map(10 * kSecond, 100 * kSecond) == 100 * kSecond);
  assert(mapper.Map(11 * kSecond, 101 * kSecond) == 101 * kSecond);

  // A small cross-stream reordering must keep the shared epoch.
  assert(mapper.Map(11 * kSecond - 5000000ULL,
                    101 * kSecond + 5000000ULL) ==
         101 * kSecond - 5000000ULL);

  // Reproduce the field failure: NTP moves the host back by 28 seconds.
  assert(mapper.Map(12 * kSecond, 74 * kSecond) == 74 * kSecond);
  assert(mapper.Map(13 * kSecond, 75 * kSecond) == 75 * kSecond);

  // A later forward correction must also create a new common epoch.
  assert(mapper.Map(14 * kSecond, 105 * kSecond) == 105 * kSecond);
  assert(mapper.Map(15 * kSecond, 106 * kSecond) == 106 * kSecond);

  return 0;
}
