#include "common/Time.hpp"

#include <chrono>

namespace onboard::common {

std::int64_t unixTimestampMs()
{
    const auto now = std::chrono::system_clock::now();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch());
    return ms.count();
}

} // namespace onboard::common
