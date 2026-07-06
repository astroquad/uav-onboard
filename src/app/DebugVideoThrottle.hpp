#pragma once

#include <algorithm>
#include <chrono>

namespace onboard::app {

// Decides whether a processed frame should be forwarded as debug video and,
// when it should, records `now` as the last send time. send_fps is clamped
// to [1, camera_fps]; at or above the camera rate every processed frame is
// sent. The first call always sends.
inline bool shouldSendDebugVideoFrame(
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::time_point& last_sent,
    int send_fps,
    int camera_fps)
{
    const int max_fps = std::max(1, camera_fps);
    const int fps = std::clamp(send_fps, 1, max_fps);
    const auto min_period = std::chrono::microseconds(1000000 / fps);
    const bool should_send =
        fps >= max_fps ||
        last_sent.time_since_epoch().count() == 0 ||
        now - last_sent >= min_period;
    if (should_send) {
        last_sent = now;
    }
    return should_send;
}

} // namespace onboard::app
