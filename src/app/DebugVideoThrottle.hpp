#pragma once

#include <algorithm>
#include <chrono>

namespace onboard::app {

// Decides whether a processed frame should be forwarded as debug video.
// send_fps is clamped to [1, camera_fps]; at or above the camera rate every
// processed frame is sent. The first call always sends.
//
// Pacing is credit-based: on a send, last_sent advances by exactly one send
// period (not to `now`), so the fractional remainder carries over to the
// next frame. A naive `last_sent = now` throttle beats against camera rates
// that do not divide the send period — e.g. 10 fps over a 12 fps camera
// (83 ms frames vs a 100 ms gate) skips every other frame and delivers only
// ~6 fps. The credit is clamped to one period so a stall cannot bank a
// burst.
inline bool shouldSendDebugVideoFrame(
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::time_point& last_sent,
    int send_fps,
    int camera_fps)
{
    const int max_fps = std::max(1, camera_fps);
    const int fps = std::clamp(send_fps, 1, max_fps);
    if (fps >= max_fps || last_sent.time_since_epoch().count() == 0) {
        last_sent = now;
        return true;
    }
    const auto min_period = std::chrono::microseconds(1000000 / fps);
    if (now - last_sent < min_period) {
        return false;
    }
    last_sent += min_period;
    if (now - last_sent > min_period) {
        last_sent = now - min_period;
    }
    return true;
}

} // namespace onboard::app
