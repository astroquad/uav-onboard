#include "app/DebugVideoThrottle.hpp"

#include <cassert>
#include <chrono>

int main()
{
    using onboard::app::shouldSendDebugVideoFrame;
    using Clock = std::chrono::steady_clock;
    using std::chrono::milliseconds;

    const auto epoch = Clock::time_point {} + std::chrono::hours(1);

    // First frame always sends and stamps last_sent.
    {
        Clock::time_point last_sent {};
        assert(shouldSendDebugVideoFrame(epoch, last_sent, 10, 12));
        assert(last_sent == epoch);
    }

    // send_fps >= camera fps sends every processed frame (clamped).
    {
        Clock::time_point last_sent {};
        for (int i = 0; i < 5; ++i) {
            assert(shouldSendDebugVideoFrame(
                epoch + milliseconds(i), last_sent, 15, 12));
        }
    }

    // 10 fps over a 12 fps frame train keeps >= 100 ms spacing.
    {
        Clock::time_point last_sent {};
        int sent = 0;
        auto previous_sent = Clock::time_point {};
        for (int i = 0; i < 24; ++i) {
            const auto now = epoch + milliseconds(i * 1000 / 12);
            if (shouldSendDebugVideoFrame(now, last_sent, 10, 12)) {
                if (sent > 0) {
                    assert(now - previous_sent >= milliseconds(100));
                }
                previous_sent = now;
                ++sent;
            }
        }
        assert(sent >= 2);
        assert(sent < 24);
    }

    // Non-positive send_fps clamps up to 1 fps instead of dividing by zero.
    {
        Clock::time_point last_sent {};
        assert(shouldSendDebugVideoFrame(epoch, last_sent, 0, 12));
        assert(!shouldSendDebugVideoFrame(
            epoch + milliseconds(500), last_sent, 0, 12));
        assert(shouldSendDebugVideoFrame(
            epoch + milliseconds(1000), last_sent, 0, 12));
    }

    return 0;
}
