// Tests are assert-based: keep assert() active even in Release
// builds (CMake adds -DNDEBUG there, which silently no-ops all checks).
#undef NDEBUG

#include "app/DebugVideoThrottle.hpp"

#include <cassert>
#include <chrono>

int main()
{
    using onboard::app::shouldSendDebugVideoFrame;
    using Clock = std::chrono::steady_clock;
    using std::chrono::microseconds;
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

    // send_fps <= 0 means "match the processing rate": every processed frame.
    {
        using onboard::app::effectiveDebugVideoFps;
        assert(effectiveDebugVideoFps(0, 12) == 12);
        assert(effectiveDebugVideoFps(-1, 12) == 12);
        assert(effectiveDebugVideoFps(10, 12) == 10);
        assert(effectiveDebugVideoFps(30, 12) == 12); // capped at camera fps

        Clock::time_point last_sent {};
        for (int i = 0; i < 5; ++i) {
            assert(shouldSendDebugVideoFrame(
                epoch + milliseconds(i), last_sent, 0, 12));
        }
    }

    // Credit pacing: 10 fps over a 12 fps frame train must average ~10 fps,
    // not beat down to ~6 fps like a naive fixed-gate throttle.
    {
        Clock::time_point last_sent {};
        int sent = 0;
        const int frames = 120; // 10 seconds at 12 fps
        for (int i = 0; i < frames; ++i) {
            const auto now = epoch + microseconds(i * 1000000LL / 12);
            if (shouldSendDebugVideoFrame(now, last_sent, 10, 12)) {
                ++sent;
            }
        }
        assert(sent >= 98);
        assert(sent <= 102);
    }

    // A stall must not bank a burst: the credit clamp allows at most one
    // extra send over the target rate in the second after a 1 s gap.
    {
        Clock::time_point last_sent {};
        assert(shouldSendDebugVideoFrame(epoch, last_sent, 10, 12));
        const auto resume = epoch + milliseconds(1000);
        int sent = 0;
        for (int i = 0; i < 12; ++i) {
            const auto now = resume + microseconds(i * 1000000LL / 12);
            if (shouldSendDebugVideoFrame(now, last_sent, 10, 12)) {
                ++sent;
            }
        }
        assert(sent <= 11);
        assert(sent >= 9);
    }

    // send_fps = 1 caps at one frame per second.
    {
        Clock::time_point last_sent {};
        assert(shouldSendDebugVideoFrame(epoch, last_sent, 1, 12));
        assert(!shouldSendDebugVideoFrame(
            epoch + milliseconds(500), last_sent, 1, 12));
        assert(shouldSendDebugVideoFrame(
            epoch + milliseconds(1000), last_sent, 1, 12));
    }

    return 0;
}
