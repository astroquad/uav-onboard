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

    // A stall must not bank a burst: after a 1 s gap at most two sends may
    // arrive back-to-back before normal pacing resumes.
    {
        Clock::time_point last_sent {};
        assert(shouldSendDebugVideoFrame(epoch, last_sent, 10, 12));
        const auto resume = epoch + milliseconds(1000);
        int consecutive = 0;
        for (int i = 0; i < 4; ++i) {
            const auto now = resume + microseconds(i * 1000000LL / 12);
            if (shouldSendDebugVideoFrame(now, last_sent, 10, 12)) {
                ++consecutive;
            } else {
                break;
            }
        }
        assert(consecutive <= 2);
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
