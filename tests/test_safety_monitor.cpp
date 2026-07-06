// Tests are assert-based: keep assert() active even in Release
// builds (CMake adds -DNDEBUG there, which silently no-ops all checks).
#undef NDEBUG

#include "safety/SafetyMonitor.hpp"

#include <cassert>
#include <chrono>
#include <string>

namespace {

using Clock = std::chrono::steady_clock;

onboard::safety::SafetyInput makeInput(Clock::time_point now)
{
    onboard::safety::SafetyInput input;
    input.heartbeat_seen = true;
    input.line_detected = true;
    input.now = now;
    input.last_heartbeat_time = now;
    input.mode_known = true;
    input.mode_guided = true;
    return input;
}

} // namespace

int main()
{
    const auto now = Clock::now();

    onboard::safety::SafetyConfig sitl_config;
    sitl_config.assume_rc_present = true;
    sitl_config.rc_required = false;
    onboard::safety::SafetyMonitor sitl_monitor(sitl_config);
    assert(sitl_monitor.update(makeInput(now)).action == onboard::safety::SafetyAction::None);

    onboard::safety::SafetyConfig strict_config;
    strict_config.assume_rc_present = false;
    strict_config.rc_required = true;
    strict_config.rc_lost_ms = 1000;

    onboard::safety::SafetyMonitor missing_rc_monitor(strict_config);
    const auto missing_rc_decision = missing_rc_monitor.update(makeInput(now));
    assert(missing_rc_decision.action == onboard::safety::SafetyAction::Land);
    assert(missing_rc_decision.reason == "rc input unavailable");

    auto fresh_rc_input = makeInput(now);
    fresh_rc_input.rc_channels_seen = true;
    fresh_rc_input.rc_channel_count = 8;
    fresh_rc_input.last_rc_channels_time = now;
    onboard::safety::SafetyMonitor fresh_rc_monitor(strict_config);
    assert(fresh_rc_monitor.update(fresh_rc_input).action == onboard::safety::SafetyAction::None);

    auto stale_rc_input = makeInput(now);
    stale_rc_input.rc_channels_seen = true;
    stale_rc_input.rc_channel_count = 8;
    stale_rc_input.last_rc_channels_time = now - std::chrono::milliseconds(1500);
    onboard::safety::SafetyMonitor stale_rc_monitor(strict_config);
    const auto stale_rc_decision = stale_rc_monitor.update(stale_rc_input);
    assert(stale_rc_decision.action == onboard::safety::SafetyAction::Land);
    assert(stale_rc_decision.reason == "rc input lost");

    auto takeover_input = fresh_rc_input;
    takeover_input.mode_guided = false;
    onboard::safety::SafetyMonitor takeover_monitor(strict_config);
    const auto takeover_decision = takeover_monitor.update(takeover_input);
    assert(takeover_decision.action == onboard::safety::SafetyAction::Abort);
    assert(takeover_decision.reason == "operator takeover: mode changed from GUIDED");

    return 0;
}
