#include "safety/SafetyMonitor.hpp"

namespace onboard::safety {

SafetyMonitor::SafetyMonitor(SafetyConfig config)
    : config_(config)
{
}

void SafetyMonitor::startMission(std::chrono::steady_clock::time_point now)
{
    mission_started_ = true;
    mission_started_at_ = now;
    last_line_seen_at_ = now;
    line_seen_ = true;
}

SafetyDecision SafetyMonitor::update(const SafetyInput& input)
{
    if (!mission_started_) {
        startMission(input.now);
    }

    if (!input.heartbeat_seen) {
        return {SafetyAction::Abort, "pixhawk heartbeat not seen"};
    }

    const auto heartbeat_age_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            input.now - input.last_heartbeat_time)
            .count();
    if (heartbeat_age_ms > config_.pixhawk_heartbeat_lost_ms) {
        return {SafetyAction::Abort, "pixhawk heartbeat lost"};
    }

    if (input.line_detected) {
        last_line_seen_at_ = input.now;
        line_seen_ = true;
    } else if (line_seen_) {
        const auto line_lost_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                input.now - last_line_seen_at_)
                .count();
        if (line_lost_ms > config_.line_lost_ms) {
            return {SafetyAction::Land, "line lost timeout"};
        }
    }

    const auto mission_age_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            input.now - mission_started_at_)
            .count();
    if (mission_age_ms > config_.mission_timeout_ms) {
        return {SafetyAction::Land, "mission timeout"};
    }

    return {};
}

} // namespace onboard::safety
