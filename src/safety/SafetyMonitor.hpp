#pragma once

#include <chrono>
#include <string>

namespace onboard::safety {

enum class SafetyAction {
    None,
    Land,
    Abort,
};

struct SafetyConfig {
    int line_lost_ms = 2000;
    int pixhawk_heartbeat_lost_ms = 2000;
    int mission_timeout_ms = 300000;
};

struct SafetyInput {
    bool heartbeat_seen = false;
    bool line_detected = true;
    std::chrono::steady_clock::time_point now;
    std::chrono::steady_clock::time_point last_heartbeat_time {};
};

struct SafetyDecision {
    SafetyAction action = SafetyAction::None;
    std::string reason;
};

class SafetyMonitor {
public:
    explicit SafetyMonitor(SafetyConfig config);

    void startMission(std::chrono::steady_clock::time_point now);
    SafetyDecision update(const SafetyInput& input);

private:
    SafetyConfig config_;
    bool mission_started_ = false;
    std::chrono::steady_clock::time_point mission_started_at_ {};
    std::chrono::steady_clock::time_point last_line_seen_at_ {};
    bool line_seen_ = false;
};

} // namespace onboard::safety
