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
    int autopilot_heartbeat_lost_ms = 2000;
    int mission_timeout_ms = 300000;
    bool rc_required = false;
    bool assume_rc_present = true;
    int rc_lost_ms = 1500;
    // Vision pipeline stall watchdog (enforced by the mission loop, not by
    // SafetyMonitor::update which only runs on fresh frames). If no new
    // processed frame arrives for vision_stale_hold_ms the loop stops
    // re-sending the last velocity command and holds position (zero
    // velocity); past vision_stale_land_ms it commands LAND. Without this a
    // camera stall would freeze the mission state machine while the last
    // non-zero velocity command kept being re-sent — a flyaway.
    int vision_stale_hold_ms = 800;
    int vision_stale_land_ms = 4000;
    // Preflight EKF readiness gate. ArduCopter accepts GUIDED only when the EKF
    // horizontal-position and velocity innovation ratios are below ~1.0, so the
    // onboard gate enforces the same bound to predict GUIDED acceptance rather
    // than passing on flag bits alone. Only enforced when the FC reports them.
    double ekf_pos_horiz_variance_max = 1.0;
    double ekf_velocity_variance_max = 1.0;
};

struct SafetyInput {
    bool heartbeat_seen = false;
    bool line_detected = true;
    std::chrono::steady_clock::time_point now;
    std::chrono::steady_clock::time_point last_heartbeat_time {};
    bool rc_channels_seen = false;
    int rc_channel_count = 0;
    std::chrono::steady_clock::time_point last_rc_channels_time {};
    bool mode_known = true;
    bool mode_guided = true;
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
