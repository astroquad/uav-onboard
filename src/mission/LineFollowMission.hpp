#pragma once

#include <chrono>
#include <string>

namespace onboard::mission {

enum class LineFollowMissionState {
    Idle,
    Takeoff,
    LineFollow,
    MarkerApproach,
    MarkerHover,
    Land,
    Complete,
    Abort,
};

struct LineFollowMissionConfig {
    double target_altitude_m = 2.0;
    double altitude_reached_ratio = 0.9;
    double takeoff_settle_s = 2.0;
    double line_follow_duration_s = 60.0;
    double line_lost_timeout_s = 3.0;
    double marker_approach_timeout_s = 5.0;
    double marker_hover_s = 3.0;
    double marker_lost_timeout_s = 5.0;
    double marker_hover_recenter_timeout_s = 1.25;
};

struct LineFollowMissionInput {
    bool altitude_available = false;
    double altitude_m = 0.0;
    bool line_detected = true;
    bool marker_detected = false;
    bool marker_centered = false;
    bool land_requested = false;
    bool abort_requested = false;
    std::chrono::steady_clock::time_point now;
};

class LineFollowMission {
public:
    explicit LineFollowMission(LineFollowMissionConfig config);

    LineFollowMissionState state() const { return state_; }
    const std::string& landingReason() const { return landing_reason_; }

    void startTakeoff(std::chrono::steady_clock::time_point now);
    LineFollowMissionState update(const LineFollowMissionInput& input);
    void markComplete();
    void abort(std::string reason);

private:
    void transition(LineFollowMissionState state, std::chrono::steady_clock::time_point now);

    LineFollowMissionConfig config_;
    LineFollowMissionState state_ = LineFollowMissionState::Idle;
    std::chrono::steady_clock::time_point state_entered_ {};
    std::chrono::steady_clock::time_point last_line_seen_at_ {};
    std::chrono::steady_clock::time_point last_marker_seen_at_ {};
    std::chrono::steady_clock::time_point last_marker_centered_at_ {};
    std::string landing_reason_;
};

const char* toString(LineFollowMissionState state);

} // namespace onboard::mission
