#pragma once

#include <chrono>
#include <string>

namespace onboard::mission {

enum class LineFollowMissionState {
    Idle,
    Takeoff,
    LineFollow,
    MarkerHover,
    Land,
    Complete,
    Abort,
};

struct LineFollowMissionConfig {
    double target_altitude_m = 1.2;
    double altitude_reached_ratio = 0.85;
    double line_follow_duration_s = 3.0;
    double marker_hover_s = 3.0;
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
    std::string landing_reason_;
};

const char* toString(LineFollowMissionState state);

} // namespace onboard::mission
