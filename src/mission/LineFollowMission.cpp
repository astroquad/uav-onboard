#include "mission/LineFollowMission.hpp"

#include <utility>

namespace onboard::mission {

LineFollowMission::LineFollowMission(LineFollowMissionConfig config)
    : config_(config)
{
}

void LineFollowMission::startTakeoff(std::chrono::steady_clock::time_point now)
{
    transition(LineFollowMissionState::Takeoff, now);
}

LineFollowMissionState LineFollowMission::update(const LineFollowMissionInput& input)
{
    if (input.abort_requested) {
        abort("abort requested");
        return state_;
    }

    if (state_ == LineFollowMissionState::Takeoff &&
        input.altitude_available &&
        input.altitude_m >= config_.target_altitude_m * config_.altitude_reached_ratio) {
        transition(LineFollowMissionState::LineFollow, input.now);
    }

    if (state_ == LineFollowMissionState::LineFollow) {
        const auto elapsed = std::chrono::duration<double>(input.now - state_entered_).count();
        if (input.land_requested) {
            landing_reason_ = "safety land";
            transition(LineFollowMissionState::Land, input.now);
        } else if (elapsed >= config_.line_follow_duration_s) {
            landing_reason_ = "duration complete";
            transition(LineFollowMissionState::Land, input.now);
        }
    }

    return state_;
}

void LineFollowMission::markComplete()
{
    state_ = LineFollowMissionState::Complete;
}

void LineFollowMission::abort(std::string reason)
{
    landing_reason_ = std::move(reason);
    state_ = LineFollowMissionState::Abort;
}

void LineFollowMission::transition(
    LineFollowMissionState state,
    std::chrono::steady_clock::time_point now)
{
    state_ = state;
    state_entered_ = now;
}

const char* toString(LineFollowMissionState state)
{
    switch (state) {
    case LineFollowMissionState::Idle:
        return "IDLE";
    case LineFollowMissionState::Takeoff:
        return "TAKEOFF";
    case LineFollowMissionState::LineFollow:
        return "LINE_FOLLOW";
    case LineFollowMissionState::Land:
        return "LAND";
    case LineFollowMissionState::Complete:
        return "COMPLETE";
    case LineFollowMissionState::Abort:
        return "ABORT";
    }
    return "UNKNOWN";
}

} // namespace onboard::mission
