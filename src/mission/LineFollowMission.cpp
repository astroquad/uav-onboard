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
    last_line_seen_at_ = now;
}

LineFollowMissionState LineFollowMission::update(const LineFollowMissionInput& input)
{
    if (input.abort_requested) {
        abort("abort requested");
        return state_;
    }

    if (input.marker_detected) {
        last_marker_seen_at_ = input.now;
    }

    if (state_ == LineFollowMissionState::Takeoff &&
        input.altitude_available &&
        input.altitude_m >= config_.target_altitude_m * config_.altitude_reached_ratio) {
        transition(LineFollowMissionState::LineFollow, input.now);
    }

    if (state_ == LineFollowMissionState::LineFollow) {
        const auto elapsed = std::chrono::duration<double>(input.now - state_entered_).count();
        if (input.line_detected) {
            last_line_seen_at_ = input.now;
        }
        const auto line_lost_elapsed =
            std::chrono::duration<double>(input.now - last_line_seen_at_).count();
        if (input.land_requested) {
            landing_reason_ = "safety land";
            transition(LineFollowMissionState::Land, input.now);
        } else if (input.marker_detected) {
            transition(LineFollowMissionState::MarkerApproach, input.now);
        } else if (!input.line_detected &&
                   line_lost_elapsed >= config_.line_lost_timeout_s) {
            landing_reason_ = "line end";
            transition(LineFollowMissionState::Land, input.now);
        } else if (elapsed >= config_.line_follow_duration_s) {
            landing_reason_ = "duration complete";
            transition(LineFollowMissionState::Land, input.now);
        }
    }

    if (state_ == LineFollowMissionState::MarkerApproach) {
        const auto elapsed = std::chrono::duration<double>(input.now - state_entered_).count();
        const auto marker_lost_elapsed =
            std::chrono::duration<double>(input.now - last_marker_seen_at_).count();
        if (input.land_requested) {
            landing_reason_ = "safety land";
            transition(LineFollowMissionState::Land, input.now);
        } else if (input.marker_centered) {
            transition(LineFollowMissionState::MarkerHover, input.now);
        } else if (elapsed >= config_.marker_approach_timeout_s &&
                   !input.marker_detected &&
                   !input.line_detected) {
            landing_reason_ = "marker approach timeout";
            transition(LineFollowMissionState::Land, input.now);
        } else if (!input.marker_detected &&
                   !input.line_detected &&
                   marker_lost_elapsed >= config_.marker_lost_timeout_s) {
            landing_reason_ = "marker and line lost";
            transition(LineFollowMissionState::Land, input.now);
        }
    }

    if (state_ == LineFollowMissionState::MarkerHover) {
        const auto elapsed = std::chrono::duration<double>(input.now - state_entered_).count();
        const auto marker_lost_elapsed =
            std::chrono::duration<double>(input.now - last_marker_seen_at_).count();
        if (input.land_requested) {
            landing_reason_ = "safety land";
            transition(LineFollowMissionState::Land, input.now);
        } else if (!input.marker_detected &&
                   input.line_detected) {
            transition(LineFollowMissionState::MarkerApproach, input.now);
        } else if (!input.marker_detected &&
                   !input.line_detected &&
                   marker_lost_elapsed >= config_.marker_lost_timeout_s) {
            landing_reason_ = "marker and line lost";
            transition(LineFollowMissionState::Land, input.now);
        } else if (input.marker_detected && !input.marker_centered) {
            transition(LineFollowMissionState::MarkerApproach, input.now);
        } else if (elapsed >= config_.marker_hover_s) {
            landing_reason_ = "marker hover complete";
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
    if (state == LineFollowMissionState::LineFollow) {
        last_line_seen_at_ = now;
    }
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
    case LineFollowMissionState::MarkerApproach:
        return "MARKER_APPROACH";
    case LineFollowMissionState::MarkerHover:
        return "MARKER_HOVER";
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
