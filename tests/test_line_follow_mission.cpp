#include "mission/LineFollowMission.hpp"

#include <cassert>
#include <chrono>

int main()
{
    onboard::mission::LineFollowMissionConfig config;
    config.target_altitude_m = 1.0;
    config.altitude_reached_ratio = 0.8;
    config.line_follow_duration_s = 30.0;
    config.marker_hover_s = 3.0;

    onboard::mission::LineFollowMission mission(config);
    const auto started = std::chrono::steady_clock::now();
    mission.startTakeoff(started);

    assert(mission.update(onboard::mission::LineFollowMissionInput {
        true,
        0.9,
        true,
        false,
        false,
        false,
        false,
        started + std::chrono::seconds(1),
    }) == onboard::mission::LineFollowMissionState::LineFollow);

    assert(mission.update(onboard::mission::LineFollowMissionInput {
        true,
        1.0,
        true,
        true,
        true,
        false,
        false,
        started + std::chrono::seconds(2),
    }) == onboard::mission::LineFollowMissionState::MarkerHover);

    assert(mission.update(onboard::mission::LineFollowMissionInput {
        true,
        1.0,
        true,
        true,
        true,
        false,
        false,
        started + std::chrono::seconds(4),
    }) == onboard::mission::LineFollowMissionState::MarkerHover);

    assert(mission.update(onboard::mission::LineFollowMissionInput {
        true,
        1.0,
        true,
        true,
        true,
        false,
        false,
        started + std::chrono::seconds(6),
    }) == onboard::mission::LineFollowMissionState::Land);
    assert(mission.landingReason() == "marker hover complete");

    return 0;
}
