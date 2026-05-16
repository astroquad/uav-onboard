#include "mission/LineFollowMission.hpp"

#include <cassert>
#include <chrono>

int main()
{
    onboard::mission::LineFollowMissionConfig config;
    config.target_altitude_m = 1.0;
    config.altitude_reached_ratio = 0.8;
    config.line_follow_duration_s = 30.0;
    config.line_lost_timeout_s = 3.0;
    config.marker_approach_timeout_s = 5.0;
    config.marker_hover_s = 3.0;
    config.marker_lost_timeout_s = 2.0;

    onboard::mission::LineFollowMission mission(config);
    const auto started = std::chrono::steady_clock::now();
    const auto input =
        [started](
            double altitude_m,
            bool line_detected,
            bool marker_detected,
            bool marker_centered,
            int seconds) {
            return onboard::mission::LineFollowMissionInput {
                true,
                altitude_m,
                line_detected,
                marker_detected,
                marker_centered,
                false,
                false,
                started + std::chrono::seconds(seconds),
            };
        };
    mission.startTakeoff(started);

    assert(mission.update(input(0.9, true, false, false, 1)) ==
           onboard::mission::LineFollowMissionState::LineFollow);

    assert(mission.update(input(1.0, true, true, false, 2)) ==
           onboard::mission::LineFollowMissionState::MarkerApproach);

    assert(mission.update(input(1.0, true, true, true, 3)) ==
           onboard::mission::LineFollowMissionState::MarkerHover);

    assert(mission.update(input(1.0, true, true, true, 5)) ==
           onboard::mission::LineFollowMissionState::MarkerHover);

    assert(mission.update(input(1.0, true, true, true, 7)) ==
           onboard::mission::LineFollowMissionState::Land);
    assert(mission.landingReason() == "marker hover complete");

    onboard::mission::LineFollowMission timeout_mission(config);
    timeout_mission.startTakeoff(started);
    assert(timeout_mission.update(input(0.9, true, false, false, 1)) ==
           onboard::mission::LineFollowMissionState::LineFollow);
    assert(timeout_mission.update(input(1.0, true, true, false, 2)) ==
           onboard::mission::LineFollowMissionState::MarkerApproach);
    assert(timeout_mission.update(input(1.0, true, false, false, 6)) ==
           onboard::mission::LineFollowMissionState::MarkerApproach);
    assert(timeout_mission.update(input(1.0, false, false, false, 8)) ==
           onboard::mission::LineFollowMissionState::Land);
    assert(timeout_mission.landingReason() == "marker approach timeout");

    onboard::mission::LineFollowMission line_end_mission(config);
    line_end_mission.startTakeoff(started);
    assert(line_end_mission.update(input(0.9, true, false, false, 1)) ==
           onboard::mission::LineFollowMissionState::LineFollow);
    assert(line_end_mission.update(input(1.0, false, false, false, 2)) ==
           onboard::mission::LineFollowMissionState::LineFollow);
    assert(line_end_mission.update(input(1.0, false, false, false, 3)) ==
           onboard::mission::LineFollowMissionState::LineFollow);
    assert(line_end_mission.update(input(1.0, false, false, false, 4)) ==
           onboard::mission::LineFollowMissionState::Land);
    assert(line_end_mission.landingReason() == "line end");

    onboard::mission::LineFollowMission reacquire_mission(config);
    reacquire_mission.startTakeoff(started);
    assert(reacquire_mission.update(input(0.9, true, false, false, 1)) ==
           onboard::mission::LineFollowMissionState::LineFollow);
    assert(reacquire_mission.update(input(1.0, false, false, false, 3)) ==
           onboard::mission::LineFollowMissionState::LineFollow);
    assert(reacquire_mission.update(input(1.0, true, false, false, 4)) ==
           onboard::mission::LineFollowMissionState::LineFollow);
    assert(reacquire_mission.update(input(1.0, false, false, false, 6)) ==
           onboard::mission::LineFollowMissionState::LineFollow);

    return 0;
}
