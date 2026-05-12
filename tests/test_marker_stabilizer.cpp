#include "common/VisionConfig.hpp"
#include "vision/MarkerStabilizer.hpp"
#include "vision/VisionTypes.hpp"

#include <cassert>

namespace {

onboard::vision::MarkerObservation makeMarker(int id, float center_x)
{
    onboard::vision::MarkerObservation marker;
    marker.id = id;
    marker.center_px = {center_x, 240.0f};
    marker.corners_px = {{
        {center_x - 20.0f, 220.0f},
        {center_x + 20.0f, 220.0f},
        {center_x + 20.0f, 260.0f},
        {center_x - 20.0f, 260.0f},
    }};
    return marker;
}

} // namespace

int main()
{
    onboard::common::ArucoConfig config;
    config.hold_frames = 3;
    onboard::vision::MarkerStabilizer stabilizer(config);

    const auto fresh = stabilizer.update({makeMarker(4, 320.0f)});
    assert(fresh.size() == 1);
    assert(fresh.front().id == 4);
    assert(stabilizer.ageFrames() == 0);

    for (int frame = 1; frame <= 3; ++frame) {
        const auto held = stabilizer.update({});
        assert(held.size() == 1);
        assert(held.front().id == 4);
        assert(stabilizer.ageFrames() == frame);
    }

    const auto expired = stabilizer.update({});
    assert(expired.empty());

    const auto refreshed = stabilizer.update({makeMarker(7, 300.0f)});
    assert(refreshed.size() == 1);
    assert(refreshed.front().id == 7);
    assert(stabilizer.ageFrames() == 0);

    onboard::common::ArucoConfig no_hold_config;
    no_hold_config.hold_frames = 0;
    onboard::vision::MarkerStabilizer no_hold(no_hold_config);
    assert(no_hold.update({makeMarker(1, 320.0f)}).size() == 1);
    assert(no_hold.update({}).empty());

    return 0;
}
