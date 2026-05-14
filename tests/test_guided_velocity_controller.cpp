#include "control/GuidedVelocityController.hpp"

#include <cassert>
#include <cmath>

int main()
{
    onboard::control::GuidedVelocityControllerConfig config;
    config.forward_mps = 0.25;
    config.offset_kp = 0.8;
    config.angle_yaw_kp = 1.2;
    config.offset_yaw_kp = 0.0;
    config.max_lateral_mps = 0.5;
    config.max_yaw_rate_rad_s = 0.8;
    config.min_confidence = 0.2;
    config.offset_deadband_norm = 0.0;
    config.angle_deadband_rad = 0.0;
    config.altitude_kp = 0.4;
    config.max_vz_down_mps = 0.35;
    config.altitude_deadband_m = 0.08;

    const onboard::control::GuidedVelocityController controller(config);
    const onboard::control::AltitudeControlInput altitude {
        true,
        2.0,
        2.0,
    };

    const auto centered = controller.updateLine({true, 0.0, 0.0, 1.0}, altitude);
    assert(std::abs(centered.vx_forward_mps - 0.25f) < 0.0001f);
    assert(centered.vy_right_mps == 0.0f);
    assert(centered.vz_down_mps == 0.0f);
    assert(centered.yaw_rate_rad_s == 0.0f);

    const auto right = controller.updateLine({true, 0.25, 0.5, 1.0}, altitude);
    assert(right.vy_right_mps > 0.0f);
    assert(right.yaw_rate_rad_s > 0.0f);

    const auto left = controller.updateLine({true, -0.25, -0.5, 1.0}, altitude);
    assert(left.vy_right_mps < 0.0f);
    assert(left.yaw_rate_rad_s < 0.0f);

    const auto clamped = controller.updateLine({true, 10.0, 10.0, 1.0}, altitude);
    assert(std::abs(clamped.vy_right_mps - 0.5f) < 0.0001f);
    assert(std::abs(clamped.yaw_rate_rad_s - 0.8f) < 0.0001f);

    const auto lost = controller.updateLine({false, 1.0, 1.0, 0.0}, altitude);
    assert(lost.vx_forward_mps == 0.0f);
    assert(lost.vy_right_mps == 0.0f);
    assert(lost.yaw_rate_rad_s == 0.0f);

    const auto climb = controller.holdAltitude({true, 1.5, 2.0});
    assert(climb.vz_down_mps < 0.0f);

    const auto descend = controller.holdAltitude({true, 2.5, 2.0});
    assert(descend.vz_down_mps > 0.0f);

    const auto altitude_deadband = controller.holdAltitude({true, 2.02, 2.0});
    assert(altitude_deadband.vz_down_mps == 0.0f);

    const auto low_confidence = controller.updateLine({true, 0.5, 0.5, 0.1}, altitude);
    assert(low_confidence.vx_forward_mps == 0.0f);
    assert(low_confidence.vy_right_mps == 0.0f);
    assert(low_confidence.yaw_rate_rad_s == 0.0f);

    return 0;
}
