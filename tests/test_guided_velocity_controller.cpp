#include "control/GuidedVelocityController.hpp"

#include <cassert>
#include <cmath>

namespace {

onboard::control::GuidedVelocityControllerConfig baseConfig()
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
    // Smoothing off by default so the P-gain math is asserted directly.
    config.output_ema_alpha = 1.0;
    config.max_lateral_rate_mps = 0.0;
    config.max_yaw_rate_change_rad_s = 0.0;
    config.forward_confidence_scale = 0.0;
    return config;
}

void runLegacyControlMath()
{
    onboard::control::GuidedVelocityController controller(baseConfig());
    const onboard::control::AltitudeControlInput altitude {true, 2.0, 2.0};

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
}

void runOutputSmoothing()
{
    auto config = baseConfig();
    config.output_ema_alpha = 0.4;
    onboard::control::GuidedVelocityController controller(config);
    const onboard::control::AltitudeControlInput altitude {true, 2.0, 2.0};

    // First sample seeds the filter and passes through unchanged.
    const auto first = controller.updateLine({true, 0.25, 0.5, 1.0}, altitude);
    const float first_lat = first.vy_right_mps;
    const float first_yaw = first.yaw_rate_rad_s;
    assert(first_lat > 0.0f);
    assert(first_yaw > 0.0f);

    // Sign flip in input must be damped: output should move toward the new
    // input but lag, so its magnitude is smaller than the raw P-gain.
    const auto flipped = controller.updateLine({true, -0.25, -0.5, 1.0}, altitude);
    assert(flipped.vy_right_mps < first_lat);
    assert(flipped.yaw_rate_rad_s < first_yaw);
    assert(std::abs(flipped.vy_right_mps) < 0.8f * std::abs(-0.25 * config.offset_kp));
    assert(std::abs(flipped.yaw_rate_rad_s) < 0.8f * std::abs(-0.5 * config.angle_yaw_kp));
}

void runRateLimit()
{
    auto config = baseConfig();
    config.output_ema_alpha = 1.0;
    config.max_lateral_rate_mps = 0.02;
    config.max_yaw_rate_change_rad_s = 0.05;
    onboard::control::GuidedVelocityController controller(config);
    const onboard::control::AltitudeControlInput altitude {true, 2.0, 2.0};

    // First sample seeds the filter to the input.
    controller.updateLine({true, 0.0, 0.0, 1.0}, altitude);
    // Demand a saturating step; output must move by at most max_*_rate per step.
    const auto step = controller.updateLine({true, 1.0, 1.0, 1.0}, altitude);
    assert(std::abs(step.vy_right_mps) <= static_cast<float>(config.max_lateral_rate_mps) + 1e-6f);
    assert(std::abs(step.yaw_rate_rad_s) <= static_cast<float>(config.max_yaw_rate_change_rad_s) + 1e-6f);
}

void runForwardConfidenceScale()
{
    auto config = baseConfig();
    config.forward_confidence_scale = 1.0;
    onboard::control::GuidedVelocityController controller(config);
    const onboard::control::AltitudeControlInput altitude {true, 2.0, 2.0};

    const auto strong = controller.updateLine({true, 0.0, 0.0, 1.0}, altitude);
    assert(std::abs(strong.vx_forward_mps - 0.25f) < 0.0001f);

    const auto weak = controller.updateLine({true, 0.0, 0.0, 0.5}, altitude);
    assert(weak.vx_forward_mps > 0.0f);
    assert(weak.vx_forward_mps < strong.vx_forward_mps);
}

void runLostLineDecays()
{
    auto config = baseConfig();
    config.output_ema_alpha = 0.4;
    onboard::control::GuidedVelocityController controller(config);
    const onboard::control::AltitudeControlInput altitude {true, 2.0, 2.0};

    // Drive the smoother into a non-zero state.
    controller.updateLine({true, 0.25, 0.5, 1.0}, altitude);
    // Lose the line: output zeros and smoother also decays toward zero.
    const auto lost = controller.updateLine({false, 0.0, 0.0, 0.0}, altitude);
    assert(lost.vx_forward_mps == 0.0f);
    assert(lost.vy_right_mps == 0.0f);
    assert(lost.yaw_rate_rad_s == 0.0f);

    // Re-acquiring should not produce a step from stale state.
    const auto reacquired = controller.updateLine({true, 0.25, 0.5, 1.0}, altitude);
    // First post-reset output is bounded by EMA-blending toward (decayed) state.
    assert(reacquired.vy_right_mps > 0.0f);
    assert(reacquired.yaw_rate_rad_s > 0.0f);
}

} // namespace

int main()
{
    runLegacyControlMath();
    runOutputSmoothing();
    runRateLimit();
    runForwardConfidenceScale();
    runLostLineDecays();
    return 0;
}
