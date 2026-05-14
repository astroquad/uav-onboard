#include "control/GuidedVelocityController.hpp"

#include <cassert>
#include <cmath>

int main()
{
    onboard::control::GuidedVelocityControllerConfig config;
    config.forward_mps = 0.3;
    config.lateral_kp = 0.8;
    config.yaw_kp = 1.2;
    config.max_lateral_mps = 0.5;
    config.max_yaw_rate_rad_s = 0.8;

    const onboard::control::GuidedVelocityController controller(config);

    const auto centered = controller.update({true, 0.0, 0.0, 1.0});
    assert(std::abs(centered.vx_forward_mps - 0.3f) < 0.0001f);
    assert(centered.vy_right_mps == 0.0f);
    assert(centered.yaw_rate_rad_s == 0.0f);

    const auto right = controller.update({true, 0.25, 0.5, 1.0});
    assert(right.vy_right_mps > 0.0f);
    assert(right.yaw_rate_rad_s > 0.0f);

    const auto left = controller.update({true, -0.25, -0.5, 1.0});
    assert(left.vy_right_mps < 0.0f);
    assert(left.yaw_rate_rad_s < 0.0f);

    const auto clamped = controller.update({true, 10.0, 10.0, 1.0});
    assert(std::abs(clamped.vy_right_mps - 0.5f) < 0.0001f);
    assert(std::abs(clamped.yaw_rate_rad_s - 0.8f) < 0.0001f);

    const auto lost = controller.update({false, 1.0, 1.0, 0.0});
    assert(lost.vx_forward_mps == 0.0f);
    assert(lost.vy_right_mps == 0.0f);
    assert(lost.yaw_rate_rad_s == 0.0f);

    return 0;
}
