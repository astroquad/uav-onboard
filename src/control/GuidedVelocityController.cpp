#include "control/GuidedVelocityController.hpp"

#include <algorithm>

namespace onboard::control {

GuidedVelocityController::GuidedVelocityController(GuidedVelocityControllerConfig config)
    : config_(config)
{
}

ControlSetpoint GuidedVelocityController::update(const LineControlInput& input) const
{
    if (!input.line_detected) {
        return {};
    }

    const double lateral = std::clamp(
        config_.lateral_kp * input.center_error_m,
        -config_.max_lateral_mps,
        config_.max_lateral_mps);
    const double yaw_rate = std::clamp(
        config_.yaw_kp * input.line_angle_rad,
        -config_.max_yaw_rate_rad_s,
        config_.max_yaw_rate_rad_s);

    return ControlSetpoint {
        static_cast<float>(config_.forward_mps),
        static_cast<float>(lateral),
        0.0f,
        static_cast<float>(yaw_rate),
    };
}

} // namespace onboard::control
