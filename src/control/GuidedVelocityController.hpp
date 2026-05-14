#pragma once

#include "control/ControlSetpoint.hpp"

namespace onboard::control {

struct GuidedVelocityControllerConfig {
    double forward_mps = 0.3;
    double lateral_kp = 0.8;
    double yaw_kp = 1.2;
    double max_lateral_mps = 0.5;
    double max_yaw_rate_rad_s = 0.8;
};

class GuidedVelocityController {
public:
    explicit GuidedVelocityController(GuidedVelocityControllerConfig config);

    ControlSetpoint update(const LineControlInput& input) const;
    ControlSetpoint stop() const { return {}; }

private:
    GuidedVelocityControllerConfig config_;
};

} // namespace onboard::control
