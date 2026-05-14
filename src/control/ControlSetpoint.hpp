#pragma once

namespace onboard::control {

struct ControlSetpoint {
    float vx_forward_mps = 0.0f;
    float vy_right_mps = 0.0f;
    float vz_down_mps = 0.0f;
    float yaw_rate_rad_s = 0.0f;
};

struct LineControlInput {
    bool line_detected = false;
    double center_error_m = 0.0;
    double line_angle_rad = 0.0;
    double confidence = 0.0;
};

} // namespace onboard::control
