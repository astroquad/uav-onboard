#pragma once

namespace onboard::control {

struct ControlSetpoint {
    float vx_forward_mps = 0.0f;
    float vy_right_mps = 0.0f;
    float vz_down_mps = 0.0f;
    float yaw_rate_rad_s = 0.0f;
};

struct AltitudeControlInput {
    bool altitude_available = false;
    double current_altitude_m = 0.0;
    double target_altitude_m = 2.0;
};

struct LineControlInput {
    bool line_detected = false;
    double center_error_norm = 0.0;
    double angle_error_rad = 0.0;
    double confidence = 0.0;
};

struct MarkerControlInput {
    bool marker_detected = false;
    double center_error_x_norm = 0.0;
    double center_error_y_norm = 0.0;
};

} // namespace onboard::control
