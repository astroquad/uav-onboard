#pragma once

#include "control/ControlSetpoint.hpp"

namespace onboard::control {

struct GuidedVelocityControllerConfig {
    double forward_mps = 0.25;
    double offset_kp = 0.35;
    double angle_yaw_kp = 1.0;
    double offset_yaw_kp = 0.25;
    double max_lateral_mps = 0.35;
    double max_yaw_rate_rad_s = 0.6;
    double min_confidence = 0.35;
    double offset_deadband_norm = 0.03;
    double angle_deadband_rad = 0.05235987755982989;
    bool invert_lateral = false;
    bool invert_yaw = false;

    double altitude_kp = 0.4;
    double max_vz_down_mps = 0.35;
    double altitude_deadband_m = 0.08;

    double marker_x_kp = 0.25;
    double marker_y_kp = 0.25;
    double max_marker_mps = 0.25;
    double marker_deadband_norm = 0.03;
    double marker_output_ema_alpha = 1.0;
    double max_marker_rate_mps = 0.0;
    bool invert_marker_x = false;
    bool invert_marker_y = false;

    // Output smoothing: EMA alpha for lateral and yaw commands.
    // Lower values = smoother output but more lag.
    // 1.0 = no smoothing (legacy behavior, default).
    double output_ema_alpha = 1.0;

    // Rate limiting: maximum absolute change in lateral / yaw per update step.
    // Caps acceleration in body-frame; ignored when <= 0.
    double max_lateral_rate_mps = 0.0;
    double max_yaw_rate_change_rad_s = 0.0;

    // Scale forward velocity by detection confidence: reduces speed when
    // the line confidence is weak. 0.0 disables (constant forward_mps);
    // values > 0 use std::pow(confidence, forward_confidence_scale).
    double forward_confidence_scale = 0.0;
};

class GuidedVelocityController {
public:
    explicit GuidedVelocityController(GuidedVelocityControllerConfig config);

    ControlSetpoint updateLine(
        const LineControlInput& line,
        const AltitudeControlInput& altitude);
    ControlSetpoint updateMarker(
        const MarkerControlInput& marker,
        const AltitudeControlInput& altitude);
    ControlSetpoint holdAltitude(const AltitudeControlInput& altitude) const;
    ControlSetpoint update(const LineControlInput& input);
    ControlSetpoint stop(const AltitudeControlInput& altitude = {});

    // Reset smoothing state (e.g. on mode transition)
    void resetSmoothing();

private:
    double altitudeVelocityDownMps(const AltitudeControlInput& altitude) const;

    // Apply EMA smoothing and rate limiting to lateral and yaw outputs
    void smoothOutput(double& lateral, double& yaw_rate);
    void smoothMarkerOutput(double& forward, double& lateral);

    GuidedVelocityControllerConfig config_;

    // Smoothing state
    bool has_prev_output_ = false;
    double prev_lateral_ = 0.0;
    double prev_yaw_rate_ = 0.0;
    bool has_prev_marker_output_ = false;
    double prev_marker_forward_ = 0.0;
    double prev_marker_lateral_ = 0.0;
};

} // namespace onboard::control
