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
    bool invert_marker_x = false;
    bool invert_marker_y = false;
};

class GuidedVelocityController {
public:
    explicit GuidedVelocityController(GuidedVelocityControllerConfig config);

    ControlSetpoint updateLine(
        const LineControlInput& line,
        const AltitudeControlInput& altitude) const;
    ControlSetpoint updateMarker(
        const MarkerControlInput& marker,
        const AltitudeControlInput& altitude) const;
    ControlSetpoint holdAltitude(const AltitudeControlInput& altitude) const;
    ControlSetpoint update(const LineControlInput& input) const;
    ControlSetpoint stop(const AltitudeControlInput& altitude = {}) const;

private:
    double altitudeVelocityDownMps(const AltitudeControlInput& altitude) const;

    GuidedVelocityControllerConfig config_;
};

} // namespace onboard::control
