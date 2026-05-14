#include "control/GuidedVelocityController.hpp"

#include <algorithm>
#include <cmath>

namespace onboard::control {
namespace {

double applyDeadband(double value, double deadband)
{
    return std::abs(value) < std::max(0.0, deadband) ? 0.0 : value;
}

} // namespace

GuidedVelocityController::GuidedVelocityController(GuidedVelocityControllerConfig config)
    : config_(config)
{
}

ControlSetpoint GuidedVelocityController::updateLine(
    const LineControlInput& line,
    const AltitudeControlInput& altitude) const
{
    ControlSetpoint output = holdAltitude(altitude);
    if (!line.line_detected || line.confidence < config_.min_confidence) {
        return output;
    }

    const double offset_error =
        applyDeadband(line.center_error_norm, config_.offset_deadband_norm);
    const double angle_error =
        applyDeadband(line.angle_error_rad, config_.angle_deadband_rad);

    double lateral = std::clamp(
        config_.offset_kp * offset_error,
        -config_.max_lateral_mps,
        config_.max_lateral_mps);
    double yaw_rate = std::clamp(
        config_.angle_yaw_kp * angle_error + config_.offset_yaw_kp * offset_error,
        -config_.max_yaw_rate_rad_s,
        config_.max_yaw_rate_rad_s);
    if (config_.invert_lateral) {
        lateral = -lateral;
    }
    if (config_.invert_yaw) {
        yaw_rate = -yaw_rate;
    }

    output.vx_forward_mps = static_cast<float>(config_.forward_mps);
    output.vy_right_mps = static_cast<float>(lateral);
    output.yaw_rate_rad_s = static_cast<float>(yaw_rate);
    return output;
}

ControlSetpoint GuidedVelocityController::updateMarker(
    const MarkerControlInput& marker,
    const AltitudeControlInput& altitude) const
{
    ControlSetpoint output = holdAltitude(altitude);
    if (!marker.marker_detected) {
        return output;
    }

    double lateral = std::clamp(
        config_.marker_x_kp * marker.center_error_x_norm,
        -config_.max_marker_mps,
        config_.max_marker_mps);
    double forward = std::clamp(
        config_.marker_y_kp * marker.center_error_y_norm,
        -config_.max_marker_mps,
        config_.max_marker_mps);
    if (config_.invert_marker_x) {
        lateral = -lateral;
    }
    if (config_.invert_marker_y) {
        forward = -forward;
    }

    output.vx_forward_mps = static_cast<float>(forward);
    output.vy_right_mps = static_cast<float>(lateral);
    output.yaw_rate_rad_s = 0.0f;
    return output;
}

ControlSetpoint GuidedVelocityController::holdAltitude(const AltitudeControlInput& altitude) const
{
    return ControlSetpoint {
        0.0f,
        0.0f,
        static_cast<float>(altitudeVelocityDownMps(altitude)),
        0.0f,
    };
}

ControlSetpoint GuidedVelocityController::update(const LineControlInput& input) const
{
    return updateLine(input, AltitudeControlInput {});
}

ControlSetpoint GuidedVelocityController::stop(const AltitudeControlInput& altitude) const
{
    return holdAltitude(altitude);
}

double GuidedVelocityController::altitudeVelocityDownMps(const AltitudeControlInput& altitude) const
{
    if (!altitude.altitude_available) {
        return 0.0;
    }
    const double error_m = altitude.target_altitude_m - altitude.current_altitude_m;
    if (std::abs(error_m) < config_.altitude_deadband_m) {
        return 0.0;
    }
    return -std::clamp(
        config_.altitude_kp * error_m,
        -config_.max_vz_down_mps,
        config_.max_vz_down_mps);
}

} // namespace onboard::control
