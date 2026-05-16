#include "control/GuidedVelocityController.hpp"

#include <algorithm>
#include <cmath>

namespace onboard::control {
namespace {

double applyDeadband(double value, double deadband)
{
    return std::abs(value) < std::max(0.0, deadband) ? 0.0 : value;
}

double rateLimitStep(double current, double target, double max_change)
{
    if (max_change <= 0.0) {
        return target;
    }
    const double delta = target - current;
    return current + std::clamp(delta, -max_change, max_change);
}

double emaFilter(double prev, double input, double alpha)
{
    return prev * (1.0 - alpha) + input * alpha;
}

} // namespace

GuidedVelocityController::GuidedVelocityController(GuidedVelocityControllerConfig config)
    : config_(config)
{
}

void GuidedVelocityController::resetSmoothing()
{
    has_prev_output_ = false;
    prev_lateral_ = 0.0;
    prev_yaw_rate_ = 0.0;
    has_prev_marker_output_ = false;
    prev_marker_forward_ = 0.0;
    prev_marker_lateral_ = 0.0;
}

void GuidedVelocityController::smoothOutput(double& lateral, double& yaw_rate)
{
    const double alpha = std::clamp(config_.output_ema_alpha, 0.05, 1.0);

    if (!has_prev_output_) {
        has_prev_output_ = true;
        prev_lateral_ = lateral;
        prev_yaw_rate_ = yaw_rate;
        return;
    }

    // EMA smoothing
    if (alpha < 1.0) {
        lateral = emaFilter(prev_lateral_, lateral, alpha);
        yaw_rate = emaFilter(prev_yaw_rate_, yaw_rate, alpha);
    }

    // Rate limiting
    lateral = rateLimitStep(prev_lateral_, lateral, config_.max_lateral_rate_mps);
    yaw_rate = rateLimitStep(prev_yaw_rate_, yaw_rate, config_.max_yaw_rate_change_rad_s);

    prev_lateral_ = lateral;
    prev_yaw_rate_ = yaw_rate;
}

void GuidedVelocityController::smoothMarkerOutput(double& forward, double& lateral)
{
    const double alpha = std::clamp(config_.marker_output_ema_alpha, 0.05, 1.0);

    if (!has_prev_marker_output_) {
        has_prev_marker_output_ = true;
        prev_marker_forward_ = 0.0;
        prev_marker_lateral_ = 0.0;
    }

    if (alpha < 1.0) {
        forward = emaFilter(prev_marker_forward_, forward, alpha);
        lateral = emaFilter(prev_marker_lateral_, lateral, alpha);
    }

    forward = rateLimitStep(prev_marker_forward_, forward, config_.max_marker_rate_mps);
    lateral = rateLimitStep(prev_marker_lateral_, lateral, config_.max_marker_rate_mps);

    prev_marker_forward_ = forward;
    prev_marker_lateral_ = lateral;
}

ControlSetpoint GuidedVelocityController::updateLine(
    const LineControlInput& line,
    const AltitudeControlInput& altitude)
{
    ControlSetpoint output = holdAltitude(altitude);
    if (!line.line_detected || line.confidence < config_.min_confidence) {
        // When line is lost, decay smoothing state toward zero so the
        // drone doesn't lurch when re-acquiring.
        double zero_lat = 0.0;
        double zero_yaw = 0.0;
        smoothOutput(zero_lat, zero_yaw);
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

    // Apply output smoothing and rate limiting
    smoothOutput(lateral, yaw_rate);

    double forward = config_.forward_mps;
    // Scale forward speed by detection confidence to slow down when uncertain
    if (config_.forward_confidence_scale > 0.0) {
        const double conf_factor = std::clamp(
            std::pow(line.confidence, config_.forward_confidence_scale), 0.0, 1.0);
        forward *= conf_factor;
    }

    output.vx_forward_mps = static_cast<float>(forward);
    output.vy_right_mps = static_cast<float>(lateral);
    output.yaw_rate_rad_s = static_cast<float>(yaw_rate);
    return output;
}

ControlSetpoint GuidedVelocityController::updateMarker(
    const MarkerControlInput& marker,
    const AltitudeControlInput& altitude)
{
    ControlSetpoint output = holdAltitude(altitude);
    if (!marker.marker_detected) {
        double zero_forward = 0.0;
        double zero_lateral = 0.0;
        smoothMarkerOutput(zero_forward, zero_lateral);
        return output;
    }

    const double marker_x_error =
        applyDeadband(marker.center_error_x_norm, config_.marker_deadband_norm);
    const double marker_y_error =
        applyDeadband(marker.center_error_y_norm, config_.marker_deadband_norm);

    double lateral = std::clamp(
        config_.marker_x_kp * marker_x_error,
        -config_.max_marker_mps,
        config_.max_marker_mps);
    double forward = std::clamp(
        config_.marker_y_kp * marker_y_error,
        -config_.max_marker_mps,
        config_.max_marker_mps);
    if (config_.invert_marker_x) {
        lateral = -lateral;
    }
    if (config_.invert_marker_y) {
        forward = -forward;
    }

    smoothMarkerOutput(forward, lateral);

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

ControlSetpoint GuidedVelocityController::update(const LineControlInput& input)
{
    return updateLine(input, AltitudeControlInput {});
}

ControlSetpoint GuidedVelocityController::stop(const AltitudeControlInput& altitude)
{
    // Decay smoothing state toward zero when stopping
    double zero_lat = 0.0;
    double zero_yaw = 0.0;
    smoothOutput(zero_lat, zero_yaw);
    double zero_forward = 0.0;
    double zero_marker_lat = 0.0;
    smoothMarkerOutput(zero_forward, zero_marker_lat);
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
