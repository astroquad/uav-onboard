#include "control/GridControlMapper.hpp"

#include <algorithm>
#include <cmath>

namespace onboard::control {

GridControlMapper::GridControlMapper(GridControlMapperConfig config,
                                     GuidedVelocityController* line_controller)
    : config_(config), line_controller_(line_controller)
{
}

double GridControlMapper::wrapAngle(double a) const
{
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

double GridControlMapper::computeAltitudeVz(bool available, double current, double target)
{
    if (!available) {
        altitude_integral_ = 0.0;   // no estimate: drop the accumulator
        return 0.0;
    }
    const double err = target - current;
    // Integral term with clamp-based anti-windup. Skipped entirely when ki == 0,
    // so the controller stays bit-for-bit the pure-P version by default.
    if (config_.altitude_ki > 0.0) {
        altitude_integral_ += err;
        const double i_clamp = config_.altitude_max_vz_mps / config_.altitude_ki;
        altitude_integral_ = std::clamp(altitude_integral_, -i_clamp, i_clamp);
    }
    double vz_up = config_.altitude_kp * err + config_.altitude_ki * altitude_integral_;
    if (vz_up > config_.altitude_max_vz_mps)  vz_up = config_.altitude_max_vz_mps;
    if (vz_up < -config_.altitude_max_vz_mps) vz_up = -config_.altitude_max_vz_mps;
    // ControlSetpoint.vz_down_mps is downward-positive; positive vz_up means rise.
    return -vz_up;
}

double GridControlMapper::computeYawRate(double current_yaw_rad, double target_yaw_rad)
{
    // Reset the accumulator when the target changes (a new turn/align goal) so
    // integral wound up against the previous target does not carry over.
    if (!yaw_target_initialized_ ||
        std::abs(wrapAngle(target_yaw_rad - prev_yaw_target_rad_)) > 1e-3) {
        yaw_integral_ = 0.0;
        prev_yaw_target_rad_ = target_yaw_rad;
        yaw_target_initialized_ = true;
    }
    const double err = wrapAngle(target_yaw_rad - current_yaw_rad);
    if (std::abs(err) <= config_.yaw_align_deadband_rad) {
        yaw_integral_ = 0.0;        // settled inside deadband: no residual wind-up
        return 0.0;
    }
    if (config_.yaw_align_ki > 0.0) {
        yaw_integral_ += err;
        const double i_clamp = config_.max_yaw_rate_rad_s / config_.yaw_align_ki;
        yaw_integral_ = std::clamp(yaw_integral_, -i_clamp, i_clamp);
    }
    double rate = config_.yaw_align_kp * err + config_.yaw_align_ki * yaw_integral_;
    if (rate > config_.max_yaw_rate_rad_s)  rate = config_.max_yaw_rate_rad_s;
    if (rate < -config_.max_yaw_rate_rad_s) rate = -config_.max_yaw_rate_rad_s;
    return rate;
}

ControlSetpoint GridControlMapper::compute(const GridControlMapperInput& input)
{
    ControlSetpoint out;
    AltitudeControlInput alt {};
    alt.altitude_available = input.altitude_available;
    alt.current_altitude_m = input.current_altitude_m;
    alt.target_altitude_m = input.target_altitude_m;

    switch (input.intent) {
    case GridControlIntent::Idle:
        // emit zero; caller may choose not to send anything.
        return out;
    case GridControlIntent::HoldPosition: {
        out.vz_down_mps = static_cast<float>(
            computeAltitudeVz(input.altitude_available, input.current_altitude_m,
                              input.target_altitude_m));
        return out;
    }
    case GridControlIntent::ForwardBlind: {
        out.vx_forward_mps = static_cast<float>(
            input.forward_speed_override_mps.value_or(config_.forward_speed_blind_mps));
        out.vz_down_mps = static_cast<float>(
            computeAltitudeVz(input.altitude_available, input.current_altitude_m,
                              input.target_altitude_m));
        if (input.yaw_available) {
            out.yaw_rate_rad_s = static_cast<float>(
                computeYawRate(input.current_yaw_rad, input.target_yaw_rad));
        }
        // Lateral line-centering while yaw stays frozen. Without
        // this the drone tracks a slight LineDetector center bias in body
        // frame which manifests as a consistent left/right tilt on one of
        // N/S only (the bias direction in grid frame depends on heading).
        // angle_error_rad is intentionally zeroed before the line_controller
        // call so the yaw lock above is the sole yaw authority.
        // populateLineInputs already nulls the line errors near intersections
        // (line freeze), so vy here only acts on confident straight-line
        // detections.
        if (input.line_detected && line_controller_ != nullptr) {
            LineControlInput line {};
            line.line_detected = true;
            line.center_error_norm = input.line_center_error_norm;
            line.angle_error_rad = 0.0;
            line.confidence = input.line_confidence;
            const ControlSetpoint sp_line = line_controller_->updateLine(line, alt);
            out.vy_right_mps = sp_line.vy_right_mps;
        }
        return out;
    }
    case GridControlIntent::LineFollow: {
        LineControlInput line {};
        line.line_detected = input.line_detected;
        line.center_error_norm = input.line_center_error_norm;
        line.angle_error_rad = input.line_angle_error_rad;
        line.confidence = input.line_confidence;
        ControlSetpoint sp = line_controller_
            ? line_controller_->updateLine(line, alt)
            : ControlSetpoint {};
        // Line_controller's yaw_rate follows the LineDetector's
        // angle_error directly. That signal drifts cumulatively across cells
        // (LineDetector has a small angle bias) and walks the body-yaw away
        // from the latched column heading. Override the yaw_rate with a lock
        // to the mission's latched target_yaw_rad so vy keeps line-centering
        // but yaw stays exactly on the column.
        if (input.yaw_available) {
            sp.yaw_rate_rad_s = static_cast<float>(
                computeYawRate(input.current_yaw_rad, input.target_yaw_rad));
        }
        if (input.advance_phase) {
            const double scale = config_.forward_speed_advance_mps /
                std::max(0.05, static_cast<double>(std::abs(sp.vx_forward_mps)) + 1e-6);
            if (std::abs(sp.vx_forward_mps) > config_.forward_speed_advance_mps) {
                sp.vx_forward_mps = static_cast<float>(
                    config_.forward_speed_advance_mps *
                    (sp.vx_forward_mps >= 0 ? 1.0 : -1.0));
            }
            (void)scale;
        }
        return sp;
    }
    case GridControlIntent::LaunchAlign: {
        LineControlInput line {};
        line.line_detected = input.line_detected;
        line.center_error_norm = input.line_center_error_norm;
        line.angle_error_rad = input.line_angle_error_rad;
        line.confidence = input.line_confidence;
        ControlSetpoint sp = line_controller_
            ? line_controller_->updateLine(line, alt)
            : ControlSetpoint {};
        sp.vx_forward_mps = 0.0f;
        // Same yaw lock as LineFollow — LaunchAlign was the main
        // contributor to per-cell yaw drift (the LineDetector angle bias gets
        // applied repeatedly during the multi-second hold) so this override
        // is the critical change.
        if (input.yaw_available) {
            sp.yaw_rate_rad_s = static_cast<float>(
                computeYawRate(input.current_yaw_rad, input.target_yaw_rad));
        }
        return sp;
    }
    case GridControlIntent::StopAndCenter: {
        out.vz_down_mps = static_cast<float>(
            computeAltitudeVz(input.altitude_available, input.current_altitude_m,
                              input.target_altitude_m));
        // Cy-feedback deceleration. While the intersection center
        // is still above the camera target (cy < stop_center_target_cy), push
        // a small forward velocity that tapers off as the gap closes. When cy
        // reaches the target we command vx=0 and the drone settles right
        // over the intersection center.
        if (input.intersection_valid &&
            input.intersection_center_y_norm < config_.stop_center_target_cy) {
            const double gap = std::max(0.0,
                config_.stop_center_target_cy - input.intersection_center_y_norm);
            const double taper = std::max(1e-3, config_.stop_center_taper_gap);
            const double scale = std::min(1.0, gap / taper);
            out.vx_forward_mps =
                static_cast<float>(config_.stop_center_max_vx_mps * scale);
        }
        return out;
    }
    case GridControlIntent::IntersectionCenter: {
        out.vz_down_mps = static_cast<float>(
            computeAltitudeVz(input.altitude_available, input.current_altitude_m,
                              input.target_altitude_m));
        if (input.yaw_available) {
            out.yaw_rate_rad_s = static_cast<float>(
                computeYawRate(input.current_yaw_rad, input.target_yaw_rad));
        }
        if (!input.intersection_valid) {
            return out;
        }
        const double y_error =
            config_.intersection_center_target_y_norm - input.intersection_center_y_norm;
        const double forward = config_.intersection_center_forward_kp * y_error;
        out.vx_forward_mps = static_cast<float>(std::clamp(
            forward,
            -config_.intersection_center_max_reverse_mps,
            config_.intersection_center_max_forward_mps));
        const double lateral =
            config_.intersection_center_lateral_kp * input.intersection_center_x_norm;
        out.vy_right_mps = static_cast<float>(std::clamp(
            lateral,
            -config_.intersection_center_max_lateral_mps,
            config_.intersection_center_max_lateral_mps));
        return out;
    }
    case GridControlIntent::YawAlign:
    case GridControlIntent::YawTurn: {
        if (input.yaw_available) {
            out.yaw_rate_rad_s = static_cast<float>(
                computeYawRate(input.current_yaw_rad, input.target_yaw_rad));
        }
        out.vz_down_mps = static_cast<float>(
            computeAltitudeVz(input.altitude_available, input.current_altitude_m,
                              input.target_altitude_m));
        return out;
    }
    case GridControlIntent::MarkerHover: {
        MarkerControlInput marker {};
        marker.marker_detected = input.marker_detected;
        marker.center_error_x_norm = input.marker_center_error_x_norm;
        marker.center_error_y_norm = input.marker_center_error_y_norm;
        ControlSetpoint sp = line_controller_
            ? line_controller_->updateMarker(marker, alt)
            : ControlSetpoint {};
        // Marker hover must also drive the body-yaw toward the
        // mission's target_yaw_rad so MarkerLockYaw can rotate while staying
        // centered on the marker. GuidedVelocityController::updateMarker
        // forces yaw_rate_rad_s = 0, so we override it here from the mapper.
        if (input.yaw_available) {
            sp.yaw_rate_rad_s = static_cast<float>(
                computeYawRate(input.current_yaw_rad, input.target_yaw_rad));
        }
        return sp;
    }
    case GridControlIntent::Land:
    default:
        return out;
    }
}

const char* gridControlIntentName(GridControlIntent intent)
{
    switch (intent) {
    case GridControlIntent::Idle:           return "idle";
    case GridControlIntent::HoldPosition:   return "hold";
    case GridControlIntent::ForwardBlind:   return "fwd_blind";
    case GridControlIntent::LineFollow:     return "line_follow";
    case GridControlIntent::StopAndCenter:  return "stop_center";
    case GridControlIntent::IntersectionCenter: return "intersection_center";
    case GridControlIntent::LaunchAlign:    return "launch_align";
    case GridControlIntent::YawAlign:       return "yaw_align";
    case GridControlIntent::YawTurn:        return "yaw_turn";
    case GridControlIntent::MarkerHover:    return "marker_hover";
    case GridControlIntent::Land:           return "land";
    }
    return "unknown";
}

} // namespace onboard::control
