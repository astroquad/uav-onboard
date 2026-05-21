#pragma once

#include "autopilot/AutopilotState.hpp"
#include "control/ControlSetpoint.hpp"
#include "control/GuidedVelocityController.hpp"
#include "vision/VisionTypes.hpp"

#include <optional>

namespace onboard::control {

enum class GridControlIntent {
    Idle,             // no command output (mission not started)
    HoldPosition,     // freeze body velocity to ~0
    ForwardBlind,     // body-forward + optional lateral line-centering, yaw locked
    LineFollow,       // line-follow controller @ current cruise alt
    StopAndCenter,    // ramp velocity to 0 while holding altitude
    IntersectionCenter, // center an intersection under the camera before origin/node lock
    LaunchAlign,      // align yaw/lateral on the outgoing line before freeze launch
    YawAlign,         // yaw-rate-only command toward target_yaw_rad
    YawTurn,          // active 90° turn (yaw-only, no lateral / forward)
    MarkerHover,      // keep marker centered (line ignored)
    Land,             // pass-through: mission_mapper does not emit setpoint; LAND mode handles it
};

struct GridControlMapperConfig {
    double forward_speed_blind_mps = 0.60;
    double forward_speed_advance_mps = 0.18;  // post-turn slow re-acquire
    double max_yaw_rate_rad_s = 0.6;
    double yaw_align_kp = 1.2;
    double yaw_align_deadband_rad = 0.0349;   // ~2°
    double altitude_max_vz_mps = 0.35;
    double altitude_kp = 0.4;
    // Cycle 12 B: cy-feedback deceleration for StopAndCenter intent.
    double stop_center_target_cy = 0.55;
    double stop_center_max_vx_mps = 0.10;
    double stop_center_taper_gap = 0.30;
    // Entry-origin centering. X is normalized around camera center, Y is
    // normalized image row [0, 1].
    double intersection_center_target_y_norm = 0.55;
    double intersection_center_forward_kp = 0.55;
    double intersection_center_lateral_kp = 0.20;
    double intersection_center_max_forward_mps = 0.20;
    double intersection_center_max_reverse_mps = 0.08;
    double intersection_center_max_lateral_mps = 0.12;
};

struct GridControlMapperInput {
    GridControlIntent intent = GridControlIntent::Idle;
    double target_altitude_m = 1.3;
    bool altitude_available = false;
    double current_altitude_m = 0.0;
    std::optional<double> forward_speed_override_mps;

    // For YawAlign / YawTurn
    bool yaw_available = false;
    double current_yaw_rad = 0.0;
    double target_yaw_rad = 0.0;

    // For LineFollow / advance-after-turn
    bool line_detected = false;
    double line_center_error_norm = 0.0;
    double line_angle_error_rad = 0.0;
    double line_confidence = 0.0;
    bool advance_phase = false;  // limit forward speed to forward_speed_advance_mps

    // For MarkerHover
    bool marker_detected = false;
    double marker_center_error_x_norm = 0.0;
    double marker_center_error_y_norm = 0.0;

    // Cycle 12 B: cy-feedback deceleration in StopAndCenter intent.
    bool intersection_valid = false;
    double intersection_center_x_norm = 0.0;
    double intersection_center_y_norm = 0.0;
};

class GridControlMapper {
public:
    GridControlMapper(GridControlMapperConfig config,
                      GuidedVelocityController* line_controller);

    ControlSetpoint compute(const GridControlMapperInput& input);

private:
    double computeAltitudeVz(bool available, double current, double target) const;
    double computeYawRate(double current_yaw_rad, double target_yaw_rad) const;
    double wrapAngle(double a) const;

    GridControlMapperConfig config_;
    GuidedVelocityController* line_controller_; // not owned
};

const char* gridControlIntentName(GridControlIntent intent);

} // namespace onboard::control
