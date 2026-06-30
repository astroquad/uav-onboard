#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

namespace onboard::autopilot {

// One autopilot STATUSTEXT line. ArduPilot reports the reason a mode change or
// arming attempt was rejected here (e.g. "Mode change failed: GUIDED requires
// position estimate"), so we retain the most recent lines to surface the cause
// of a setMode timeout instead of an opaque "timed out" error.
struct StatusTextEntry {
    std::uint8_t severity = 6;   // MAV_SEVERITY (0 = emergency .. 7 = debug)
    std::string  text;
    std::chrono::steady_clock::time_point time {};
};

struct AutopilotState {
    bool heartbeat_seen = false;
    bool armed = false;
    std::uint8_t target_system = 1;
    std::uint8_t target_component = 1;
    std::uint32_t custom_mode = 0;
    std::string mode_name = "unknown";
    std::optional<double> distance_sensor_m;
    std::optional<double> local_x_m;
    std::optional<double> local_y_m;
    std::optional<double> local_z_m;
    std::optional<double> local_altitude_m;
    std::optional<double> local_vx_mps;
    std::optional<double> local_vy_mps;
    std::optional<double> local_vz_mps;
    std::optional<double> relative_altitude_m;
    std::optional<int> optical_flow_quality;
    std::optional<double> optical_flow_ground_distance_m;
    std::optional<std::uint16_t> ekf_flags;
    // EKF normalised innovation test ratios from EKF_STATUS_REPORT. ArduCopter
    // gates position-controller modes (GUIDED/LOITER) on these being below ~1.0,
    // so we track them to predict whether a GUIDED request will be accepted.
    std::optional<double> ekf_velocity_variance;
    std::optional<double> ekf_pos_horiz_variance;
    std::optional<double> ekf_pos_vert_variance;
    std::optional<double> ekf_compass_variance;
    std::optional<double> ekf_terrain_alt_variance;
    std::optional<int> rc_channel_count;
    std::optional<int> rc_rssi;
    std::array<std::uint16_t, 18> rc_channels_pwm {};
    std::array<std::uint16_t, 8> servo_outputs_pwm {};
    bool servo_outputs_seen = false;
    std::optional<double> attitude_roll_rad;
    std::optional<double> attitude_pitch_rad;
    std::optional<double> attitude_yaw_rad;
    std::optional<double> attitude_yawspeed_rad_s;
    // Most recent autopilot STATUSTEXT lines, newest at the back. Capped to a
    // small window; used to attach the FC's own rejection reason to a failed
    // mode change. See kMaxStatusTexts in the adapter.
    std::deque<StatusTextEntry> recent_statustexts;
    std::chrono::steady_clock::time_point last_heartbeat_time {};
    std::chrono::steady_clock::time_point last_optical_flow_time {};
    std::chrono::steady_clock::time_point last_ekf_status_time {};
    std::chrono::steady_clock::time_point last_rc_channels_time {};
    std::chrono::steady_clock::time_point last_attitude_time {};
};

struct MavlinkIds {
    std::uint8_t system_id = 191;
    std::uint8_t component_id = 191;
    std::uint8_t target_system = 1;
    std::uint8_t target_component = 1;
};

struct BodyVelocityCommand {
    float vx_forward_mps = 0.0f;
    float vy_right_mps = 0.0f;
    float vz_down_mps = 0.0f;
    float yaw_rate_rad_s = 0.0f;
};

struct LocalNedPositionTargetCommand {
    float x_m = 0.0f;
    float y_m = 0.0f;
    std::optional<float> z_m;
    std::optional<float> vz_down_mps;
    float yaw_rate_rad_s = 0.0f;
};

} // namespace onboard::autopilot
