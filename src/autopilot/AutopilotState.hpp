#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace onboard::autopilot {

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
    std::optional<int> rc_channel_count;
    std::optional<int> rc_rssi;
    std::array<std::uint16_t, 18> rc_channels_pwm {};
    std::optional<double> attitude_roll_rad;
    std::optional<double> attitude_pitch_rad;
    std::optional<double> attitude_yaw_rad;
    std::optional<double> attitude_yawspeed_rad_s;
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
