#pragma once

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
    std::optional<double> local_altitude_m;
    std::optional<double> relative_altitude_m;
    std::chrono::steady_clock::time_point last_heartbeat_time {};
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

} // namespace onboard::autopilot
