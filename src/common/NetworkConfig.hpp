#pragma once

#include <cstdint>
#include <string>

namespace onboard::common {

struct NetworkConfig {
    std::string gcs_ip = "127.0.0.1";
    std::uint16_t telemetry_port = 14550;
    std::uint16_t command_port = 14551;
    std::uint16_t video_port = 5600;
    int telemetry_interval_ms = 1000;
};

NetworkConfig loadNetworkConfig(const std::string& config_dir);

} // namespace onboard::common
