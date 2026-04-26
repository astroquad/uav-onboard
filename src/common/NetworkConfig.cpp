#include "common/NetworkConfig.hpp"

#include <toml++/toml.hpp>

namespace onboard::common {
namespace {

std::string joinConfigPath(const std::string& config_dir)
{
    if (config_dir.empty()) {
        return "config/network.toml";
    }
    const char last = config_dir.back();
    if (last == '/' || last == '\\') {
        return config_dir + "network.toml";
    }
    return config_dir + "/network.toml";
}

} // namespace

NetworkConfig loadNetworkConfig(const std::string& config_dir)
{
    NetworkConfig config;

    toml::table table;
    try {
        table = toml::parse_file(joinConfigPath(config_dir));
    } catch (const toml::parse_error&) {
        return config;
    }

    if (const auto gcs = table["gcs"]) {
        config.gcs_ip = gcs["ip"].value_or(config.gcs_ip);
        config.telemetry_port = static_cast<std::uint16_t>(
            gcs["telemetry_port"].value_or(static_cast<int>(config.telemetry_port)));
        config.command_port = static_cast<std::uint16_t>(
            gcs["command_port"].value_or(static_cast<int>(config.command_port)));
        config.video_port = static_cast<std::uint16_t>(
            gcs["video_port"].value_or(static_cast<int>(config.video_port)));
    }

    if (const auto telemetry = table["telemetry"]) {
        config.telemetry_interval_ms =
            telemetry["send_interval_ms"].value_or(config.telemetry_interval_ms);
    }

    return config;
}

} // namespace onboard::common
