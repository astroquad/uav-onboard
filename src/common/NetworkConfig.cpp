#include "common/NetworkConfig.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>

namespace onboard::common {
namespace {

std::string trim(std::string value)
{
    const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    return value;
}

std::string unquote(std::string value)
{
    value = trim(value);
    if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
        return value.substr(1, value.size() - 2);
    }
    return value;
}

int parseInt(const std::string& value, int fallback)
{
    try {
        return std::stoi(trim(value));
    } catch (...) {
        return fallback;
    }
}

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
    std::ifstream file(joinConfigPath(config_dir));
    if (!file) {
        return config;
    }

    std::string section;
    std::string line;
    while (std::getline(file, line)) {
        const auto comment = line.find('#');
        if (comment != std::string::npos) {
            line = line.substr(0, comment);
        }
        line = trim(line);
        if (line.empty()) {
            continue;
        }
        if (line.front() == '[' && line.back() == ']') {
            section = trim(line.substr(1, line.size() - 2));
            continue;
        }

        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            continue;
        }

        const std::string key = trim(line.substr(0, eq));
        const std::string value = trim(line.substr(eq + 1));

        if (section == "gcs") {
            if (key == "ip") {
                config.gcs_ip = unquote(value);
            } else if (key == "telemetry_port") {
                config.telemetry_port = static_cast<std::uint16_t>(
                    parseInt(value, config.telemetry_port));
            } else if (key == "command_port") {
                config.command_port = static_cast<std::uint16_t>(
                    parseInt(value, config.command_port));
            } else if (key == "video_port") {
                config.video_port = static_cast<std::uint16_t>(
                    parseInt(value, config.video_port));
            }
        } else if (section == "telemetry" && key == "send_interval_ms") {
            config.telemetry_interval_ms = parseInt(value, config.telemetry_interval_ms);
        }
    }

    return config;
}

} // namespace onboard::common
