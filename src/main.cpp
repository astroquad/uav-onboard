#include "common/NetworkConfig.hpp"
#include "common/Time.hpp"
#include "network/UdpTelemetrySender.hpp"
#include "protocol/TelemetryMessage.hpp"

#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

namespace {

struct Options {
    std::string config_dir = "../config";
    std::string gcs_ip_override;
    int count = 0;
    int interval_ms = 0;
    int telemetry_port_override = 0;
};

void printUsage()
{
    std::cout
        << "Usage: uav_onboard [options]\n"
        << "\n"
        << "Options:\n"
        << "  --config <dir>       Config directory containing network.toml\n"
        << "  --gcs-ip <ip>        Override GCS telemetry destination IP\n"
        << "  --port <n>           Override GCS telemetry destination UDP port\n"
        << "  --count <n>          Send n telemetry packets, 0 means forever\n"
        << "  --interval-ms <n>    Override telemetry send interval\n"
        << "  -h, --help           Show this help\n";
}

int parseInt(const std::string& value, int fallback)
{
    try {
        return std::stoi(value);
    } catch (...) {
        return fallback;
    }
}

Options parseOptions(int argc, char** argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            options.config_dir = argv[++i];
        } else if (arg == "--gcs-ip" && i + 1 < argc) {
            options.gcs_ip_override = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            options.telemetry_port_override = parseInt(argv[++i], options.telemetry_port_override);
        } else if (arg == "--count" && i + 1 < argc) {
            options.count = parseInt(argv[++i], options.count);
        } else if (arg == "--interval-ms" && i + 1 < argc) {
            options.interval_ms = parseInt(argv[++i], options.interval_ms);
        } else if (arg == "-h" || arg == "--help") {
            printUsage();
            std::exit(0);
        } else {
            std::cerr << "unknown or incomplete option: " << arg << "\n";
            printUsage();
            std::exit(2);
        }
    }
    return options;
}

} // namespace

int main(int argc, char** argv)
{
    const Options options = parseOptions(argc, argv);
    auto config = onboard::common::loadNetworkConfig(options.config_dir);
    if (!options.gcs_ip_override.empty()) {
        config.gcs_ip = options.gcs_ip_override;
    }
    if (options.telemetry_port_override > 0) {
        config.telemetry_port = static_cast<std::uint16_t>(options.telemetry_port_override);
    }
    if (options.interval_ms > 0) {
        config.telemetry_interval_ms = options.interval_ms;
    }

    onboard::network::UdpTelemetrySender sender;
    if (!sender.open(config.gcs_ip, config.telemetry_port)) {
        std::cerr << "failed to open UDP telemetry sender: " << sender.lastError() << "\n";
        return 1;
    }

    std::cout << "uav_onboard bring-up telemetry\n"
              << "  destination: " << config.gcs_ip << ':' << config.telemetry_port << "\n"
              << "  interval_ms: " << config.telemetry_interval_ms << "\n"
              << "  count: " << (options.count == 0 ? std::string("forever") : std::to_string(options.count))
              << "\n";

    std::uint32_t seq = 1;
    int sent_count = 0;
    while (options.count == 0 || sent_count < options.count) {
        onboard::protocol::BringupTelemetry telemetry;
        telemetry.seq = seq++;
        telemetry.timestamp_ms = onboard::common::unixTimestampMs();
        telemetry.camera.status = "not_checked";
        telemetry.note = "camera_preview is the dedicated camera bring-up tool";

        const std::string payload = onboard::protocol::buildTelemetryJson(telemetry);
        if (!sender.send(payload)) {
            std::cerr << "failed to send telemetry: " << sender.lastError() << "\n";
            return 1;
        }

        std::cout << "sent TELEMETRY seq=" << telemetry.seq
                  << " timestamp_ms=" << telemetry.timestamp_ms << "\n";
        ++sent_count;
        std::this_thread::sleep_for(std::chrono::milliseconds(config.telemetry_interval_ms));
    }

    return 0;
}
