#include "app/VisionDebugPipeline.hpp"
#include "common/NetworkConfig.hpp"
#include "common/VisionConfig.hpp"

#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace {

constexpr std::uint16_t kGcsDiscoveryPort = 5601;
constexpr int kGcsDiscoveryTimeoutMs = 3000;

struct Options {
    std::string config_dir = "config";
    std::string gcs_ip_override;
    int count = 0;
    int video_port_override = 0;
    bool send_video = true;
    bool send_telemetry = true;
    bool enable_aruco = true;
    bool enable_line = true;
    std::string line_mode_override;
    int line_threshold_override = -1;
};

struct GcsDiscoveryResult {
    std::string ip;
    std::uint16_t video_port = 0;
};

void printUsage()
{
    std::cout
        << "Usage: vision_debug_node [options]\n\n"
        << "Options:\n"
        << "  --config <dir>       Config directory, default config\n"
        << "  --gcs-ip <ip>        Override GCS destination IP\n"
        << "  --port <n>           Override video destination UDP port\n"
        << "  --count <n>          Process n frames, 0 means forever\n"
        << "  --no-video           Disable best-effort MJPEG video streaming\n"
        << "  --no-telemetry       Disable vision telemetry streaming\n"
        << "  --disable-aruco      Disable ArUco detection\n"
        << "  --disable-line       Disable line detection\n"
        << "  --aruco-only         Enable ArUco and disable line detection\n"
        << "  --line-only          Enable line detection and disable ArUco\n"
        << "  --line-mode <mode>   auto, light_on_dark, or dark_on_light\n"
        << "  --line-threshold <n> Override fixed threshold; 0 uses Otsu/auto\n"
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
            options.video_port_override = parseInt(argv[++i], options.video_port_override);
        } else if (arg == "--count" && i + 1 < argc) {
            options.count = parseInt(argv[++i], options.count);
        } else if (arg == "--no-video") {
            options.send_video = false;
        } else if (arg == "--no-telemetry") {
            options.send_telemetry = false;
        } else if (arg == "--disable-aruco") {
            options.enable_aruco = false;
        } else if (arg == "--disable-line") {
            options.enable_line = false;
        } else if (arg == "--aruco-only") {
            options.enable_aruco = true;
            options.enable_line = false;
        } else if (arg == "--line-only") {
            options.enable_aruco = false;
            options.enable_line = true;
        } else if (arg == "--line-mode" && i + 1 < argc) {
            options.line_mode_override = argv[++i];
        } else if (arg == "--line-threshold" && i + 1 < argc) {
            options.line_threshold_override = parseInt(argv[++i], options.line_threshold_override);
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

bool isBroadcastAddress(const std::string& ip)
{
    return ip == "255.255.255.255" || ip == "0.0.0.0";
}

std::optional<std::uint16_t> parseBeaconVideoPort(const std::string& message)
{
    constexpr const char* key = "video_port=";
    const auto position = message.find(key);
    if (position == std::string::npos) {
        return std::nullopt;
    }

    std::size_t index = position + std::string(key).size();
    std::string value;
    while (index < message.size() && std::isdigit(static_cast<unsigned char>(message[index]))) {
        value.push_back(message[index++]);
    }
    if (value.empty()) {
        return std::nullopt;
    }

    const int port = parseInt(value, 0);
    if (port <= 0 || port > 65535) {
        return std::nullopt;
    }
    return static_cast<std::uint16_t>(port);
}

std::optional<GcsDiscoveryResult> discoverGcsVideoTarget(std::uint16_t fallback_video_port)
{
#ifdef _WIN32
    WSADATA data {};
    WSAStartup(MAKEWORD(2, 2), &data);
#endif

    const auto raw_socket = socket(AF_INET, SOCK_DGRAM, 0);
#ifdef _WIN32
    if (raw_socket == INVALID_SOCKET) {
        WSACleanup();
        return std::nullopt;
    }
    const auto socket_handle = static_cast<SOCKET>(raw_socket);
#else
    if (raw_socket < 0) {
        return std::nullopt;
    }
    const auto socket_handle = static_cast<int>(raw_socket);
#endif

    const int reuse = 1;
    setsockopt(
        socket_handle,
        SOL_SOCKET,
        SO_REUSEADDR,
#ifdef _WIN32
        reinterpret_cast<const char*>(&reuse),
#else
        &reuse,
#endif
        sizeof(reuse));

    sockaddr_in address {};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(kGcsDiscoveryPort);
    if (bind(socket_handle, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
#ifdef _WIN32
        closesocket(socket_handle);
        WSACleanup();
#else
        close(socket_handle);
#endif
        return std::nullopt;
    }

    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(socket_handle, &read_set);
    timeval timeout {};
    timeout.tv_sec = kGcsDiscoveryTimeoutMs / 1000;
    timeout.tv_usec = (kGcsDiscoveryTimeoutMs % 1000) * 1000;
    const int ready = select(static_cast<int>(socket_handle) + 1, &read_set, nullptr, nullptr, &timeout);
    if (ready > 0) {
        char buffer[128] {};
        sockaddr_in sender {};
#ifdef _WIN32
        int sender_size = sizeof(sender);
#else
        socklen_t sender_size = sizeof(sender);
#endif
        const int received = recvfrom(
            socket_handle,
            buffer,
            static_cast<int>(sizeof(buffer) - 1),
            0,
            reinterpret_cast<sockaddr*>(&sender),
            &sender_size);
        if (received > 0) {
            const std::string message(buffer, buffer + received);
            if (message.rfind("AQGCS1", 0) == 0) {
                char ip_buffer[INET_ADDRSTRLEN] {};
                if (inet_ntop(AF_INET, &sender.sin_addr, ip_buffer, sizeof(ip_buffer))) {
                    GcsDiscoveryResult result;
                    result.ip = ip_buffer;
                    result.video_port = parseBeaconVideoPort(message).value_or(fallback_video_port);
#ifdef _WIN32
                    closesocket(socket_handle);
                    WSACleanup();
#else
                    close(socket_handle);
#endif
                    return result;
                }
            }
        }
    }

#ifdef _WIN32
    closesocket(socket_handle);
    WSACleanup();
#else
    close(socket_handle);
#endif
    return std::nullopt;
}

} // namespace

int main(int argc, char** argv)
{
    const Options options = parseOptions(argc, argv);
    auto network_config = onboard::common::loadNetworkConfig(options.config_dir);
    auto vision_config = onboard::common::loadVisionConfig(options.config_dir);

    if (!options.gcs_ip_override.empty()) {
        network_config.gcs_ip = options.gcs_ip_override;
    }
    if (!options.line_mode_override.empty()) {
        vision_config.line.mode = options.line_mode_override;
    }
    if (options.line_threshold_override >= 0) {
        vision_config.line.threshold = options.line_threshold_override;
    }
    if (options.video_port_override > 0) {
        network_config.video_port = static_cast<std::uint16_t>(options.video_port_override);
    }

    if (options.gcs_ip_override.empty() && isBroadcastAddress(network_config.gcs_ip)) {
        std::cout << "discovering GCS video receiver for "
                  << kGcsDiscoveryTimeoutMs << " ms...\n";
        const auto discovered = discoverGcsVideoTarget(network_config.video_port);
        if (discovered) {
            network_config.gcs_ip = discovered->ip;
            network_config.video_port = discovered->video_port;
            std::cout << "discovered GCS video receiver at "
                      << network_config.gcs_ip << ':' << network_config.video_port << "\n";
        } else {
            std::cout << "no GCS discovery beacon received; falling back to "
                      << network_config.gcs_ip << ':' << network_config.video_port << "\n";
        }
    }

    onboard::app::VisionDebugPipelineOptions pipeline_options;
    pipeline_options.network = network_config;
    pipeline_options.vision = vision_config;
    pipeline_options.count = options.count;
    pipeline_options.send_video = options.send_video;
    pipeline_options.send_telemetry = options.send_telemetry;
    pipeline_options.enable_aruco = options.enable_aruco;
    pipeline_options.enable_line = options.enable_line;

    onboard::app::VisionDebugPipeline pipeline;
    return pipeline.run(pipeline_options);
}
