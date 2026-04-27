#include "camera/CameraFrame.hpp"
#include "camera/RpicamMjpegSource.hpp"
#include "common/NetworkConfig.hpp"
#include "common/VisionConfig.hpp"
#include "network/UdpTelemetrySender.hpp"
#include "protocol/TelemetryMessage.hpp"
#include "video/UdpMjpegStreamer.hpp"
#include "vision/ArucoDetector.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

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
        << "  --no-video           Disable MJPEG video streaming\n"
        << "  --no-telemetry       Disable marker telemetry streaming\n"
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

    const auto started_at = std::chrono::steady_clock::now();
    while (true) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - started_at);
        const int remaining_ms = kGcsDiscoveryTimeoutMs - static_cast<int>(elapsed.count());
        if (remaining_ms <= 0) {
            break;
        }

        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(socket_handle, &read_set);
        timeval timeout {};
        timeout.tv_sec = remaining_ms / 1000;
        timeout.tv_usec = (remaining_ms % 1000) * 1000;
        const int ready = select(static_cast<int>(socket_handle) + 1, &read_set, nullptr, nullptr, &timeout);
        if (ready <= 0) {
            break;
        }

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
        if (received <= 0) {
            continue;
        }

        const std::string message(buffer, buffer + received);
        if (message.rfind("AQGCS1", 0) != 0) {
            continue;
        }

        char ip_buffer[INET_ADDRSTRLEN] {};
        if (!inet_ntop(AF_INET, &sender.sin_addr, ip_buffer, sizeof(ip_buffer))) {
            continue;
        }

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

#ifdef _WIN32
    closesocket(socket_handle);
    WSACleanup();
#else
    close(socket_handle);
#endif
    return std::nullopt;
}

onboard::protocol::Point2f toProtocolPoint(const onboard::vision::Point2f& point)
{
    return {point.x, point.y};
}

onboard::protocol::MarkerTelemetry toProtocolMarker(const onboard::vision::MarkerObservation& marker)
{
    onboard::protocol::MarkerTelemetry output;
    output.id = marker.id;
    output.center_px = toProtocolPoint(marker.center_px);
    for (std::size_t index = 0; index < output.corners_px.size(); ++index) {
        output.corners_px[index] = toProtocolPoint(marker.corners_px[index]);
    }
    output.orientation_deg = marker.orientation_deg;
    return output;
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

    onboard::video::UdpMjpegStreamer video_streamer;
    if (options.send_video && !video_streamer.open(network_config.gcs_ip, network_config.video_port)) {
        std::cerr << "failed to open UDP video streamer: " << video_streamer.lastError() << "\n";
        return 1;
    }

    onboard::network::UdpTelemetrySender telemetry_sender;
    if (options.send_telemetry && !telemetry_sender.open(network_config.gcs_ip, network_config.telemetry_port)) {
        std::cerr << "failed to open UDP telemetry sender: " << telemetry_sender.lastError() << "\n";
        return 1;
    }

    onboard::camera::RpicamMjpegSource camera;
    onboard::camera::RpicamOptions camera_options;
    camera_options.width = vision_config.video.width;
    camera_options.height = vision_config.video.height;
    camera_options.fps = vision_config.video.fps;
    camera_options.jpeg_quality = vision_config.video.jpeg_quality;
    if (!camera.open(camera_options)) {
        std::cerr << "failed to open rpicam source: " << camera.lastError() << "\n";
        return 1;
    }

    const onboard::vision::ArucoDetector detector(vision_config.aruco);

    std::cout << "vision_debug_node\n"
              << "  destination: " << network_config.gcs_ip << "\n"
              << "  telemetry UDP port: " << network_config.telemetry_port << "\n"
              << "  video UDP port: " << network_config.video_port << "\n"
              << "  size: " << vision_config.video.width << 'x' << vision_config.video.height << "\n"
              << "  fps: " << vision_config.video.fps << "\n"
              << "  aruco_dictionary: " << detector.dictionaryName() << "\n"
              << "  video: " << (options.send_video ? "on" : "off") << "\n"
              << "  telemetry: " << (options.send_telemetry ? "on" : "off") << "\n"
              << "  count: " << (options.count == 0 ? std::string("forever") : std::to_string(options.count))
              << "\n";

    std::uint32_t telemetry_seq = 1;
    int processed_count = 0;
    while (options.count == 0 || processed_count < options.count) {
        onboard::camera::CameraFrame frame;
        if (!camera.readFrame(frame)) {
            std::cerr << "failed to read rpicam frame: " << camera.lastError() << "\n";
            return 1;
        }

        const auto processing_started = std::chrono::steady_clock::now();
        const cv::Mat encoded(1, static_cast<int>(frame.jpeg_data.size()), CV_8UC1, frame.jpeg_data.data());
        const cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);

        onboard::vision::VisionResult result;
        const auto aruco_started = std::chrono::steady_clock::now();
        if (!image.empty()) {
            result = detector.detect(image, frame.frame_id, frame.timestamp_ms);
        } else {
            result.frame_seq = frame.frame_id;
            result.timestamp_ms = frame.timestamp_ms;
            result.width = frame.width;
            result.height = frame.height;
        }
        const auto aruco_finished = std::chrono::steady_clock::now();
        const auto processing_finished = aruco_finished;

        if (options.send_telemetry) {
            onboard::protocol::BringupTelemetry telemetry;
            telemetry.seq = telemetry_seq++;
            telemetry.timestamp_ms = frame.timestamp_ms;
            telemetry.camera.status = image.empty() ? "decode_failed" : "streaming";
            telemetry.camera.width = frame.width;
            telemetry.camera.height = frame.height;
            telemetry.camera.fps = vision_config.video.fps;
            telemetry.camera.frame_seq = frame.frame_id;
            telemetry.vision.marker_detected = !result.markers.empty();
            telemetry.vision.marker_id = result.markers.empty() ? -1 : result.markers.front().id;
            for (const auto& marker : result.markers) {
                telemetry.vision.markers.push_back(toProtocolMarker(marker));
            }
            telemetry.debug.aruco_latency_ms = std::chrono::duration<double, std::milli>(
                aruco_finished - aruco_started).count();
            telemetry.debug.processing_latency_ms = std::chrono::duration<double, std::milli>(
                processing_finished - processing_started).count();
            telemetry.note = "vision_debug_node";

            const std::string payload = onboard::protocol::buildTelemetryJson(telemetry);
            if (!telemetry_sender.send(payload)) {
                std::cerr << "failed to send marker telemetry: " << telemetry_sender.lastError() << "\n";
                return 1;
            }
        }

        if (options.send_video && !video_streamer.sendFrame(frame)) {
            std::cerr << "failed to send video frame: " << video_streamer.lastError() << "\n";
            return 1;
        }

        ++processed_count;
        std::cout << "frame=" << frame.frame_id
                  << " markers=" << result.markers.size()
                  << " jpeg_bytes=" << frame.jpeg_data.size() << "\n";
    }

    return 0;
}
