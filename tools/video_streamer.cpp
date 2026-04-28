#include "camera/CameraFrame.hpp"
#include "camera/RpicamMjpegSource.hpp"
#include "common/NetworkConfig.hpp"
#include "common/Time.hpp"
#include "common/VisionConfig.hpp"
#include "video/UdpMjpegStreamer.hpp"

#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace {

constexpr std::uint16_t kGcsDiscoveryPort = 5601;
constexpr int kGcsDiscoveryTimeoutMs = 3000;

constexpr const char* kTinyTestJpegBase64 =
    "/9j/4AAQSkZJRgABAQEAYABgAAD/2wBDAAMCAgMCAgMDAwMEAwMEBQgFBQQEBQoHBwYIDAoM"
    "DAsKCwsNDhIQDQ4RDgsLEBYQERMUFRUVDA8XGBYUGBIUFRT/2wBDAQMEBAUEBQkFBQkUDQsN"
    "FBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBT/wAAR"
    "CAAQABADASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAA"
    "AgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkK"
    "FhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWG"
    "h4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl"
    "5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREA"
    "AgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYk"
    "NOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOE"
    "hYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk"
    "5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwCvc/660/66n/0BqsXn/Idv/wAP/QnpbjUb"
    "tZrYC6mAMhBxIefkapru/ul1q9QXMwRcYUOcD5n6D8BSnPGc0P3cf4Mvtvb20P7nf/h+gp1M"
    "fzU/3UP93n9uW31in/073v07a36H/9k=";

struct Options {
    std::string config_dir = "config";
    std::string source = "rpicam";
    std::string image_path;
    std::string gcs_ip_override;
    int port_override = 0;
    int width_override = 0;
    int height_override = 0;
    int fps_override = 0;
    int count = 0;
};

void printUsage()
{
    std::cout
        << "Usage: video_streamer [options]\n\n"
        << "Options:\n"
        << "  --config <dir>       Config directory, default config\n"
        << "  --source <mode>      rpicam, test-pattern, or image\n"
        << "  --image <path>       JPEG path for --source image\n"
        << "  --gcs-ip <ip>        Override destination IP\n"
        << "  --port <n>           Override destination UDP port\n"
        << "  --width <n>          Override stream width\n"
        << "  --height <n>         Override stream height\n"
        << "  --fps <n>            Override stream FPS\n"
        << "  --count <n>          Send n frames, 0 means forever\n"
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
        } else if (arg == "--source" && i + 1 < argc) {
            options.source = argv[++i];
        } else if (arg == "--image" && i + 1 < argc) {
            options.image_path = argv[++i];
        } else if (arg == "--gcs-ip" && i + 1 < argc) {
            options.gcs_ip_override = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            options.port_override = parseInt(argv[++i], options.port_override);
        } else if (arg == "--width" && i + 1 < argc) {
            options.width_override = parseInt(argv[++i], options.width_override);
        } else if (arg == "--height" && i + 1 < argc) {
            options.height_override = parseInt(argv[++i], options.height_override);
        } else if (arg == "--fps" && i + 1 < argc) {
            options.fps_override = parseInt(argv[++i], options.fps_override);
        } else if (arg == "--count" && i + 1 < argc) {
            options.count = parseInt(argv[++i], options.count);
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

std::vector<std::uint8_t> decodeBase64(const std::string& input)
{
    const std::string alphabet =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::vector<std::uint8_t> output;
    int value = 0;
    int bits = -8;
    for (const unsigned char ch : input) {
        if (ch == '=') {
            break;
        }
        const auto index = alphabet.find(static_cast<char>(ch));
        if (index == std::string::npos) {
            continue;
        }
        value = (value << 6) + static_cast<int>(index);
        bits += 6;
        if (bits >= 0) {
            output.push_back(static_cast<std::uint8_t>((value >> bits) & 0xff));
            bits -= 8;
        }
    }
    return output;
}

std::vector<std::uint8_t> readBinaryFile(const std::string& path)
{
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return {};
    }
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

bool isJpeg(const std::vector<std::uint8_t>& data)
{
    return data.size() >= 4 && data[0] == 0xff && data[1] == 0xd8 &&
           data[data.size() - 2] == 0xff && data[data.size() - 1] == 0xd9;
}

struct GcsDiscoveryResult {
    std::string ip;
    std::uint16_t video_port = 0;
};

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

} // namespace

int main(int argc, char** argv)
{
    const Options options = parseOptions(argc, argv);
    auto network_config = onboard::common::loadNetworkConfig(options.config_dir);
    auto vision_config = onboard::common::loadVisionConfig(options.config_dir);

    if (!options.gcs_ip_override.empty()) {
        network_config.gcs_ip = options.gcs_ip_override;
    }
    if (options.port_override > 0) {
        network_config.video_port = static_cast<std::uint16_t>(options.port_override);
        vision_config.video.port = options.port_override;
    }
    if (options.width_override > 0) {
        vision_config.camera.width = options.width_override;
    }
    if (options.height_override > 0) {
        vision_config.camera.height = options.height_override;
    }
    if (options.fps_override > 0) {
        vision_config.camera.fps = options.fps_override;
    }

    if (options.gcs_ip_override.empty() && isBroadcastAddress(network_config.gcs_ip)) {
        std::cout << "discovering GCS video receiver for "
                  << kGcsDiscoveryTimeoutMs << " ms...\n";
        const auto discovered = discoverGcsVideoTarget(network_config.video_port);
        if (discovered) {
            network_config.gcs_ip = discovered->ip;
            network_config.video_port = discovered->video_port;
            vision_config.video.port = discovered->video_port;
            std::cout << "discovered GCS video receiver at "
                      << network_config.gcs_ip << ':' << network_config.video_port << "\n";
        } else {
            std::cout << "no GCS discovery beacon received; falling back to "
                      << network_config.gcs_ip << ':' << network_config.video_port << "\n";
        }
    }

    onboard::video::UdpMjpegStreamer streamer;
    if (!streamer.open(network_config.gcs_ip, network_config.video_port)) {
        std::cerr << "failed to open UDP video streamer: " << streamer.lastError() << "\n";
        return 1;
    }

    std::cout << "video_streamer\n"
              << "  source: " << options.source << "\n"
              << "  destination: " << network_config.gcs_ip << ':' << network_config.video_port << "\n"
              << "  size: " << vision_config.camera.width << 'x' << vision_config.camera.height << "\n"
              << "  fps: " << vision_config.camera.fps << "\n"
              << "  count: " << (options.count == 0 ? std::string("forever") : std::to_string(options.count))
              << "\n";

    std::uint32_t sent_count = 0;
    if (options.source == "rpicam") {
        onboard::camera::RpicamMjpegSource source;
        onboard::camera::RpicamOptions rpicam_options;
        rpicam_options.camera_index = vision_config.camera.device;
        rpicam_options.width = vision_config.camera.width;
        rpicam_options.height = vision_config.camera.height;
        rpicam_options.fps = vision_config.camera.fps;
        rpicam_options.jpeg_quality = vision_config.camera.jpeg_quality;
        rpicam_options.codec = vision_config.camera.codec;
        rpicam_options.autofocus_mode = vision_config.camera.autofocus_mode;
        rpicam_options.autofocus_range = vision_config.camera.autofocus_range;
        rpicam_options.autofocus_speed = vision_config.camera.autofocus_speed;
        rpicam_options.autofocus_window = vision_config.camera.autofocus_window;
        rpicam_options.lens_position = vision_config.camera.lens_position;
        rpicam_options.exposure = vision_config.camera.exposure;
        rpicam_options.shutter_us = vision_config.camera.shutter_us;
        rpicam_options.gain = vision_config.camera.gain;
        rpicam_options.ev = vision_config.camera.ev;
        rpicam_options.awb = vision_config.camera.awb;
        rpicam_options.awbgains = vision_config.camera.awbgains;
        rpicam_options.metering = vision_config.camera.metering;
        rpicam_options.denoise = vision_config.camera.denoise;
        rpicam_options.sharpness = vision_config.camera.sharpness;
        rpicam_options.contrast = vision_config.camera.contrast;
        rpicam_options.brightness = vision_config.camera.brightness;
        rpicam_options.saturation = vision_config.camera.saturation;
        rpicam_options.roi = vision_config.camera.roi;
        rpicam_options.tuning_file = vision_config.camera.tuning_file;
        rpicam_options.hflip = vision_config.camera.hflip;
        rpicam_options.vflip = vision_config.camera.vflip;
        rpicam_options.rotation = vision_config.camera.rotation;
        if (!source.open(rpicam_options)) {
            std::cerr << "failed to open rpicam source: " << source.lastError() << "\n";
            return 1;
        }
        while (options.count == 0 || static_cast<int>(sent_count) < options.count) {
            onboard::camera::CameraFrame frame;
            if (!source.readFrame(frame)) {
                std::cerr << "failed to read rpicam frame: " << source.lastError() << "\n";
                return 1;
            }
            if (!streamer.sendFrame(frame)) {
                std::cerr << "failed to send video frame: " << streamer.lastError() << "\n";
                return 1;
            }
            ++sent_count;
            std::cout << "sent video frame id=" << frame.frame_id
                      << " bytes=" << frame.jpeg_data.size() << "\n";
        }
        return 0;
    }

    std::vector<std::uint8_t> jpeg;
    if (options.source == "test-pattern") {
        jpeg = decodeBase64(kTinyTestJpegBase64);
    } else if (options.source == "image") {
        if (options.image_path.empty()) {
            std::cerr << "--source image requires --image <path>\n";
            return 2;
        }
        jpeg = readBinaryFile(options.image_path);
    } else {
        std::cerr << "unknown source: " << options.source << "\n";
        return 2;
    }

    if (!isJpeg(jpeg)) {
        std::cerr << "source did not produce a valid JPEG frame\n";
        return 1;
    }

    const auto frame_interval = std::chrono::milliseconds(
        vision_config.camera.fps > 0 ? 1000 / vision_config.camera.fps : 66);
    while (options.count == 0 || static_cast<int>(sent_count) < options.count) {
        onboard::camera::CameraFrame frame;
        frame.frame_id = sent_count + 1;
        frame.timestamp_ms = onboard::common::unixTimestampMs();
        frame.width = vision_config.camera.width;
        frame.height = vision_config.camera.height;
        frame.jpeg_data = jpeg;
        if (!streamer.sendFrame(frame)) {
            std::cerr << "failed to send video frame: " << streamer.lastError() << "\n";
            return 1;
        }
        ++sent_count;
        std::cout << "sent video frame id=" << frame.frame_id
                  << " bytes=" << frame.jpeg_data.size() << "\n";
        std::this_thread::sleep_for(frame_interval);
    }

    return 0;
}
