#include "camera/CameraFrame.hpp"
#include "camera/RpicamMjpegSource.hpp"
#include "common/NetworkConfig.hpp"
#include "common/Time.hpp"
#include "common/VisionConfig.hpp"
#include "video/UdpMjpegStreamer.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <thread>
#include <vector>

namespace {

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
        vision_config.video.width = options.width_override;
    }
    if (options.height_override > 0) {
        vision_config.video.height = options.height_override;
    }
    if (options.fps_override > 0) {
        vision_config.video.fps = options.fps_override;
    }

    onboard::video::UdpMjpegStreamer streamer;
    if (!streamer.open(network_config.gcs_ip, network_config.video_port)) {
        std::cerr << "failed to open UDP video streamer: " << streamer.lastError() << "\n";
        return 1;
    }

    std::cout << "video_streamer\n"
              << "  source: " << options.source << "\n"
              << "  destination: " << network_config.gcs_ip << ':' << network_config.video_port << "\n"
              << "  size: " << vision_config.video.width << 'x' << vision_config.video.height << "\n"
              << "  fps: " << vision_config.video.fps << "\n"
              << "  count: " << (options.count == 0 ? std::string("forever") : std::to_string(options.count))
              << "\n";

    std::uint32_t sent_count = 0;
    if (options.source == "rpicam") {
        onboard::camera::RpicamMjpegSource source;
        onboard::camera::RpicamOptions rpicam_options;
        rpicam_options.width = vision_config.video.width;
        rpicam_options.height = vision_config.video.height;
        rpicam_options.fps = vision_config.video.fps;
        rpicam_options.jpeg_quality = vision_config.video.jpeg_quality;
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
        vision_config.video.fps > 0 ? 1000 / vision_config.video.fps : 66);
    while (options.count == 0 || static_cast<int>(sent_count) < options.count) {
        onboard::camera::CameraFrame frame;
        frame.frame_id = sent_count + 1;
        frame.timestamp_ms = onboard::common::unixTimestampMs();
        frame.width = vision_config.video.width;
        frame.height = vision_config.video.height;
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
