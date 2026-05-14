#include "app/VisionDebugPublisher.hpp"
#include "autopilot/AutopilotMavlinkAdapter.hpp"
#include "autopilot/UdpMavlinkTransport.hpp"
#include "common/NetworkConfig.hpp"
#include "common/VisionConfig.hpp"
#include "control/GuidedVelocityController.hpp"
#include "mission/LineFollowMission.hpp"
#include "safety/SafetyMonitor.hpp"

#if defined(ONBOARD_LINE_FOLLOW_HAS_VISION)
#include "vision/FakeFrameSource.hpp"
#include "vision/GazeboCameraSource.hpp"
#include "vision/RpicamFrameSource.hpp"
#include "vision/VisionProcessor.hpp"
#endif

#include <toml++/toml.hpp>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

enum class TransportKind {
    Udp,
    Serial,
};

struct EndpointConfig {
    TransportKind kind = TransportKind::Udp;
    std::string label = "sitl";
    std::string listen_host = "0.0.0.0";
    std::uint16_t listen_port = 14550;
    std::string serial_device = "/dev/serial0";
    int serial_baudrate = 115200;
};

struct RuntimeConfig {
    EndpointConfig endpoint;
    onboard::autopilot::MavlinkIds mavlink;
    onboard::common::NetworkConfig network;
    onboard::common::VisionConfig vision;
    int setpoint_rate_hz = 20;
    onboard::mission::LineFollowMissionConfig mission;
    onboard::control::GuidedVelocityControllerConfig controller;
    onboard::safety::SafetyConfig safety;
    double fake_center_error_m = 0.0;
    double fake_line_angle_deg = 0.0;
    double marker_center_tolerance_px = 80.0;
    std::string vision_source = "fake";
};

struct Options {
    std::string config_dir = "config";
    std::string target;
    std::string autopilot_uri;
    std::string vision;
    std::string gcs_ip_override;
    std::string gazebo_topic_override;
    std::optional<int> mission_timeout_ms;
    int telemetry_port_override = 0;
    int video_port_override = 0;
    int vision_smoke_count = 0;
    bool send_video = false;
    bool send_video_overridden = false;
    bool send_telemetry = true;
};

std::string joinConfigPath(const std::string& config_dir, const std::string& filename)
{
    if (config_dir.empty()) {
        return "config/" + filename;
    }
    const char last = config_dir.back();
    if (last == '/' || last == '\\') {
        return config_dir + filename;
    }
    return config_dir + "/" + filename;
}

void printUsage()
{
    std::cout
        << "Usage: line_follow_node [options]\n\n"
        << "Options:\n"
        << "  --config <dir>              Config directory\n"
        << "  --target <sitl|pixhawk1>    Runtime target profile\n"
        << "  --autopilot <uri>           Override endpoint, e.g. udp://0.0.0.0:14550\n"
        << "                              or serial:///dev/serial0:115200\n"
        << "  --vision <fake|gazebo|rpicam>\n"
        << "                              Vision source for this MVP node\n"
        << "  --gcs-ip <ip>               Override GCS telemetry/video destination IP\n"
        << "  --telemetry-port <n>        Override GCS telemetry UDP port\n"
        << "  --video-port <n>            Override GCS video UDP port\n"
        << "  --gazebo-topic <topic>      Override Gazebo camera image topic\n"
        << "  --video                     Enable best-effort GCS MJPEG debug streaming\n"
        << "  --no-video                  Disable GCS MJPEG debug streaming\n"
        << "  --no-telemetry              Disable GCS telemetry streaming\n"
        << "  --vision-smoke-count <n>    Read/process n vision frames and exit before MAVLink\n"
        << "  --mission-timeout-ms <n>    Override safety mission timeout\n"
        << "  -h, --help                  Show this help\n";
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
        } else if (arg == "--target" && i + 1 < argc) {
            options.target = argv[++i];
        } else if (arg == "--autopilot" && i + 1 < argc) {
            options.autopilot_uri = argv[++i];
        } else if (arg == "--vision" && i + 1 < argc) {
            options.vision = argv[++i];
        } else if (arg == "--gcs-ip" && i + 1 < argc) {
            options.gcs_ip_override = argv[++i];
        } else if (arg == "--telemetry-port" && i + 1 < argc) {
            options.telemetry_port_override = parseInt(argv[++i], 0);
        } else if (arg == "--video-port" && i + 1 < argc) {
            options.video_port_override = parseInt(argv[++i], 0);
        } else if (arg == "--gazebo-topic" && i + 1 < argc) {
            options.gazebo_topic_override = argv[++i];
        } else if (arg == "--video") {
            options.send_video = true;
            options.send_video_overridden = true;
        } else if (arg == "--no-video") {
            options.send_video = false;
            options.send_video_overridden = true;
        } else if (arg == "--no-telemetry") {
            options.send_telemetry = false;
        } else if (arg == "--vision-smoke-count" && i + 1 < argc) {
            options.vision_smoke_count = parseInt(argv[++i], 0);
        } else if (arg == "--mission-timeout-ms" && i + 1 < argc) {
            options.mission_timeout_ms = parseInt(argv[++i], 0);
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

std::uint16_t parsePort(const std::string& value, std::uint16_t fallback)
{
    const int parsed = parseInt(value, fallback);
    if (parsed <= 0 || parsed > 65535) {
        return fallback;
    }
    return static_cast<std::uint16_t>(parsed);
}

void applyAutopilotUri(RuntimeConfig& config, const std::string& uri)
{
    if (uri.rfind("udp://", 0) == 0) {
        const std::string rest = uri.substr(6);
        const auto colon = rest.rfind(':');
        config.endpoint.kind = TransportKind::Udp;
        config.endpoint.label = "sitl";
        if (colon == std::string::npos) {
            config.endpoint.listen_port = parsePort(rest, config.endpoint.listen_port);
            return;
        }
        config.endpoint.listen_host = rest.substr(0, colon);
        config.endpoint.listen_port =
            parsePort(rest.substr(colon + 1), config.endpoint.listen_port);
        return;
    }

    if (uri.rfind("serial://", 0) == 0) {
        std::string rest = uri.substr(9);
        while (!rest.empty() && rest.front() == '/') {
            if (rest.rfind("/dev/", 0) == 0) {
                break;
            }
            rest.erase(rest.begin());
        }
        const auto colon = rest.rfind(':');
        config.endpoint.kind = TransportKind::Serial;
        config.endpoint.label = "pixhawk1";
        if (colon == std::string::npos) {
            config.endpoint.serial_device = rest;
            return;
        }
        config.endpoint.serial_device = rest.substr(0, colon);
        config.endpoint.serial_baudrate =
            parseInt(rest.substr(colon + 1), config.endpoint.serial_baudrate);
        return;
    }

    throw std::runtime_error("unsupported --autopilot URI: " + uri);
}

void applyRuntimeOverlay(RuntimeConfig& config, const std::string& config_dir, const std::string& target)
{
    if (target.empty()) {
        return;
    }

    toml::table table;
    try {
        table = toml::parse_file(joinConfigPath(config_dir, "runtime." + target + ".toml"));
    } catch (const toml::parse_error&) {
        return;
    }

    if (const auto runtime = table["runtime"]) {
        config.vision_source = runtime["vision"].value_or(config.vision_source);
    }
    if (const auto transport = table["transport"]) {
        const std::string kind = transport["kind"].value_or(std::string(""));
        if (kind == "udp") {
            config.endpoint.kind = TransportKind::Udp;
        } else if (kind == "serial") {
            config.endpoint.kind = TransportKind::Serial;
        }
        config.endpoint.listen_host =
            transport["listen_host"].value_or(config.endpoint.listen_host);
        config.endpoint.listen_port = static_cast<std::uint16_t>(
            transport["listen_port"].value_or(static_cast<int>(config.endpoint.listen_port)));
    }
    if (const auto serial = table["serial"]) {
        config.endpoint.serial_device =
            serial["device"].value_or(config.endpoint.serial_device);
        config.endpoint.serial_baudrate =
            serial["baudrate"].value_or(config.endpoint.serial_baudrate);
    }
    if (const auto mavlink = table["mavlink"]) {
        config.setpoint_rate_hz =
            mavlink["setpoint_rate_hz"].value_or(config.setpoint_rate_hz);
    }
    if (const auto safety = table["safety"]) {
        if (const auto timeouts = safety["timeouts"]) {
            config.safety.line_lost_ms =
                timeouts["line_lost_ms"].value_or(config.safety.line_lost_ms);
            config.safety.pixhawk_heartbeat_lost_ms =
                timeouts["pixhawk_heartbeat_lost_ms"].value_or(config.safety.pixhawk_heartbeat_lost_ms);
            config.safety.mission_timeout_ms =
                timeouts["mission_timeout_ms"].value_or(config.safety.mission_timeout_ms);
        }
    }
    if (const auto vision = table["vision"]) {
        if (const auto source = vision["source"]) {
            config.vision.source.gazebo_topic =
                source["gazebo_topic"].value_or(config.vision.source.gazebo_topic);
            config.vision.source.read_timeout_ms =
                source["read_timeout_ms"].value_or(config.vision.source.read_timeout_ms);
        }
    }
    if (const auto debug_video = table["debug_video"]) {
        config.vision.debug_video.enabled =
            debug_video["enabled"].value_or(config.vision.debug_video.enabled);
        config.vision.debug_video.send_fps =
            debug_video["send_fps"].value_or(config.vision.debug_video.send_fps);
        config.vision.debug_video.jpeg_quality =
            debug_video["jpeg_quality"].value_or(config.vision.debug_video.jpeg_quality);
        config.vision.debug_video.chunk_pacing_us =
            debug_video["chunk_pacing_us"].value_or(config.vision.debug_video.chunk_pacing_us);
        config.vision.debug_video.send_width =
            debug_video["send_width"].value_or(config.vision.debug_video.send_width);
        config.vision.debug_video.send_height =
            debug_video["send_height"].value_or(config.vision.debug_video.send_height);
    }
    if (const auto line_follow = table["line_follow"]) {
        config.controller.forward_mps =
            line_follow["forward_mps"].value_or(config.controller.forward_mps);
        config.fake_center_error_m =
            line_follow["center_error_m"].value_or(config.fake_center_error_m);
        config.fake_line_angle_deg =
            line_follow["line_angle_deg"].value_or(config.fake_line_angle_deg);
        config.mission.line_follow_duration_s =
            line_follow["duration_s"].value_or(config.mission.line_follow_duration_s);
    }
    if (const auto marker_hover = table["marker_hover"]) {
        config.mission.marker_hover_s =
            marker_hover["hold_s"].value_or(config.mission.marker_hover_s);
        config.marker_center_tolerance_px =
            marker_hover["center_tolerance_px"].value_or(config.marker_center_tolerance_px);
    }
}

RuntimeConfig loadRuntimeConfig(const Options& options)
{
    RuntimeConfig config;
    config.network = onboard::common::loadNetworkConfig(options.config_dir);
    config.vision = onboard::common::loadVisionConfig(options.config_dir);

    try {
        const auto table = toml::parse_file(joinConfigPath(options.config_dir, "autopilot.toml"));
        if (const auto transport = table["transport"]) {
            const std::string kind = transport["kind"].value_or(std::string("udp"));
            config.endpoint.kind = kind == "serial" ? TransportKind::Serial : TransportKind::Udp;
            config.endpoint.listen_host =
                transport["listen_host"].value_or(config.endpoint.listen_host);
            config.endpoint.listen_port = static_cast<std::uint16_t>(
                transport["listen_port"].value_or(static_cast<int>(config.endpoint.listen_port)));
        }
        if (const auto serial = table["serial"]) {
            config.endpoint.serial_device =
                serial["device"].value_or(config.endpoint.serial_device);
            config.endpoint.serial_baudrate =
                serial["baudrate"].value_or(config.endpoint.serial_baudrate);
        }
        if (const auto mavlink = table["mavlink"]) {
            config.mavlink.system_id = static_cast<std::uint8_t>(
                mavlink["system_id"].value_or(static_cast<int>(config.mavlink.system_id)));
            config.mavlink.component_id = static_cast<std::uint8_t>(
                mavlink["component_id"].value_or(static_cast<int>(config.mavlink.component_id)));
            config.mavlink.target_system = static_cast<std::uint8_t>(
                mavlink["target_system"].value_or(static_cast<int>(config.mavlink.target_system)));
            config.mavlink.target_component = static_cast<std::uint8_t>(
                mavlink["target_component"].value_or(static_cast<int>(config.mavlink.target_component)));
            config.setpoint_rate_hz =
                mavlink["setpoint_rate_hz"].value_or(config.setpoint_rate_hz);
        }
    } catch (const toml::parse_error&) {
    }

    try {
        const auto table = toml::parse_file(joinConfigPath(options.config_dir, "mission.toml"));
        if (const auto takeoff = table["takeoff"]) {
            config.mission.target_altitude_m =
                takeoff["target_altitude_m"].value_or(config.mission.target_altitude_m);
        }
        if (const auto line_follow = table["line_follow"]) {
            config.controller.forward_mps =
                line_follow["forward_mps"].value_or(config.controller.forward_mps);
            config.fake_center_error_m =
                line_follow["center_error_m"].value_or(config.fake_center_error_m);
            config.fake_line_angle_deg =
                line_follow["line_angle_deg"].value_or(config.fake_line_angle_deg);
            config.mission.line_follow_duration_s =
                line_follow["duration_s"].value_or(config.mission.line_follow_duration_s);
        }
        if (const auto marker_hover = table["marker_hover"]) {
            config.mission.marker_hover_s =
                marker_hover["hold_s"].value_or(config.mission.marker_hover_s);
            config.marker_center_tolerance_px =
                marker_hover["center_tolerance_px"].value_or(config.marker_center_tolerance_px);
        }
    } catch (const toml::parse_error&) {
    }

    try {
        const auto table = toml::parse_file(joinConfigPath(options.config_dir, "safety.toml"));
        if (const auto timeouts = table["timeouts"]) {
            config.safety.line_lost_ms =
                timeouts["line_lost_ms"].value_or(config.safety.line_lost_ms);
            config.safety.pixhawk_heartbeat_lost_ms =
                timeouts["pixhawk_heartbeat_lost_ms"].value_or(config.safety.pixhawk_heartbeat_lost_ms);
            config.safety.mission_timeout_ms =
                timeouts["mission_timeout_ms"].value_or(config.safety.mission_timeout_ms);
        }
    } catch (const toml::parse_error&) {
    }

    if (options.target == "sitl") {
        config.endpoint.kind = TransportKind::Udp;
        config.endpoint.label = "sitl";
        config.vision_source = "fake";
    } else if (options.target == "pixhawk1") {
        config.endpoint.kind = TransportKind::Serial;
        config.endpoint.label = "pixhawk1";
        config.vision_source = "rpicam";
    } else if (!options.target.empty()) {
        throw std::runtime_error("unknown --target: " + options.target);
    }

    if (options.target.empty()) {
        config.vision_source = "fake";
    }

    applyRuntimeOverlay(config, options.config_dir, options.target);

    if (!options.vision.empty()) {
        config.vision_source = options.vision;
    }
    if (config.vision_source != "fake" &&
        config.vision_source != "gazebo" &&
        config.vision_source != "rpicam") {
        throw std::runtime_error("unknown --vision: " + config.vision_source);
    }

    if (options.mission_timeout_ms && *options.mission_timeout_ms > 0) {
        config.safety.mission_timeout_ms = *options.mission_timeout_ms;
    }

    if (!options.autopilot_uri.empty()) {
        applyAutopilotUri(config, options.autopilot_uri);
    }
    if (!options.gcs_ip_override.empty()) {
        config.network.gcs_ip = options.gcs_ip_override;
    }
    if (options.telemetry_port_override > 0 && options.telemetry_port_override <= 65535) {
        config.network.telemetry_port =
            static_cast<std::uint16_t>(options.telemetry_port_override);
    }
    if (options.video_port_override > 0 && options.video_port_override <= 65535) {
        config.network.video_port =
            static_cast<std::uint16_t>(options.video_port_override);
    }
    if (!options.gazebo_topic_override.empty()) {
        config.vision.source.gazebo_topic = options.gazebo_topic_override;
    }

    return config;
}

onboard::autopilot::BodyVelocityCommand toAutopilotCommand(
    const onboard::control::ControlSetpoint& setpoint)
{
    return onboard::autopilot::BodyVelocityCommand {
        setpoint.vx_forward_mps,
        setpoint.vy_right_mps,
        setpoint.vz_down_mps,
        setpoint.yaw_rate_rad_s,
    };
}

double degToRad(double degrees)
{
    constexpr double pi = 3.14159265358979323846;
    return degrees * pi / 180.0;
}

#if defined(ONBOARD_LINE_FOLLOW_HAS_VISION)
std::unique_ptr<onboard::vision::FrameSource> createFrameSource(const std::string& vision_source)
{
    if (vision_source == "fake") {
        return std::make_unique<onboard::vision::FakeFrameSource>();
    }
    if (vision_source == "gazebo") {
        return std::make_unique<onboard::vision::GazeboCameraSource>();
    }
    if (vision_source == "rpicam") {
        return std::make_unique<onboard::vision::RpicamFrameSource>();
    }
    throw std::runtime_error("unknown vision source: " + vision_source);
}

onboard::control::LineControlInput toLineControlInput(const onboard::vision::VisionResult& result)
{
    const double half_width = std::max(1.0, static_cast<double>(result.width) * 0.5);
    return onboard::control::LineControlInput {
        result.line.detected,
        result.line.center_offset_px / half_width,
        degToRad(result.line.angle_deg),
        result.line.confidence,
    };
}

bool markerCentered(const onboard::vision::VisionResult& result, double tolerance_px)
{
    if (result.markers.empty() || result.width <= 0 || result.height <= 0) {
        return false;
    }
    const auto& marker = result.markers.front();
    const double dx = marker.center_px.x - static_cast<double>(result.width) * 0.5;
    const double dy = marker.center_px.y - static_cast<double>(result.height) * 0.5;
    return std::hypot(dx, dy) <= tolerance_px;
}

onboard::vision::VisionResult readVisionResult(
    onboard::vision::FrameSource& source,
    onboard::vision::VisionProcessor& processor)
{
    onboard::vision::Frame frame;
    if (!source.read(frame)) {
        throw std::runtime_error("vision frame read failed: " + source.lastError());
    }
    const auto output = processor.process(
        frame.image_bgr,
        onboard::vision::VisionFrameMetadata {
            frame.frame_id,
            frame.timestamp_ms,
            frame.width,
            frame.height,
        });
    return output.result;
}

struct VisionFrameResult {
    onboard::vision::Frame frame;
    onboard::vision::VisionProcessingOutput output;
    double read_frame_ms = 0.0;
    double processing_latency_ms = 0.0;
};

VisionFrameResult readVisionFrameResult(
    onboard::vision::FrameSource& source,
    onboard::vision::VisionProcessor& processor)
{
    VisionFrameResult result;
    const auto read_started = Clock::now();
    if (!source.read(result.frame)) {
        throw std::runtime_error("vision frame read failed: " + source.lastError());
    }
    const auto read_finished = Clock::now();
    result.read_frame_ms = std::chrono::duration<double, std::milli>(
        read_finished - read_started).count();
    if (result.frame.width <= 0) {
        result.frame.width = result.frame.image_bgr.cols;
    }
    if (result.frame.height <= 0) {
        result.frame.height = result.frame.image_bgr.rows;
    }

    const auto processing_started = Clock::now();
    result.output = processor.process(
        result.frame.image_bgr,
        onboard::vision::VisionFrameMetadata {
            result.frame.frame_id,
            result.frame.timestamp_ms,
            result.frame.width,
            result.frame.height,
        });
    const auto processing_finished = Clock::now();
    result.processing_latency_ms = std::chrono::duration<double, std::milli>(
        processing_finished - processing_started).count();
    return result;
}

class RateMeter {
public:
    double note(std::chrono::steady_clock::time_point now)
    {
        if (window_start_.time_since_epoch().count() == 0) {
            window_start_ = now;
        }
        ++count_;
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - window_start_);
        if (elapsed.count() >= 1000) {
            rate_ = count_ * 1000.0 / std::max(1, static_cast<int>(elapsed.count()));
            count_ = 0;
            window_start_ = now;
        }
        return rate_;
    }

private:
    std::chrono::steady_clock::time_point window_start_ {};
    int count_ = 0;
    double rate_ = 0.0;
};

int runVisionSmoke(const RuntimeConfig& config, int count)
{
    auto source = createFrameSource(config.vision_source);
    if (!source->open(onboard::vision::FrameSourceOptions {config.vision})) {
        throw std::runtime_error("failed to open vision source: " + source->lastError());
    }

    onboard::vision::VisionProcessor processor(onboard::vision::VisionProcessorOptions {
        config.vision,
        true,
        true,
    });
    for (int index = 0; index < count; ++index) {
        const auto result = readVisionResult(*source, processor);
        std::cout << "[vision] frame=" << result.frame_seq
                  << " source=" << config.vision_source
                  << " size=" << result.width << 'x' << result.height
                  << " line=" << (result.line.detected ? "yes" : "no")
                  << " offset_px=" << result.line.center_offset_px
                  << " angle_deg=" << result.line.angle_deg
                  << " markers=" << result.markers.size();
        if (!result.markers.empty()) {
            std::cout << " marker_id=" << result.markers.front().id;
        }
        std::cout << "\n";
    }
    source->close();
    return 0;
}
#endif

} // namespace

int main(int argc, char** argv)
{
    try {
        const Options options = parseOptions(argc, argv);
        const RuntimeConfig config = loadRuntimeConfig(options);

#if !defined(ONBOARD_LINE_FOLLOW_HAS_VISION)
        if (config.vision_source != "fake" || options.vision_smoke_count > 0) {
            throw std::runtime_error(
                "this build does not include OpenCV vision sources; rebuild with BUILD_TOOLS=ON");
        }
#else
        if (options.vision_smoke_count > 0) {
            return runVisionSmoke(config, options.vision_smoke_count);
        }
#endif

        if (config.endpoint.kind == TransportKind::Serial) {
            std::cerr
                << "target " << config.endpoint.label
                << " selected serial endpoint " << config.endpoint.serial_device
                << ':' << config.endpoint.serial_baudrate << "\n"
                << "serial transport is intentionally not implemented in this step; "
                << "use --target sitl for the approved SITL smoke test\n";
            return 2;
        }

        std::cout << "[line_follow_node] target=" << config.endpoint.label
                  << " vision=" << config.vision_source
                  << " udp_port=" << config.endpoint.listen_port
                  << " gcs=" << config.network.gcs_ip << ':'
                  << config.network.telemetry_port << '/'
                  << config.network.video_port
                  << " takeoff_alt=" << config.mission.target_altitude_m
                  << " duration=" << config.mission.line_follow_duration_s
                  << " rate_hz=" << config.setpoint_rate_hz << "\n";

        auto transport = std::make_unique<onboard::autopilot::UdpMavlinkTransport>(
            config.endpoint.listen_port,
            MAVLINK_COMM_0,
            config.endpoint.label);
        onboard::autopilot::AutopilotMavlinkAdapter autopilot(
            std::move(transport),
            config.mavlink);
        onboard::control::GuidedVelocityController controller(config.controller);
        onboard::mission::LineFollowMission mission(config.mission);
        onboard::safety::SafetyMonitor safety(config.safety);

#if defined(ONBOARD_LINE_FOLLOW_HAS_VISION)
        std::unique_ptr<onboard::vision::FrameSource> frame_source;
        std::unique_ptr<onboard::vision::VisionProcessor> vision_processor;
        std::unique_ptr<onboard::app::VisionDebugPublisher> gcs_publisher;
        RateMeter capture_rate;
        RateMeter processing_rate;
        if (config.vision_source != "fake") {
            frame_source = createFrameSource(config.vision_source);
            if (!frame_source->open(onboard::vision::FrameSourceOptions {config.vision})) {
                throw std::runtime_error("failed to open vision source: " + frame_source->lastError());
            }
            vision_processor = std::make_unique<onboard::vision::VisionProcessor>(
                onboard::vision::VisionProcessorOptions {
                    config.vision,
                    true,
                    true,
                });
            std::cout << "[vision] source=" << config.vision_source << " opened\n";

            const bool send_video = options.send_video_overridden
                ? options.send_video
                : config.vision.debug_video.enabled;
            if (send_video || options.send_telemetry) {
                auto publisher = std::make_unique<onboard::app::VisionDebugPublisher>();
                onboard::app::VisionDebugPublisherOptions publisher_options;
                publisher_options.network = config.network;
                publisher_options.vision = config.vision;
                publisher_options.send_video = send_video;
                publisher_options.send_telemetry = options.send_telemetry;
                publisher_options.note = "line_follow_node source=" + config.vision_source;
                publisher_options.camera_sensor_model =
                    config.vision_source == "gazebo" ? "gazebo_downward_camera" : config.vision.camera.sensor_model;
                publisher_options.camera_index = config.vision.camera.device;
                if (!publisher->open(publisher_options)) {
                    throw std::runtime_error("failed to open GCS publisher: " + publisher->lastError());
                }
                gcs_publisher = std::move(publisher);
                std::cout << "[gcs] telemetry=" << (options.send_telemetry ? "on" : "off")
                          << " video=" << (send_video ? "on" : "off")
                          << " dest=" << config.network.gcs_ip << ':'
                          << config.network.telemetry_port << '/'
                          << config.network.video_port << "\n";
            }
        }
#endif

        std::cout << "[mavlink] waiting for heartbeat...\n";
        autopilot.waitHeartbeat(std::chrono::seconds(30));
        std::cout << "[mavlink] heartbeat ok system="
                  << static_cast<int>(autopilot.state().target_system)
                  << " component=" << static_cast<int>(autopilot.state().target_component)
                  << " mode=" << autopilot.state().mode_name << "\n";

        autopilot.requestDefaultStreams();
        autopilot.setGuidedMode(std::chrono::seconds(10));
        std::cout << "[mavlink] GUIDED confirmed\n";
        autopilot.arm(std::chrono::seconds(30));
        std::cout << "[mavlink] armed\n";

        const auto mission_started_at = Clock::now();
        mission.startTakeoff(mission_started_at);
        safety.startMission(mission_started_at);

        autopilot.takeoff(config.mission.target_altitude_m);
        std::cout << "[mission] TAKEOFF target=" << config.mission.target_altitude_m << "m\n";
        if (!autopilot.waitAltitudeReached(
                config.mission.target_altitude_m,
                config.mission.altitude_reached_ratio,
                std::chrono::seconds(30))) {
            throw std::runtime_error("timed out waiting for takeoff altitude");
        }

        const auto altitude = autopilot.bestAltitudeM();
        mission.update(onboard::mission::LineFollowMissionInput {
            altitude.has_value(),
            altitude.value_or(0.0),
            true,
            false,
            false,
            false,
            false,
            Clock::now(),
        });
        std::cout << "[mission] " << toString(mission.state()) << "\n";

        const auto period = std::chrono::duration<double>(
            1.0 / static_cast<double>(std::max(1, config.setpoint_rate_hz)));
        const auto line_angle_rad = degToRad(config.fake_line_angle_deg);
        const onboard::control::LineControlInput fake_line {
            true,
            config.fake_center_error_m,
            line_angle_rad,
            1.0,
        };

        while (mission.state() == onboard::mission::LineFollowMissionState::LineFollow ||
               mission.state() == onboard::mission::LineFollowMissionState::MarkerHover) {
            const auto loop_start = Clock::now();
            autopilot.poll(1);

            onboard::control::LineControlInput line_input = fake_line;
            bool marker_detected = false;
            bool marker_is_centered = false;
#if defined(ONBOARD_LINE_FOLLOW_HAS_VISION)
            if (frame_source && vision_processor) {
                try {
                    auto frame_result = readVisionFrameResult(*frame_source, *vision_processor);
                    const auto& result = frame_result.output.result;
                    const double capture_fps = capture_rate.note(Clock::now());
                    const double processing_fps = processing_rate.note(Clock::now());
                    line_input = toLineControlInput(result);
                    marker_detected = !result.markers.empty();
                    marker_is_centered = markerCentered(result, config.marker_center_tolerance_px);
                    if (gcs_publisher) {
                        onboard::app::VisionDebugPublishInput publish_input;
                        publish_input.frame = frame_result.frame;
                        publish_input.image_bgr = frame_result.frame.image_bgr;
                        publish_input.vision_output = frame_result.output;
                        publish_input.read_frame_ms = frame_result.read_frame_ms;
                        publish_input.processing_latency_ms = frame_result.processing_latency_ms;
                        publish_input.capture_fps = capture_fps;
                        publish_input.processing_fps = processing_fps;
                        const auto publish_stats = gcs_publisher->publish(std::move(publish_input));
                        const std::string publish_error = gcs_publisher->lastError();
                        if (!publish_error.empty()) {
                            std::cerr << "[gcs] warning: " << publish_error << "\n";
                        }
                        (void)publish_stats;
                    }
                    if (marker_detected) {
                        std::cout << "[vision] marker id=" << result.markers.front().id
                                  << " centered=" << (marker_is_centered ? "yes" : "no")
                                  << " line=" << (line_input.line_detected ? "yes" : "no")
                                  << " offset_px=" << result.line.center_offset_px << "\n";
                    }
                } catch (const std::exception& error) {
                    std::cerr << "[vision] warning: " << error.what() << "\n";
                    line_input = {};
                }
            }
#endif

            const auto safety_decision = safety.update(onboard::safety::SafetyInput {
                autopilot.state().heartbeat_seen,
                line_input.line_detected,
                loop_start,
                autopilot.state().last_heartbeat_time,
            });

            if (safety_decision.action == onboard::safety::SafetyAction::Abort) {
                mission.abort(safety_decision.reason);
                break;
            }

            const bool safety_land =
                safety_decision.action == onboard::safety::SafetyAction::Land;
            mission.update(onboard::mission::LineFollowMissionInput {
                true,
                autopilot.bestAltitudeM().value_or(0.0),
                line_input.line_detected,
                marker_detected,
                marker_is_centered,
                safety_land,
                false,
                loop_start,
            });
            if (mission.state() != onboard::mission::LineFollowMissionState::LineFollow &&
                mission.state() != onboard::mission::LineFollowMissionState::MarkerHover) {
                break;
            }

            if (mission.state() == onboard::mission::LineFollowMissionState::MarkerHover) {
                autopilot.sendBodyVelocity({});
            } else {
                autopilot.sendBodyVelocity(toAutopilotCommand(controller.update(line_input)));
            }
            std::this_thread::sleep_until(loop_start + period);
        }

        autopilot.sendBodyVelocity({});
        if (mission.state() == onboard::mission::LineFollowMissionState::Abort) {
            std::cerr << "[mission] ABORT reason=" << mission.landingReason() << "\n";
        } else {
            std::cout << "[mission] LAND reason=" << mission.landingReason() << "\n";
        }
        autopilot.setLandMode(std::chrono::seconds(10));
        autopilot.waitDisarmed(std::chrono::seconds(30));
        mission.markComplete();
        std::cout << "[mission] COMPLETE\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[error] " << error.what() << "\n";
        return 1;
    }
}
