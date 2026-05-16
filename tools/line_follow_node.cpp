#include "autopilot/AutopilotMavlinkAdapter.hpp"
#include "autopilot/SerialMavlinkTransport.hpp"
#include "autopilot/UdpMavlinkTransport.hpp"
#include "common/NetworkConfig.hpp"
#include "common/VisionConfig.hpp"
#include "control/GuidedVelocityController.hpp"
#include "mission/LineFollowMission.hpp"
#include "safety/SafetyMonitor.hpp"

#if defined(ONBOARD_LINE_FOLLOW_HAS_VISION)
#include "app/VisionDebugPublisher.hpp"
#include "vision/FakeFrameSource.hpp"
#include "vision/GazeboCameraSource.hpp"
#include "vision/RpicamFrameSource.hpp"
#include "vision/VisionProcessor.hpp"
#endif

#include <toml++/toml.hpp>

#include <atomic>
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
    double fake_center_error_norm = 0.0;
    double fake_line_angle_deg = 90.0;
    double desired_line_angle_deg = 90.0;
    double marker_center_tolerance_px = 80.0;
    std::string vision_source = "fake";
};

struct Options {
    std::string config_dir = "config";
    std::string target;
    std::string autopilot_uri;
    std::string vision;
    std::string line_mode_override;
    std::string gcs_ip_override;
    std::string gazebo_topic_override;
    std::optional<int> mission_timeout_ms;
    int telemetry_port_override = 0;
    int video_port_override = 0;
    int debug_video_fps_override = 0;
    bool debug_video_fps_specified = false;
    int vision_smoke_count = 0;
    int smoke_duration_ms = 10000;
    bool send_video = false;
    bool send_video_overridden = false;
    bool send_telemetry = true;
    bool mavlink_smoke = false;
    bool no_arm = false;
    bool dry_run = false;
    bool allow_arm_takeoff = false;
    bool unsafe_assume_rc_present = false;
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
        << "  --line-mode <mode>          auto, light_on_dark, or dark_on_light\n"
        << "  --gcs-ip <ip>               Override GCS telemetry/video destination IP\n"
        << "  --telemetry-port <n>        Override GCS telemetry UDP port\n"
        << "  --video-port <n>            Override GCS video UDP port\n"
        << "  --fps <n>                   Override GCS debug video send FPS\n"
        << "  --gazebo-topic <topic>      Override Gazebo camera image topic\n"
        << "  --video                     Enable best-effort GCS MJPEG debug streaming\n"
        << "  --no-video                  Disable GCS MJPEG debug streaming\n"
        << "  --no-telemetry              Disable GCS telemetry streaming\n"
        << "  --vision-smoke-count <n>    Read/process n vision frames and exit before MAVLink\n"
        << "  --mavlink-smoke             Heartbeat/stream smoke only; no mode/arm/takeoff\n"
        << "  --no-arm                    Real Pixhawk safety mode; no mode/arm/takeoff\n"
        << "  --dry-run                   Real Pixhawk safety mode; no command-producing mission\n"
        << "  --allow-arm-takeoff         Explicitly allow real serial arm/takeoff path\n"
        << "  --unsafe-assume-rc-present  Bypass MAVLink RC gate for real-flight debugging\n"
        << "  --smoke-duration-ms <n>     MAVLink smoke duration, default 10000\n"
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

bool isValidLineMode(const std::string& mode)
{
    return mode == "auto" ||
           mode == "light_on_dark" ||
           mode == "dark_on_light";
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
        } else if (arg == "--line-mode" && i + 1 < argc) {
            options.line_mode_override = argv[++i];
        } else if (arg == "--gcs-ip" && i + 1 < argc) {
            options.gcs_ip_override = argv[++i];
        } else if (arg == "--telemetry-port" && i + 1 < argc) {
            options.telemetry_port_override = parseInt(argv[++i], 0);
        } else if (arg == "--video-port" && i + 1 < argc) {
            options.video_port_override = parseInt(argv[++i], 0);
        } else if (arg == "--fps" && i + 1 < argc) {
            options.debug_video_fps_override = parseInt(argv[++i], 0);
            options.debug_video_fps_specified = true;
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
        } else if (arg == "--mavlink-smoke") {
            options.mavlink_smoke = true;
        } else if (arg == "--no-arm") {
            options.no_arm = true;
        } else if (arg == "--dry-run") {
            options.dry_run = true;
        } else if (arg == "--allow-arm-takeoff") {
            options.allow_arm_takeoff = true;
        } else if (arg == "--unsafe-assume-rc-present") {
            options.unsafe_assume_rc_present = true;
        } else if (arg == "--smoke-duration-ms" && i + 1 < argc) {
            options.smoke_duration_ms = parseInt(argv[++i], options.smoke_duration_ms);
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

double degToRad(double degrees);

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
        if (const auto rc = safety["rc"]) {
            config.safety.rc_required =
                rc["rc_required"].value_or(config.safety.rc_required);
            config.safety.assume_rc_present =
                rc["assume_rc_present"].value_or(config.safety.assume_rc_present);
            config.safety.rc_lost_ms =
                rc["rc_lost_ms"].value_or(config.safety.rc_lost_ms);
        }
    }
    if (const auto takeoff = table["takeoff"]) {
        config.mission.target_altitude_m =
            takeoff["target_altitude_m"].value_or(config.mission.target_altitude_m);
        config.mission.altitude_reached_ratio =
            takeoff["altitude_reached_ratio"].value_or(config.mission.altitude_reached_ratio);
        config.mission.takeoff_settle_s =
            takeoff["settle_s"].value_or(config.mission.takeoff_settle_s);
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
        config.fake_center_error_norm =
            line_follow["center_error_norm"].value_or(
                line_follow["center_error_m"].value_or(config.fake_center_error_norm));
        config.fake_line_angle_deg =
            line_follow["line_angle_deg"].value_or(config.fake_line_angle_deg);
        config.desired_line_angle_deg =
            line_follow["desired_angle_deg"].value_or(config.desired_line_angle_deg);
        config.mission.line_follow_duration_s =
            line_follow["duration_s"].value_or(config.mission.line_follow_duration_s);
        config.mission.line_lost_timeout_s =
            line_follow["lost_timeout_s"].value_or(config.mission.line_lost_timeout_s);
    }
    if (const auto altitude_hold = table["altitude_hold"]) {
        config.controller.altitude_kp =
            altitude_hold["kp"].value_or(config.controller.altitude_kp);
        config.controller.max_vz_down_mps =
            altitude_hold["max_vz_mps"].value_or(config.controller.max_vz_down_mps);
        config.controller.altitude_deadband_m =
            altitude_hold["deadband_m"].value_or(config.controller.altitude_deadband_m);
    }
    if (const auto line_controller = table["line_controller"]) {
        config.controller.offset_kp =
            line_controller["offset_kp"].value_or(config.controller.offset_kp);
        config.controller.angle_yaw_kp =
            line_controller["angle_yaw_kp"].value_or(config.controller.angle_yaw_kp);
        config.controller.offset_yaw_kp =
            line_controller["offset_yaw_kp"].value_or(config.controller.offset_yaw_kp);
        config.controller.max_lateral_mps =
            line_controller["max_lateral_mps"].value_or(config.controller.max_lateral_mps);
        config.controller.max_yaw_rate_rad_s =
            line_controller["max_yaw_rate_rad_s"].value_or(config.controller.max_yaw_rate_rad_s);
        config.controller.min_confidence =
            line_controller["min_confidence"].value_or(config.controller.min_confidence);
        config.controller.offset_deadband_norm =
            line_controller["offset_deadband_norm"].value_or(config.controller.offset_deadband_norm);
        const double angle_deadband_deg =
            line_controller["angle_deadband_deg"].value_or(
                config.controller.angle_deadband_rad * 180.0 / 3.14159265358979323846);
        config.controller.angle_deadband_rad = degToRad(angle_deadband_deg);
        config.controller.invert_lateral =
            line_controller["invert_lateral"].value_or(config.controller.invert_lateral);
        config.controller.invert_yaw =
            line_controller["invert_yaw"].value_or(config.controller.invert_yaw);
    }
    if (const auto marker_hover = table["marker_hover"]) {
        config.mission.marker_hover_s =
            marker_hover["hold_s"].value_or(config.mission.marker_hover_s);
        config.mission.marker_approach_timeout_s =
            marker_hover["approach_timeout_s"].value_or(config.mission.marker_approach_timeout_s);
        config.mission.marker_lost_timeout_s =
            marker_hover["lost_timeout_s"].value_or(config.mission.marker_lost_timeout_s);
        config.marker_center_tolerance_px =
            marker_hover["center_tolerance_px"].value_or(config.marker_center_tolerance_px);
        config.controller.marker_x_kp =
            marker_hover["center_x_kp"].value_or(
                marker_hover["center_kp"].value_or(config.controller.marker_x_kp));
        config.controller.marker_y_kp =
            marker_hover["center_y_kp"].value_or(
                marker_hover["center_kp"].value_or(config.controller.marker_y_kp));
        config.controller.max_marker_mps =
            marker_hover["max_lateral_mps"].value_or(
                marker_hover["max_marker_mps"].value_or(config.controller.max_marker_mps));
        config.controller.invert_marker_x =
            marker_hover["invert_x"].value_or(config.controller.invert_marker_x);
        config.controller.invert_marker_y =
            marker_hover["invert_y"].value_or(config.controller.invert_marker_y);
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
            config.mission.altitude_reached_ratio =
                takeoff["altitude_reached_ratio"].value_or(config.mission.altitude_reached_ratio);
            config.mission.takeoff_settle_s =
                takeoff["settle_s"].value_or(config.mission.takeoff_settle_s);
        }
        if (const auto line_follow = table["line_follow"]) {
            config.controller.forward_mps =
                line_follow["forward_mps"].value_or(config.controller.forward_mps);
            config.fake_center_error_norm =
                line_follow["center_error_norm"].value_or(
                    line_follow["center_error_m"].value_or(config.fake_center_error_norm));
            config.fake_line_angle_deg =
                line_follow["line_angle_deg"].value_or(config.fake_line_angle_deg);
            config.desired_line_angle_deg =
                line_follow["desired_angle_deg"].value_or(config.desired_line_angle_deg);
            config.mission.line_follow_duration_s =
                line_follow["duration_s"].value_or(config.mission.line_follow_duration_s);
            config.mission.line_lost_timeout_s =
                line_follow["lost_timeout_s"].value_or(config.mission.line_lost_timeout_s);
        }
        if (const auto altitude_hold = table["altitude_hold"]) {
            config.controller.altitude_kp =
                altitude_hold["kp"].value_or(config.controller.altitude_kp);
            config.controller.max_vz_down_mps =
                altitude_hold["max_vz_mps"].value_or(config.controller.max_vz_down_mps);
            config.controller.altitude_deadband_m =
                altitude_hold["deadband_m"].value_or(config.controller.altitude_deadband_m);
        }
        if (const auto line_controller = table["line_controller"]) {
            config.controller.offset_kp =
                line_controller["offset_kp"].value_or(config.controller.offset_kp);
            config.controller.angle_yaw_kp =
                line_controller["angle_yaw_kp"].value_or(config.controller.angle_yaw_kp);
            config.controller.offset_yaw_kp =
                line_controller["offset_yaw_kp"].value_or(config.controller.offset_yaw_kp);
            config.controller.max_lateral_mps =
                line_controller["max_lateral_mps"].value_or(config.controller.max_lateral_mps);
            config.controller.max_yaw_rate_rad_s =
                line_controller["max_yaw_rate_rad_s"].value_or(config.controller.max_yaw_rate_rad_s);
            config.controller.min_confidence =
                line_controller["min_confidence"].value_or(config.controller.min_confidence);
            config.controller.offset_deadband_norm =
                line_controller["offset_deadband_norm"].value_or(config.controller.offset_deadband_norm);
            const double angle_deadband_deg =
                line_controller["angle_deadband_deg"].value_or(
                    config.controller.angle_deadband_rad * 180.0 / 3.14159265358979323846);
            config.controller.angle_deadband_rad = degToRad(angle_deadband_deg);
            config.controller.invert_lateral =
                line_controller["invert_lateral"].value_or(config.controller.invert_lateral);
            config.controller.invert_yaw =
                line_controller["invert_yaw"].value_or(config.controller.invert_yaw);
        }
        if (const auto marker_hover = table["marker_hover"]) {
            config.mission.marker_hover_s =
                marker_hover["hold_s"].value_or(config.mission.marker_hover_s);
            config.mission.marker_approach_timeout_s =
                marker_hover["approach_timeout_s"].value_or(config.mission.marker_approach_timeout_s);
            config.mission.marker_lost_timeout_s =
                marker_hover["lost_timeout_s"].value_or(config.mission.marker_lost_timeout_s);
            config.marker_center_tolerance_px =
                marker_hover["center_tolerance_px"].value_or(config.marker_center_tolerance_px);
            config.controller.marker_x_kp =
                marker_hover["center_x_kp"].value_or(
                    marker_hover["center_kp"].value_or(config.controller.marker_x_kp));
            config.controller.marker_y_kp =
                marker_hover["center_y_kp"].value_or(
                    marker_hover["center_kp"].value_or(config.controller.marker_y_kp));
            config.controller.max_marker_mps =
                marker_hover["max_lateral_mps"].value_or(
                    marker_hover["max_marker_mps"].value_or(config.controller.max_marker_mps));
            config.controller.invert_marker_x =
                marker_hover["invert_x"].value_or(config.controller.invert_marker_x);
            config.controller.invert_marker_y =
                marker_hover["invert_y"].value_or(config.controller.invert_marker_y);
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
        if (const auto rc = table["rc"]) {
            config.safety.rc_required =
                rc["rc_required"].value_or(config.safety.rc_required);
            config.safety.assume_rc_present =
                rc["assume_rc_present"].value_or(config.safety.assume_rc_present);
            config.safety.rc_lost_ms =
                rc["rc_lost_ms"].value_or(config.safety.rc_lost_ms);
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
    if (options.debug_video_fps_specified) {
        if (options.debug_video_fps_override <= 0) {
            throw std::runtime_error("--fps must be positive");
        }
        config.vision.debug_video.send_fps = options.debug_video_fps_override;
        config.vision.debug_video.enabled = true;
    }
    if (!options.gazebo_topic_override.empty()) {
        config.vision.source.gazebo_topic = options.gazebo_topic_override;
    }
    if (!options.line_mode_override.empty()) {
        if (!isValidLineMode(options.line_mode_override)) {
            throw std::runtime_error("unknown --line-mode: " + options.line_mode_override);
        }
        config.vision.line.mode = options.line_mode_override;
    }
    if (options.unsafe_assume_rc_present) {
        config.safety.assume_rc_present = true;
        config.safety.rc_required = false;
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

double radToDeg(double radians)
{
    constexpr double pi = 3.14159265358979323846;
    return radians * 180.0 / pi;
}

double axialAngleDeltaDeg(double measured_deg, double desired_deg)
{
    double delta = measured_deg - desired_deg;
    while (delta > 90.0) {
        delta -= 180.0;
    }
    while (delta < -90.0) {
        delta += 180.0;
    }
    return delta;
}

bool rcInputFresh(
    const onboard::autopilot::AutopilotState& state,
    const onboard::safety::SafetyConfig& safety,
    Clock::time_point now)
{
    if (safety.assume_rc_present || !safety.rc_required) {
        return true;
    }
    if (!state.rc_channel_count || *state.rc_channel_count <= 0) {
        return false;
    }
    if (state.last_rc_channels_time.time_since_epoch().count() == 0) {
        return false;
    }
    const auto rc_age_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now - state.last_rc_channels_time)
            .count();
    return rc_age_ms <= safety.rc_lost_ms;
}

bool waitRcReady(
    onboard::autopilot::AutopilotMavlinkAdapter& autopilot,
    const onboard::safety::SafetyConfig& safety,
    std::chrono::seconds timeout)
{
    if (safety.assume_rc_present || !safety.rc_required) {
        std::cout << "[safety] RC gate assumed-present or optional\n";
        return true;
    }

    std::cout << "[safety] waiting for RC input via MAVLink RC_CHANNELS...\n";
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        autopilot.poll(200);
        if (rcInputFresh(autopilot.state(), safety, Clock::now())) {
            const auto& state = autopilot.state();
            std::cout << "[safety] RC input ready channels="
                      << state.rc_channel_count.value_or(0)
                      << " rssi=" << state.rc_rssi.value_or(-1) << "\n";
            return true;
        }
    }
    const auto& state = autopilot.state();
    std::cerr << "[safety] RC gate failed: channels="
              << state.rc_channel_count.value_or(0)
              << " rssi=" << state.rc_rssi.value_or(-1)
              << " required=true\n";
    return false;
}

bool nearGroundAndStable(const onboard::autopilot::AutopilotState& state)
{
    if (!state.distance_sensor_m || *state.distance_sensor_m > 0.30) {
        return false;
    }
    const double vx = std::abs(state.local_vx_mps.value_or(0.0));
    const double vy = std::abs(state.local_vy_mps.value_or(0.0));
    const double vz = std::abs(state.local_vz_mps.value_or(0.0));
    return vx < 0.35 && vy < 0.35 && vz < 0.35;
}

struct LocalHoldAnchor {
    double x_m = 0.0;
    double y_m = 0.0;
    std::optional<double> z_m;
};

std::optional<LocalHoldAnchor> captureLocalHoldAnchor(
    const onboard::autopilot::AutopilotState& state)
{
    if (!state.local_x_m || !state.local_y_m) {
        return std::nullopt;
    }
    return LocalHoldAnchor {
        *state.local_x_m,
        *state.local_y_m,
        state.local_z_m,
    };
}

void sendAnchoredVelocitySetpoint(
    onboard::autopilot::AutopilotMavlinkAdapter& autopilot,
    const LocalHoldAnchor& anchor,
    const onboard::control::ControlSetpoint& setpoint)
{
    autopilot.sendLocalNedPositionTarget(onboard::autopilot::LocalNedPositionTargetCommand {
        static_cast<float>(anchor.x_m),
        static_cast<float>(anchor.y_m),
        std::nullopt,
        std::optional<float> {setpoint.vz_down_mps},
        setpoint.yaw_rate_rad_s,
    });
}

void sendAnchoredDescentSetpoint(
    onboard::autopilot::AutopilotMavlinkAdapter& autopilot,
    const LocalHoldAnchor& anchor,
    float vz_down_mps)
{
    autopilot.sendLocalNedPositionTarget(onboard::autopilot::LocalNedPositionTargetCommand {
        static_cast<float>(anchor.x_m),
        static_cast<float>(anchor.y_m),
        std::nullopt,
        std::optional<float> {vz_down_mps},
        0.0f,
    });
}

bool lineTrackingActive(
    const onboard::control::LineControlInput& line,
    const onboard::control::GuidedVelocityControllerConfig& controller)
{
    return line.line_detected && line.confidence >= controller.min_confidence;
}

bool recent(Clock::time_point timestamp, Clock::time_point now, int max_age_ms)
{
    if (timestamp.time_since_epoch().count() == 0) {
        return false;
    }
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - timestamp).count();
    return age_ms >= 0 && age_ms <= max_age_ms;
}

bool ekfRelativeAidingReady(
    const onboard::autopilot::AutopilotState& state,
    Clock::time_point now)
{
    constexpr std::uint16_t kEkfAttitude = 1u << 0;
    constexpr std::uint16_t kEkfVelocityHoriz = 1u << 1;
    constexpr std::uint16_t kEkfPosHorizRel = 1u << 3;
    constexpr std::uint16_t kEkfPredPosHorizRel = 1u << 8;

    if (!state.ekf_flags || !recent(state.last_ekf_status_time, now, 1500)) {
        return false;
    }
    const auto flags = *state.ekf_flags;
    return (flags & kEkfAttitude) != 0 &&
           (flags & kEkfVelocityHoriz) != 0 &&
           ((flags & kEkfPosHorizRel) != 0 || (flags & kEkfPredPosHorizRel) != 0);
}

bool opticalFlowReady(
    const onboard::autopilot::AutopilotState& state,
    Clock::time_point now)
{
    if (!state.optical_flow_quality || !recent(state.last_optical_flow_time, now, 1500)) {
        return false;
    }
    return *state.optical_flow_quality >= 50;
}

bool localHoldEstimateReady(
    const onboard::autopilot::AutopilotState& state,
    Clock::time_point now)
{
    const bool local_position_ready =
        state.local_x_m.has_value() &&
        state.local_y_m.has_value() &&
        state.local_z_m.has_value() &&
        state.local_vx_mps.has_value() &&
        state.local_vy_mps.has_value() &&
        state.local_vz_mps.has_value();
    const bool range_ready =
        state.distance_sensor_m.has_value() &&
        *state.distance_sensor_m >= 0.03 &&
        *state.distance_sensor_m <= 8.0;
    return local_position_ready &&
           range_ready &&
           opticalFlowReady(state, now) &&
           ekfRelativeAidingReady(state, now);
}

void printLocalEstimateStatus(const onboard::autopilot::AutopilotState& state)
{
    std::cout << "[preflight] local_xy=(" << state.local_x_m.value_or(-999.0)
              << ',' << state.local_y_m.value_or(-999.0) << ')'
              << " z=" << state.local_z_m.value_or(-999.0)
              << " vel_ned=(" << state.local_vx_mps.value_or(-999.0)
              << ',' << state.local_vy_mps.value_or(-999.0)
              << ',' << state.local_vz_mps.value_or(-999.0) << ')'
              << " range=" << state.distance_sensor_m.value_or(-1.0)
              << " flow_q=" << state.optical_flow_quality.value_or(-1)
              << " flow_dist=" << state.optical_flow_ground_distance_m.value_or(-1.0)
              << " ekf_flags=0x" << std::hex << state.ekf_flags.value_or(0)
              << std::dec << "\n";
}

bool waitLocalHoldEstimateReady(
    onboard::autopilot::AutopilotMavlinkAdapter& autopilot,
    std::chrono::seconds timeout)
{
    std::cout << "[preflight] waiting for local XY hold estimate: local position, range, optical flow, EKF\n";
    const auto deadline = Clock::now() + timeout;
    auto ready_since = Clock::time_point {};
    auto last_log = Clock::now() - std::chrono::seconds(1);
    while (Clock::now() < deadline) {
        autopilot.poll(100);
        const auto now = Clock::now();
        const bool ready = localHoldEstimateReady(autopilot.state(), now);
        if (ready) {
            if (ready_since.time_since_epoch().count() == 0) {
                ready_since = now;
            }
            if (now - ready_since >= std::chrono::milliseconds(1500)) {
                printLocalEstimateStatus(autopilot.state());
                std::cout << "[preflight] local XY hold estimate ready\n";
                return true;
            }
        } else {
            ready_since = Clock::time_point {};
        }
        if (now - last_log >= std::chrono::seconds(1)) {
            printLocalEstimateStatus(autopilot.state());
            last_log = now;
        }
    }
    std::cerr << "[preflight] local XY hold estimate not ready before timeout\n";
    printLocalEstimateStatus(autopilot.state());
    return false;
}

bool waitTakeoffAltitudeWithHold(
    onboard::autopilot::AutopilotMavlinkAdapter& autopilot,
    const onboard::control::GuidedVelocityController& controller,
    const LocalHoldAnchor& anchor,
    double target_altitude_m,
    double ratio,
    std::chrono::seconds timeout,
    std::chrono::duration<double> period)
{
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        const auto loop_start = Clock::now();
        autopilot.poll(20);
        const auto altitude = autopilot.bestAltitudeM();
        const onboard::control::AltitudeControlInput altitude_input {
            altitude.has_value(),
            altitude.value_or(target_altitude_m),
            target_altitude_m,
        };
        sendAnchoredVelocitySetpoint(
            autopilot,
            anchor,
            controller.holdAltitude(altitude_input));
        if (altitude && *altitude >= target_altitude_m * ratio) {
            return true;
        }
        std::this_thread::sleep_until(loop_start + period);
    }
    return false;
}

void holdAnchoredAltitude(
    onboard::autopilot::AutopilotMavlinkAdapter& autopilot,
    const onboard::control::GuidedVelocityController& controller,
    const LocalHoldAnchor& anchor,
    double target_altitude_m,
    std::chrono::duration<double> duration,
    std::chrono::duration<double> period)
{
    const auto deadline = Clock::now() + duration;
    while (Clock::now() < deadline) {
        const auto loop_start = Clock::now();
        autopilot.poll(20);
        const auto altitude = autopilot.bestAltitudeM();
        const onboard::control::AltitudeControlInput altitude_input {
            altitude.has_value(),
            altitude.value_or(target_altitude_m),
            target_altitude_m,
        };
        sendAnchoredVelocitySetpoint(
            autopilot,
            anchor,
            controller.stop(altitude_input));
        std::this_thread::sleep_until(loop_start + period);
    }
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

onboard::control::LineControlInput toLineControlInput(
    const onboard::vision::VisionResult& result,
    double desired_line_angle_deg)
{
    const double half_width = std::max(1.0, static_cast<double>(result.width) * 0.5);
    return onboard::control::LineControlInput {
        result.line.detected,
        result.line.center_offset_px / half_width,
        degToRad(axialAngleDeltaDeg(result.line.angle_deg, desired_line_angle_deg)),
        result.line.confidence,
    };
}

std::optional<onboard::vision::MarkerObservation> selectTargetMarker(
    const onboard::vision::VisionResult& result)
{
    if (result.markers.empty()) {
        return std::nullopt;
    }
    for (const auto& marker : result.markers) {
        if (marker.id == 1) {
            return marker;
        }
    }
    return result.markers.front();
}

onboard::control::MarkerControlInput toMarkerControlInput(
    const onboard::vision::VisionResult& result)
{
    const auto marker = selectTargetMarker(result);
    if (!marker || result.width <= 0 || result.height <= 0) {
        return {};
    }
    const double half_width = std::max(1.0, static_cast<double>(result.width) * 0.5);
    const double half_height = std::max(1.0, static_cast<double>(result.height) * 0.5);
    return onboard::control::MarkerControlInput {
        true,
        (marker->center_px.x - static_cast<double>(result.width) * 0.5) / half_width,
        (marker->center_px.y - static_cast<double>(result.height) * 0.5) / half_height,
    };
}

bool markerCentered(const onboard::vision::VisionResult& result, double tolerance_px)
{
    const auto marker = selectTargetMarker(result);
    if (!marker || result.width <= 0 || result.height <= 0) {
        return false;
    }
    const double dx = marker->center_px.x - static_cast<double>(result.width) * 0.5;
    const double dy = marker->center_px.y - static_cast<double>(result.height) * 0.5;
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
    onboard::app::VisionDebugPublishStats publish_stats;
    double read_frame_ms = 0.0;
    double processing_latency_ms = 0.0;
    bool published = false;
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

VisionFrameResult readAndPublishVisionFrame(
    onboard::vision::FrameSource& source,
    onboard::vision::VisionProcessor& processor,
    onboard::app::VisionDebugPublisher* publisher,
    RateMeter& capture_rate,
    RateMeter& processing_rate)
{
    auto frame_result = readVisionFrameResult(source, processor);
    const double capture_fps = capture_rate.note(Clock::now());
    const double processing_fps = processing_rate.note(Clock::now());

    if (publisher) {
        onboard::app::VisionDebugPublishInput publish_input;
        publish_input.frame = frame_result.frame;
        publish_input.image_bgr = frame_result.frame.image_bgr;
        publish_input.vision_output = frame_result.output;
        publish_input.read_frame_ms = frame_result.read_frame_ms;
        publish_input.processing_latency_ms = frame_result.processing_latency_ms;
        publish_input.capture_fps = capture_fps;
        publish_input.processing_fps = processing_fps;
        const auto publish_stats = publisher->publish(std::move(publish_input));
        frame_result.publish_stats = publish_stats;
        frame_result.published = true;
        const std::string publish_error = publisher->lastError();
        if (!publish_error.empty()) {
            std::cerr << "[gcs] warning: " << publish_error << "\n";
        }
    }

    return frame_result;
}

class StartupVisionStreamer {
public:
    StartupVisionStreamer(
        onboard::vision::FrameSource& source,
        onboard::vision::VisionProcessor& processor,
        onboard::app::VisionDebugPublisher& publisher,
        RateMeter& capture_rate,
        RateMeter& processing_rate,
        std::chrono::milliseconds period)
        : source_(source)
        , processor_(processor)
        , publisher_(publisher)
        , capture_rate_(capture_rate)
        , processing_rate_(processing_rate)
        , period_(period)
    {
    }

    ~StartupVisionStreamer()
    {
        stop();
    }

    StartupVisionStreamer(const StartupVisionStreamer&) = delete;
    StartupVisionStreamer& operator=(const StartupVisionStreamer&) = delete;

    void start()
    {
        if (worker_.joinable()) {
            return;
        }
        running_ = true;
        worker_ = std::thread([this] { run(); });
    }

    void stop()
    {
        running_ = false;
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    int frames() const
    {
        return frames_.load();
    }

private:
    void run()
    {
        while (running_) {
            const auto loop_start = Clock::now();
            try {
                (void)readAndPublishVisionFrame(
                    source_,
                    processor_,
                    &publisher_,
                    capture_rate_,
                    processing_rate_);
                ++frames_;
            } catch (const std::exception& error) {
                std::cerr << "[vision] startup warning: " << error.what() << "\n";
            }
            std::this_thread::sleep_until(loop_start + period_);
        }
    }

    onboard::vision::FrameSource& source_;
    onboard::vision::VisionProcessor& processor_;
    onboard::app::VisionDebugPublisher& publisher_;
    RateMeter& capture_rate_;
    RateMeter& processing_rate_;
    std::chrono::milliseconds period_;
    std::atomic_bool running_ {false};
    std::atomic<int> frames_ {0};
    std::thread worker_;
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

        const bool real_serial_target = config.endpoint.kind == TransportKind::Serial;
        if (real_serial_target &&
            !options.mavlink_smoke &&
            !options.no_arm &&
            !options.dry_run &&
            !options.allow_arm_takeoff) {
            std::cerr << "target " << config.endpoint.label
                      << " selected real serial endpoint " << config.endpoint.serial_device
                      << ':' << config.endpoint.serial_baudrate << "\n"
                      << "refusing to run the automatic arm/takeoff mission on real Pixhawk without "
                      << "--mavlink-smoke, --no-arm, --dry-run, or --allow-arm-takeoff\n";
            return 2;
        }

        std::cout << "[line_follow_node] target=" << config.endpoint.label
                  << " vision=" << config.vision_source
                  << " endpoint="
                  << (config.endpoint.kind == TransportKind::Serial
                          ? config.endpoint.serial_device + ":" + std::to_string(config.endpoint.serial_baudrate)
                          : "udp:" + std::to_string(config.endpoint.listen_port))
                  << " gcs=" << config.network.gcs_ip << ':'
                  << config.network.telemetry_port << '/'
                  << config.network.video_port
                  << " takeoff_alt=" << config.mission.target_altitude_m
                  << " takeoff_settle=" << config.mission.takeoff_settle_s
                  << " duration=" << config.mission.line_follow_duration_s
                  << " line_lost_timeout=" << config.mission.line_lost_timeout_s
                  << " desired_angle=" << config.desired_line_angle_deg
                  << " line_mode=" << config.vision.line.mode
                  << " video_fps=" << config.vision.debug_video.send_fps
                  << " rc_required=" << (config.safety.rc_required ? "true" : "false")
                  << " assume_rc=" << (config.safety.assume_rc_present ? "true" : "false")
                  << " rate_hz=" << config.setpoint_rate_hz << "\n";
        if (options.unsafe_assume_rc_present) {
            std::cerr
                << "[safety] WARNING: --unsafe-assume-rc-present bypasses the MAVLink RC gate\n";
        }

        std::unique_ptr<onboard::autopilot::MavlinkTransport> transport;
        if (config.endpoint.kind == TransportKind::Serial) {
            transport = std::make_unique<onboard::autopilot::SerialMavlinkTransport>(
                config.endpoint.serial_device,
                config.endpoint.serial_baudrate,
                MAVLINK_COMM_0,
                config.endpoint.label);
        } else {
            transport = std::make_unique<onboard::autopilot::UdpMavlinkTransport>(
                config.endpoint.listen_port,
                MAVLINK_COMM_0,
                config.endpoint.label);
        }
        onboard::autopilot::AutopilotMavlinkAdapter autopilot(
            std::move(transport),
            config.mavlink);
        onboard::control::GuidedVelocityController controller(config.controller);
        onboard::mission::LineFollowMission mission(config.mission);
        onboard::safety::SafetyMonitor safety(config.safety);

        if (options.mavlink_smoke || options.no_arm || options.dry_run) {
            std::cout << "[mavlink] safety smoke mode: no mode/arm/takeoff/velocity commands\n";
            std::cout << "[mavlink] waiting for heartbeat...\n";
            autopilot.waitHeartbeat(std::chrono::seconds(30));
            std::cout << "[mavlink] heartbeat ok system="
                      << static_cast<int>(autopilot.state().target_system)
                      << " component=" << static_cast<int>(autopilot.state().target_component)
                      << " mode=" << autopilot.state().mode_name
                      << " armed=" << (autopilot.state().armed ? "true" : "false") << "\n";
            autopilot.requestDefaultStreams();
            const auto smoke_deadline =
                Clock::now() + std::chrono::milliseconds(std::max(1000, options.smoke_duration_ms));
            auto last_smoke_log = Clock::now() - std::chrono::seconds(1);
            while (Clock::now() < smoke_deadline) {
                autopilot.poll(200);
                const auto now = Clock::now();
                if (now - last_smoke_log >= std::chrono::seconds(1)) {
                    const auto& ap_state = autopilot.state();
                    std::cout << "[mavlink-smoke] mode=" << ap_state.mode_name
                              << " armed=" << (ap_state.armed ? "true" : "false")
                              << " alt=" << autopilot.bestAltitudeM().value_or(-1.0)
                              << " local_xy=(" << ap_state.local_x_m.value_or(-999.0)
                              << ',' << ap_state.local_y_m.value_or(-999.0) << ')'
                              << " local_z=" << ap_state.local_z_m.value_or(-999.0)
                              << " vel_ned=(" << ap_state.local_vx_mps.value_or(-999.0)
                              << ',' << ap_state.local_vy_mps.value_or(-999.0)
                              << ',' << ap_state.local_vz_mps.value_or(-999.0) << ')'
                              << " range=" << ap_state.distance_sensor_m.value_or(-1.0)
                              << " flow_q=" << ap_state.optical_flow_quality.value_or(-1)
                              << " ekf_flags=0x" << std::hex
                              << ap_state.ekf_flags.value_or(0) << std::dec
                              << " rc_channels=" << ap_state.rc_channel_count.value_or(0)
                              << " rc_rssi=" << ap_state.rc_rssi.value_or(-1)
                              << "\n";
                    last_smoke_log = now;
                }
            }
            return 0;
        }

#if defined(ONBOARD_LINE_FOLLOW_HAS_VISION)
        std::unique_ptr<onboard::vision::FrameSource> frame_source;
        std::unique_ptr<onboard::vision::VisionProcessor> vision_processor;
        std::unique_ptr<onboard::app::VisionDebugPublisher> gcs_publisher;
        RateMeter capture_rate;
        RateMeter processing_rate;
        std::unique_ptr<StartupVisionStreamer> startup_vision_streamer;
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
                if (send_video) {
                    const int startup_fps = std::max(1, config.vision.camera.fps);
                    startup_vision_streamer = std::make_unique<StartupVisionStreamer>(
                        *frame_source,
                        *vision_processor,
                        *gcs_publisher,
                        capture_rate,
                        processing_rate,
                        std::chrono::milliseconds(1000 / startup_fps));
                    startup_vision_streamer->start();
                    std::cout << "[gcs] startup video streaming until line-follow control starts\n";
                }
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
        if (!waitRcReady(autopilot, config.safety, std::chrono::seconds(10))) {
            std::cerr << "[safety] refusing GUIDED/arm/takeoff until RC input is visible\n";
            return 2;
        }
        if (real_serial_target &&
            !waitLocalHoldEstimateReady(autopilot, std::chrono::seconds(15))) {
            std::cerr << "[safety] refusing GUIDED/arm/takeoff until optical-flow local hold is ready\n";
            return 2;
        }
        const auto takeoff_anchor = captureLocalHoldAnchor(autopilot.state());
        if (real_serial_target && !takeoff_anchor) {
            std::cerr << "[safety] refusing takeoff: local XY anchor is unavailable\n";
            return 2;
        }
        autopilot.setGuidedMode(std::chrono::seconds(10));
        std::cout << "[mavlink] GUIDED confirmed\n";
        autopilot.arm(std::chrono::seconds(30));
        std::cout << "[mavlink] armed\n";

        const auto period = std::chrono::duration<double>(
            1.0 / static_cast<double>(std::max(1, config.setpoint_rate_hz)));
        const auto mission_started_at = Clock::now();
        mission.startTakeoff(mission_started_at);
        safety.startMission(mission_started_at);

        autopilot.takeoff(config.mission.target_altitude_m);
        std::cout << "[mission] TAKEOFF target=" << config.mission.target_altitude_m << "m";
        if (takeoff_anchor) {
            std::cout << " hold_xy=(" << takeoff_anchor->x_m << ',' << takeoff_anchor->y_m << ')';
        }
        std::cout << "\n";
        const bool takeoff_altitude_reached = takeoff_anchor
            ? waitTakeoffAltitudeWithHold(
                  autopilot,
                  controller,
                  *takeoff_anchor,
                  config.mission.target_altitude_m,
                  config.mission.altitude_reached_ratio,
                  std::chrono::seconds(30),
                  period)
            : autopilot.waitAltitudeReached(
                  config.mission.target_altitude_m,
                  config.mission.altitude_reached_ratio,
                  std::chrono::seconds(30));
        if (!takeoff_altitude_reached) {
            throw std::runtime_error("timed out waiting for takeoff altitude");
        }
        if (takeoff_anchor && config.mission.takeoff_settle_s > 0.0) {
            std::cout << "[mission] TAKEOFF altitude reached; holding XY settle_s="
                      << config.mission.takeoff_settle_s << "\n";
            holdAnchoredAltitude(
                autopilot,
                controller,
                *takeoff_anchor,
                config.mission.target_altitude_m,
                std::chrono::duration<double>(config.mission.takeoff_settle_s),
                period);
        }

#if defined(ONBOARD_LINE_FOLLOW_HAS_VISION)
        if (startup_vision_streamer) {
            startup_vision_streamer->stop();
            std::cout << "[gcs] startup_video_frames=" << startup_vision_streamer->frames() << "\n";
        }
#endif

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

        const auto fake_angle_error_rad =
            degToRad(axialAngleDeltaDeg(
                config.fake_line_angle_deg,
                config.desired_line_angle_deg));
        const onboard::control::LineControlInput fake_line {
            true,
            config.fake_center_error_norm,
            fake_angle_error_rad,
            1.0,
        };
        const onboard::control::MarkerControlInput fake_marker {};
        auto last_control_log = Clock::now();
        auto last_state = mission.state();
        bool operator_takeover = false;
        std::optional<LocalHoldAnchor> active_hold_anchor;
        bool hold_anchor_warning_logged = false;
#if defined(ONBOARD_LINE_FOLLOW_HAS_VISION)
        int mission_vision_frames = 0;
        onboard::app::VisionDebugPublishStats mission_publish_stats;
#endif

        while (mission.state() == onboard::mission::LineFollowMissionState::LineFollow ||
               mission.state() == onboard::mission::LineFollowMissionState::MarkerApproach ||
               mission.state() == onboard::mission::LineFollowMissionState::MarkerHover) {
            const auto loop_start = Clock::now();
            autopilot.poll(1);

            onboard::control::LineControlInput line_input = fake_line;
            onboard::control::MarkerControlInput marker_input = fake_marker;
            bool marker_detected = false;
            bool marker_is_centered = false;
#if defined(ONBOARD_LINE_FOLLOW_HAS_VISION)
            if (frame_source && vision_processor) {
                try {
                    auto frame_result = readAndPublishVisionFrame(
                        *frame_source,
                        *vision_processor,
                        gcs_publisher.get(),
                        capture_rate,
                        processing_rate);
                    const auto& result = frame_result.output.result;
                    ++mission_vision_frames;
                    if (frame_result.published) {
                        mission_publish_stats = frame_result.publish_stats;
                    }
                    line_input = toLineControlInput(result, config.desired_line_angle_deg);
                    marker_input = toMarkerControlInput(result);
                    marker_detected = marker_input.marker_detected;
                    marker_is_centered = markerCentered(result, config.marker_center_tolerance_px);
                    if (marker_detected) {
                        const auto marker = selectTargetMarker(result);
                        std::cout << "[vision] marker id=" << (marker ? marker->id : -1)
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

            const bool line_tracking_ready = lineTrackingActive(line_input, config.controller);
            const auto& ap_state_for_safety = autopilot.state();
            const auto safety_decision = safety.update(onboard::safety::SafetyInput {
                ap_state_for_safety.heartbeat_seen,
                line_tracking_ready || marker_detected,
                loop_start,
                ap_state_for_safety.last_heartbeat_time,
                ap_state_for_safety.rc_channel_count.has_value(),
                ap_state_for_safety.rc_channel_count.value_or(0),
                ap_state_for_safety.last_rc_channels_time,
                ap_state_for_safety.heartbeat_seen,
                ap_state_for_safety.custom_mode == COPTER_MODE_GUIDED,
            });

            if (safety_decision.action == onboard::safety::SafetyAction::Abort) {
                operator_takeover =
                    safety_decision.reason.find("operator takeover") != std::string::npos;
                mission.abort(safety_decision.reason);
                break;
            }

            const bool safety_land =
                safety_decision.action == onboard::safety::SafetyAction::Land;
            const auto altitude_now = autopilot.bestAltitudeM();
            mission.update(onboard::mission::LineFollowMissionInput {
                altitude_now.has_value(),
                altitude_now.value_or(0.0),
                line_tracking_ready,
                marker_detected,
                marker_is_centered,
                safety_land,
                false,
                loop_start,
            });
            if (mission.state() != last_state) {
                std::cout << "[mission] " << toString(mission.state()) << "\n";
                last_state = mission.state();
            }
            if (mission.state() != onboard::mission::LineFollowMissionState::LineFollow &&
                mission.state() != onboard::mission::LineFollowMissionState::MarkerApproach &&
                mission.state() != onboard::mission::LineFollowMissionState::MarkerHover) {
                break;
            }

            const onboard::control::AltitudeControlInput altitude_input {
                altitude_now.has_value(),
                altitude_now.value_or(config.mission.target_altitude_m),
                config.mission.target_altitude_m,
            };
            onboard::control::ControlSetpoint setpoint;
            bool use_local_hold = false;
            std::string hold_reason;
            if (mission.state() == onboard::mission::LineFollowMissionState::MarkerHover) {
                setpoint = controller.stop(altitude_input);
                use_local_hold = true;
                hold_reason = "marker_hover";
            } else if (mission.state() == onboard::mission::LineFollowMissionState::MarkerApproach) {
                if (marker_detected) {
                    setpoint = controller.updateMarker(marker_input, altitude_input);
                    active_hold_anchor.reset();
                } else if (line_tracking_ready) {
                    setpoint = controller.updateLine(line_input, altitude_input);
                    active_hold_anchor.reset();
                } else {
                    setpoint = controller.stop(altitude_input);
                    use_local_hold = true;
                    hold_reason = "marker_approach_no_target";
                }
            } else {
                if (line_tracking_ready) {
                    setpoint = controller.updateLine(line_input, altitude_input);
                    active_hold_anchor.reset();
                } else {
                    setpoint = controller.stop(altitude_input);
                    use_local_hold = true;
                    hold_reason = line_input.line_detected ? "line_low_confidence" : "line_lost";
                }
            }
            std::string command_frame = "body_vel";
            if (use_local_hold) {
                if (!active_hold_anchor) {
                    active_hold_anchor = captureLocalHoldAnchor(autopilot.state());
                    if (active_hold_anchor) {
                        std::cout << "[control] hold_anchor reason=" << hold_reason
                                  << " xy=(" << active_hold_anchor->x_m
                                  << ',' << active_hold_anchor->y_m << ")\n";
                    } else if (!hold_anchor_warning_logged) {
                        std::cerr << "[control] warning: local hold requested but local XY is unavailable; using body zero velocity\n";
                        hold_anchor_warning_logged = true;
                    }
                }
                if (active_hold_anchor) {
                    sendAnchoredVelocitySetpoint(autopilot, *active_hold_anchor, setpoint);
                    command_frame = "local_hold";
                } else {
                    autopilot.sendBodyVelocity(toAutopilotCommand(setpoint));
                }
            } else {
                autopilot.sendBodyVelocity(toAutopilotCommand(setpoint));
            }

            if (loop_start - last_control_log >= std::chrono::seconds(1)) {
                const auto& ap_state = autopilot.state();
                std::cout << "[control] state=" << toString(mission.state())
                          << " mode=" << ap_state.mode_name
                          << " alt=" << altitude_now.value_or(-1.0)
                          << " local_xy=(" << ap_state.local_x_m.value_or(-999.0)
                          << ',' << ap_state.local_y_m.value_or(-999.0) << ')'
                          << " vel_ned=(" << ap_state.local_vx_mps.value_or(-999.0)
                          << ',' << ap_state.local_vy_mps.value_or(-999.0)
                          << ',' << ap_state.local_vz_mps.value_or(-999.0) << ')'
                          << " line=" << (line_input.line_detected ? "yes" : "no")
                          << " line_conf=" << line_input.confidence
                          << " off_norm=" << line_input.center_error_norm
                          << " angle_err_deg=" << radToDeg(line_input.angle_error_rad)
                          << " marker=" << (marker_detected ? "yes" : "no")
                          << " marker_x_norm=" << marker_input.center_error_x_norm
                          << " marker_y_norm=" << marker_input.center_error_y_norm
                          << " marker_centered=" << (marker_is_centered ? "yes" : "no")
                          << " vx=" << setpoint.vx_forward_mps
                          << " vy=" << setpoint.vy_right_mps
                          << " vz_down=" << setpoint.vz_down_mps
                          << " yaw_rate=" << setpoint.yaw_rate_rad_s
                          << " cmd=" << command_frame;
                if (active_hold_anchor) {
                    std::cout << " hold_xy=(" << active_hold_anchor->x_m
                              << ',' << active_hold_anchor->y_m << ')';
                }
#if defined(ONBOARD_LINE_FOLLOW_HAS_VISION)
                if (gcs_publisher) {
                    std::cout << " vision_frames=" << mission_vision_frames
                              << " video_sent=" << mission_publish_stats.video_sent_frames
                              << " video_drop=" << mission_publish_stats.video_dropped_frames
                              << " video_skip=" << mission_publish_stats.video_skipped_frames
                              << " video_fail=" << mission_publish_stats.video_send_failures;
                }
#endif
                std::cout << "\n";
                last_control_log = loop_start;
            }
            std::this_thread::sleep_until(loop_start + period);
        }

        if (operator_takeover) {
            std::cerr << "[mission] ABORT reason=" << mission.landingReason() << "\n";
            std::cerr << "[safety] operator takeover detected; not sending LAND or velocity commands\n";
            return 2;
        }

#if defined(ONBOARD_LINE_FOLLOW_HAS_VISION)
        if (gcs_publisher) {
            std::cout << "[gcs] mission_vision_frames=" << mission_vision_frames
                      << " video_sent=" << mission_publish_stats.video_sent_frames
                      << " video_drop=" << mission_publish_stats.video_dropped_frames
                      << " video_skip=" << mission_publish_stats.video_skipped_frames
                      << " video_fail=" << mission_publish_stats.video_send_failures << "\n";
        }
#endif

        auto landing_anchor = captureLocalHoldAnchor(autopilot.state());
        if (!landing_anchor && active_hold_anchor) {
            landing_anchor = active_hold_anchor;
        }
        const onboard::control::AltitudeControlInput landing_altitude_input {
            autopilot.bestAltitudeM().has_value(),
            autopilot.bestAltitudeM().value_or(config.mission.target_altitude_m),
            config.mission.target_altitude_m,
        };
        const auto stop_setpoint = controller.stop(landing_altitude_input);
        if (landing_anchor) {
            sendAnchoredVelocitySetpoint(autopilot, *landing_anchor, stop_setpoint);
        } else {
            autopilot.sendBodyVelocity(toAutopilotCommand(stop_setpoint));
        }
        if (mission.state() == onboard::mission::LineFollowMissionState::Abort) {
            std::cerr << "[mission] ABORT reason=" << mission.landingReason() << "\n";
        } else {
            std::cout << "[mission] LAND reason=" << mission.landingReason() << "\n";
        }
        if (landing_anchor) {
            std::cout << "[mission] GUIDED_DESCENT hold_xy=("
                      << landing_anchor->x_m << ',' << landing_anchor->y_m << ")\n";
        } else {
            std::cerr << "[mission] local XY unavailable; falling back to LAND mode\n";
            autopilot.setLandMode(std::chrono::seconds(10));
        }
        const auto disarm_deadline = Clock::now() + std::chrono::seconds(60);
        bool disarmed = false;
        bool disarm_requested = false;
        auto near_ground_since = Clock::time_point {};
        int landing_video_frames = 0;
#if defined(ONBOARD_LINE_FOLLOW_HAS_VISION)
        onboard::app::VisionDebugPublishStats landing_publish_stats;
#endif
        while (Clock::now() < disarm_deadline) {
            const auto loop_start = Clock::now();
            autopilot.poll(1);
#if defined(ONBOARD_LINE_FOLLOW_HAS_VISION)
            if (frame_source && vision_processor && gcs_publisher) {
                try {
                    auto frame_result = readAndPublishVisionFrame(
                        *frame_source,
                        *vision_processor,
                        gcs_publisher.get(),
                        capture_rate,
                        processing_rate);
                    if (frame_result.published) {
                        landing_publish_stats = frame_result.publish_stats;
                    }
                    ++landing_video_frames;
                } catch (const std::exception& error) {
                    std::cerr << "[vision] landing warning: " << error.what() << "\n";
                }
            }
#endif
            if (autopilot.state().heartbeat_seen && !autopilot.state().armed) {
                disarmed = true;
                break;
            }
            if (nearGroundAndStable(autopilot.state())) {
                if (landing_anchor) {
                    sendAnchoredDescentSetpoint(autopilot, *landing_anchor, 0.0f);
                }
                if (near_ground_since.time_since_epoch().count() == 0) {
                    near_ground_since = loop_start;
                }
                const auto near_ground_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        loop_start - near_ground_since)
                        .count();
                if (!disarm_requested && near_ground_ms >= 2000) {
                    std::cout << "[mission] landed estimate range="
                              << autopilot.state().distance_sensor_m.value_or(-1.0)
                              << "m; sending disarm\n";
                    disarm_requested = true;
                    try {
                        autopilot.disarm(std::chrono::seconds(10));
                        disarmed = true;
                        break;
                    } catch (const std::exception& error) {
                        std::cerr << "[safety] disarm retry failed: "
                                  << error.what() << "\n";
                    }
                }
            } else {
                near_ground_since = Clock::time_point {};
            }
            if (landing_anchor) {
                float descent_mps = 0.25f;
                const auto range = autopilot.state().distance_sensor_m;
                if (range && *range < 0.8) {
                    descent_mps = 0.15f;
                }
                if (range && *range < 0.45) {
                    descent_mps = 0.10f;
                }
                sendAnchoredDescentSetpoint(autopilot, *landing_anchor, descent_mps);
            }
            std::this_thread::sleep_until(loop_start + period);
        }
        if (!disarmed && landing_anchor) {
            std::cerr << "[mission] guided landing timeout; falling back to LAND mode\n";
            autopilot.setLandMode(std::chrono::seconds(10));
            const auto land_mode_deadline = Clock::now() + std::chrono::seconds(30);
            while (Clock::now() < land_mode_deadline) {
                autopilot.poll(200);
                if (autopilot.state().heartbeat_seen && !autopilot.state().armed) {
                    disarmed = true;
                    break;
                }
                if (nearGroundAndStable(autopilot.state())) {
                    std::cout << "[mission] landed estimate after LAND fallback range="
                              << autopilot.state().distance_sensor_m.value_or(-1.0)
                              << "m; sending disarm\n";
                    autopilot.disarm(std::chrono::seconds(10));
                    disarmed = true;
                    break;
                }
            }
        }
        if (!disarmed) {
            if (nearGroundAndStable(autopilot.state())) {
                std::cout << "[mission] final near-ground disarm attempt\n";
                autopilot.disarm(std::chrono::seconds(10));
                disarmed = true;
            } else {
                throw std::runtime_error("timed out waiting for disarmed state");
            }
        }
#if defined(ONBOARD_LINE_FOLLOW_HAS_VISION)
        if (gcs_publisher) {
            std::cout << "[gcs] landing_video_frames=" << landing_video_frames
                      << " video_sent=" << landing_publish_stats.video_sent_frames
                      << " video_drop=" << landing_publish_stats.video_dropped_frames
                      << " video_skip=" << landing_publish_stats.video_skipped_frames
                      << " video_fail=" << landing_publish_stats.video_send_failures << "\n";
        }
#endif
        mission.markComplete();
        std::cout << "[mission] COMPLETE\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[error] " << error.what() << "\n";
        return 1;
    }
}
