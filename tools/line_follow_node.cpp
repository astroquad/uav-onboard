#include "autopilot/AutopilotMavlinkAdapter.hpp"
#include "autopilot/SerialMavlinkTransport.hpp"
#include "autopilot/UdpMavlinkTransport.hpp"
#include "common/NetworkConfig.hpp"
#include "common/VisionConfig.hpp"
#include "control/GuidedVelocityController.hpp"
#include "mission/LineFollowMission.hpp"
#include "safety/SafetyMonitor.hpp"

#if defined(ONBOARD_LINE_FOLLOW_HAS_VISION)
#include "app/GcsTelemetryPublisher.hpp"
#include "vision/FakeFrameSource.hpp"
#include "vision/GazeboCameraSource.hpp"
#include "vision/RpicamFrameSource.hpp"
#include "vision/VisionProcessor.hpp"
#endif

#include <toml++/toml.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <csignal>
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

volatile std::sig_atomic_t g_shutdown_requested = 0;

void handleShutdownSignal(int)
{
    g_shutdown_requested = 1;
}

bool shutdownRequested()
{
    return g_shutdown_requested != 0;
}

void installShutdownSignalHandlers()
{
    std::signal(SIGINT, handleShutdownSignal);
    std::signal(SIGTERM, handleShutdownSignal);
}

enum class TransportKind {
    Udp,
    Serial,
};

enum class ControlBackend {
    GuidedVelocity,
    AltHoldRcOverride,
};

const char* toString(ControlBackend backend)
{
    switch (backend) {
    case ControlBackend::GuidedVelocity:
        return "guided_velocity";
    case ControlBackend::AltHoldRcOverride:
        return "alt_hold_rc_override";
    }
    return "unknown";
}

ControlBackend parseControlBackend(const std::string& value)
{
    if (value == "guided_velocity") {
        return ControlBackend::GuidedVelocity;
    }
    if (value == "alt_hold_rc_override") {
        return ControlBackend::AltHoldRcOverride;
    }
    throw std::runtime_error("unknown --control-backend: " + value);
}

struct EndpointConfig {
    TransportKind kind = TransportKind::Udp;
    std::string label = "sitl";
    std::string listen_host = "0.0.0.0";
    std::uint16_t listen_port = 14550;
    std::string serial_device = "/dev/serial0";
    int serial_baudrate = 115200;
};

struct RcOverrideConfig {
    int sender_system_id = 255;
    int trim_pwm = 1500;
    int arm_throttle_pwm = 1000;
    int throttle_hold_pwm = 1500;
    int takeoff_throttle_pwm = 1600;
    int max_roll_delta_pwm = 80;
    int max_pitch_delta_pwm = 80;
    int max_yaw_delta_pwm = 80;
    double roll_full_scale_mps = 0.8;
    double pitch_full_scale_mps = 1.0;
    double yaw_full_scale_rad_s = 0.8;
    double marker_roll_full_scale_mps = 0.45;
    double marker_pitch_full_scale_mps = 0.35;
    double marker_servo_kp_x = 0.35;
    double marker_servo_kp_y = 0.35;
    double marker_servo_kd_x = 0.08;
    double marker_servo_kd_y = 0.14;
    double marker_servo_max_lateral_mps = 0.22;
    double marker_servo_max_forward_mps = 0.10;
    double marker_servo_max_reverse_mps = 0.30;
    double marker_servo_max_rate_mps = 0.04;
    double marker_servo_derivative_alpha = 0.35;
    double takeoff_timeout_s = 20.0;
    bool override_throttle = true;
    bool invert_roll = false;
    bool invert_pitch = false;
    bool invert_yaw = false;
};

struct RuntimeConfig {
    EndpointConfig endpoint;
    onboard::autopilot::MavlinkIds mavlink;
    onboard::common::NetworkConfig network;
    onboard::common::VisionConfig vision;
    int setpoint_rate_hz = 20;
    onboard::mission::LineFollowMissionConfig mission;
    onboard::control::GuidedVelocityControllerConfig controller;
    RcOverrideConfig rc_override;
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
    ControlBackend control_backend = ControlBackend::GuidedVelocity;
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
    bool allow_rc_override = false;
    bool rc_override_smoke = false;
    bool alt_hold_set_mode = false;
    bool alt_hold_auto_takeoff = false;
    bool unsafe_assume_rc_present = false;
    bool profile_vision = false;
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
        << "  --target <sitl|ardupilot_serial>\n"
        << "                              Runtime target profile\n"
        << "  --autopilot <uri>           Override endpoint, e.g. udp://0.0.0.0:14550\n"
        << "                              or serial:///dev/serial0:115200\n"
        << "  --vision <fake|gazebo|rpicam>\n"
        << "                              Vision source for this MVP node\n"
        << "  --control-backend <guided_velocity|alt_hold_rc_override>\n"
        << "                              Default guided_velocity. alt_hold_rc_override is line-follow only.\n"
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
        << "  --no-arm                    Real serial safety mode; no mode/arm/takeoff\n"
        << "  --dry-run                   Real serial safety mode; no command-producing mission\n"
        << "  --allow-arm-takeoff         Explicitly allow real serial arm/takeoff path\n"
        << "  --allow-rc-override         Explicitly allow ALT_HOLD RC override control\n"
        << "  --rc-override-smoke         No-arm RC override channel smoke; sends neutral/pulses/release\n"
        << "  --alt-hold-set-mode         In alt_hold_rc_override, request ALT_HOLD before waiting ready\n"
        << "  --alt-hold-auto-takeoff     In alt_hold_rc_override, arm and climb using RC throttle override\n"
        << "  --unsafe-assume-rc-present  Bypass MAVLink RC gate for real-flight debugging\n"
        << "  --profile-vision            Log per-stage vision latency in control log\n"
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
        } else if (arg == "--control-backend" && i + 1 < argc) {
            options.control_backend = parseControlBackend(argv[++i]);
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
        } else if (arg == "--allow-rc-override") {
            options.allow_rc_override = true;
        } else if (arg == "--rc-override-smoke") {
            options.rc_override_smoke = true;
        } else if (arg == "--alt-hold-set-mode") {
            options.alt_hold_set_mode = true;
        } else if (arg == "--alt-hold-auto-takeoff") {
            options.alt_hold_auto_takeoff = true;
        } else if (arg == "--unsafe-assume-rc-present") {
            options.unsafe_assume_rc_present = true;
        } else if (arg == "--profile-vision") {
            options.profile_vision = true;
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
        config.endpoint.label = "ardupilot_serial";
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
            config.safety.autopilot_heartbeat_lost_ms =
                timeouts["autopilot_heartbeat_lost_ms"].value_or(
                    config.safety.autopilot_heartbeat_lost_ms);
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
        config.controller.output_ema_alpha =
            line_controller["output_ema_alpha"].value_or(config.controller.output_ema_alpha);
        config.controller.max_lateral_rate_mps =
            line_controller["max_lateral_rate_mps"].value_or(config.controller.max_lateral_rate_mps);
        config.controller.max_yaw_rate_change_rad_s =
            line_controller["max_yaw_rate_change_rad_s"].value_or(config.controller.max_yaw_rate_change_rad_s);
        config.controller.forward_confidence_scale =
            line_controller["forward_confidence_scale"].value_or(config.controller.forward_confidence_scale);
    }
    if (const auto marker_hover = table["marker_hover"]) {
        config.mission.marker_hover_s =
            marker_hover["hold_s"].value_or(config.mission.marker_hover_s);
        config.mission.marker_approach_timeout_s =
            marker_hover["approach_timeout_s"].value_or(config.mission.marker_approach_timeout_s);
        config.mission.marker_lost_timeout_s =
            marker_hover["lost_timeout_s"].value_or(config.mission.marker_lost_timeout_s);
        config.mission.marker_hover_recenter_timeout_s =
            marker_hover["recenter_timeout_s"].value_or(
                config.mission.marker_hover_recenter_timeout_s);
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
        config.controller.marker_deadband_norm =
            marker_hover["deadband_norm"].value_or(
                marker_hover["center_deadband_norm"].value_or(
                    config.controller.marker_deadband_norm));
        config.controller.marker_output_ema_alpha =
            marker_hover["output_ema_alpha"].value_or(
                config.controller.marker_output_ema_alpha);
        config.controller.max_marker_rate_mps =
            marker_hover["max_marker_rate_mps"].value_or(
                marker_hover["max_lateral_rate_mps"].value_or(
                    config.controller.max_marker_rate_mps));
        config.controller.invert_marker_x =
            marker_hover["invert_x"].value_or(config.controller.invert_marker_x);
        config.controller.invert_marker_y =
            marker_hover["invert_y"].value_or(config.controller.invert_marker_y);
    }
    if (const auto rc_override = table["rc_override"]) {
        config.rc_override.sender_system_id =
            rc_override["sender_system_id"].value_or(config.rc_override.sender_system_id);
        config.rc_override.trim_pwm =
            rc_override["trim_pwm"].value_or(config.rc_override.trim_pwm);
        config.rc_override.arm_throttle_pwm =
            rc_override["arm_throttle_pwm"].value_or(config.rc_override.arm_throttle_pwm);
        config.rc_override.throttle_hold_pwm =
            rc_override["throttle_hold_pwm"].value_or(config.rc_override.throttle_hold_pwm);
        config.rc_override.takeoff_throttle_pwm =
            rc_override["takeoff_throttle_pwm"].value_or(config.rc_override.takeoff_throttle_pwm);
        config.rc_override.max_roll_delta_pwm =
            rc_override["max_roll_delta_pwm"].value_or(config.rc_override.max_roll_delta_pwm);
        config.rc_override.max_pitch_delta_pwm =
            rc_override["max_pitch_delta_pwm"].value_or(config.rc_override.max_pitch_delta_pwm);
        config.rc_override.max_yaw_delta_pwm =
            rc_override["max_yaw_delta_pwm"].value_or(config.rc_override.max_yaw_delta_pwm);
        config.rc_override.roll_full_scale_mps =
            rc_override["roll_full_scale_mps"].value_or(config.rc_override.roll_full_scale_mps);
        config.rc_override.pitch_full_scale_mps =
            rc_override["pitch_full_scale_mps"].value_or(config.rc_override.pitch_full_scale_mps);
        config.rc_override.yaw_full_scale_rad_s =
            rc_override["yaw_full_scale_rad_s"].value_or(config.rc_override.yaw_full_scale_rad_s);
        config.rc_override.marker_roll_full_scale_mps =
            rc_override["marker_roll_full_scale_mps"].value_or(
                config.rc_override.marker_roll_full_scale_mps);
        config.rc_override.marker_pitch_full_scale_mps =
            rc_override["marker_pitch_full_scale_mps"].value_or(
                config.rc_override.marker_pitch_full_scale_mps);
        config.rc_override.marker_servo_kp_x =
            rc_override["marker_servo_kp_x"].value_or(config.rc_override.marker_servo_kp_x);
        config.rc_override.marker_servo_kp_y =
            rc_override["marker_servo_kp_y"].value_or(config.rc_override.marker_servo_kp_y);
        config.rc_override.marker_servo_kd_x =
            rc_override["marker_servo_kd_x"].value_or(config.rc_override.marker_servo_kd_x);
        config.rc_override.marker_servo_kd_y =
            rc_override["marker_servo_kd_y"].value_or(config.rc_override.marker_servo_kd_y);
        config.rc_override.marker_servo_max_lateral_mps =
            rc_override["marker_servo_max_lateral_mps"].value_or(
                config.rc_override.marker_servo_max_lateral_mps);
        config.rc_override.marker_servo_max_forward_mps =
            rc_override["marker_servo_max_forward_mps"].value_or(
                rc_override["marker_approach_max_forward_mps"].value_or(
                    config.rc_override.marker_servo_max_forward_mps));
        config.rc_override.marker_servo_max_reverse_mps =
            rc_override["marker_servo_max_reverse_mps"].value_or(
                rc_override["marker_brake_reverse_mps"].value_or(
                    config.rc_override.marker_servo_max_reverse_mps));
        config.rc_override.marker_servo_max_rate_mps =
            rc_override["marker_servo_max_rate_mps"].value_or(
                config.rc_override.marker_servo_max_rate_mps);
        config.rc_override.marker_servo_derivative_alpha =
            rc_override["marker_servo_derivative_alpha"].value_or(
                config.rc_override.marker_servo_derivative_alpha);
        config.rc_override.takeoff_timeout_s =
            rc_override["takeoff_timeout_s"].value_or(config.rc_override.takeoff_timeout_s);
        config.rc_override.override_throttle =
            rc_override["override_throttle"].value_or(config.rc_override.override_throttle);
        config.rc_override.invert_roll =
            rc_override["invert_roll"].value_or(config.rc_override.invert_roll);
        config.rc_override.invert_pitch =
            rc_override["invert_pitch"].value_or(config.rc_override.invert_pitch);
        config.rc_override.invert_yaw =
            rc_override["invert_yaw"].value_or(config.rc_override.invert_yaw);
    }
}

RuntimeConfig loadRuntimeConfig(const Options& options)
{
    RuntimeConfig config;
    const std::string target = options.target;
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
            config.controller.output_ema_alpha =
                line_controller["output_ema_alpha"].value_or(config.controller.output_ema_alpha);
            config.controller.max_lateral_rate_mps =
                line_controller["max_lateral_rate_mps"].value_or(config.controller.max_lateral_rate_mps);
            config.controller.max_yaw_rate_change_rad_s =
                line_controller["max_yaw_rate_change_rad_s"].value_or(config.controller.max_yaw_rate_change_rad_s);
            config.controller.forward_confidence_scale =
                line_controller["forward_confidence_scale"].value_or(config.controller.forward_confidence_scale);
        }
        if (const auto marker_hover = table["marker_hover"]) {
            config.mission.marker_hover_s =
                marker_hover["hold_s"].value_or(config.mission.marker_hover_s);
            config.mission.marker_approach_timeout_s =
                marker_hover["approach_timeout_s"].value_or(config.mission.marker_approach_timeout_s);
            config.mission.marker_lost_timeout_s =
                marker_hover["lost_timeout_s"].value_or(config.mission.marker_lost_timeout_s);
            config.mission.marker_hover_recenter_timeout_s =
                marker_hover["recenter_timeout_s"].value_or(
                    config.mission.marker_hover_recenter_timeout_s);
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
            config.controller.marker_deadband_norm =
                marker_hover["deadband_norm"].value_or(
                    marker_hover["center_deadband_norm"].value_or(
                        config.controller.marker_deadband_norm));
            config.controller.marker_output_ema_alpha =
                marker_hover["output_ema_alpha"].value_or(
                    config.controller.marker_output_ema_alpha);
            config.controller.max_marker_rate_mps =
                marker_hover["max_marker_rate_mps"].value_or(
                    marker_hover["max_lateral_rate_mps"].value_or(
                        config.controller.max_marker_rate_mps));
            config.controller.invert_marker_x =
                marker_hover["invert_x"].value_or(config.controller.invert_marker_x);
            config.controller.invert_marker_y =
                marker_hover["invert_y"].value_or(config.controller.invert_marker_y);
        }
        if (const auto rc_override = table["rc_override"]) {
            config.rc_override.sender_system_id =
                rc_override["sender_system_id"].value_or(config.rc_override.sender_system_id);
            config.rc_override.trim_pwm =
                rc_override["trim_pwm"].value_or(config.rc_override.trim_pwm);
            config.rc_override.arm_throttle_pwm =
                rc_override["arm_throttle_pwm"].value_or(config.rc_override.arm_throttle_pwm);
            config.rc_override.throttle_hold_pwm =
                rc_override["throttle_hold_pwm"].value_or(config.rc_override.throttle_hold_pwm);
            config.rc_override.takeoff_throttle_pwm =
                rc_override["takeoff_throttle_pwm"].value_or(config.rc_override.takeoff_throttle_pwm);
            config.rc_override.max_roll_delta_pwm =
                rc_override["max_roll_delta_pwm"].value_or(config.rc_override.max_roll_delta_pwm);
            config.rc_override.max_pitch_delta_pwm =
                rc_override["max_pitch_delta_pwm"].value_or(config.rc_override.max_pitch_delta_pwm);
            config.rc_override.max_yaw_delta_pwm =
                rc_override["max_yaw_delta_pwm"].value_or(config.rc_override.max_yaw_delta_pwm);
            config.rc_override.roll_full_scale_mps =
                rc_override["roll_full_scale_mps"].value_or(config.rc_override.roll_full_scale_mps);
            config.rc_override.pitch_full_scale_mps =
                rc_override["pitch_full_scale_mps"].value_or(config.rc_override.pitch_full_scale_mps);
            config.rc_override.yaw_full_scale_rad_s =
                rc_override["yaw_full_scale_rad_s"].value_or(config.rc_override.yaw_full_scale_rad_s);
            config.rc_override.marker_roll_full_scale_mps =
                rc_override["marker_roll_full_scale_mps"].value_or(
                    config.rc_override.marker_roll_full_scale_mps);
            config.rc_override.marker_pitch_full_scale_mps =
                rc_override["marker_pitch_full_scale_mps"].value_or(
                    config.rc_override.marker_pitch_full_scale_mps);
            config.rc_override.marker_servo_kp_x =
                rc_override["marker_servo_kp_x"].value_or(config.rc_override.marker_servo_kp_x);
            config.rc_override.marker_servo_kp_y =
                rc_override["marker_servo_kp_y"].value_or(config.rc_override.marker_servo_kp_y);
            config.rc_override.marker_servo_kd_x =
                rc_override["marker_servo_kd_x"].value_or(config.rc_override.marker_servo_kd_x);
            config.rc_override.marker_servo_kd_y =
                rc_override["marker_servo_kd_y"].value_or(config.rc_override.marker_servo_kd_y);
            config.rc_override.marker_servo_max_lateral_mps =
                rc_override["marker_servo_max_lateral_mps"].value_or(
                    config.rc_override.marker_servo_max_lateral_mps);
            config.rc_override.marker_servo_max_forward_mps =
                rc_override["marker_servo_max_forward_mps"].value_or(
                    rc_override["marker_approach_max_forward_mps"].value_or(
                        config.rc_override.marker_servo_max_forward_mps));
            config.rc_override.marker_servo_max_reverse_mps =
                rc_override["marker_servo_max_reverse_mps"].value_or(
                    rc_override["marker_brake_reverse_mps"].value_or(
                        config.rc_override.marker_servo_max_reverse_mps));
            config.rc_override.marker_servo_max_rate_mps =
                rc_override["marker_servo_max_rate_mps"].value_or(
                    config.rc_override.marker_servo_max_rate_mps);
            config.rc_override.marker_servo_derivative_alpha =
                rc_override["marker_servo_derivative_alpha"].value_or(
                    config.rc_override.marker_servo_derivative_alpha);
            config.rc_override.takeoff_timeout_s =
                rc_override["takeoff_timeout_s"].value_or(config.rc_override.takeoff_timeout_s);
            config.rc_override.override_throttle =
                rc_override["override_throttle"].value_or(config.rc_override.override_throttle);
            config.rc_override.invert_roll =
                rc_override["invert_roll"].value_or(config.rc_override.invert_roll);
            config.rc_override.invert_pitch =
                rc_override["invert_pitch"].value_or(config.rc_override.invert_pitch);
            config.rc_override.invert_yaw =
                rc_override["invert_yaw"].value_or(config.rc_override.invert_yaw);
        }
    } catch (const toml::parse_error&) {
    }

    try {
        const auto table = toml::parse_file(joinConfigPath(options.config_dir, "safety.toml"));
        if (const auto timeouts = table["timeouts"]) {
            config.safety.line_lost_ms =
                timeouts["line_lost_ms"].value_or(config.safety.line_lost_ms);
            config.safety.autopilot_heartbeat_lost_ms =
                timeouts["autopilot_heartbeat_lost_ms"].value_or(
                    config.safety.autopilot_heartbeat_lost_ms);
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

    if (target == "sitl") {
        config.endpoint.kind = TransportKind::Udp;
        config.endpoint.label = "sitl";
        config.vision_source = "fake";
    } else if (target == "ardupilot_serial") {
        config.endpoint.kind = TransportKind::Serial;
        config.endpoint.label = "ardupilot_serial";
        config.vision_source = "rpicam";
    } else if (!target.empty()) {
        throw std::runtime_error("unknown --target: " + options.target);
    }

    if (target.empty()) {
        config.vision_source = "fake";
    }

    applyRuntimeOverlay(config, options.config_dir, target);

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
    config.rc_override.sender_system_id =
        std::clamp(config.rc_override.sender_system_id, 1, 255);
    config.rc_override.roll_full_scale_mps =
        std::max(0.05, config.rc_override.roll_full_scale_mps);
    config.rc_override.pitch_full_scale_mps =
        std::max(0.05, config.rc_override.pitch_full_scale_mps);
    config.rc_override.yaw_full_scale_rad_s =
        std::max(0.05, config.rc_override.yaw_full_scale_rad_s);
    config.rc_override.marker_roll_full_scale_mps =
        std::max(0.05, config.rc_override.marker_roll_full_scale_mps);
    config.rc_override.marker_pitch_full_scale_mps =
        std::max(0.05, config.rc_override.marker_pitch_full_scale_mps);
    config.rc_override.marker_servo_kp_x =
        std::max(0.0, config.rc_override.marker_servo_kp_x);
    config.rc_override.marker_servo_kp_y =
        std::max(0.0, config.rc_override.marker_servo_kp_y);
    config.rc_override.marker_servo_kd_x =
        std::max(0.0, config.rc_override.marker_servo_kd_x);
    config.rc_override.marker_servo_kd_y =
        std::max(0.0, config.rc_override.marker_servo_kd_y);
    config.rc_override.marker_servo_max_lateral_mps =
        std::max(0.0, config.rc_override.marker_servo_max_lateral_mps);
    config.rc_override.marker_servo_max_forward_mps =
        std::max(0.0, config.rc_override.marker_servo_max_forward_mps);
    config.rc_override.marker_servo_max_reverse_mps =
        std::max(0.0, config.rc_override.marker_servo_max_reverse_mps);
    config.rc_override.marker_servo_max_rate_mps =
        std::max(0.0, config.rc_override.marker_servo_max_rate_mps);
    config.rc_override.marker_servo_derivative_alpha =
        std::clamp(config.rc_override.marker_servo_derivative_alpha, 0.05, 1.0);

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
        if (shutdownRequested()) {
            std::cerr << "[safety] shutdown requested while waiting for RC input\n";
            return false;
        }
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

std::optional<double> validRangefinderAltitudeM(
    const onboard::autopilot::AutopilotState& state)
{
    if (state.distance_sensor_m &&
        *state.distance_sensor_m >= 0.03 &&
        *state.distance_sensor_m <= 8.0) {
        return state.distance_sensor_m;
    }
    return std::nullopt;
}

std::optional<double> altHoldTakeoffAltitudeM(
    const onboard::autopilot::AutopilotState& state)
{
    if (const auto range = validRangefinderAltitudeM(state)) {
        return range;
    }
    if (state.relative_altitude_m) {
        return state.relative_altitude_m;
    }
    return state.local_altitude_m;
}

std::optional<double> landingHeightM(const onboard::autopilot::AutopilotState& state)
{
    if (state.distance_sensor_m) {
        return state.distance_sensor_m;
    }
    if (state.local_altitude_m) {
        return state.local_altitude_m;
    }
    if (state.relative_altitude_m) {
        return state.relative_altitude_m;
    }
    return std::nullopt;
}

bool lowEnoughForLandMode(const onboard::autopilot::AutopilotState& state)
{
    const auto height = landingHeightM(state);
    return height && *height <= 0.55;
}

bool touchdownLikely(const onboard::autopilot::AutopilotState& state)
{
    const auto height = landingHeightM(state);
    if (!height || *height > 0.14) {
        return false;
    }
    const double vx = std::abs(state.local_vx_mps.value_or(0.0));
    const double vy = std::abs(state.local_vy_mps.value_or(0.0));
    const double vz = std::abs(state.local_vz_mps.value_or(0.0));
    return vx < 0.12 && vy < 0.12 && vz < 0.10;
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

// Hover at the captured X,Y anchor and the supplied target altitude (m above
// origin). Emits a position-only SET_POSITION_TARGET_LOCAL_NED so ArduCopter's
// guided position controller holds 3D position; mixing X,Y position with VZ
// velocity hits ArduCopter's "unsupported combination" branch and calls
// hold_position() instead of honoring the command.
void sendAnchoredVelocitySetpoint(
    onboard::autopilot::AutopilotMavlinkAdapter& autopilot,
    const LocalHoldAnchor& anchor,
    const onboard::control::ControlSetpoint& setpoint,
    double target_altitude_m)
{
    autopilot.sendLocalNedPositionTarget(onboard::autopilot::LocalNedPositionTargetCommand {
        static_cast<float>(anchor.x_m),
        static_cast<float>(anchor.y_m),
        std::optional<float> {static_cast<float>(-target_altitude_m)},
        std::nullopt,
        setpoint.yaw_rate_rad_s,
    });
}

// Descent emits a velocity-only SET_POSITION_TARGET_LOCAL_NED (VX=VY=0,
// VZ=descent rate). ArduCopter's velocity controller holds horizontal velocity
// at zero so XY drift stays small even without a position anchor.
void sendAnchoredDescentSetpoint(
    onboard::autopilot::AutopilotMavlinkAdapter& autopilot,
    const LocalHoldAnchor& /*anchor*/,
    float vz_down_mps)
{
    autopilot.sendLocalNedPositionTarget(onboard::autopilot::LocalNedPositionTargetCommand {
        0.0f,
        0.0f,
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

// Local-XY position source by runtime target:
//   * Gazebo SITL (target=sitl, vision=gazebo|fake): ArduCopter SITL EKF
//     publishes LOCAL_POSITION_NED fused from the simulated GPS+IMU. There is
//     no MTF-01; opticalFlowReady() returns false. We therefore only enforce
//     waitLocalHoldEstimateReady() / opticalFlowReady() when the transport is
//     real serial (target=ardupilot_serial). Setpoints go out as either body-frame
//     velocities (line follow) or position-anchored velocities (hover/land).
//   * Real serial target (target=ardupilot_serial, vision=rpicam): EKF3 fuses MTF-01
//     OPTICAL_FLOW + range finder + IMU into LOCAL_POSITION_NED. Drift is
//     higher than the SITL GPS path, so the ardupilot_serial config uses
//     smaller gains, stronger EMA smoothing, and a lower max forward speed.
//     localHoldEstimateReady() additionally requires optical_flow_quality and
//     the EKF relative-aiding bits before we permit GUIDED arm/takeoff.
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

// We must NOT send any SET_POSITION_TARGET_LOCAL_NED while NAV_TAKEOFF is in
// progress. In GUIDED mode ArduCopter's TakeOff sub-mode is interrupted by the
// first position/velocity setpoint, so even a pos-only "hold at takeoff target"
// races the takeoff controller and usually leaves the vehicle stuck just above
// the ground. Use autopilot.waitAltitudeReached() to poll without sending.

void holdAnchoredAltitude(
    onboard::autopilot::AutopilotMavlinkAdapter& autopilot,
    onboard::control::GuidedVelocityController& controller,
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
            controller.stop(altitude_input),
            target_altitude_m);
        std::this_thread::sleep_until(loop_start + period);
    }
}

int clampPwm(int pwm)
{
    return std::clamp(pwm, 1000, 2000);
}

double rateLimitStep(double current, double target, double max_change)
{
    if (max_change <= 0.0) {
        return target;
    }
    const double delta = target - current;
    return current + std::clamp(delta, -max_change, max_change);
}

std::uint16_t pwmFromInt(int pwm)
{
    return static_cast<std::uint16_t>(clampPwm(pwm));
}

std::array<std::uint16_t, 18> neutralRcOverrideChannels(
    const RcOverrideConfig& rc_config)
{
    std::array<std::uint16_t, 18> channels {};
    channels.fill(UINT16_MAX);
    channels[0] = pwmFromInt(rc_config.trim_pwm);
    channels[1] = pwmFromInt(rc_config.trim_pwm);
    channels[2] = rc_config.override_throttle
        ? pwmFromInt(rc_config.throttle_hold_pwm)
        : UINT16_MAX;
    channels[3] = pwmFromInt(rc_config.trim_pwm);
    return channels;
}

std::array<std::uint16_t, 18> throttleRcOverrideChannels(
    const RcOverrideConfig& rc_config,
    int throttle_pwm)
{
    auto channels = neutralRcOverrideChannels(rc_config);
    channels[2] = pwmFromInt(throttle_pwm);
    return channels;
}

std::array<std::uint16_t, 18> toRcOverrideChannels(
    const onboard::control::ControlSetpoint& setpoint,
    const onboard::control::GuidedVelocityControllerConfig& controller_config,
    const RcOverrideConfig& rc_config,
    std::optional<double> roll_full_scale_mps = std::nullopt,
    std::optional<double> pitch_full_scale_mps = std::nullopt)
{
    auto channels = neutralRcOverrideChannels(rc_config);

    const double forward_scale =
        std::max(0.05, pitch_full_scale_mps.value_or(rc_config.pitch_full_scale_mps));
    const double lateral_scale =
        std::max(0.05, roll_full_scale_mps.value_or(rc_config.roll_full_scale_mps));
    const double yaw_scale = std::max(0.05, rc_config.yaw_full_scale_rad_s);

    double roll_norm = std::clamp(
        static_cast<double>(setpoint.vy_right_mps) / lateral_scale,
        -1.0,
        1.0);
    double pitch_norm = std::clamp(
        static_cast<double>(setpoint.vx_forward_mps) / forward_scale,
        -1.0,
        1.0);
    double yaw_norm = std::clamp(
        static_cast<double>(setpoint.yaw_rate_rad_s) / yaw_scale,
        -1.0,
        1.0);

    if (rc_config.invert_roll) {
        roll_norm = -roll_norm;
    }
    if (rc_config.invert_pitch) {
        pitch_norm = -pitch_norm;
    }
    if (rc_config.invert_yaw) {
        yaw_norm = -yaw_norm;
    }

    channels[0] = pwmFromInt(
        rc_config.trim_pwm +
        static_cast<int>(std::lround(roll_norm * rc_config.max_roll_delta_pwm)));
    channels[1] = pwmFromInt(
        rc_config.trim_pwm -
        static_cast<int>(std::lround(pitch_norm * rc_config.max_pitch_delta_pwm)));
    channels[3] = pwmFromInt(
        rc_config.trim_pwm +
        static_cast<int>(std::lround(yaw_norm * rc_config.max_yaw_delta_pwm)));
    return channels;
}

class RcMarkerVisualServo {
public:
    onboard::control::ControlSetpoint update(
        const onboard::control::MarkerControlInput& marker,
        const onboard::control::GuidedVelocityControllerConfig& controller_config,
        const RcOverrideConfig& rc_config,
        Clock::time_point now)
    {
        if (!marker.marker_detected) {
            reset();
            return {};
        }

        double error_x = controller_config.invert_marker_x
            ? -marker.center_error_x_norm
            : marker.center_error_x_norm;
        double error_y = controller_config.invert_marker_y
            ? -marker.center_error_y_norm
            : marker.center_error_y_norm;
        if (std::abs(error_x) < controller_config.marker_deadband_norm) {
            error_x = 0.0;
        }
        if (std::abs(error_y) < controller_config.marker_deadband_norm) {
            error_y = 0.0;
        }

        double error_x_rate = 0.0;
        double error_y_rate = 0.0;
        if (has_prev_error_) {
            const double dt_s = std::clamp(
                std::chrono::duration<double>(now - prev_time_).count(),
                0.02,
                0.50);
            error_x_rate = (error_x - prev_error_x_) / dt_s;
            error_y_rate = (error_y - prev_error_y_) / dt_s;
            const double alpha = rc_config.marker_servo_derivative_alpha;
            filtered_error_x_rate_ =
                filtered_error_x_rate_ * (1.0 - alpha) + error_x_rate * alpha;
            filtered_error_y_rate_ =
                filtered_error_y_rate_ * (1.0 - alpha) + error_y_rate * alpha;
        } else {
            has_prev_error_ = true;
            filtered_error_x_rate_ = 0.0;
            filtered_error_y_rate_ = 0.0;
        }
        prev_error_x_ = error_x;
        prev_error_y_ = error_y;
        prev_time_ = now;

        double lateral = rc_config.marker_servo_kp_x * error_x +
            rc_config.marker_servo_kd_x * filtered_error_x_rate_;
        double forward = rc_config.marker_servo_kp_y * error_y +
            rc_config.marker_servo_kd_y * filtered_error_y_rate_;

        lateral = std::clamp(
            lateral,
            -rc_config.marker_servo_max_lateral_mps,
            rc_config.marker_servo_max_lateral_mps);
        forward = std::clamp(
            forward,
            -rc_config.marker_servo_max_reverse_mps,
            rc_config.marker_servo_max_forward_mps);

        if (has_prev_output_ && rc_config.marker_servo_max_rate_mps > 0.0) {
            lateral = rateLimitStep(
                prev_lateral_,
                lateral,
                rc_config.marker_servo_max_rate_mps);
            forward = rateLimitStep(
                prev_forward_,
                forward,
                rc_config.marker_servo_max_rate_mps);
        } else {
            has_prev_output_ = true;
        }
        prev_lateral_ = lateral;
        prev_forward_ = forward;

        return onboard::control::ControlSetpoint {
            static_cast<float>(forward),
            static_cast<float>(lateral),
            0.0f,
            0.0f,
        };
    }

    void reset()
    {
        has_prev_error_ = false;
        has_prev_output_ = false;
        prev_error_x_ = 0.0;
        prev_error_y_ = 0.0;
        filtered_error_x_rate_ = 0.0;
        filtered_error_y_rate_ = 0.0;
        prev_lateral_ = 0.0;
        prev_forward_ = 0.0;
        prev_time_ = {};
    }

private:
    bool has_prev_error_ = false;
    bool has_prev_output_ = false;
    double prev_error_x_ = 0.0;
    double prev_error_y_ = 0.0;
    double filtered_error_x_rate_ = 0.0;
    double filtered_error_y_rate_ = 0.0;
    double prev_lateral_ = 0.0;
    double prev_forward_ = 0.0;
    Clock::time_point prev_time_ {};
};

class RcOverrideReleaseGuard {
public:
    explicit RcOverrideReleaseGuard(onboard::autopilot::AutopilotMavlinkAdapter& autopilot)
        : autopilot_(autopilot)
    {
    }

    ~RcOverrideReleaseGuard()
    {
        releaseNoThrow();
    }

    RcOverrideReleaseGuard(const RcOverrideReleaseGuard&) = delete;
    RcOverrideReleaseGuard& operator=(const RcOverrideReleaseGuard&) = delete;

    void markActive()
    {
        active_ = true;
    }

    void release()
    {
        if (!active_) {
            return;
        }
        autopilot_.releaseRcChannelsOverride();
        active_ = false;
    }

    void releaseNoThrow()
    {
        if (!active_) {
            return;
        }
        try {
            autopilot_.releaseRcChannelsOverride();
        } catch (...) {
        }
        active_ = false;
    }

private:
    onboard::autopilot::AutopilotMavlinkAdapter& autopilot_;
    bool active_ = false;
};

void sendRcOverrideSetpoint(
    onboard::autopilot::AutopilotMavlinkAdapter& autopilot,
    RcOverrideReleaseGuard& guard,
    const onboard::control::ControlSetpoint& setpoint,
    const onboard::control::GuidedVelocityControllerConfig& controller_config,
    const RcOverrideConfig& rc_config,
    bool use_marker_stick_scale = false)
{
    guard.markActive();
    autopilot.sendRcChannelsOverride(
        toRcOverrideChannels(
            setpoint,
            controller_config,
            rc_config,
            use_marker_stick_scale
                ? std::optional<double> { rc_config.marker_roll_full_scale_mps }
                : std::nullopt,
            use_marker_stick_scale
                ? std::optional<double> { rc_config.marker_pitch_full_scale_mps }
                : std::nullopt));
}

void sendRcOverrideNeutral(
    onboard::autopilot::AutopilotMavlinkAdapter& autopilot,
    RcOverrideReleaseGuard& guard,
    const RcOverrideConfig& rc_config)
{
    guard.markActive();
    autopilot.sendRcChannelsOverride(neutralRcOverrideChannels(rc_config));
}

bool requestShutdownLandAndDisarm(
    onboard::autopilot::AutopilotMavlinkAdapter& autopilot,
    RcOverrideReleaseGuard& guard,
    const RcOverrideConfig& rc_config,
    std::chrono::duration<double> period)
{
    std::cerr << "[safety] shutdown requested; neutral/release RC override and request LAND\n";
    try {
        sendRcOverrideNeutral(autopilot, guard, rc_config);
    } catch (const std::exception& error) {
        std::cerr << "[safety] failed to send neutral RC override: " << error.what() << "\n";
    }
    guard.releaseNoThrow();
    autopilot.poll(100);

    if (!autopilot.state().armed) {
        std::cout << "[safety] vehicle already disarmed\n";
        return true;
    }

    try {
        autopilot.setLandMode(std::chrono::seconds(10));
    } catch (const std::exception& error) {
        std::cerr << "[safety] LAND mode request failed: " << error.what() << "\n";
    }

    const auto deadline = Clock::now() + std::chrono::seconds(90);
    auto last_disarm_request = Clock::time_point {};
    while (Clock::now() < deadline) {
        const auto loop_start = Clock::now();
        autopilot.poll(100);
        if (autopilot.state().heartbeat_seen && !autopilot.state().armed) {
            std::cout << "[safety] shutdown landing disarmed\n";
            return true;
        }
        const bool retry_due =
            last_disarm_request.time_since_epoch().count() == 0 ||
            loop_start - last_disarm_request >= std::chrono::seconds(4);
        if (touchdownLikely(autopilot.state()) && retry_due) {
            std::cout << "[safety] shutdown touchdown likely height="
                      << landingHeightM(autopilot.state()).value_or(-1.0)
                      << "m; requesting disarm\n";
            autopilot.requestDisarm();
            last_disarm_request = loop_start;
        }
        std::this_thread::sleep_until(loop_start + period);
    }

    std::cerr << "[safety] shutdown landing timed out waiting for disarm\n";
    return false;
}

bool waitAltHoldPilotReady(
    onboard::autopilot::AutopilotMavlinkAdapter& autopilot,
    const onboard::safety::SafetyConfig& safety,
    double target_altitude_m,
    double altitude_reached_ratio,
    bool request_alt_hold_mode,
    std::chrono::seconds timeout)
{
    if (!waitRcReady(autopilot, safety, std::chrono::seconds(10))) {
        std::cerr << "[safety] refusing ALT_HOLD RC override until RC input is visible\n";
        return false;
    }

    if (request_alt_hold_mode && autopilot.state().custom_mode != COPTER_MODE_ALT_HOLD) {
        std::cout << "[mavlink] requesting ALT_HOLD mode\n";
        autopilot.setAltHoldMode(std::chrono::seconds(10));
    }

    std::cout << "[preflight] waiting for pilot ALT_HOLD hover: armed=true mode=ALT_HOLD altitude>="
              << target_altitude_m * altitude_reached_ratio << "m\n";
    const auto deadline = Clock::now() + timeout;
    auto last_log = Clock::now() - std::chrono::seconds(1);
    while (Clock::now() < deadline) {
        if (shutdownRequested()) {
            std::cerr << "[safety] shutdown requested while waiting for ALT_HOLD hover\n";
            return false;
        }
        autopilot.poll(100);
        const auto now = Clock::now();
        const auto altitude = autopilot.bestAltitudeM();
        const bool ready =
            autopilot.state().armed &&
            autopilot.state().custom_mode == COPTER_MODE_ALT_HOLD &&
            altitude &&
            *altitude >= target_altitude_m * altitude_reached_ratio &&
            rcInputFresh(autopilot.state(), safety, now);
        if (ready) {
            std::cout << "[preflight] ALT_HOLD hover ready alt="
                      << altitude.value_or(-1.0)
                      << " mode=" << autopilot.state().mode_name << "\n";
            return true;
        }
        if (now - last_log >= std::chrono::seconds(1)) {
            const auto& state = autopilot.state();
            std::cout << "[preflight] armed=" << (state.armed ? "true" : "false")
                      << " mode=" << state.mode_name
                      << " alt=" << altitude.value_or(-1.0)
                      << " rc_channels=" << state.rc_channel_count.value_or(0)
                      << "\n";
            last_log = now;
        }
    }
    std::cerr << "[preflight] ALT_HOLD hover not ready before timeout\n";
    return false;
}

bool runAltHoldRcOverrideAutoTakeoff(
    onboard::autopilot::AutopilotMavlinkAdapter& autopilot,
    RcOverrideReleaseGuard& guard,
    const onboard::safety::SafetyConfig& safety,
    const RcOverrideConfig& rc_config,
    double target_altitude_m,
    double altitude_reached_ratio,
    double settle_s,
    std::chrono::duration<double> period)
{
    if (!waitRcReady(autopilot, safety, std::chrono::seconds(10))) {
        std::cerr << "[safety] refusing ALT_HOLD auto-takeoff until RC input is visible\n";
        return false;
    }

    guard.markActive();
    const auto low_throttle = throttleRcOverrideChannels(rc_config, rc_config.arm_throttle_pwm);
    const auto neutral = neutralRcOverrideChannels(rc_config);
    const auto climb = throttleRcOverrideChannels(rc_config, rc_config.takeoff_throttle_pwm);

    std::cout << "[mission] ALT_HOLD_RC_TAKEOFF arm_throttle="
              << low_throttle[2]
              << " climb_throttle=" << climb[2]
              << " target=" << target_altitude_m << "m\n";

    const auto arm_prep_deadline = Clock::now() + std::chrono::seconds(1);
    while (Clock::now() < arm_prep_deadline) {
        if (shutdownRequested()) {
            std::cerr << "[safety] shutdown requested before ALT_HOLD auto-takeoff arm\n";
            autopilot.sendRcChannelsOverride(neutral);
            guard.releaseNoThrow();
            return false;
        }
        const auto loop_start = Clock::now();
        autopilot.poll(1);
        autopilot.sendRcChannelsOverride(low_throttle);
        std::this_thread::sleep_until(loop_start + period);
    }

    if (autopilot.state().custom_mode != COPTER_MODE_ALT_HOLD) {
        std::cout << "[mavlink] requesting ALT_HOLD mode\n";
        autopilot.setAltHoldMode(std::chrono::seconds(10));
    }
    if (!autopilot.state().armed) {
        const auto arm_deadline = Clock::now() + std::chrono::seconds(30);
        while (!autopilot.state().armed && Clock::now() < arm_deadline) {
            if (shutdownRequested()) {
                std::cerr << "[safety] shutdown requested while waiting for armed state\n";
                autopilot.sendRcChannelsOverride(neutral);
                guard.releaseNoThrow();
                return false;
            }
            autopilot.requestArm();
            const auto attempt_deadline = Clock::now() + std::chrono::seconds(3);
            while (!autopilot.state().armed && Clock::now() < attempt_deadline) {
                if (shutdownRequested()) {
                    std::cerr << "[safety] shutdown requested while waiting for armed state\n";
                    autopilot.sendRcChannelsOverride(neutral);
                    guard.releaseNoThrow();
                    return false;
                }
                autopilot.poll(200);
            }
        }
        if (!autopilot.state().armed) {
            throw std::runtime_error("timed out waiting for armed state");
        }
        std::cout << "[mavlink] armed\n";
    }

    const auto target_reached_m = target_altitude_m * altitude_reached_ratio;
    const auto takeoff_started_at = Clock::now();
    const auto takeoff_deadline =
        takeoff_started_at +
        std::chrono::milliseconds(
            static_cast<int>(std::max(3.0, rc_config.takeoff_timeout_s) * 1000.0));
    auto last_log = Clock::now() - std::chrono::seconds(1);
    auto first_altitude = altHoldTakeoffAltitudeM(autopilot.state());
    if (!validRangefinderAltitudeM(autopilot.state())) {
        std::cerr << "[safety] warning: ALT_HOLD auto-takeoff has no valid rangefinder altitude; "
                     "falling back to EKF/baro altitude\n";
    }
    bool climb_seen = false;

    while (Clock::now() < takeoff_deadline) {
        if (shutdownRequested()) {
            std::cerr << "[safety] shutdown requested during ALT_HOLD auto-takeoff\n";
            autopilot.sendRcChannelsOverride(neutral);
            return false;
        }
        const auto loop_start = Clock::now();
        autopilot.poll(1);
        const auto& ap_state = autopilot.state();
        const auto altitude = altHoldTakeoffAltitudeM(ap_state);
        const auto now = Clock::now();

        if (ap_state.custom_mode != COPTER_MODE_ALT_HOLD) {
            std::cerr << "[safety] operator takeover during ALT_HOLD auto-takeoff\n";
            guard.releaseNoThrow();
            return false;
        }
        if (!ap_state.armed) {
            std::cerr << "[safety] vehicle disarmed during ALT_HOLD auto-takeoff\n";
            guard.releaseNoThrow();
            return false;
        }
        if (!rcInputFresh(ap_state, safety, now)) {
            std::cerr << "[safety] RC input not fresh during ALT_HOLD auto-takeoff\n";
            autopilot.sendRcChannelsOverride(neutral);
            return false;
        }
        if (altitude && *altitude >= target_reached_m) {
            autopilot.sendRcChannelsOverride(neutral);
            std::cout << "[mission] ALT_HOLD_RC_TAKEOFF altitude reached alt="
                      << *altitude
                      << "m range=" << ap_state.distance_sensor_m.value_or(-1.0)
                      << " rel_alt=" << ap_state.relative_altitude_m.value_or(-1.0)
                      << " local_alt=" << ap_state.local_altitude_m.value_or(-1.0)
                      << "\n";
            if (settle_s > 0.0) {
                const auto settle_deadline =
                    Clock::now() +
                    std::chrono::milliseconds(static_cast<int>(settle_s * 1000.0));
                while (Clock::now() < settle_deadline) {
                    if (shutdownRequested()) {
                        std::cerr << "[safety] shutdown requested during ALT_HOLD takeoff settle\n";
                        autopilot.sendRcChannelsOverride(neutral);
                        return false;
                    }
                    const auto settle_loop_start = Clock::now();
                    autopilot.poll(1);
                    autopilot.sendRcChannelsOverride(neutral);
                    std::this_thread::sleep_until(settle_loop_start + period);
                }
            }
            return true;
        }

        if (!first_altitude && altitude) {
            first_altitude = altitude;
        }
        if (first_altitude && altitude && *altitude - *first_altitude >= 0.20) {
            climb_seen = true;
        }
        const auto elapsed_s = std::chrono::duration<double>(now - takeoff_started_at).count();
        if (!climb_seen && elapsed_s >= 5.0) {
            std::cerr << "[safety] no climb detected during ALT_HOLD auto-takeoff\n";
            autopilot.sendRcChannelsOverride(neutral);
            if (nearGroundAndStable(autopilot.state())) {
                autopilot.requestDisarm();
            }
            return false;
        }

        autopilot.sendRcChannelsOverride(climb);
        if (now - last_log >= std::chrono::seconds(1)) {
            std::cout << "[mission] ALT_HOLD_RC_TAKEOFF alt="
                      << altitude.value_or(-1.0)
                      << " range=" << ap_state.distance_sensor_m.value_or(-1.0)
                      << " rel_alt=" << ap_state.relative_altitude_m.value_or(-1.0)
                      << " local_alt=" << ap_state.local_altitude_m.value_or(-1.0)
                      << " target_reached=" << target_reached_m
                      << " throttle=" << climb[2] << "\n";
            last_log = now;
        }
        std::this_thread::sleep_until(loop_start + period);
    }

    std::cerr << "[safety] ALT_HOLD auto-takeoff timeout\n";
    autopilot.sendRcChannelsOverride(neutral);
    if (autopilot.state().armed) {
        autopilot.setLandMode(std::chrono::seconds(10));
    }
    return false;
}

int runRcOverrideSmoke(
    onboard::autopilot::AutopilotMavlinkAdapter& autopilot,
    const RuntimeConfig& config)
{
    RcOverrideReleaseGuard guard(autopilot);
    if (autopilot.state().armed) {
        std::cerr << "[rc-override-smoke] refusing to send test pulses while armed\n";
        return 2;
    }

    const auto period = std::chrono::milliseconds(100);
    const auto send_for = [&](const char* label, std::array<std::uint16_t, 18> channels) {
        std::cout << "[rc-override-smoke] " << label
                  << " ch1=" << channels[0]
                  << " ch2=" << channels[1]
                  << " ch3=" << channels[2]
                  << " ch4=" << channels[3] << "\n";
        guard.markActive();
        const auto deadline = Clock::now() + std::chrono::milliseconds(900);
        while (Clock::now() < deadline) {
            const auto loop_start = Clock::now();
            autopilot.poll(1);
            autopilot.sendRcChannelsOverride(channels);
            std::this_thread::sleep_until(loop_start + period);
        }
    };

    auto neutral = neutralRcOverrideChannels(config.rc_override);
    send_for("neutral", neutral);

    auto roll_right = neutral;
    roll_right[0] = pwmFromInt(
        config.rc_override.trim_pwm + config.rc_override.max_roll_delta_pwm);
    send_for("roll_right", roll_right);

    auto pitch_forward = neutral;
    pitch_forward[1] = pwmFromInt(
        config.rc_override.trim_pwm - config.rc_override.max_pitch_delta_pwm);
    send_for("pitch_forward", pitch_forward);

    auto yaw_clockwise = neutral;
    yaw_clockwise[3] = pwmFromInt(
        config.rc_override.trim_pwm + config.rc_override.max_yaw_delta_pwm);
    send_for("yaw_clockwise", yaw_clockwise);

    guard.release();
    std::cout << "[rc-override-smoke] release sent\n";
    return 0;
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
    onboard::app::GcsTelemetryPublishStats publish_stats;
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
    onboard::app::GcsTelemetryPublisher* publisher,
    RateMeter& capture_rate,
    RateMeter& processing_rate)
{
    auto frame_result = readVisionFrameResult(source, processor);
    const double capture_fps = capture_rate.note(Clock::now());
    const double processing_fps = processing_rate.note(Clock::now());

    if (publisher) {
        onboard::app::GcsTelemetryPublishInput publish_input;
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
        onboard::app::GcsTelemetryPublisher& publisher,
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
    onboard::app::GcsTelemetryPublisher& publisher_;
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
        installShutdownSignalHandlers();
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
        const bool rc_override_backend =
            options.control_backend == ControlBackend::AltHoldRcOverride;
        if (options.alt_hold_auto_takeoff && !rc_override_backend) {
            std::cerr << "--alt-hold-auto-takeoff requires --control-backend alt_hold_rc_override\n";
            return 2;
        }
        if (real_serial_target && options.rc_override_smoke && !options.allow_rc_override) {
            std::cerr << "target " << config.endpoint.label
                      << " selected real serial endpoint " << config.endpoint.serial_device
                      << ':' << config.endpoint.serial_baudrate << "\n"
                      << "refusing RC override smoke on a real serial target without "
                      << "--allow-rc-override\n";
            return 2;
        }
        if (real_serial_target &&
            rc_override_backend &&
            !options.allow_rc_override) {
            std::cerr << "target " << config.endpoint.label
                      << " selected real serial endpoint " << config.endpoint.serial_device
                      << ':' << config.endpoint.serial_baudrate << "\n"
                      << "refusing ALT_HOLD RC override control on a real serial target without "
                      << "--allow-rc-override\n";
            return 2;
        }
        if (real_serial_target &&
            rc_override_backend &&
            options.alt_hold_auto_takeoff &&
            !options.allow_arm_takeoff) {
            std::cerr << "target " << config.endpoint.label
                      << " selected real serial endpoint " << config.endpoint.serial_device
                      << ':' << config.endpoint.serial_baudrate << "\n"
                      << "refusing ALT_HOLD RC auto-takeoff on a real serial target without "
                      << "--allow-arm-takeoff\n";
            return 2;
        }
        if (real_serial_target &&
            !rc_override_backend &&
            !options.rc_override_smoke &&
            !options.mavlink_smoke &&
            !options.no_arm &&
            !options.dry_run &&
            !options.allow_arm_takeoff) {
            std::cerr << "target " << config.endpoint.label
                      << " selected real serial endpoint " << config.endpoint.serial_device
                      << ':' << config.endpoint.serial_baudrate << "\n"
                      << "refusing to run the automatic arm/takeoff mission on a real serial target without "
                      << "--mavlink-smoke, --no-arm, --dry-run, or --allow-arm-takeoff\n";
            return 2;
        }

        std::cout << "[line_follow_node] target=" << config.endpoint.label
                  << " backend=" << toString(options.control_backend)
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
        std::cout << "[control] gains offset_kp=" << config.controller.offset_kp
                  << " angle_yaw_kp=" << config.controller.angle_yaw_kp
                  << " offset_yaw_kp=" << config.controller.offset_yaw_kp
                  << " max_lat=" << config.controller.max_lateral_mps
                  << " max_yaw=" << config.controller.max_yaw_rate_rad_s
                  << " ema_alpha=" << config.controller.output_ema_alpha
                  << " lat_rate=" << config.controller.max_lateral_rate_mps
                  << " yaw_rate_change=" << config.controller.max_yaw_rate_change_rad_s
                  << " fwd_conf_scale=" << config.controller.forward_confidence_scale << "\n";
        std::cout << "[control] marker center_x_kp=" << config.controller.marker_x_kp
                  << " center_y_kp=" << config.controller.marker_y_kp
                  << " max_marker=" << config.controller.max_marker_mps
                  << " deadband=" << config.controller.marker_deadband_norm
                  << " ema_alpha=" << config.controller.marker_output_ema_alpha
                  << " marker_rate=" << config.controller.max_marker_rate_mps
                  << " hover_recenter_timeout="
                  << config.mission.marker_hover_recenter_timeout_s << "\n";
        if (rc_override_backend || options.rc_override_smoke) {
            std::cout << "[rc_override] sender_system_id=" << config.rc_override.sender_system_id
                      << " trim=" << config.rc_override.trim_pwm
                      << " arm_throttle=" << config.rc_override.arm_throttle_pwm
                      << " throttle_hold=" << config.rc_override.throttle_hold_pwm
                      << " takeoff_throttle=" << config.rc_override.takeoff_throttle_pwm
                      << " max_delta roll/pitch/yaw="
                      << config.rc_override.max_roll_delta_pwm << '/'
                      << config.rc_override.max_pitch_delta_pwm << '/'
                      << config.rc_override.max_yaw_delta_pwm
                      << " full_scale roll/pitch/yaw="
                      << config.rc_override.roll_full_scale_mps << '/'
                      << config.rc_override.pitch_full_scale_mps << '/'
                      << config.rc_override.yaw_full_scale_rad_s
                      << " marker_scale roll/pitch="
                      << config.rc_override.marker_roll_full_scale_mps << '/'
                      << config.rc_override.marker_pitch_full_scale_mps
                      << " marker_servo kp_xy="
                      << config.rc_override.marker_servo_kp_x << '/'
                      << config.rc_override.marker_servo_kp_y
                      << " kd_xy="
                      << config.rc_override.marker_servo_kd_x << '/'
                      << config.rc_override.marker_servo_kd_y
                      << " max_lat/fwd/rev="
                      << config.rc_override.marker_servo_max_lateral_mps << '/'
                      << config.rc_override.marker_servo_max_forward_mps << '/'
                      << config.rc_override.marker_servo_max_reverse_mps
                      << " servo_rate=" << config.rc_override.marker_servo_max_rate_mps
                      << " takeoff_timeout_s=" << config.rc_override.takeoff_timeout_s
                      << " override_throttle="
                      << (config.rc_override.override_throttle ? "true" : "false")
                      << " invert roll/pitch/yaw="
                      << (config.rc_override.invert_roll ? "true" : "false") << '/'
                      << (config.rc_override.invert_pitch ? "true" : "false") << '/'
                      << (config.rc_override.invert_yaw ? "true" : "false") << "\n";
        }
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
        auto mavlink_ids = config.mavlink;
        if (rc_override_backend || options.rc_override_smoke) {
            mavlink_ids.system_id =
                static_cast<std::uint8_t>(config.rc_override.sender_system_id);
        }
        onboard::autopilot::AutopilotMavlinkAdapter autopilot(
            std::move(transport),
            mavlink_ids);
        onboard::control::GuidedVelocityController controller(config.controller);
        onboard::mission::LineFollowMission mission(config.mission);
        onboard::safety::SafetyMonitor safety(config.safety);
        RcOverrideReleaseGuard rc_override_guard(autopilot);

        if (options.rc_override_smoke) {
            std::cout << "[rc-override-smoke] waiting for heartbeat...\n";
            autopilot.waitHeartbeat(std::chrono::seconds(30));
            std::cout << "[rc-override-smoke] heartbeat ok system="
                      << static_cast<int>(autopilot.state().target_system)
                      << " component=" << static_cast<int>(autopilot.state().target_component)
                      << " mode=" << autopilot.state().mode_name
                      << " armed=" << (autopilot.state().armed ? "true" : "false") << "\n";
            autopilot.requestDefaultStreams();
            if (!waitRcReady(autopilot, config.safety, std::chrono::seconds(10))) {
                std::cerr << "[rc-override-smoke] RC input is not visible; refusing smoke\n";
                return 2;
            }
            return runRcOverrideSmoke(autopilot, config);
        }

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
        std::unique_ptr<onboard::app::GcsTelemetryPublisher> gcs_publisher;
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
                auto publisher = std::make_unique<onboard::app::GcsTelemetryPublisher>();
                onboard::app::GcsTelemetryPublisherOptions publisher_options;
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
        const auto period = std::chrono::duration<double>(
            1.0 / static_cast<double>(std::max(1, config.setpoint_rate_hz)));

        std::optional<LocalHoldAnchor> takeoff_anchor;
        if (rc_override_backend) {
            if (options.alt_hold_auto_takeoff) {
                if (!runAltHoldRcOverrideAutoTakeoff(
                        autopilot,
                        rc_override_guard,
                        config.safety,
                        config.rc_override,
                        config.mission.target_altitude_m,
                        config.mission.altitude_reached_ratio,
                        config.mission.takeoff_settle_s,
                        period)) {
                    if (shutdownRequested()) {
                        const bool disarmed = requestShutdownLandAndDisarm(
                            autopilot,
                            rc_override_guard,
                            config.rc_override,
                            period);
                        return disarmed ? 130 : 2;
                    }
                    return 2;
                }
            } else {
                if (!waitAltHoldPilotReady(
                        autopilot,
                        config.safety,
                        config.mission.target_altitude_m,
                        config.mission.altitude_reached_ratio,
                        options.alt_hold_set_mode,
                        std::chrono::seconds(60))) {
                    if (shutdownRequested()) {
                        const bool disarmed = requestShutdownLandAndDisarm(
                            autopilot,
                            rc_override_guard,
                            config.rc_override,
                            period);
                        return disarmed ? 130 : 2;
                    }
                    return 2;
                }
            }
        } else {
            if (!waitRcReady(autopilot, config.safety, std::chrono::seconds(10))) {
                std::cerr << "[safety] refusing GUIDED/arm/takeoff until RC input is visible\n";
                return 2;
            }
            if (real_serial_target &&
                !waitLocalHoldEstimateReady(autopilot, std::chrono::seconds(15))) {
                std::cerr << "[safety] refusing GUIDED/arm/takeoff until optical-flow local hold is ready\n";
                return 2;
            }
            takeoff_anchor = captureLocalHoldAnchor(autopilot.state());
            if (real_serial_target && !takeoff_anchor) {
                std::cerr << "[safety] refusing takeoff: local XY anchor is unavailable\n";
                return 2;
            }
            autopilot.setGuidedMode(std::chrono::seconds(10));
            std::cout << "[mavlink] GUIDED confirmed\n";
            autopilot.arm(std::chrono::seconds(30));
            std::cout << "[mavlink] armed\n";
        }

        const auto mission_started_at = Clock::now();
        mission.startTakeoff(mission_started_at);
        safety.startMission(mission_started_at);

        if (rc_override_backend) {
            std::cout << "[mission] ALT_HOLD_RC_OVERRIDE control start";
            if (options.alt_hold_auto_takeoff) {
                std::cout << "; auto-takeoff complete";
            } else {
                std::cout << "; pilot takeoff already complete";
            }
            std::cout << "\n";
        } else {
            autopilot.takeoff(config.mission.target_altitude_m);
            std::cout << "[mission] TAKEOFF target=" << config.mission.target_altitude_m << "m";
            if (takeoff_anchor) {
                std::cout << " hold_xy=(" << takeoff_anchor->x_m << ',' << takeoff_anchor->y_m << ')';
            }
            std::cout << "\n";
            const bool takeoff_altitude_reached = autopilot.waitAltitudeReached(
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
        RcMarkerVisualServo marker_servo;
        std::optional<Clock::time_point> rc_marker_hover_started_at;
        bool rc_marker_hover_land_logged = false;
        bool shutdown_land_logged = false;
#if defined(ONBOARD_LINE_FOLLOW_HAS_VISION)
        int mission_vision_frames = 0;
        onboard::app::GcsTelemetryPublishStats mission_publish_stats;
        onboard::vision::VisionProcessingMetrics last_vision_metrics;
        double last_read_frame_ms = 0.0;
        double last_processing_latency_ms = 0.0;
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
                    last_vision_metrics = frame_result.output.metrics;
                    last_read_frame_ms = frame_result.read_frame_ms;
                    last_processing_latency_ms = frame_result.processing_latency_ms;
                    line_input = toLineControlInput(result, config.desired_line_angle_deg);
                    marker_input = toMarkerControlInput(result);
                    marker_detected = marker_input.marker_detected;
                    marker_is_centered = markerCentered(result, config.marker_center_tolerance_px);
                    if (marker_detected) {
                        const auto marker = selectTargetMarker(result);
                        const double marker_dx_px = marker
                            ? marker->center_px.x - static_cast<double>(result.width) * 0.5
                            : 0.0;
                        const double marker_dy_px = marker
                            ? marker->center_px.y - static_cast<double>(result.height) * 0.5
                            : 0.0;
                        std::cout << "[vision] marker id=" << (marker ? marker->id : -1)
                                  << " centered=" << (marker_is_centered ? "yes" : "no")
                                  << " line=" << (line_input.line_detected ? "yes" : "no")
                                  << " marker_dx_px=" << marker_dx_px
                                  << " marker_dy_px=" << marker_dy_px
                                  << " line_offset_px=" << result.line.center_offset_px << "\n";
                    }
                } catch (const std::exception& error) {
                    std::cerr << "[vision] warning: " << error.what() << "\n";
                    line_input = {};
                }
            }
#endif

            if (rc_override_backend && marker_detected && !rc_marker_hover_started_at) {
                rc_marker_hover_started_at = loop_start;
                std::cout << "[mission] ALT_HOLD_RC_MARKER_HOVER timer start hold_s="
                          << config.mission.marker_hover_s << "\n";
            }
            const bool rc_marker_hover_timer_active =
                rc_override_backend && rc_marker_hover_started_at.has_value();
            const bool rc_marker_hover_complete =
                rc_marker_hover_timer_active &&
                std::chrono::duration<double>(
                    loop_start - *rc_marker_hover_started_at).count() >=
                    config.mission.marker_hover_s;
            if (rc_marker_hover_complete && !rc_marker_hover_land_logged) {
                std::cout << "[mission] ALT_HOLD_RC_MARKER_HOVER complete; requesting LAND\n";
                rc_marker_hover_land_logged = true;
            }
            const bool shutdown_land = shutdownRequested();
            if (shutdown_land && !shutdown_land_logged) {
                std::cerr << "[safety] Ctrl+C/SIGTERM received; requesting LAND\n";
                shutdown_land_logged = true;
            }

            const bool line_tracking_ready = lineTrackingActive(line_input, config.controller);
            const auto& ap_state_for_safety = autopilot.state();
            const bool expected_control_mode = rc_override_backend
                ? ap_state_for_safety.custom_mode == COPTER_MODE_ALT_HOLD
                : ap_state_for_safety.custom_mode == COPTER_MODE_GUIDED;
            const auto safety_decision = safety.update(onboard::safety::SafetyInput {
                ap_state_for_safety.heartbeat_seen,
                line_tracking_ready || marker_detected,
                loop_start,
                ap_state_for_safety.last_heartbeat_time,
                ap_state_for_safety.rc_channel_count.has_value(),
                ap_state_for_safety.rc_channel_count.value_or(0),
                ap_state_for_safety.last_rc_channels_time,
                ap_state_for_safety.heartbeat_seen,
                expected_control_mode,
            });

            if (safety_decision.action == onboard::safety::SafetyAction::Abort) {
                std::string abort_reason = safety_decision.reason;
                if (rc_override_backend &&
                    abort_reason == "operator takeover: mode changed from GUIDED") {
                    abort_reason = "operator takeover: mode changed from ALT_HOLD";
                }
                operator_takeover =
                    abort_reason.find("operator takeover") != std::string::npos;
                mission.abort(abort_reason);
                break;
            }

            const bool safety_land =
                safety_decision.action == onboard::safety::SafetyAction::Land;
            const auto altitude_now = autopilot.bestAltitudeM();
            const bool marker_centered_for_mission =
                marker_is_centered || (rc_marker_hover_timer_active && marker_detected);
            const bool rc_marker_hover_forced_land =
                rc_marker_hover_complete &&
                mission.state() != onboard::mission::LineFollowMissionState::MarkerHover;
            mission.update(onboard::mission::LineFollowMissionInput {
                altitude_now.has_value(),
                altitude_now.value_or(0.0),
                line_tracking_ready,
                marker_detected,
                marker_centered_for_mission,
                safety_land || rc_marker_hover_forced_land || shutdown_land,
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
            bool use_marker_stick_scale = false;
            std::string hold_reason;
            if (mission.state() == onboard::mission::LineFollowMissionState::MarkerHover) {
                if (rc_override_backend && marker_detected) {
                    setpoint = marker_servo.update(
                        marker_input,
                        config.controller,
                        config.rc_override,
                        loop_start);
                    use_marker_stick_scale = true;
                    hold_reason = "marker_hover_recenter";
                } else {
                    marker_servo.reset();
                    setpoint = controller.stop(altitude_input);
                    use_local_hold = true;
                    hold_reason = "marker_hover";
                }
            } else if (mission.state() == onboard::mission::LineFollowMissionState::MarkerApproach) {
                if (marker_detected) {
                    if (rc_override_backend) {
                        setpoint = marker_servo.update(
                            marker_input,
                            config.controller,
                            config.rc_override,
                            loop_start);
                        use_marker_stick_scale = true;
                    } else {
                        setpoint = controller.updateMarker(marker_input, altitude_input);
                    }
                    active_hold_anchor.reset();
                } else if (line_tracking_ready) {
                    marker_servo.reset();
                    setpoint = controller.updateLine(line_input, altitude_input);
                    active_hold_anchor.reset();
                } else {
                    marker_servo.reset();
                    setpoint = controller.stop(altitude_input);
                    use_local_hold = true;
                    hold_reason = "marker_approach_no_target";
                }
            } else {
                marker_servo.reset();
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
            if (rc_override_backend) {
                sendRcOverrideSetpoint(
                    autopilot,
                    rc_override_guard,
                    setpoint,
                    config.controller,
                    config.rc_override,
                    use_marker_stick_scale);
                command_frame = use_local_hold ? "rc_override_neutral" :
                    (use_marker_stick_scale ? "rc_override_marker_servo" : "rc_override");
            } else if (use_local_hold) {
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
                    sendAnchoredVelocitySetpoint(
                        autopilot,
                        *active_hold_anchor,
                        setpoint,
                        config.mission.target_altitude_m);
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
                if (options.profile_vision && frame_source && vision_processor) {
                    std::cout << " vision_read_ms=" << last_read_frame_ms
                              << " vision_process_ms=" << last_processing_latency_ms
                              << " aruco_ms=" << last_vision_metrics.aruco_latency_ms
                              << " line_ms=" << last_vision_metrics.line_latency_ms
                              << " intersect_ms=" << last_vision_metrics.intersection_latency_ms;
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
            if (rc_override_backend) {
                rc_override_guard.releaseNoThrow();
                std::cerr << "[safety] RC override release sent\n";
            }
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

        auto landing_anchor = rc_override_backend
            ? std::optional<LocalHoldAnchor> {}
            : captureLocalHoldAnchor(autopilot.state());
        if (!rc_override_backend && !landing_anchor && active_hold_anchor) {
            landing_anchor = active_hold_anchor;
        }
        const onboard::control::AltitudeControlInput landing_altitude_input {
            autopilot.bestAltitudeM().has_value(),
            autopilot.bestAltitudeM().value_or(config.mission.target_altitude_m),
            config.mission.target_altitude_m,
        };
        const auto stop_setpoint = controller.stop(landing_altitude_input);
        if (rc_override_backend) {
            sendRcOverrideNeutral(autopilot, rc_override_guard, config.rc_override);
            rc_override_guard.release();
            std::cout << "[mission] RC override neutral/release before LAND\n";
        } else if (landing_anchor) {
            sendAnchoredVelocitySetpoint(
                autopilot,
                *landing_anchor,
                stop_setpoint,
                config.mission.target_altitude_m);
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
                      << landing_anchor->x_m << ',' << landing_anchor->y_m
                      << ") until low altitude, then LAND mode\n";
        } else if (rc_override_backend) {
            std::cout << "[mission] ALT_HOLD_RC_OVERRIDE complete; requesting LAND mode\n";
            autopilot.setLandMode(std::chrono::seconds(10));
        } else {
            std::cerr << "[mission] local XY unavailable; using LAND mode\n";
            autopilot.setLandMode(std::chrono::seconds(10));
        }
        const auto landing_started_at = Clock::now();
        const auto disarm_deadline = landing_started_at + std::chrono::seconds(90);
        bool disarmed = false;
        bool land_mode_requested = !landing_anchor;
        bool disarm_requested = false;
        auto land_mode_since = land_mode_requested ? Clock::now() : Clock::time_point {};
        auto touchdown_stable_since = Clock::time_point {};
        auto last_disarm_request = Clock::time_point {};
        int landing_video_frames = 0;
#if defined(ONBOARD_LINE_FOLLOW_HAS_VISION)
        onboard::app::GcsTelemetryPublishStats landing_publish_stats;
#endif
        while (Clock::now() < disarm_deadline) {
            const auto loop_start = Clock::now();
            autopilot.poll(1);
#if defined(ONBOARD_LINE_FOLLOW_HAS_VISION)
            const bool skip_landing_video =
                land_mode_requested && touchdownLikely(autopilot.state());
            if (frame_source && vision_processor && gcs_publisher && !skip_landing_video) {
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

            if (!land_mode_requested &&
                (lowEnoughForLandMode(autopilot.state()) ||
                 loop_start - landing_started_at >= std::chrono::seconds(20))) {
                std::cout << "[mission] LAND_MODE height="
                          << landingHeightM(autopilot.state()).value_or(-1.0)
                          << "m\n";
                autopilot.setLandMode(std::chrono::seconds(10));
                land_mode_requested = true;
                land_mode_since = Clock::now();
            }

            if (!land_mode_requested && landing_anchor) {
                float descent_mps = 0.25f;
                const auto range = autopilot.state().distance_sensor_m;
                if (range && *range < 0.8) {
                    descent_mps = 0.15f;
                }
                if (range && *range < 0.45) {
                    descent_mps = 0.10f;
                }
                sendAnchoredDescentSetpoint(autopilot, *landing_anchor, descent_mps);
            } else if (land_mode_requested && touchdownLikely(autopilot.state())) {
                if (touchdown_stable_since.time_since_epoch().count() == 0) {
                    touchdown_stable_since = loop_start;
                }
                const auto touchdown_stable_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        loop_start - touchdown_stable_since)
                        .count();
                const bool land_mode_settled =
                    land_mode_since.time_since_epoch().count() != 0 &&
                    loop_start - land_mode_since >= std::chrono::seconds(5);
                const bool retry_due =
                    last_disarm_request.time_since_epoch().count() == 0 ||
                    loop_start - last_disarm_request >= std::chrono::seconds(4);
                if (touchdown_stable_ms >= 6000 && land_mode_settled && retry_due) {
                    std::cout << "[mission] touchdown stable height="
                              << landingHeightM(autopilot.state()).value_or(-1.0)
                              << "m; requesting disarm\n";
                    autopilot.requestDisarm();
                    disarm_requested = true;
                    last_disarm_request = loop_start;
                }
            } else {
                touchdown_stable_since = Clock::time_point {};
            }
            std::this_thread::sleep_until(loop_start + period);
        }
        if (!disarmed) {
            if (!land_mode_requested) {
                std::cerr << "[mission] landing timeout before LAND mode; switching to LAND\n";
                autopilot.setLandMode(std::chrono::seconds(10));
            }
            if (touchdownLikely(autopilot.state())) {
                std::cout << "[mission] final touchdown disarm request height="
                          << landingHeightM(autopilot.state()).value_or(-1.0) << "m\n";
                autopilot.requestDisarm();
                disarmed = autopilot.waitDisarmed(std::chrono::seconds(10));
            }
            if (!disarmed && !disarm_requested) {
                disarmed = autopilot.waitDisarmed(std::chrono::seconds(10));
            }
            if (!disarmed) {
                throw std::runtime_error("timed out waiting for disarmed state");
            }
            if (disarmed) {
                disarmed = true;
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
