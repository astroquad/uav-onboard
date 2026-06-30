// astroquad-onboard — composition root for the full grid snake mission.
//
// grid_mission_node is kept as a compatibility target that builds this same
// source while the team transitions scripts and habits to astroquad-onboard.
//
// Composition root that wires VisionRuntime + AutopilotMavlinkAdapter +
// GridMission state machine + GridControlMapper. Reuses line_follow_node
// helpers in spirit but lives in its own translation unit so the MVP
// line-following path keeps working unchanged.
//
// Algorithm rules (see development-log plan §0):
//   - Never use Gazebo ground-truth pose. Only MAVLink LOCAL_POSITION_NED,
//     ATTITUDE, rangefinder, vision events, and time are allowed inputs.
//   - LOCAL_POSITION_NED is an EKF estimate (same on SITL and real ArduPilot serial),
//     so it is OK as a distance/hover source within short windows.

#include "app/AstroquadOnboardApp.hpp"
#include "app/GcsTelemetryPublisher.hpp"
#include "autopilot/AutopilotMavlinkAdapter.hpp"
#include "autopilot/SerialMavlinkTransport.hpp"
#include "autopilot/UdpMavlinkTransport.hpp"

#include <mavlink/v2.0/ardupilotmega/mavlink.h>
#include "common/NetworkConfig.hpp"
#include "common/VisionConfig.hpp"
#include "control/GridControlMapper.hpp"
#include "control/GuidedVelocityController.hpp"
#include "mission/AltitudePolicy.hpp"
#include "mission/GridCoordinateTracker.hpp"
#include "mission/GridMission.hpp"
#include "mission/IntersectionDecision.hpp"
#include "mission/MarkerRegistry.hpp"
#include "mission/SnakePlanner.hpp"
#include "safety/SafetyMonitor.hpp"
#include "vision/FakeFrameSource.hpp"
#include "vision/GazeboCameraSource.hpp"
#include "vision/RpicamFrameSource.hpp"
#include "vision/VisionProcessor.hpp"

#include <toml++/toml.hpp>

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

#ifndef ONBOARD_PROGRAM_NAME
#define ONBOARD_PROGRAM_NAME "astroquad-onboard"
#endif

using Clock = std::chrono::steady_clock;
namespace omission = onboard::mission;
namespace oapp = onboard::app;
namespace ocontrol = onboard::control;
namespace ovision = onboard::vision;
namespace oautopilot = onboard::autopilot;
namespace ocommon = onboard::common;
namespace osafety = onboard::safety;

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

struct Options {
    std::string config_dir = "config";
    std::string target = "sitl";
    std::string autopilot_uri;
    std::string vision;
    std::string world = "grid";
    std::string line_mode_override = "dark_on_light";
    std::string gcs_ip_override;
    std::string gazebo_topic_override;
    int marker_count = -1;
    int vertiport_marker_id = 23;
    int max_intersections_override = 0;
    int debug_video_fps_override = 0;
    std::string snake_initial_turn = "auto";
    std::string revisit_order;
    bool debug_video_fps_specified = false;
    bool send_video = false;
    bool send_video_overridden = false;
    bool send_telemetry = true;
    bool allow_arm_takeoff = false;
    bool no_arm = false;
    bool unsafe_assume_rc_present = false;
};

std::atomic_bool g_shutdown_requested {false};

void handleSigint(int) { g_shutdown_requested.store(true); }

std::string joinPath(const std::string& dir, const std::string& file)
{
    if (dir.empty()) return "config/" + file;
    return dir.back() == '/' ? dir + file : dir + "/" + file;
}

int parseInt(const std::string& s, int fallback)
{
    try { return std::stoi(s); } catch (...) { return fallback; }
}

void printUsage()
{
    std::cout
        << "Usage: " << ONBOARD_PROGRAM_NAME << " [options]\n\n"
        << "Required:\n"
        << "  --marker-count <n>           Expected number of grid ArUco markers (vertiport excluded)\n"
        << "\n"
        << "Options:\n"
        << "  --config <dir>               Config directory (default: config)\n"
        << "  --target <sitl|ardupilot_serial>\n"
        << "                               Runtime target (default: sitl)\n"
        << "  --autopilot <uri>            Override endpoint, e.g. udp://0.0.0.0:14550\n"
        << "                               or serial:///dev/serial0:115200\n"
        << "  --vision <fake|gazebo|rpicam>\n"
        << "                               Vision source (default: target runtime profile)\n"
        << "  --world <grid|line>          Gazebo world profile (default: grid)\n"
        << "  --line-mode <mode>           light_on_dark | dark_on_light | auto\n"
        << "  --gcs-ip <ip>                Override GCS destination IP\n"
        << "  --gazebo-topic <topic>       Override Gazebo camera image topic\n"
        << "  --vertiport-marker-id <id>   Override vertiport ArUco ID (default: 23)\n"
        << "  --max-intersections <n>      Safety cap on recorded nodes\n"
        << "  --snake-initial-turn <auto|left|right>\n"
        << "  --revisit-order <asc|desc>     Marker revisit order (default: desc)\n"
        << "  --fps <n>                   Override GCS debug video send FPS\n"
        << "  --video                      Enable GCS MJPEG streaming\n"
        << "  --no-video                   Disable GCS MJPEG streaming\n"
        << "  --no-telemetry               Disable GCS telemetry sending\n"
        << "  --allow-arm-takeoff          Permit real serial arm+takeoff\n"
        << "  --no-arm                     Skip arm/takeoff (vision/state-machine smoke)\n"
        << "  --unsafe-assume-rc-present   Bypass MAVLink RC gate for real-flight debugging\n"
        << "  -h, --help                   Show this help\n";
}

Options parseOptions(int argc, char** argv)
{
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* what) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "missing argument for " << what << "\n";
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--config") o.config_dir = next("--config");
        else if (a == "--target") o.target = next("--target");
        else if (a == "--autopilot") o.autopilot_uri = next("--autopilot");
        else if (a == "--vision") o.vision = next("--vision");
        else if (a == "--world") o.world = next("--world");
        else if (a == "--line-mode") o.line_mode_override = next("--line-mode");
        else if (a == "--gcs-ip") o.gcs_ip_override = next("--gcs-ip");
        else if (a == "--gazebo-topic") o.gazebo_topic_override = next("--gazebo-topic");
        else if (a == "--marker-count") o.marker_count = parseInt(next("--marker-count"), -1);
        else if (a == "--vertiport-marker-id") o.vertiport_marker_id = parseInt(next("--vertiport-marker-id"), 23);
        else if (a == "--max-intersections") o.max_intersections_override = parseInt(next("--max-intersections"), 0);
        else if (a == "--fps") {
            o.debug_video_fps_override = parseInt(next("--fps"), 0);
            o.debug_video_fps_specified = true;
        }
        else if (a == "--snake-initial-turn") o.snake_initial_turn = next("--snake-initial-turn");
        else if (a == "--revisit-order") o.revisit_order = next("--revisit-order");
        else if (a == "--video") { o.send_video = true; o.send_video_overridden = true; }
        else if (a == "--no-video") { o.send_video = false; o.send_video_overridden = true; }
        else if (a == "--no-telemetry") o.send_telemetry = false;
        else if (a == "--allow-arm-takeoff") o.allow_arm_takeoff = true;
        else if (a == "--no-arm") o.no_arm = true;
        else if (a == "--unsafe-assume-rc-present") o.unsafe_assume_rc_present = true;
        else if (a == "-h" || a == "--help") { printUsage(); std::exit(0); }
        else {
            std::cerr << "unknown option: " << a << "\n";
            printUsage();
            std::exit(2);
        }
    }
    return o;
}

struct Configs {
    EndpointConfig endpoint;
    oautopilot::MavlinkIds mavlink;
    ocommon::NetworkConfig network;
    ocommon::VisionConfig vision;
    omission::GridMissionConfig mission;
    ocontrol::GuidedVelocityControllerConfig controller;
    ocontrol::GridControlMapperConfig mapper;
    omission::AltitudePolicyConfig altitude;
    omission::SnakePlannerConfig snake;
    osafety::SafetyConfig safety;
    std::string vision_source = "gazebo";
    int setpoint_rate_hz = 20;
};

std::uint16_t parsePort(const std::string& value, std::uint16_t fallback)
{
    const int parsed = parseInt(value, fallback);
    if (parsed <= 0 || parsed > 65535) {
        return fallback;
    }
    return static_cast<std::uint16_t>(parsed);
}

void applyAutopilotUri(Configs& cfg, const std::string& uri)
{
    if (uri.rfind("udp://", 0) == 0) {
        const std::string rest = uri.substr(6);
        const auto colon = rest.rfind(':');
        cfg.endpoint.kind = TransportKind::Udp;
        cfg.endpoint.label = "sitl";
        if (colon == std::string::npos) {
            cfg.endpoint.listen_port = parsePort(rest, cfg.endpoint.listen_port);
            return;
        }
        cfg.endpoint.listen_host = rest.substr(0, colon);
        cfg.endpoint.listen_port =
            parsePort(rest.substr(colon + 1), cfg.endpoint.listen_port);
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
        cfg.endpoint.kind = TransportKind::Serial;
        cfg.endpoint.label = "ardupilot_serial";
        if (colon == std::string::npos) {
            cfg.endpoint.serial_device = rest;
            return;
        }
        cfg.endpoint.serial_device = rest.substr(0, colon);
        cfg.endpoint.serial_baudrate =
            parseInt(rest.substr(colon + 1), cfg.endpoint.serial_baudrate);
        return;
    }

    throw std::runtime_error("unsupported --autopilot URI: " + uri);
}

void applyTransportConfig(const toml::node_view<toml::node> transport, Configs& cfg)
{
    if (!transport) return;
    const std::string kind = transport["kind"].value_or(std::string(""));
    if (kind == "udp") {
        cfg.endpoint.kind = TransportKind::Udp;
    } else if (kind == "serial") {
        cfg.endpoint.kind = TransportKind::Serial;
    }
    cfg.endpoint.listen_host =
        transport["listen_host"].value_or(cfg.endpoint.listen_host);
    cfg.endpoint.listen_port = static_cast<std::uint16_t>(
        transport["listen_port"].value_or(static_cast<int>(cfg.endpoint.listen_port)));
}

void applySerialConfig(const toml::node_view<toml::node> serial, Configs& cfg)
{
    if (!serial) return;
    cfg.endpoint.serial_device =
        serial["device"].value_or(cfg.endpoint.serial_device);
    cfg.endpoint.serial_baudrate =
        serial["baudrate"].value_or(cfg.endpoint.serial_baudrate);
}

void applyMavlinkConfig(const toml::node_view<toml::node> mavlink, Configs& cfg)
{
    if (!mavlink) return;
    cfg.mavlink.system_id = static_cast<std::uint8_t>(
        mavlink["system_id"].value_or(static_cast<int>(cfg.mavlink.system_id)));
    cfg.mavlink.component_id = static_cast<std::uint8_t>(
        mavlink["component_id"].value_or(static_cast<int>(cfg.mavlink.component_id)));
    cfg.mavlink.target_system = static_cast<std::uint8_t>(
        mavlink["target_system"].value_or(static_cast<int>(cfg.mavlink.target_system)));
    cfg.mavlink.target_component = static_cast<std::uint8_t>(
        mavlink["target_component"].value_or(static_cast<int>(cfg.mavlink.target_component)));
    cfg.setpoint_rate_hz =
        mavlink["setpoint_rate_hz"].value_or(cfg.setpoint_rate_hz);
}

void applySafetyTimeouts(const toml::node_view<toml::node> timeouts, Configs& cfg)
{
    if (!timeouts) return;
    cfg.safety.line_lost_ms =
        timeouts["line_lost_ms"].value_or(cfg.safety.line_lost_ms);
    cfg.safety.autopilot_heartbeat_lost_ms =
        timeouts["autopilot_heartbeat_lost_ms"].value_or(
            cfg.safety.autopilot_heartbeat_lost_ms);
    cfg.safety.mission_timeout_ms =
        timeouts["mission_timeout_ms"].value_or(cfg.safety.mission_timeout_ms);
    cfg.mission.autopilot_heartbeat_timeout_s =
        static_cast<double>(cfg.safety.autopilot_heartbeat_lost_ms) / 1000.0;
    cfg.mission.mission_timeout_s =
        static_cast<double>(cfg.safety.mission_timeout_ms) / 1000.0;
}

void applySafetyRc(const toml::node_view<toml::node> rc, Configs& cfg)
{
    if (!rc) return;
    cfg.safety.rc_required =
        rc["rc_required"].value_or(cfg.safety.rc_required);
    cfg.safety.assume_rc_present =
        rc["assume_rc_present"].value_or(cfg.safety.assume_rc_present);
    cfg.safety.rc_lost_ms =
        rc["rc_lost_ms"].value_or(cfg.safety.rc_lost_ms);
}

void applySafetyPreflight(const toml::node_view<toml::node> preflight, Configs& cfg)
{
    if (!preflight) return;
    cfg.safety.ekf_pos_horiz_variance_max =
        preflight["ekf_pos_horiz_variance_max"].value_or(
            cfg.safety.ekf_pos_horiz_variance_max);
    cfg.safety.ekf_velocity_variance_max =
        preflight["ekf_velocity_variance_max"].value_or(
            cfg.safety.ekf_velocity_variance_max);
}

void applyDebugVideoConfig(const toml::node_view<toml::node> debug_video, Configs& cfg)
{
    if (!debug_video) return;
    cfg.vision.debug_video.enabled =
        debug_video["enabled"].value_or(cfg.vision.debug_video.enabled);
    cfg.vision.debug_video.send_fps =
        debug_video["send_fps"].value_or(cfg.vision.debug_video.send_fps);
    cfg.vision.debug_video.jpeg_quality =
        debug_video["jpeg_quality"].value_or(cfg.vision.debug_video.jpeg_quality);
    cfg.vision.debug_video.chunk_pacing_us =
        debug_video["chunk_pacing_us"].value_or(cfg.vision.debug_video.chunk_pacing_us);
    cfg.vision.debug_video.send_width =
        debug_video["send_width"].value_or(cfg.vision.debug_video.send_width);
    cfg.vision.debug_video.send_height =
        debug_video["send_height"].value_or(cfg.vision.debug_video.send_height);
}

void applyLineControllerConfig(const toml::node_view<toml::node> line_controller, Configs& cfg)
{
    if (!line_controller) return;
    cfg.controller.offset_kp =
        line_controller["offset_kp"].value_or(cfg.controller.offset_kp);
    cfg.controller.offset_ki =
        line_controller["offset_ki"].value_or(cfg.controller.offset_ki);
    cfg.controller.angle_yaw_kp =
        line_controller["angle_yaw_kp"].value_or(cfg.controller.angle_yaw_kp);
    cfg.controller.offset_yaw_kp =
        line_controller["offset_yaw_kp"].value_or(cfg.controller.offset_yaw_kp);
    cfg.controller.max_lateral_mps =
        line_controller["max_lateral_mps"].value_or(cfg.controller.max_lateral_mps);
    cfg.controller.max_yaw_rate_rad_s =
        line_controller["max_yaw_rate_rad_s"].value_or(cfg.controller.max_yaw_rate_rad_s);
    cfg.controller.min_confidence =
        line_controller["min_confidence"].value_or(cfg.controller.min_confidence);
    cfg.controller.offset_deadband_norm =
        line_controller["offset_deadband_norm"].value_or(cfg.controller.offset_deadband_norm);
    const double angle_deadband_deg =
        line_controller["angle_deadband_deg"].value_or(
            cfg.controller.angle_deadband_rad * 180.0 / M_PI);
    cfg.controller.angle_deadband_rad = angle_deadband_deg * M_PI / 180.0;
    cfg.controller.invert_lateral =
        line_controller["invert_lateral"].value_or(cfg.controller.invert_lateral);
    cfg.controller.invert_yaw =
        line_controller["invert_yaw"].value_or(cfg.controller.invert_yaw);
    cfg.controller.output_ema_alpha =
        line_controller["output_ema_alpha"].value_or(cfg.controller.output_ema_alpha);
    cfg.controller.max_lateral_rate_mps =
        line_controller["max_lateral_rate_mps"].value_or(cfg.controller.max_lateral_rate_mps);
    cfg.controller.max_yaw_rate_change_rad_s =
        line_controller["max_yaw_rate_change_rad_s"].value_or(
            cfg.controller.max_yaw_rate_change_rad_s);
    cfg.controller.forward_confidence_scale =
        line_controller["forward_confidence_scale"].value_or(
            cfg.controller.forward_confidence_scale);
}

void applyAltitudeHoldConfig(const toml::node_view<toml::node> altitude_hold, Configs& cfg)
{
    if (!altitude_hold) return;
    cfg.controller.altitude_kp =
        altitude_hold["kp"].value_or(cfg.controller.altitude_kp);
    cfg.controller.max_vz_down_mps =
        altitude_hold["max_vz_mps"].value_or(cfg.controller.max_vz_down_mps);
    cfg.controller.altitude_deadband_m =
        altitude_hold["deadband_m"].value_or(cfg.controller.altitude_deadband_m);
}

void applyMarkerHoverConfig(const toml::node_view<toml::node> marker_hover,
                            Configs& cfg)
{
    if (!marker_hover) return;

    cfg.mission.snake_marker_hover_s =
        marker_hover["hold_s"].value_or(cfg.mission.snake_marker_hover_s);
    cfg.mission.marker_hover_min_s =
        marker_hover["min_s"].value_or(
            marker_hover["min_center_s"].value_or(cfg.mission.marker_hover_min_s));
    cfg.mission.marker_hover_center_stable_frames =
        marker_hover["center_stable_frames"].value_or(
            cfg.mission.marker_hover_center_stable_frames);

    if (const auto center_tol_norm =
            marker_hover["center_tolerance_norm"].value<double>()) {
        cfg.mission.marker_hover_center_tolerance_norm = *center_tol_norm;
    } else if (const auto center_tol_px =
                   marker_hover["center_tolerance_px"].value<double>()) {
        const double half_w =
            std::max(1.0, static_cast<double>(cfg.vision.camera.width) * 0.5);
        const double half_h =
            std::max(1.0, static_cast<double>(cfg.vision.camera.height) * 0.5);
        cfg.mission.marker_hover_center_tolerance_norm =
            std::max(*center_tol_px / half_w, *center_tol_px / half_h);
    }

    if (const auto center_kp = marker_hover["center_kp"].value<double>()) {
        cfg.controller.marker_x_kp = *center_kp;
        cfg.controller.marker_y_kp = *center_kp;
    }
    cfg.controller.marker_x_kp =
        marker_hover["center_x_kp"].value_or(cfg.controller.marker_x_kp);
    cfg.controller.marker_y_kp =
        marker_hover["center_y_kp"].value_or(cfg.controller.marker_y_kp);
    cfg.controller.max_marker_mps =
        marker_hover["max_lateral_mps"].value_or(
            marker_hover["max_marker_mps"].value_or(cfg.controller.max_marker_mps));
    cfg.controller.marker_deadband_norm =
        marker_hover["deadband_norm"].value_or(
            marker_hover["center_deadband_norm"].value_or(
                cfg.controller.marker_deadband_norm));
    cfg.controller.marker_output_ema_alpha =
        marker_hover["output_ema_alpha"].value_or(
            cfg.controller.marker_output_ema_alpha);
    cfg.controller.max_marker_rate_mps =
        marker_hover["max_marker_rate_mps"].value_or(
            marker_hover["max_lateral_rate_mps"].value_or(
                cfg.controller.max_marker_rate_mps));
    cfg.controller.invert_marker_x =
        marker_hover["invert_x"].value_or(cfg.controller.invert_marker_x);
    cfg.controller.invert_marker_y =
        marker_hover["invert_y"].value_or(cfg.controller.invert_marker_y);
}

void applyMarkerHoverConfigFile(const std::string& path, Configs& cfg)
{
    try {
        auto table = toml::parse_file(path);
        applyMarkerHoverConfig(table["marker_hover"], cfg);
    } catch (const toml::parse_error& e) {
        std::cerr << "[config] marker_hover parse warning (" << path
                  << "): " << e.what() << "\n";
    }
}

void loadConfigs(const Options& opt, Configs& cfg)
{
    const std::string target = opt.target;

    // Network (shared with line-follow path)
    cfg.network = ocommon::loadNetworkConfig(opt.config_dir);

    // Vision
    cfg.vision = ocommon::loadVisionConfig(opt.config_dir);
    if (!opt.line_mode_override.empty()) {
        cfg.vision.line.mode = opt.line_mode_override;
    }
    if (!opt.gazebo_topic_override.empty()) {
        cfg.vision.source.gazebo_topic = opt.gazebo_topic_override;
    }
    if (!opt.gcs_ip_override.empty()) {
        cfg.network.gcs_ip = opt.gcs_ip_override;
    }

    try {
        auto ap = toml::parse_file(joinPath(opt.config_dir, "autopilot.toml"));
        applyTransportConfig(ap["transport"], cfg);
        applySerialConfig(ap["serial"], cfg);
        applyMavlinkConfig(ap["mavlink"], cfg);
    } catch (const toml::parse_error& e) {
        std::cerr << "[config] autopilot.toml parse warning: " << e.what() << "\n";
    }

    if (target == "sitl") {
        cfg.endpoint.kind = TransportKind::Udp;
        cfg.endpoint.label = "sitl";
        cfg.vision_source = "gazebo";
    } else if (target == "ardupilot_serial") {
        cfg.endpoint.kind = TransportKind::Serial;
        cfg.endpoint.label = "ardupilot_serial";
        cfg.vision_source = "rpicam";
    } else {
        throw std::runtime_error("unknown --target: " + opt.target);
    }

    try {
        auto s = toml::parse_file(joinPath(opt.config_dir, "safety.toml"));
        applySafetyTimeouts(s["timeouts"], cfg);
        applySafetyRc(s["rc"], cfg);
        applySafetyPreflight(s["preflight"], cfg);
    } catch (const toml::parse_error& e) {
        std::cerr << "[config] safety.toml parse warning: " << e.what() << "\n";
    }

    // Mission config from mission.toml [grid_mission]
    try {
        auto m = toml::parse_file(joinPath(opt.config_dir, "mission.toml"));
        auto gm = m["grid_mission"];
        if (gm) {
            auto& g = cfg.mission;
            g.vertiport_marker_id = gm["vertiport_marker_id"].value_or(g.vertiport_marker_id);
            g.markers_expected = gm["markers_expected"].value_or(g.markers_expected);
            g.vertiport_altitude_m = gm["vertiport_altitude_m"].value_or(g.vertiport_altitude_m);
            g.cruise_altitude_m = gm["cruise_altitude_m"].value_or(g.cruise_altitude_m);
            g.vertiport_verify_timeout_s = gm["vertiport_verify_timeout_s"].value_or(g.vertiport_verify_timeout_s);
            g.marker_lock_center_tol_norm =
                gm["marker_lock_center_tol_norm"].value_or(g.marker_lock_center_tol_norm);
            const double marker_lock_yaw_deg = gm["marker_lock_yaw_delta_deg"].value_or(
                g.marker_lock_yaw_delta_rad * 180.0 / M_PI);
            g.marker_lock_yaw_delta_rad = marker_lock_yaw_deg * M_PI / 180.0;
            g.entry_forward_timeout_s = gm["entry_forward_timeout_s"].value_or(g.entry_forward_timeout_s);
            g.entry_forward_speed_mps = gm["entry_forward_speed_mps"].value_or(g.entry_forward_speed_mps);
            g.entry_blind_clear_distance_m =
                gm["entry_blind_clear_distance_m"].value_or(g.entry_blind_clear_distance_m);
            g.entry_blind_min_s = gm["entry_blind_min_s"].value_or(g.entry_blind_min_s);
            g.entry_blind_min_frames =
                gm["entry_blind_min_frames"].value_or(g.entry_blind_min_frames);
            g.entry_intersection_min_distance_m =
                gm["entry_intersection_min_distance_m"].value_or(g.entry_intersection_min_distance_m);
            g.entry_center_timeout_s = gm["entry_center_timeout_s"].value_or(g.entry_center_timeout_s);
            g.entry_center_target_y_norm =
                gm["entry_center_target_y_norm"].value_or(g.entry_center_target_y_norm);
            g.entry_center_late_y_norm =
                gm["entry_center_late_y_norm"].value_or(g.entry_center_late_y_norm);
            g.entry_center_x_tolerance_norm =
                gm["entry_center_x_tolerance_norm"].value_or(g.entry_center_x_tolerance_norm);
            g.entry_center_y_tolerance_norm =
                gm["entry_center_y_tolerance_norm"].value_or(g.entry_center_y_tolerance_norm);
            g.entry_center_velocity_threshold_mps =
                gm["entry_center_velocity_threshold_mps"].value_or(g.entry_center_velocity_threshold_mps);
            g.entry_center_stable_frames =
                gm["entry_center_stable_frames"].value_or(g.entry_center_stable_frames);
            g.cell_size_m = gm["cell_size_m"].value_or(g.cell_size_m);
            g.snake_record_lockout_s = gm["snake_record_lockout_s"].value_or(g.snake_record_lockout_s);
            g.snake_turn_lockout_s = gm["snake_turn_lockout_s"].value_or(g.snake_turn_lockout_s);
            g.snake_marker_hover_s = gm["snake_marker_hover_s"].value_or(g.snake_marker_hover_s);
            g.marker_hover_min_s = gm["marker_hover_min_s"].value_or(g.marker_hover_min_s);
            g.marker_hover_center_tolerance_norm =
                gm["marker_hover_center_tolerance_norm"].value_or(
                    g.marker_hover_center_tolerance_norm);
            g.marker_hover_center_stable_frames =
                gm["marker_hover_center_stable_frames"].value_or(
                    g.marker_hover_center_stable_frames);
            g.snake_advance_timeout_s = gm["snake_advance_timeout_s"].value_or(g.snake_advance_timeout_s);
            g.max_intersections = gm["max_intersections"].value_or(g.max_intersections);
            g.mission_timeout_s = gm["mission_timeout_s"].value_or(g.mission_timeout_s);
            g.altitude_ceiling_m = gm["altitude_ceiling_m"].value_or(g.altitude_ceiling_m);
            g.snake_complete_hover_s = gm["snake_complete_hover_s"].value_or(g.snake_complete_hover_s);
            g.snake_record_dwell_s =
                gm["snake_record_dwell_s"].value_or(g.snake_record_dwell_s);
            g.snake_boundary_record_dwell_s =
                gm["snake_boundary_record_dwell_s"].value_or(g.snake_boundary_record_dwell_s);
            g.snake_passthrough_regular_nodes =
                gm["snake_passthrough_regular_nodes"].value_or(
                    g.snake_passthrough_regular_nodes);
            g.revisit_passthrough_regular_nodes =
                gm["revisit_passthrough_regular_nodes"].value_or(
                    g.revisit_passthrough_regular_nodes);
            if (const auto revisit_order = gm["revisit_order"].value<std::string>()) {
                if (const auto parsed = omission::parseRevisitOrder(*revisit_order)) {
                    g.revisit_order = *parsed;
                } else {
                    std::cerr << "[config] invalid revisit_order: "
                              << *revisit_order << " (expected asc or desc)\n";
                }
            }
            g.snake_post_record_grace_s =
                gm["snake_post_record_grace_s"].value_or(g.snake_post_record_grace_s);
            g.snake_post_turn_blind_s =
                gm["snake_post_turn_blind_s"].value_or(g.snake_post_turn_blind_s);
            g.hop_max_distance_m = gm["hop_max_distance_m"].value_or(g.hop_max_distance_m);
            g.hop_intersection_min_distance_m =
                gm["hop_intersection_min_distance_m"].value_or(g.hop_intersection_min_distance_m);
            g.snake_launch_align_timeout_s =
                gm["snake_launch_align_timeout_s"].value_or(g.snake_launch_align_timeout_s);
            g.snake_launch_align_stable_frames =
                gm["snake_launch_align_stable_frames"].value_or(g.snake_launch_align_stable_frames);
            g.snake_launch_line_min_confidence =
                gm["snake_launch_line_min_confidence"].value_or(g.snake_launch_line_min_confidence);
            g.snake_launch_line_center_tolerance_norm =
                gm["snake_launch_line_center_tolerance_norm"].value_or(
                    g.snake_launch_line_center_tolerance_norm);
            const double launch_angle_tol_deg =
                gm["snake_launch_line_angle_tolerance_deg"].value_or(
                    g.snake_launch_line_angle_tolerance_rad * 180.0 / M_PI);
            g.snake_launch_line_angle_tolerance_rad =
                launch_angle_tol_deg * M_PI / 180.0;
            g.marker_window_frames = gm["marker_window_frames"].value_or(g.marker_window_frames);
            g.marker_window_min_count = gm["marker_window_min_count"].value_or(g.marker_window_min_count);
            cfg.mapper.stop_center_target_cy =
                gm["stop_center_target_cy"].value_or(cfg.mapper.stop_center_target_cy);
            cfg.mapper.stop_center_max_vx_mps =
                gm["stop_center_max_vx_mps"].value_or(cfg.mapper.stop_center_max_vx_mps);
            cfg.mapper.stop_center_taper_gap =
                gm["stop_center_taper_gap"].value_or(cfg.mapper.stop_center_taper_gap);
            cfg.mapper.intersection_center_target_y_norm =
                gm["entry_center_target_y_norm"].value_or(cfg.mapper.intersection_center_target_y_norm);
            cfg.mapper.intersection_center_forward_kp =
                gm["entry_center_forward_kp"].value_or(cfg.mapper.intersection_center_forward_kp);
            cfg.mapper.intersection_center_lateral_kp =
                gm["entry_center_lateral_kp"].value_or(cfg.mapper.intersection_center_lateral_kp);
            cfg.mapper.intersection_center_max_forward_mps =
                gm["entry_center_max_forward_mps"].value_or(cfg.mapper.intersection_center_max_forward_mps);
            cfg.mapper.intersection_center_max_reverse_mps =
                gm["entry_center_max_reverse_mps"].value_or(cfg.mapper.intersection_center_max_reverse_mps);
            cfg.mapper.intersection_center_max_lateral_mps =
                gm["entry_center_max_lateral_mps"].value_or(cfg.mapper.intersection_center_max_lateral_mps);
            // Expose forward_speed_advance_mps so the slow speed
            // applied during SnakeAdvanceOneCell's line-follow phase can be
            // tuned without recompiling.
            cfg.mapper.forward_speed_blind_mps =
                gm["forward_speed_blind_mps"].value_or(cfg.mapper.forward_speed_blind_mps);
            cfg.mapper.forward_speed_advance_mps =
                gm["forward_speed_advance_mps"].value_or(cfg.mapper.forward_speed_advance_mps);
            // Optional integral gains for altitude/yaw hold (anti-windup PI);
            // default 0.0 keeps the pure-P behaviour.
            cfg.mapper.altitude_ki =
                gm["altitude_ki"].value_or(cfg.mapper.altitude_ki);
            cfg.mapper.yaw_align_ki =
                gm["yaw_align_ki"].value_or(cfg.mapper.yaw_align_ki);
            const double yaw_deg = gm["vertiport_yaw_tolerance_deg"].value_or(5.0);
            g.vertiport_yaw_tolerance_rad = yaw_deg * M_PI / 180.0;
        }
    } catch (const toml::parse_error& e) {
        std::cerr << "[config] mission.toml parse warning: " << e.what() << "\n";
    }

    // CLI overrides
    cfg.mission.vertiport_marker_id = opt.vertiport_marker_id;
    if (opt.marker_count > 0) cfg.mission.markers_expected = opt.marker_count;
    if (opt.max_intersections_override > 0) cfg.mission.max_intersections = opt.max_intersections_override;
    if (opt.snake_initial_turn == "left")  cfg.snake.initial_turn = omission::SnakeTurnDir::Left;
    if (opt.snake_initial_turn == "right") cfg.snake.initial_turn = omission::SnakeTurnDir::Right;
    if (!opt.revisit_order.empty()) {
        const auto parsed = omission::parseRevisitOrder(opt.revisit_order);
        if (!parsed.has_value()) {
            std::cerr << "error: --revisit-order must be asc or desc\n";
            std::exit(2);
        }
        cfg.mission.revisit_order = *parsed;
    }
    cfg.mission.initial_snake_turn = cfg.snake.initial_turn;

    cfg.altitude.vertiport_altitude_m = cfg.mission.vertiport_altitude_m;
    cfg.altitude.cruise_altitude_m = cfg.mission.cruise_altitude_m;

    // Reasonable controller defaults for grid mission (lower yaw rate to settle in turns)
    cfg.controller.forward_mps = 0.25;
    cfg.controller.offset_kp = 0.35;
    cfg.controller.angle_yaw_kp = 0.5;
    cfg.controller.offset_yaw_kp = 0.0;
    cfg.controller.max_lateral_mps = 0.35;
    cfg.controller.max_yaw_rate_rad_s = 0.6;
    cfg.controller.output_ema_alpha = 0.4;
    cfg.controller.altitude_kp = 0.4;
    cfg.controller.max_vz_down_mps = 0.35;

    // Marker hover settings from mission.toml apply before runtime overrides.
    applyMarkerHoverConfigFile(joinPath(opt.config_dir, "mission.toml"), cfg);

    // Load target runtime profile. SITL keeps the existing grid/line split;
    // ardupilot_serial uses the real serial+rpicam profile.
    const std::string runtime_file = (target == "sitl")
        ? joinPath(opt.config_dir, opt.world == "grid"
            ? "runtime.sitl.grid.toml"
            : "runtime.sitl.toml")
        : joinPath(opt.config_dir, "runtime.ardupilot_serial.toml");
    try {
        auto rt = toml::parse_file(runtime_file);
        if (const auto runtime = rt["runtime"]) {
            cfg.vision_source = runtime["vision"].value_or(cfg.vision_source);
        }
        applyTransportConfig(rt["transport"], cfg);
        applySerialConfig(rt["serial"], cfg);
        applyMavlinkConfig(rt["mavlink"], cfg);
        applySafetyTimeouts(rt["safety"]["timeouts"], cfg);
        applySafetyRc(rt["safety"]["rc"], cfg);
        applySafetyPreflight(rt["safety"]["preflight"], cfg);
        applyDebugVideoConfig(rt["debug_video"], cfg);
        applyLineControllerConfig(rt["line_controller"], cfg);
        applyAltitudeHoldConfig(rt["altitude_hold"], cfg);
        applyMarkerHoverConfig(rt["marker_hover"], cfg);
        const auto src = rt["vision"]["source"];
        if (src && opt.gazebo_topic_override.empty()) {
            cfg.vision.source.gazebo_topic = src["gazebo_topic"].value_or(std::string{cfg.vision.source.gazebo_topic});
            cfg.vision.source.read_timeout_ms =
                src["read_timeout_ms"].value_or(cfg.vision.source.read_timeout_ms);
        }
    } catch (const toml::parse_error& e) {
        std::cerr << "[config] runtime profile parse warning (" << runtime_file << "): " << e.what() << "\n";
    }

    if (!opt.vision.empty()) {
        cfg.vision_source = opt.vision;
    }
    if (cfg.vision_source != "fake" &&
        cfg.vision_source != "gazebo" &&
        cfg.vision_source != "rpicam") {
        throw std::runtime_error("unknown --vision: " + cfg.vision_source);
    }

    if (!opt.gazebo_topic_override.empty()) {
        cfg.vision.source.gazebo_topic = opt.gazebo_topic_override;
    }
    if (!opt.autopilot_uri.empty()) {
        applyAutopilotUri(cfg, opt.autopilot_uri);
    }
    if (!opt.gcs_ip_override.empty()) {
        cfg.network.gcs_ip = opt.gcs_ip_override;
    }
    if (opt.debug_video_fps_specified) {
        if (opt.debug_video_fps_override <= 0) {
            throw std::runtime_error("--fps must be positive");
        }
        cfg.vision.debug_video.send_fps = opt.debug_video_fps_override;
        cfg.vision.debug_video.enabled = true;
    }
    if (opt.unsafe_assume_rc_present) {
        cfg.safety.assume_rc_present = true;
        cfg.safety.rc_required = false;
    }
}

std::unique_ptr<ovision::FrameSource> createFrameSource(const std::string& kind)
{
    if (kind == "gazebo") return std::make_unique<ovision::GazeboCameraSource>();
    if (kind == "rpicam") return std::make_unique<ovision::RpicamFrameSource>();
    return std::make_unique<ovision::FakeFrameSource>();
}

omission::GridHeading headingFromTracker(const omission::GridCoordinateTracker& t)
{
    return t.currentHeading();
}

const char* headingName(omission::GridHeading h)
{
    return omission::gridHeadingName(h);
}

const char* intersectionTypeNameShort(onboard::vision::IntersectionType t)
{
    switch (t) {
    case onboard::vision::IntersectionType::None:     return "none";
    case onboard::vision::IntersectionType::Unknown:  return "unk";
    case onboard::vision::IntersectionType::Straight: return "str";
    case onboard::vision::IntersectionType::L:        return "L";
    case onboard::vision::IntersectionType::T:        return "T";
    case onboard::vision::IntersectionType::Cross:    return "+";
    }
    return "?";
}

bool recent(Clock::time_point timestamp, Clock::time_point now, int max_age_ms)
{
    if (timestamp.time_since_epoch().count() == 0) {
        return false;
    }
    const auto age_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - timestamp).count();
    return age_ms >= 0 && age_ms <= max_age_ms;
}

bool heartbeatRecent(
    const oautopilot::AutopilotState& state,
    const omission::GridMissionConfig& mission,
    Clock::time_point now)
{
    if (!state.heartbeat_seen) {
        return false;
    }
    const int timeout_ms = std::max(
        1,
        static_cast<int>(mission.autopilot_heartbeat_timeout_s * 1000.0));
    return recent(state.last_heartbeat_time, now, timeout_ms);
}

bool rcInputFresh(
    const oautopilot::AutopilotState& state,
    const osafety::SafetyConfig& safety,
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
    oautopilot::AutopilotMavlinkAdapter& autopilot,
    const osafety::SafetyConfig& safety,
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

// Returns true when the EKF reports it is aiding horizontal position/velocity
// well enough for ArduCopter to accept a GUIDED request. Checks both the
// EKF_STATUS_REPORT flag bits AND the innovation test ratios: the flags can be
// set while ArduCopter still refuses GUIDED because pos/vel variance is above
// its position_ok bound. The variance gate is only enforced when the FC
// actually reports the ratios, so a firmware that omits them falls back to the
// flags-only behaviour instead of being blocked.
bool ekfRelativeAidingReady(
    const oautopilot::AutopilotState& state,
    Clock::time_point now,
    double pos_horiz_variance_max,
    double velocity_variance_max)
{
    constexpr std::uint16_t kEkfAttitude = 1u << 0;
    constexpr std::uint16_t kEkfVelocityHoriz = 1u << 1;
    constexpr std::uint16_t kEkfPosHorizRel = 1u << 3;
    constexpr std::uint16_t kEkfPredPosHorizRel = 1u << 8;

    if (!state.ekf_flags || !recent(state.last_ekf_status_time, now, 1500)) {
        return false;
    }
    const auto flags = *state.ekf_flags;
    const bool flags_ready =
        (flags & kEkfAttitude) != 0 &&
        (flags & kEkfVelocityHoriz) != 0 &&
        ((flags & kEkfPosHorizRel) != 0 ||
         (flags & kEkfPredPosHorizRel) != 0);
    if (!flags_ready) {
        return false;
    }
    if (state.ekf_pos_horiz_variance &&
        *state.ekf_pos_horiz_variance > pos_horiz_variance_max) {
        return false;
    }
    if (state.ekf_velocity_variance &&
        *state.ekf_velocity_variance > velocity_variance_max) {
        return false;
    }
    return true;
}

bool opticalFlowReady(
    const oautopilot::AutopilotState& state,
    Clock::time_point now)
{
    if (!state.optical_flow_quality ||
        !recent(state.last_optical_flow_time, now, 1500)) {
        return false;
    }
    return *state.optical_flow_quality >= 50;
}

bool localHoldEstimateReady(
    const oautopilot::AutopilotState& state,
    Clock::time_point now,
    double ekf_pos_horiz_variance_max,
    double ekf_velocity_variance_max)
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
           ekfRelativeAidingReady(state, now,
                                  ekf_pos_horiz_variance_max,
                                  ekf_velocity_variance_max);
}

void printLocalEstimateStatus(const oautopilot::AutopilotState& state)
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
              << " ekf_flags=0x" << std::hex << state.ekf_flags.value_or(0) << std::dec
              // Variances are the gate ArduCopter actually uses for GUIDED; show
              // them so a stuck preflight points at the real culprit.
              << " ekf_var(posH/vel/posV)="
              << state.ekf_pos_horiz_variance.value_or(-1.0) << '/'
              << state.ekf_velocity_variance.value_or(-1.0) << '/'
              << state.ekf_pos_vert_variance.value_or(-1.0);
    if (!state.recent_statustexts.empty()) {
        std::cout << " last_statustext=\"" << state.recent_statustexts.back().text << '"';
    }
    std::cout << "\n";
}

bool waitLocalHoldEstimateReady(
    oautopilot::AutopilotMavlinkAdapter& autopilot,
    std::chrono::seconds timeout,
    double ekf_pos_horiz_variance_max,
    double ekf_velocity_variance_max)
{
    std::cout << "[preflight] waiting for local XY hold estimate: local position, range, optical flow, EKF\n";
    const auto deadline = Clock::now() + timeout;
    auto ready_since = Clock::time_point {};
    auto last_log = Clock::now() - std::chrono::seconds(1);
    while (Clock::now() < deadline) {
        autopilot.poll(100);
        const auto now = Clock::now();
        const bool ready = localHoldEstimateReady(autopilot.state(), now,
                                                  ekf_pos_horiz_variance_max,
                                                  ekf_velocity_variance_max);
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

void logState(const omission::GridMissionOutput& out,
              const oautopilot::AutopilotState& ap,
              const onboard::vision::VisionResult& vis,
              const omission::IntersectionDecision& idec,
              const omission::GridMission& mission,
              const omission::MarkerRegistry& registry,
              double now_s)
{
    static double last = 0.0;
    if (now_s - last < 0.5) return;
    last = now_s;
    auto fmt = [](const std::optional<double>& v) -> std::string {
        if (!v.has_value()) return "n/a";
        char buf[32];
        snprintf(buf, sizeof(buf), "%.2f", *v);
        return buf;
    };
    char bm_buf[8];
    snprintf(bm_buf, sizeof(bm_buf), "0x%02X", idec.accepted_branch_mask);
    char cx_buf[16];
    snprintf(cx_buf, sizeof(cx_buf), "%.2f", out.intersection_center_x_norm);
    char cy_buf[16];
    snprintf(cy_buf, sizeof(cy_buf), "%.2f", idec.center_y_norm);
    char hop_buf[16];
    snprintf(hop_buf, sizeof(hop_buf), "%.2f", out.hop_distance_m);
    char mkx_buf[16];
    char mky_buf[16];
    if (out.marker_detected) {
        snprintf(mkx_buf, sizeof(mkx_buf), "%.2f", out.marker_center_error_x_norm);
        snprintf(mky_buf, sizeof(mky_buf), "%.2f", out.marker_center_error_y_norm);
    } else {
        snprintf(mkx_buf, sizeof(mkx_buf), "-");
        snprintf(mky_buf, sizeof(mky_buf), "-");
    }
    // Registry ID list (committed markers).
    std::string regids;
    {
        std::vector<int> ids(registry.seenIds().begin(), registry.seenIds().end());
        std::sort(ids.begin(), ids.end());
        for (size_t i = 0; i < ids.size(); ++i) {
            if (i) regids += ',';
            regids += std::to_string(ids[i]);
        }
        if (regids.empty()) regids = "-";
    }
    std::cout << "[grid-mission] t=" << now_s
              << " st=" << omission::gridStateName(out.state)
              << " intent=" << ocontrol::gridControlIntentName(out.intent)
              << " mode=" << ap.mode_name
              << " armed=" << (ap.armed ? 1 : 0)
              << " agl=" << fmt(ap.distance_sensor_m)
              << " lz=" << fmt(ap.local_altitude_m)
              << " yaw=" << fmt(ap.attitude_yaw_rad)
              << " line=" << (vis.line.detected ? 1 : 0)
              << " mks=" << vis.markers.size()
              << " mkerr=(" << mkx_buf << "," << mky_buf << ")"
              << " coord=(" << out.current_coord.x << "," << out.current_coord.y << ")"
              << " hd=" << headingName(out.current_heading)
              << " nodes=" << out.intersections_recorded
              << " idec=" << omission::decisionStateName(idec.state)
              << " type=" << intersectionTypeNameShort(idec.accepted_type)
              << " cx=" << cx_buf
              << " cy=" << cy_buf
              << " hop=" << hop_buf
              << " bm=" << bm_buf
              << " mkstable=" << mission.stableMarkerCandidateCount()
              << " regids=" << regids
              << (out.reason.empty() ? "" : (" reason=" + out.reason))
              << (out.last_safety_event.empty() ? "" : (" safety=" + out.last_safety_event))
              << "\n";
}

} // namespace

int onboard::app::AstroquadOnboardApp::run(int argc, char** argv)
{
    std::signal(SIGINT, handleSigint);
    std::signal(SIGTERM, handleSigint);

    const Options opt = parseOptions(argc, argv);
    if (opt.marker_count <= 0) {
        std::cerr << "error: --marker-count is required (positive integer)\n";
        return 2;
    }
    Configs cfg;
    try {
        loadConfigs(opt, cfg);
    } catch (const std::exception& e) {
        std::cerr << "config error: " << e.what() << "\n";
        return 2;
    }

    const bool real_serial_target = cfg.endpoint.kind == TransportKind::Serial;
    if (real_serial_target && !opt.allow_arm_takeoff && !opt.no_arm) {
        std::cerr << "error: ardupilot_serial target requires --allow-arm-takeoff (or --no-arm for bench)\n";
        return 2;
    }
    if (opt.unsafe_assume_rc_present) {
        std::cerr
            << "[safety] WARNING: --unsafe-assume-rc-present bypasses the MAVLink RC gate\n";
    }

    std::cout << "[grid-mission] starting target=" << opt.target
              << " vision=" << cfg.vision_source
              << " world=" << opt.world
              << " line_mode=" << opt.line_mode_override
              << " marker_count=" << opt.marker_count
              << " vertiport_id=" << cfg.mission.vertiport_marker_id
              << " revisit_order=" << omission::revisitOrderName(cfg.mission.revisit_order)
              << " autopilot="
              << (cfg.endpoint.kind == TransportKind::Serial
                      ? cfg.endpoint.serial_device + ":" + std::to_string(cfg.endpoint.serial_baudrate)
                      : "udp:" + std::to_string(cfg.endpoint.listen_port))
              << " gcs=" << cfg.network.gcs_ip << ":" << cfg.network.telemetry_port
              << " rc_required=" << (cfg.safety.rc_required ? "true" : "false")
              << " assume_rc=" << (cfg.safety.assume_rc_present ? "true" : "false")
              << " rate_hz=" << cfg.setpoint_rate_hz
              << "\n";
    std::cout << "[control] gains offset_kp=" << cfg.controller.offset_kp
              << " angle_yaw_kp=" << cfg.controller.angle_yaw_kp
              << " offset_yaw_kp=" << cfg.controller.offset_yaw_kp
              << " max_lat=" << cfg.controller.max_lateral_mps
              << " max_yaw=" << cfg.controller.max_yaw_rate_rad_s
              << " ema_alpha=" << cfg.controller.output_ema_alpha
              << " lat_rate=" << cfg.controller.max_lateral_rate_mps
              << " yaw_rate_change=" << cfg.controller.max_yaw_rate_change_rad_s
              << " fwd_conf_scale=" << cfg.controller.forward_confidence_scale << "\n";

    // Vision setup
    auto frame_source = createFrameSource(cfg.vision_source);
    ovision::FrameSourceOptions fs_opts {cfg.vision};
    if (!frame_source->open(fs_opts)) {
        std::cerr << "vision source open failed: " << frame_source->lastError() << "\n";
        return 1;
    }
    ovision::VisionProcessor processor(ovision::VisionProcessorOptions {
        cfg.vision, true, true,
    });

    // GCS publisher
    oapp::GcsTelemetryPublisher publisher;
    oapp::GcsTelemetryPublisherOptions pub_opts;
    pub_opts.network = cfg.network;
    pub_opts.vision = cfg.vision;
    pub_opts.send_video = opt.send_video_overridden
        ? opt.send_video
        : cfg.vision.debug_video.enabled;
    pub_opts.send_telemetry = opt.send_telemetry;
    pub_opts.note = "grid_mission";
    pub_opts.camera_sensor_model =
        cfg.vision_source == "gazebo" ? "gazebo_downward_camera" : cfg.vision.camera.sensor_model;
    if (!publisher.open(pub_opts)) {
        std::cerr << "[gcs] warning: publisher open failed: " << publisher.lastError() << "\n";
    }

    std::unique_ptr<oautopilot::MavlinkTransport> transport;
    if (cfg.endpoint.kind == TransportKind::Serial) {
        transport = std::make_unique<oautopilot::SerialMavlinkTransport>(
            cfg.endpoint.serial_device,
            cfg.endpoint.serial_baudrate,
            static_cast<std::uint8_t>(MAVLINK_COMM_0),
            cfg.endpoint.label);
    } else {
        transport = std::make_unique<oautopilot::UdpMavlinkTransport>(
            cfg.endpoint.listen_port,
            static_cast<std::uint8_t>(MAVLINK_COMM_0),
            cfg.endpoint.label);
    }
    oautopilot::AutopilotMavlinkAdapter autopilot(std::move(transport), cfg.mavlink);

    std::cout << "[mavlink] waiting for heartbeat on "
              << (cfg.endpoint.kind == TransportKind::Serial
                      ? cfg.endpoint.serial_device
                      : cfg.endpoint.listen_host + ":" + std::to_string(cfg.endpoint.listen_port))
              << "...\n";
    autopilot.waitHeartbeat(std::chrono::seconds(30));
    std::cout << "[mavlink] heartbeat received system="
              << static_cast<int>(autopilot.state().target_system)
              << " component=" << static_cast<int>(autopilot.state().target_component)
              << " mode=" << autopilot.state().mode_name << "\n";
    autopilot.requestDefaultStreams();

    // Build mission stack
    ocontrol::GuidedVelocityController line_controller(cfg.controller);
    omission::GridCoordinateTracker tracker(cfg.vision.intersection_decision);
    omission::MarkerRegistry registry;
    omission::AltitudePolicy altitude(cfg.altitude);
    omission::SnakePlanner snake(cfg.snake);
    omission::IntersectionDecisionEngine decision_engine(cfg.vision.intersection_decision);
    omission::GridMission mission(cfg.mission, &tracker, &registry, &altitude, &snake,
                                  &decision_engine,
                                  cfg.vision.intersection_decision.node_record_y_min,
                                  cfg.vision.intersection_decision.node_record_y_max,
                                  /*node_record_y_min_line_enter=*/0.30);
    ocontrol::GridControlMapper mapper(cfg.mapper, &line_controller);
    // Independent safety watchdog. Its main job for the grid mission is to abort
    // if the flight controller leaves GUIDED (operator/RC takeover or an FC-side
    // mode drop); heartbeat/timeout overlap GridMission::isHardFailsafe and both
    // route to EMERGENCY_LAND, which is harmless.
    osafety::SafetyMonitor safety(cfg.safety);

    if (!opt.no_arm) {
        if (!waitRcReady(autopilot, cfg.safety, std::chrono::seconds(10))) {
            std::cerr << "[safety] refusing GUIDED/arm/takeoff until RC input is visible\n";
            return 2;
        }
        if (real_serial_target &&
            !waitLocalHoldEstimateReady(autopilot, std::chrono::seconds(15),
                                        cfg.safety.ekf_pos_horiz_variance_max,
                                        cfg.safety.ekf_velocity_variance_max)) {
            std::cerr << "[safety] refusing GUIDED/arm/takeoff until optical-flow local hold is ready\n";
            return 2;
        }
        std::cout << "[mavlink] setting GUIDED mode...\n";
        autopilot.setGuidedMode(std::chrono::seconds(10));
        std::cout << "[mavlink] arming...\n";
        autopilot.arm(std::chrono::seconds(15));
        std::cout << "[mavlink] takeoff -> " << cfg.mission.vertiport_altitude_m << "m\n";
        autopilot.takeoff(cfg.mission.vertiport_altitude_m);
    }

    const auto start_time = Clock::now();
    auto now_s = [&]() {
        return std::chrono::duration<double>(Clock::now() - start_time).count();
    };
    mission.start(now_s());
    safety.startMission(Clock::now());

    const auto period = std::chrono::milliseconds(1000 / std::max(1, cfg.setpoint_rate_hz));
    bool first_frame = true;
    std::string last_safety_abort_reason;
    // Latch the last committed node event so we can keep retransmitting
    // it to the GCS each frame (UDP loss redundancy + no peek-stage ghost nodes).
    omission::GridNodeEvent last_committed_event;

    // Vision capture + processing runs on its own thread so a slow frame never
    // stalls the control loop. The thread reads and processes frames as fast as
    // the camera allows and publishes only the latest result; the control loop
    // consumes that latest result at the fixed setpoint rate. The heavy OpenCV
    // work (capture + process) is the producer's; the stateful decision engine,
    // tracker, and mission run in the consumer so per-frame ordering is kept.
    struct VisionSlot {
        ovision::Frame frame;
        ovision::VisionProcessingOutput vp_out;
        double latency_ms = 0.0;
        std::uint64_t seq = 0;
        bool valid = false;
    };
    std::mutex vision_mtx;
    std::condition_variable vision_cv;
    VisionSlot vision_latest;          // guarded by vision_mtx
    std::string vision_error;          // guarded by vision_mtx
    std::atomic_bool vision_stop{false};

    std::thread vision_thread([&]() {
        std::uint64_t seq = 0;
        while (!vision_stop.load() && !g_shutdown_requested.load()) {
            ovision::Frame f;
            try {
                if (!frame_source->read(f)) {
                    {
                        std::lock_guard<std::mutex> lk(vision_mtx);
                        vision_error = "read failed: " + frame_source->lastError();
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    continue;
                }
            } catch (const std::exception& e) {
                {
                    std::lock_guard<std::mutex> lk(vision_mtx);
                    vision_error = std::string("exception: ") + e.what();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            const auto t0 = Clock::now();
            auto out = processor.process(f.image_bgr,
                ovision::VisionFrameMetadata{f.frame_id, f.timestamp_ms, f.width, f.height});
            const double latency = std::chrono::duration<double, std::milli>(
                Clock::now() - t0).count();
            {
                std::lock_guard<std::mutex> lk(vision_mtx);
                vision_latest.frame = std::move(f);
                vision_latest.vp_out = std::move(out);
                vision_latest.latency_ms = latency;
                vision_latest.seq = ++seq;
                vision_latest.valid = true;
            }
            vision_cv.notify_one();
        }
    });

    // Control-output state retained between frames so idle ticks (no new vision)
    // can re-send the last velocity command and hold the setpoint cadence.
    std::uint64_t last_processed_seq = 0;
    oautopilot::BodyVelocityCommand last_cmd{};
    bool have_control_output = false;
    bool last_cmd_sendable = false;

    while (!g_shutdown_requested.load() && mission.state() != omission::GridState::Done) {
        const auto loop_start = Clock::now();
        autopilot.poll(1);

        // Grab the latest processed frame from the vision thread.
        ovision::Frame frame;
        ovision::VisionProcessingOutput vp_out;
        double processing_latency_ms = 0.0;
        std::uint64_t vision_seq = 0;
        bool vision_valid = false;
        {
            std::unique_lock<std::mutex> lk(vision_mtx);
            if (!vision_latest.valid) {
                vision_cv.wait_for(lk, std::chrono::milliseconds(50));
            }
            if (!vision_error.empty()) {
                std::cerr << "[vision] " << vision_error << "\n";
                vision_error.clear();
            }
            vision_valid = vision_latest.valid;
            if (vision_valid) {
                frame = vision_latest.frame;          // refcounted cv::Mat copy
                vp_out = vision_latest.vp_out;
                processing_latency_ms = vision_latest.latency_ms;
                vision_seq = vision_latest.seq;
            }
        }

        // No new frame this tick: re-send the last command to keep the setpoint
        // cadence steady, then wait for the next tick.
        const bool new_frame = vision_valid && (vision_seq != last_processed_seq);
        if (!new_frame) {
            if (have_control_output && last_cmd_sendable) {
                autopilot.sendBodyVelocity(last_cmd);
            }
            std::this_thread::sleep_until(loop_start + period);
            continue;
        }
        last_processed_seq = vision_seq;

        // Intersection + tracker
        auto idec = decision_engine.update(
            vp_out.result.intersection,
            frame.width, frame.height,
            frame.frame_id, frame.timestamp_ms,
            false);
        // Only let the grid tracker peek regular snake nodes after the
        // synthetic origin has been centered and latched. EntryForward and
        // EntryCenterOrigin publish (0,0) directly so their early candidates
        // must not advance tracker frame lockouts.
        const auto cur_state = mission.state();
        const bool tracker_enabled =
            cur_state == omission::GridState::SnakeForward ||
            cur_state == omission::GridState::SnakeRecordNode ||
            cur_state == omission::GridState::SnakeStopAtCenter ||
            cur_state == omission::GridState::SnakeTurn90 ||
            cur_state == omission::GridState::SnakeAdvanceOneCell ||
            cur_state == omission::GridState::SnakeTurn90Again ||
            cur_state == omission::GridState::RevisitForward ||
            cur_state == omission::GridState::ReturnHomeForward;
        omission::GridNodeEvent node_event;
        if (tracker_enabled) {
            node_event = tracker.update(idec, frame.frame_id, frame.timestamp_ms);
        }

        // Compose mission input
        omission::GridMissionInput min;
        min.now_s = now_s();
        min.timestamp_ms = frame.timestamp_ms;
        min.frame_seq = frame.frame_id;
        const auto& apst = autopilot.state();
        min.armed = apst.armed;
        min.guided_mode = apst.mode_name == "GUIDED";
        min.heartbeat_recent = heartbeatRecent(apst, cfg.mission, loop_start);
        min.autopilot_ready = min.heartbeat_recent;
        min.rangefinder_m = apst.distance_sensor_m;
        min.local_x_m = apst.local_x_m;
        min.local_y_m = apst.local_y_m;
        min.local_altitude_m = apst.local_altitude_m;
        if (apst.local_vx_mps.has_value() && apst.local_vy_mps.has_value()) {
            const double vx = *apst.local_vx_mps;
            const double vy = *apst.local_vy_mps;
            min.local_velocity_xy_mps = std::sqrt(vx*vx + vy*vy);
        }
        min.attitude_yaw_rad = apst.attitude_yaw_rad;
        min.optical_flow_quality = apst.optical_flow_quality;
        min.vision = &vp_out.result;
        min.intersection_decision = idec;
        min.node_event = node_event;
        min.land_requested = false;

        // Safety watchdog. The grid mission flies long blind-forward / yaw-turn
        // phases with no line in view, so the line-lost check is disabled here
        // (line_detected held true). The mode-from-GUIDED check is only enforced
        // while the mission still expects GUIDED: skip it before commanding
        // (no_arm) and once the mission has entered a landing/terminal state, so
        // the controller switching to LAND at mission end is not flagged as a
        // takeover. Any Abort/Land routes through abort_requested into the same
        // centralized EMERGENCY_LAND path GridMission already uses.
        const bool mission_expects_guided =
            !opt.no_arm &&
            cur_state != omission::GridState::Idle &&
            cur_state != omission::GridState::Land &&
            cur_state != omission::GridState::EmergencyLand &&
            cur_state != omission::GridState::MissionComplete &&
            cur_state != omission::GridState::Done;
        osafety::SafetyInput safety_in;
        safety_in.heartbeat_seen = apst.heartbeat_seen;
        safety_in.line_detected = true;
        safety_in.now = loop_start;
        safety_in.last_heartbeat_time = apst.last_heartbeat_time;
        safety_in.rc_channels_seen = apst.rc_channel_count.has_value();
        safety_in.rc_channel_count = apst.rc_channel_count.value_or(0);
        safety_in.last_rc_channels_time = apst.last_rc_channels_time;
        safety_in.mode_known = mission_expects_guided;
        safety_in.mode_guided = min.guided_mode;
        const auto safety_decision = safety.update(safety_in);
        min.abort_requested =
            safety_decision.action != osafety::SafetyAction::None;
        if (min.abort_requested &&
            safety_decision.reason != last_safety_abort_reason) {
            std::cerr << "[safety] " << safety_decision.reason
                      << " -> EMERGENCY_LAND\n";
            last_safety_abort_reason = safety_decision.reason;
        }

        // Mission update
        auto mout = mission.update(min);

        // Tracker advances only when GridMission says so (post all
        // gates). The earlier tracker.update call above was peek-only.
        // GridMission may attach a synthetic event for boundary
        // commits where idec skipped NodeRecord and tracker peek returned
        // valid=false. Prefer the synthetic event when present.
        if (mout.commit_tracker_advance) {
            const omission::GridNodeEvent& commit_ev =
                mout.synthetic_commit_event.has_value()
                    ? *mout.synthetic_commit_event
                    : node_event;
            if (commit_ev.valid) {
                tracker.commitAdvance(commit_ev);
                last_committed_event = commit_ev;
            }
        }

        // One-shot synthetic origin event from GRID_ORIGIN_LOCK so
        // the GCS map gets the (0,0) node + start-marker placement before any
        // real commit fires.
        if (mout.origin_publish_event.has_value()) {
            last_committed_event = *mout.origin_publish_event;
        }

        // Build control inputs
        ocontrol::GridControlMapperInput cmin;
        cmin.intent = mout.intent;
        cmin.target_altitude_m = mout.target_altitude_m;
        cmin.altitude_available = apst.local_altitude_m.has_value() || apst.distance_sensor_m.has_value();
        cmin.current_altitude_m = apst.distance_sensor_m.value_or(apst.local_altitude_m.value_or(0.0));
        cmin.forward_speed_override_mps = mout.forward_speed_override_mps;
        cmin.yaw_available = mout.yaw_available;
        cmin.current_yaw_rad = mout.current_yaw_rad;
        cmin.target_yaw_rad = mout.target_yaw_rad;
        cmin.line_detected = mout.line_detected;
        cmin.line_center_error_norm = mout.line_center_error_norm;
        cmin.line_angle_error_rad = mout.line_angle_error_rad;
        cmin.line_confidence = mout.line_confidence;
        cmin.advance_phase = mout.advance_phase;
        cmin.marker_detected = mout.marker_detected;
        cmin.marker_center_error_x_norm = mout.marker_center_error_x_norm;
        cmin.marker_center_error_y_norm = mout.marker_center_error_y_norm;
        // Forward intersection cy for StopAndCenter cy-feedback decel.
        cmin.intersection_valid = mout.intersection_valid;
        cmin.intersection_center_x_norm = mout.intersection_center_x_norm;
        cmin.intersection_center_y_norm = mout.intersection_center_y_norm;

        const auto sp = mapper.compute(cmin);

        // Land request
        have_control_output = true;
        if (!opt.no_arm && mout.request_land_mode) {
            last_cmd_sendable = false;   // LAND mode owns the descent; stop velocity
            try {
                autopilot.setLandMode(std::chrono::seconds(5));
            } catch (const std::exception& e) {
                std::cerr << "[mavlink] setLandMode error: " << e.what() << "\n";
            }
        } else if (!opt.no_arm &&
                   mout.intent != ocontrol::GridControlIntent::Idle &&
                   mout.intent != ocontrol::GridControlIntent::Land) {
            oautopilot::BodyVelocityCommand cmd;
            cmd.vx_forward_mps = sp.vx_forward_mps;
            cmd.vy_right_mps = sp.vy_right_mps;
            cmd.vz_down_mps = sp.vz_down_mps;
            cmd.yaw_rate_rad_s = sp.yaw_rate_rad_s;
            autopilot.sendBodyVelocity(cmd);
            // Remember it so idle ticks (no new frame) can re-send and keep the
            // setpoint cadence steady at the configured rate.
            last_cmd = cmd;
            last_cmd_sendable = true;
        } else {
            last_cmd_sendable = false;
        }

        // Publish telemetry / video
        if (publisher.lastError().empty() || first_frame) {
            oapp::GcsTelemetryPublishInput pin;
            pin.frame = frame;
            pin.image_bgr = frame.image_bgr;
            pin.vision_output = vp_out;
            pin.intersection_decision = idec;
            // Send the last committed node every frame (not the peek
            // event). Peek events would leak an uncommitted "ghost" node ahead
            // of the drone into the GCS map; resending the last commit gives
            // UDP loss redundancy so GCS edges never have gaps.
            pin.grid_node = mout.grid_map_finalized
                ? omission::GridNodeEvent {}
                : last_committed_event;
            // Forward drone fractional position so the GCS can
            // render the heading arrow at a sub-cell position between commits.
            pin.drone_position_valid =
                mout.grid_pose_visible && mout.drone_position_valid;
            pin.drone_cell_progress = mout.drone_cell_progress;
            pin.drone_grid_offset_x = mout.drone_grid_offset_x;
            pin.drone_grid_offset_y = mout.drone_grid_offset_y;
            // Populate mission telemetry so the GCS sees the
            // discovered-marker registry + mission state. Without this the
            // mission JSON block stayed empty and GCS could not render the
            // markers panel.
            pin.mission.present = true;
            pin.mission.state = omission::gridStateName(mout.state);
            pin.mission.control_intent = ocontrol::gridControlIntentName(mout.intent);
            pin.mission.target_altitude_m = mout.target_altitude_m;
            pin.mission.altitude_off_pad_confirmed = mout.altitude_off_pad_confirmed;
            pin.mission.grid.x = mout.current_coord.x;
            pin.mission.grid.y = mout.current_coord.y;
            pin.mission.grid.heading = omission::gridHeadingName(mout.current_heading);
            pin.mission.grid.snake_dir = omission::snakeTurnDirName(mout.snake_dir);
            pin.mission.grid.valid =
                mout.grid_pose_visible &&
                mout.current_heading != omission::GridHeading::Unknown;
            pin.mission.vertiport.verified = mout.vertiport_verified;
            // Report the marker id MarkerLockYaw actually latched
            // (which may differ from cfg default if the dynamic-id path
            // picked a different first marker).
            pin.mission.vertiport.marker_id = mission.activeVertiportMarkerId();
            pin.mission.markers_expected = cfg.mission.markers_expected;
            pin.mission.snake_complete =
                mout.state == omission::GridState::SnakeComplete ||
                mout.state == omission::GridState::RevisitInit ||
                mout.state == omission::GridState::RevisitForward ||
                mout.state == omission::GridState::RevisitStopAtTurn ||
                mout.state == omission::GridState::RevisitTurn90 ||
                mout.state == omission::GridState::RevisitMarkerHover ||
                mout.state == omission::GridState::RevisitComplete ||
                mout.state == omission::GridState::ReturnHomeInit ||
                mout.state == omission::GridState::ReturnHomeForward ||
                mout.state == omission::GridState::ReturnHomeStopAtTurn ||
                mout.state == omission::GridState::ReturnHomeTurn90 ||
                mout.state == omission::GridState::ReturnHomeAlignOrigin ||
                mout.state == omission::GridState::ReturnHomeFaceSouth ||
                mout.state == omission::GridState::ReturnVertiportForward ||
                mout.state == omission::GridState::ReturnVertiportMarkerHover ||
                mout.state == omission::GridState::MissionComplete ||
                mout.state == omission::GridState::Land ||
                mout.state == omission::GridState::Done;
            pin.mission.revisit_active = mout.revisit_active;
            pin.mission.return_active = mout.return_active;
            pin.mission.return_phase = mout.return_phase;
            pin.mission.grid_map_finalized = mout.grid_map_finalized;
            pin.mission.grid_pose_visible = mout.grid_pose_visible;
            pin.mission.vertiport_return_active = mout.vertiport_return_active;
            pin.mission.vertiport_acquired = mout.vertiport_acquired;
            pin.mission.landing_success = mout.landing_success;
            pin.mission.mission_complete = mout.mission_complete;
            pin.mission.revisit_order = mout.revisit_order;
            pin.mission.revisit_target_id = mout.revisit_target_id;
            pin.mission.revisit_remaining = mout.revisit_remaining;
            pin.mission.last_safety_event = mout.last_safety_event;
            pin.mission.markers_found.clear();
            const double mission_now_s = now_s();
            for (const auto& m : registry.records()) {
                // Skip records without a valid grid coord. Vertiport
                // (and any other pre-grid sighting) is recorded with
                // grid_coord_valid=false; including those here would inflate
                // markers_found count past markers_expected on GCS.
                if (!m.grid_coord_valid) continue;
                onboard::protocol::MissionMarkerEntry e;
                e.id = m.aruco_id;
                e.grid_x = m.grid_coord.x;
                e.grid_y = m.grid_coord.y;
                e.grid_valid = m.grid_coord_valid;
                e.orientation_deg = static_cast<double>(m.orientation_deg);
                // Convert absolute first_seen_ms (camera frame clock) into
                // mission elapsed seconds by anchoring against the current
                // frame's timestamp_ms and now_s(). first_seen <= now_s.
                e.first_seen_s = mission_now_s +
                    (m.first_seen_ms - frame.timestamp_ms) / 1000.0;
                e.revisited = m.revisited;
                e.revisited_s = m.revisited
                    ? mission_now_s + (m.revisited_ms - frame.timestamp_ms) / 1000.0
                    : 0.0;
                pin.mission.markers_found.push_back(e);
            }
            pin.processing_latency_ms = processing_latency_ms;
            pin.read_frame_ms = 0.0;
            pin.capture_fps = 0.0;
            pin.processing_fps = 0.0;
            pin.camera_status = "streaming";
            (void)publisher.publish(std::move(pin));
        }
        first_frame = false;

        // Log
        logState(mout, apst, vp_out.result, idec, mission, registry, now_s());

        std::this_thread::sleep_until(loop_start + period);
    }

    // Stop the vision thread before tearing down the frame source it uses.
    vision_stop.store(true);
    if (vision_thread.joinable()) {
        vision_thread.join();
    }

    std::cout << "[grid-mission] shutdown requested or mission DONE. waiting disarm...\n";
    if (!opt.no_arm) {
        try {
            autopilot.setLandMode(std::chrono::seconds(3));
        } catch (...) {}
        const auto land_deadline = Clock::now() + std::chrono::seconds(20);
        while (Clock::now() < land_deadline) {
            autopilot.poll(100);
            if (!autopilot.state().armed) break;
        }
    }
    publisher.close();
    frame_source->close();
    std::cout << "[grid-mission] done.\n";
    return 0;
}
