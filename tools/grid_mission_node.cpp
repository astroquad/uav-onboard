// grid_mission_node — staging executable for full grid snake mission.
//
// Composition root that wires VisionRuntime + AutopilotMavlinkAdapter +
// GridMission state machine + GridControlMapper. Reuses line_follow_node
// helpers in spirit but lives in its own translation unit so the MVP
// line-following path keeps working unchanged.
//
// Algorithm rules (see development-log plan §0):
//   - Never use Gazebo ground-truth pose. Only MAVLink LOCAL_POSITION_NED,
//     ATTITUDE, rangefinder, vision events, and time are allowed inputs.
//   - LOCAL_POSITION_NED is an EKF estimate (same on SITL and real Pixhawk),
//     so it is OK as a distance/hover source within short windows.

#include "app/VisionDebugPublisher.hpp"
#include "autopilot/AutopilotMavlinkAdapter.hpp"
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
#include "vision/FakeFrameSource.hpp"
#include "vision/GazeboCameraSource.hpp"
#include "vision/RpicamFrameSource.hpp"
#include "vision/VisionProcessor.hpp"

#include <toml++/toml.hpp>

#include <atomic>
#include <algorithm>
#include <chrono>
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

namespace {

using Clock = std::chrono::steady_clock;
namespace omission = onboard::mission;
namespace oapp = onboard::app;
namespace ocontrol = onboard::control;
namespace ovision = onboard::vision;
namespace oautopilot = onboard::autopilot;
namespace ocommon = onboard::common;

struct Options {
    std::string config_dir = "config";
    std::string target = "sitl";
    std::string vision = "gazebo";
    std::string world = "grid";
    std::string line_mode_override = "dark_on_light";
    std::string gcs_ip_override;
    std::string gazebo_topic_override;
    int marker_count = -1;
    int vertiport_marker_id = 23;
    int max_intersections_override = 0;
    std::string snake_initial_turn = "auto";
    bool send_video = false;
    bool send_video_overridden = false;
    bool send_telemetry = true;
    bool allow_arm_takeoff = false;
    bool no_arm = false;
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
        << "Usage: grid_mission_node [options]\n\n"
        << "Required:\n"
        << "  --marker-count <n>           Expected number of grid ArUco markers (vertiport excluded)\n"
        << "\n"
        << "Options:\n"
        << "  --config <dir>               Config directory (default: config)\n"
        << "  --target <sitl|pixhawk1>     Runtime target (default: sitl)\n"
        << "  --vision <fake|gazebo|rpicam>\n"
        << "                               Vision source (default: gazebo)\n"
        << "  --world <grid|line>          Gazebo world profile (default: grid)\n"
        << "  --line-mode <mode>           light_on_dark | dark_on_light | auto\n"
        << "  --gcs-ip <ip>                Override GCS destination IP\n"
        << "  --gazebo-topic <topic>       Override Gazebo camera image topic\n"
        << "  --vertiport-marker-id <id>   Override vertiport ArUco ID (default: 23)\n"
        << "  --max-intersections <n>      Safety cap on recorded nodes\n"
        << "  --snake-initial-turn <auto|left|right>\n"
        << "  --video                      Enable GCS MJPEG streaming\n"
        << "  --no-video                   Disable GCS MJPEG streaming\n"
        << "  --no-telemetry               Disable GCS telemetry sending\n"
        << "  --allow-arm-takeoff          Permit real arm+takeoff on Pixhawk1 target\n"
        << "  --no-arm                     Skip arm/takeoff (vision/state-machine smoke)\n"
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
        else if (a == "--vision") o.vision = next("--vision");
        else if (a == "--world") o.world = next("--world");
        else if (a == "--line-mode") o.line_mode_override = next("--line-mode");
        else if (a == "--gcs-ip") o.gcs_ip_override = next("--gcs-ip");
        else if (a == "--gazebo-topic") o.gazebo_topic_override = next("--gazebo-topic");
        else if (a == "--marker-count") o.marker_count = parseInt(next("--marker-count"), -1);
        else if (a == "--vertiport-marker-id") o.vertiport_marker_id = parseInt(next("--vertiport-marker-id"), 23);
        else if (a == "--max-intersections") o.max_intersections_override = parseInt(next("--max-intersections"), 0);
        else if (a == "--snake-initial-turn") o.snake_initial_turn = next("--snake-initial-turn");
        else if (a == "--video") { o.send_video = true; o.send_video_overridden = true; }
        else if (a == "--no-video") { o.send_video = false; o.send_video_overridden = true; }
        else if (a == "--no-telemetry") o.send_telemetry = false;
        else if (a == "--allow-arm-takeoff") o.allow_arm_takeoff = true;
        else if (a == "--no-arm") o.no_arm = true;
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
    ocommon::NetworkConfig network;
    ocommon::VisionConfig vision;
    omission::GridMissionConfig mission;
    ocontrol::GuidedVelocityControllerConfig controller;
    ocontrol::GridControlMapperConfig mapper;
    omission::AltitudePolicyConfig altitude;
    omission::SnakePlannerConfig snake;
    std::string udp_listen_host = "0.0.0.0";
    std::uint16_t udp_listen_port = 14550;
    int setpoint_rate_hz = 20;
};

void loadConfigs(const Options& opt, Configs& cfg)
{
    // Network (shared with line-follow path)
    cfg.network = ocommon::loadNetworkConfig(joinPath(opt.config_dir, "network.toml"));

    // Vision
    cfg.vision = ocommon::loadVisionConfig(joinPath(opt.config_dir, "vision.toml"));
    if (!opt.line_mode_override.empty()) {
        cfg.vision.line.mode = opt.line_mode_override;
    }
    if (!opt.gazebo_topic_override.empty()) {
        cfg.vision.source.gazebo_topic = opt.gazebo_topic_override;
    }
    if (!opt.gcs_ip_override.empty()) {
        cfg.network.gcs_ip = opt.gcs_ip_override;
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
            g.snake_line_lost_warn_s = gm["snake_line_lost_warn_s"].value_or(g.snake_line_lost_warn_s);
            g.snake_line_lost_emergency_s = gm["snake_line_lost_emergency_s"].value_or(g.snake_line_lost_emergency_s);
            g.snake_advance_timeout_s = gm["snake_advance_timeout_s"].value_or(g.snake_advance_timeout_s);
            g.max_intersections = gm["max_intersections"].value_or(g.max_intersections);
            g.mission_timeout_s = gm["mission_timeout_s"].value_or(g.mission_timeout_s);
            g.altitude_ceiling_m = gm["altitude_ceiling_m"].value_or(g.altitude_ceiling_m);
            g.snake_complete_hover_s = gm["snake_complete_hover_s"].value_or(g.snake_complete_hover_s);
            g.snake_record_dwell_s =
                gm["snake_record_dwell_s"].value_or(g.snake_record_dwell_s);
            g.snake_post_record_grace_s =
                gm["snake_post_record_grace_s"].value_or(g.snake_post_record_grace_s);
            g.snake_post_turn_blind_s =
                gm["snake_post_turn_blind_s"].value_or(g.snake_post_turn_blind_s);
            g.hop_align_start_m = gm["hop_align_start_m"].value_or(g.hop_align_start_m);
            g.hop_align_end_m = gm["hop_align_end_m"].value_or(g.hop_align_end_m);
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
            // Cycle 13: expose forward_speed_advance_mps so the slow speed
            // applied during SnakeAdvanceOneCell's line-follow phase can be
            // tuned without recompiling.
            cfg.mapper.forward_speed_advance_mps =
                gm["forward_speed_advance_mps"].value_or(cfg.mapper.forward_speed_advance_mps);
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
    cfg.mission.initial_snake_turn = cfg.snake.initial_turn;

    cfg.altitude.vertiport_altitude_m = cfg.mission.vertiport_altitude_m;
    cfg.altitude.cruise_altitude_m = cfg.mission.cruise_altitude_m;

    // Load runtime profile (UDP transport for SITL)
    const std::string runtime_file = (opt.target == "sitl")
        ? joinPath(opt.config_dir, opt.world == "grid"
            ? "runtime.sitl.grid.toml"
            : "runtime.sitl.toml")
        : joinPath(opt.config_dir, "autopilot.toml");
    try {
        auto rt = toml::parse_file(runtime_file);
        auto tr = rt["transport"];
        if (tr) {
            cfg.udp_listen_host = tr["listen_host"].value_or(std::string{cfg.udp_listen_host});
            cfg.udp_listen_port = static_cast<std::uint16_t>(
                tr["listen_port"].value_or(static_cast<int>(cfg.udp_listen_port)));
        }
        auto src = rt["vision"]["source"];
        if (src && opt.gazebo_topic_override.empty()) {
            cfg.vision.source.gazebo_topic = src["gazebo_topic"].value_or(std::string{cfg.vision.source.gazebo_topic});
        }
    } catch (const toml::parse_error& e) {
        std::cerr << "[config] runtime profile parse warning (" << runtime_file << "): " << e.what() << "\n";
    }

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
    // Cycle 10: registry ID list (committed markers).
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
              << (out.last_safety_event.empty() ? "" : (" safety=" + out.last_safety_event))
              << "\n";
}

} // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, handleSigint);
    std::signal(SIGTERM, handleSigint);

    const Options opt = parseOptions(argc, argv);
    if (opt.marker_count <= 0) {
        std::cerr << "error: --marker-count is required (positive integer)\n";
        return 2;
    }
    if (opt.target == "pixhawk1" && !opt.allow_arm_takeoff && !opt.no_arm) {
        std::cerr << "error: pixhawk1 target requires --allow-arm-takeoff (or --no-arm for bench)\n";
        return 2;
    }

    Configs cfg;
    loadConfigs(opt, cfg);

    std::cout << "[grid-mission] starting target=" << opt.target
              << " vision=" << opt.vision
              << " world=" << opt.world
              << " line_mode=" << opt.line_mode_override
              << " marker_count=" << opt.marker_count
              << " vertiport_id=" << cfg.mission.vertiport_marker_id
              << " udp=" << cfg.udp_listen_host << ":" << cfg.udp_listen_port
              << " gcs=" << cfg.network.gcs_ip << ":" << cfg.network.telemetry_port
              << "\n";

    // Vision setup
    auto frame_source = createFrameSource(opt.vision);
    ovision::FrameSourceOptions fs_opts {cfg.vision};
    if (!frame_source->open(fs_opts)) {
        std::cerr << "vision source open failed: " << frame_source->lastError() << "\n";
        return 1;
    }
    ovision::VisionProcessor processor(ovision::VisionProcessorOptions {
        cfg.vision, true, true,
    });

    // GCS publisher
    oapp::VisionDebugPublisher publisher;
    oapp::VisionDebugPublisherOptions pub_opts;
    pub_opts.network = cfg.network;
    pub_opts.vision = cfg.vision;
    pub_opts.send_video = opt.send_video_overridden
        ? opt.send_video
        : cfg.vision.debug_video.enabled;
    pub_opts.send_telemetry = opt.send_telemetry;
    pub_opts.note = "grid_mission";
    pub_opts.camera_sensor_model = cfg.vision.camera.sensor_model;
    if (!publisher.open(pub_opts)) {
        std::cerr << "[gcs] warning: publisher open failed: " << publisher.lastError() << "\n";
    }

    // Autopilot setup (UDP only for now; pixhawk1 path requires serial — out of scope first pass)
    if (opt.target != "sitl" && !opt.no_arm) {
        std::cerr << "error: pixhawk1 grid mission not yet implemented in grid_mission_node (use --no-arm for vision smoke)\n";
        return 2;
    }
    auto udp_transport = std::make_unique<oautopilot::UdpMavlinkTransport>(
        cfg.udp_listen_port,
        static_cast<std::uint8_t>(MAVLINK_COMM_0),
        std::string("grid_mission"));
    oautopilot::MavlinkIds mavlink_ids; // defaults
    oautopilot::AutopilotMavlinkAdapter autopilot(std::move(udp_transport), mavlink_ids);

    std::cout << "[mavlink] waiting for heartbeat on " << cfg.udp_listen_host
              << ":" << cfg.udp_listen_port << "...\n";
    autopilot.waitHeartbeat(std::chrono::seconds(30));
    std::cout << "[mavlink] heartbeat received.\n";
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

    if (!opt.no_arm) {
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

    const auto period = std::chrono::milliseconds(1000 / std::max(1, cfg.setpoint_rate_hz));
    bool first_frame = true;
    // Cycle 10: latch the last committed node event so we can keep retransmitting
    // it to the GCS each frame (UDP loss redundancy + no peek-stage ghost nodes).
    omission::GridNodeEvent last_committed_event;

    while (!g_shutdown_requested.load() && mission.state() != omission::GridState::Done) {
        const auto loop_start = Clock::now();
        autopilot.poll(1);

        // Read + process a frame
        ovision::Frame frame;
        try {
            if (!frame_source->read(frame)) {
                std::cerr << "[vision] read failed: " << frame_source->lastError() << "\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
        } catch (const std::exception& e) {
            std::cerr << "[vision] exception: " << e.what() << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        const auto vp_started = Clock::now();
        auto vp_out = processor.process(frame.image_bgr,
            ovision::VisionFrameMetadata{frame.frame_id, frame.timestamp_ms, frame.width, frame.height});
        const auto vp_finished = Clock::now();
        const double processing_latency_ms = std::chrono::duration<double, std::milli>(
            vp_finished - vp_started).count();

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
            cur_state == omission::GridState::SnakeTurn90Again;
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
        min.heartbeat_recent = apst.heartbeat_seen;
        min.autopilot_ready = apst.heartbeat_seen;
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
        min.abort_requested = false;

        // Mission update
        auto mout = mission.update(min);

        // Cycle 9: tracker advances only when GridMission says so (post all
        // gates). The earlier tracker.update call above was peek-only.
        // Cycle 23: GridMission may attach a synthetic event for boundary
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

        // Cycle 12 A: one-shot synthetic origin event from GRID_ORIGIN_LOCK so
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
        // Cycle 12 B: forward intersection cy for StopAndCenter cy-feedback decel.
        cmin.intersection_valid = mout.intersection_valid;
        cmin.intersection_center_x_norm = mout.intersection_center_x_norm;
        cmin.intersection_center_y_norm = mout.intersection_center_y_norm;

        const auto sp = mapper.compute(cmin);

        // Land request
        if (mout.request_land_mode) {
            try {
                autopilot.setLandMode(std::chrono::seconds(5));
            } catch (const std::exception& e) {
                std::cerr << "[mavlink] setLandMode error: " << e.what() << "\n";
            }
        } else if (mout.intent != ocontrol::GridControlIntent::Idle &&
                   mout.intent != ocontrol::GridControlIntent::Land) {
            oautopilot::BodyVelocityCommand cmd;
            cmd.vx_forward_mps = sp.vx_forward_mps;
            cmd.vy_right_mps = sp.vy_right_mps;
            cmd.vz_down_mps = sp.vz_down_mps;
            cmd.yaw_rate_rad_s = sp.yaw_rate_rad_s;
            autopilot.sendBodyVelocity(cmd);
        }

        // Publish telemetry / video
        if (publisher.lastError().empty() || first_frame) {
            oapp::VisionDebugPublishInput pin;
            pin.frame = frame;
            pin.image_bgr = frame.image_bgr;
            pin.vision_output = vp_out;
            pin.intersection_decision = idec;
            // Cycle 10: send the last committed node every frame (not the peek
            // event). Peek events would leak an uncommitted "ghost" node ahead
            // of the drone into the GCS map; resending the last commit gives
            // UDP loss redundancy so GCS edges never have gaps.
            pin.grid_node = last_committed_event;
            // Cycle 13: forward drone fractional position so the GCS can
            // render the heading arrow at a sub-cell position between commits.
            pin.drone_position_valid = mout.drone_position_valid;
            pin.drone_cell_progress = mout.drone_cell_progress;
            pin.drone_grid_offset_x = mout.drone_grid_offset_x;
            pin.drone_grid_offset_y = mout.drone_grid_offset_y;
            // Cycle 23: populate mission telemetry so the GCS sees the
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
            pin.mission.grid.valid = mout.current_heading != omission::GridHeading::Unknown;
            pin.mission.vertiport.verified = mout.vertiport_verified;
            pin.mission.vertiport.marker_id = cfg.mission.vertiport_marker_id;
            pin.mission.markers_expected = cfg.mission.markers_expected;
            pin.mission.snake_complete =
                mout.state == omission::GridState::SnakeComplete ||
                mout.state == omission::GridState::Land ||
                mout.state == omission::GridState::Done;
            pin.mission.last_safety_event = mout.last_safety_event;
            pin.mission.markers_found.clear();
            const double mission_now_s = now_s();
            for (const auto& m : registry.records()) {
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
