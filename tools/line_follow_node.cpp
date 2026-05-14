#include "autopilot/AutopilotMavlinkAdapter.hpp"
#include "autopilot/UdpMavlinkTransport.hpp"
#include "control/GuidedVelocityController.hpp"
#include "mission/LineFollowMission.hpp"
#include "safety/SafetyMonitor.hpp"

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
    int setpoint_rate_hz = 20;
    onboard::mission::LineFollowMissionConfig mission;
    onboard::control::GuidedVelocityControllerConfig controller;
    onboard::safety::SafetyConfig safety;
    double fake_center_error_m = 0.0;
    double fake_line_angle_deg = 0.0;
};

struct Options {
    std::string config_dir = "config";
    std::string target;
    std::string autopilot_uri;
    std::string vision = "fake";
    std::optional<int> mission_timeout_ms;
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
        << "  --vision <fake>             Vision source for this MVP node\n"
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

RuntimeConfig loadRuntimeConfig(const Options& options)
{
    RuntimeConfig config;

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
    } else if (options.target == "pixhawk1") {
        config.endpoint.kind = TransportKind::Serial;
        config.endpoint.label = "pixhawk1";
    } else if (!options.target.empty()) {
        throw std::runtime_error("unknown --target: " + options.target);
    }

    if (options.mission_timeout_ms && *options.mission_timeout_ms > 0) {
        config.safety.mission_timeout_ms = *options.mission_timeout_ms;
    }

    if (!options.autopilot_uri.empty()) {
        applyAutopilotUri(config, options.autopilot_uri);
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

} // namespace

int main(int argc, char** argv)
{
    try {
        const Options options = parseOptions(argc, argv);
        if (options.vision != "fake") {
            throw std::runtime_error("only --vision fake is implemented in this MVP node");
        }

        const RuntimeConfig config = loadRuntimeConfig(options);
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
                  << " udp_port=" << config.endpoint.listen_port
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
            Clock::now(),
        });
        std::cout << "[mission] " << toString(mission.state()) << "\n";

        const auto period = std::chrono::duration<double>(
            1.0 / static_cast<double>(std::max(1, config.setpoint_rate_hz)));
        constexpr double pi = 3.14159265358979323846;
        const auto line_angle_rad = config.fake_line_angle_deg * pi / 180.0;
        const onboard::control::LineControlInput fake_line {
            true,
            config.fake_center_error_m,
            line_angle_rad,
            1.0,
        };

        while (mission.state() == onboard::mission::LineFollowMissionState::LineFollow) {
            const auto loop_start = Clock::now();
            autopilot.poll(1);

            const auto safety_decision = safety.update(onboard::safety::SafetyInput {
                autopilot.state().heartbeat_seen,
                fake_line.line_detected,
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
                fake_line.line_detected,
                safety_land,
                false,
                loop_start,
            });
            if (mission.state() != onboard::mission::LineFollowMissionState::LineFollow) {
                break;
            }

            autopilot.sendBodyVelocity(toAutopilotCommand(controller.update(fake_line)));
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
