#include "autopilot/SerialMavlinkTransport.hpp"
#include "autopilot/UdpMavlinkTransport.hpp"

#include <toml++/toml.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
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
    std::uint16_t listen_port = 14550;
    std::string serial_device = "/dev/ttyACM0";
    int serial_baudrate = 115200;
};

struct Options {
    std::string config_dir = "config";
    std::string target;
    std::string autopilot_uri;
    std::string param_file;
    int duration_ms = 10000;
    int heartbeat_timeout_ms = 30000;
    bool dump_params = false;
    bool apply_params = false;
    bool allow_param_write = false;
    bool strict_local_estimate = false;
    bool strict_rc = false;
    bool strict_battery = false;
};

struct ParamValue {
    float value = 0.0f;
    std::uint8_t type = MAV_PARAM_TYPE_REAL32;
    std::uint16_t index = 0;
    std::uint16_t count = 0;
};

struct ParamSetRequest {
    std::string name;
    float value = 0.0f;
};

struct ProbeState {
    bool heartbeat_seen = false;
    std::uint8_t target_system = 1;
    std::uint8_t target_component = 1;
    std::uint32_t custom_mode = 0;
    std::uint8_t base_mode = 0;
    std::uint8_t system_status = 0;
    std::string mode_name = "unknown";
    bool armed = false;
    std::map<std::uint32_t, int> message_counts;
    std::map<std::string, ParamValue> params;
    std::optional<double> battery_voltage_v;
    std::optional<double> battery_current_a;
    std::optional<int> battery_remaining_pct;
    std::optional<int> gps_fix_type;
    std::optional<int> gps_satellites;
    std::optional<double> relative_altitude_m;
    std::optional<double> local_x_m;
    std::optional<double> local_y_m;
    std::optional<double> local_z_m;
    std::optional<double> local_vx_mps;
    std::optional<double> local_vy_mps;
    std::optional<double> local_vz_mps;
    std::optional<double> distance_sensor_m;
    std::optional<double> rangefinder_m;
    std::optional<int> optical_flow_quality;
    std::optional<double> optical_flow_ground_distance_m;
    std::optional<int> rc_channel_count;
    std::optional<int> rc_rssi;
    std::optional<std::uint16_t> ekf_flags;
    std::vector<std::string> statustext;
    std::optional<std::uint32_t> autopilot_capabilities;
    std::chrono::steady_clock::time_point last_heartbeat_time {};
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
        << "Usage: mavlink_probe [options]\n\n"
        << "Options:\n"
        << "  --config <dir>              Config directory\n"
        << "  --target <sitl|ardupilot_serial>\n"
        << "                              Runtime target profile\n"
        << "  --autopilot <uri>           udp://0.0.0.0:14550 or serial:///dev/ttyACM0:115200\n"
        << "  --duration-ms <n>           Probe duration after heartbeat, default 10000\n"
        << "  --heartbeat-timeout-ms <n>  Heartbeat timeout, default 30000\n"
        << "  --dump-params               Request and print full parameter list\n"
        << "  --param-file <path>         Compare/apply parameter file entries\n"
        << "  --apply-params              Apply parameter file entries to the autopilot\n"
        << "  --i-understand-this-writes-autopilot-params\n"
        << "                              Required with --apply-params\n"
        << "  --strict-local-estimate     Fail if local/range/flow telemetry is missing\n"
        << "  --strict-rc                 Fail if RC_CHANNELS are missing\n"
        << "  --strict-battery            Fail if battery telemetry is unavailable\n"
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

std::uint16_t parsePort(const std::string& value, std::uint16_t fallback)
{
    const int parsed = parseInt(value, fallback);
    if (parsed <= 0 || parsed > 65535) {
        return fallback;
    }
    return static_cast<std::uint16_t>(parsed);
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
        } else if (arg == "--duration-ms" && i + 1 < argc) {
            options.duration_ms = parseInt(argv[++i], options.duration_ms);
        } else if (arg == "--heartbeat-timeout-ms" && i + 1 < argc) {
            options.heartbeat_timeout_ms = parseInt(argv[++i], options.heartbeat_timeout_ms);
        } else if (arg == "--dump-params") {
            options.dump_params = true;
        } else if (arg == "--param-file" && i + 1 < argc) {
            options.param_file = argv[++i];
        } else if (arg == "--apply-params") {
            options.apply_params = true;
        } else if (arg == "--i-understand-this-writes-autopilot-params") {
            options.allow_param_write = true;
        } else if (arg == "--strict-local-estimate") {
            options.strict_local_estimate = true;
        } else if (arg == "--strict-rc") {
            options.strict_rc = true;
        } else if (arg == "--strict-battery") {
            options.strict_battery = true;
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

void applyAutopilotUri(EndpointConfig& endpoint, const std::string& uri)
{
    if (uri.rfind("udp://", 0) == 0) {
        const std::string rest = uri.substr(6);
        const auto colon = rest.rfind(':');
        endpoint.kind = TransportKind::Udp;
        endpoint.label = "udp";
        endpoint.listen_port = colon == std::string::npos
            ? parsePort(rest, endpoint.listen_port)
            : parsePort(rest.substr(colon + 1), endpoint.listen_port);
        return;
    }

    if (uri.rfind("serial://", 0) == 0) {
        const std::string rest = uri.substr(9);
        const auto colon = rest.rfind(':');
        endpoint.kind = TransportKind::Serial;
        endpoint.label = "serial";
        if (colon == std::string::npos) {
            endpoint.serial_device = rest;
        } else {
            endpoint.serial_device = rest.substr(0, colon);
            endpoint.serial_baudrate = parseInt(
                rest.substr(colon + 1),
                endpoint.serial_baudrate);
        }
        return;
    }

    throw std::runtime_error("unsupported --autopilot URI: " + uri);
}

EndpointConfig loadEndpointConfig(const Options& options)
{
    EndpointConfig endpoint;
    const std::string target = options.target;

    try {
        const auto table = toml::parse_file(joinConfigPath(options.config_dir, "autopilot.toml"));
        if (const auto transport = table["transport"]) {
            const std::string kind = transport["kind"].value_or(std::string("udp"));
            endpoint.kind = kind == "serial" ? TransportKind::Serial : TransportKind::Udp;
            endpoint.listen_port = static_cast<std::uint16_t>(
                transport["listen_port"].value_or(static_cast<int>(endpoint.listen_port)));
        }
        if (const auto serial = table["serial"]) {
            endpoint.serial_device = serial["device"].value_or(endpoint.serial_device);
            endpoint.serial_baudrate = serial["baudrate"].value_or(endpoint.serial_baudrate);
        }
    } catch (const toml::parse_error&) {
    }

    if (target == "sitl") {
        endpoint.kind = TransportKind::Udp;
        endpoint.label = "sitl";
    } else if (target == "ardupilot_serial") {
        endpoint.kind = TransportKind::Serial;
        endpoint.label = "ardupilot_serial";
    } else if (!target.empty()) {
        throw std::runtime_error("unknown --target: " + options.target);
    }

    if (!target.empty()) {
        try {
            const auto table = toml::parse_file(
                joinConfigPath(options.config_dir, "runtime." + target + ".toml"));
            if (const auto transport = table["transport"]) {
                const std::string kind = transport["kind"].value_or(std::string(""));
                if (kind == "serial") {
                    endpoint.kind = TransportKind::Serial;
                } else if (kind == "udp") {
                    endpoint.kind = TransportKind::Udp;
                }
                endpoint.listen_port = static_cast<std::uint16_t>(
                    transport["listen_port"].value_or(static_cast<int>(endpoint.listen_port)));
            }
            if (const auto serial = table["serial"]) {
                endpoint.serial_device = serial["device"].value_or(endpoint.serial_device);
                endpoint.serial_baudrate = serial["baudrate"].value_or(endpoint.serial_baudrate);
            }
        } catch (const toml::parse_error&) {
        }
    }

    if (!options.autopilot_uri.empty()) {
        applyAutopilotUri(endpoint, options.autopilot_uri);
    }

    return endpoint;
}

std::unique_ptr<onboard::autopilot::MavlinkTransport> createTransport(const EndpointConfig& endpoint)
{
    if (endpoint.kind == TransportKind::Serial) {
        return std::make_unique<onboard::autopilot::SerialMavlinkTransport>(
            endpoint.serial_device,
            endpoint.serial_baudrate,
            MAVLINK_COMM_0,
            endpoint.label);
    }
    return std::make_unique<onboard::autopilot::UdpMavlinkTransport>(
        endpoint.listen_port,
        MAVLINK_COMM_0,
        endpoint.label);
}

std::string copterModeName(std::uint32_t custom_mode)
{
    switch (custom_mode) {
    case COPTER_MODE_STABILIZE:
        return "STABILIZE";
    case COPTER_MODE_ALT_HOLD:
        return "ALT_HOLD";
    case COPTER_MODE_LOITER:
        return "LOITER";
    case COPTER_MODE_GUIDED:
        return "GUIDED";
    case COPTER_MODE_LAND:
        return "LAND";
    case COPTER_MODE_RTL:
        return "RTL";
    default:
        return "mode_" + std::to_string(custom_mode);
    }
}

bool isAutopilotHeartbeat(const mavlink_heartbeat_t& heartbeat)
{
    return heartbeat.type != MAV_TYPE_GCS &&
           heartbeat.autopilot != MAV_AUTOPILOT_INVALID;
}

void sendGcsHeartbeat(
    onboard::autopilot::MavlinkTransport& transport,
    std::uint8_t system_id,
    std::uint8_t component_id)
{
    mavlink_message_t message {};
    mavlink_msg_heartbeat_pack(
        system_id,
        component_id,
        &message,
        MAV_TYPE_GCS,
        MAV_AUTOPILOT_INVALID,
        0,
        0,
        MAV_STATE_ACTIVE);
    transport.sendMessage(message);
}

void sendCommandLong(
    onboard::autopilot::MavlinkTransport& transport,
    const ProbeState& state,
    std::uint16_t command,
    float p1 = 0.0f,
    float p2 = 0.0f,
    float p3 = 0.0f,
    float p4 = 0.0f,
    float p5 = 0.0f,
    float p6 = 0.0f,
    float p7 = 0.0f)
{
    mavlink_message_t message {};
    mavlink_msg_command_long_pack(
        191,
        191,
        &message,
        state.target_system,
        state.target_component,
        command,
        0,
        p1,
        p2,
        p3,
        p4,
        p5,
        p6,
        p7);
    transport.sendMessage(message);
}

void requestMessageInterval(
    onboard::autopilot::MavlinkTransport& transport,
    const ProbeState& state,
    std::uint32_t message_id,
    double rate_hz)
{
    if (rate_hz <= 0.0) {
        return;
    }
    sendCommandLong(
        transport,
        state,
        MAV_CMD_SET_MESSAGE_INTERVAL,
        static_cast<float>(message_id),
        static_cast<float>(1000000.0 / rate_hz));
}

void requestDataStream(onboard::autopilot::MavlinkTransport& transport, const ProbeState& state)
{
    mavlink_message_t message {};
    mavlink_msg_request_data_stream_pack(
        191,
        191,
        &message,
        state.target_system,
        state.target_component,
        MAV_DATA_STREAM_ALL,
        10,
        1);
    transport.sendMessage(message);
}

void requestDefaultStreams(onboard::autopilot::MavlinkTransport& transport, const ProbeState& state)
{
    requestDataStream(transport, state);
    requestMessageInterval(transport, state, MAVLINK_MSG_ID_SYS_STATUS, 2.0);
    requestMessageInterval(transport, state, MAVLINK_MSG_ID_GPS_RAW_INT, 2.0);
    requestMessageInterval(transport, state, MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10.0);
    requestMessageInterval(transport, state, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5.0);
    requestMessageInterval(transport, state, MAVLINK_MSG_ID_DISTANCE_SENSOR, 10.0);
#ifdef MAVLINK_MSG_ID_RANGEFINDER
    requestMessageInterval(transport, state, MAVLINK_MSG_ID_RANGEFINDER, 10.0);
#endif
#ifdef MAVLINK_MSG_ID_OPTICAL_FLOW
    requestMessageInterval(transport, state, MAVLINK_MSG_ID_OPTICAL_FLOW, 10.0);
#endif
#ifdef MAVLINK_MSG_ID_OPTICAL_FLOW_RAD
    requestMessageInterval(transport, state, MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, 10.0);
#endif
#ifdef MAVLINK_MSG_ID_EKF_STATUS_REPORT
    requestMessageInterval(transport, state, MAVLINK_MSG_ID_EKF_STATUS_REPORT, 5.0);
#endif
#ifdef MAVLINK_MSG_ID_RC_CHANNELS
    requestMessageInterval(transport, state, MAVLINK_MSG_ID_RC_CHANNELS, 5.0);
#endif
#ifdef MAVLINK_MSG_ID_AUTOPILOT_VERSION
    requestMessageInterval(transport, state, MAVLINK_MSG_ID_AUTOPILOT_VERSION, 1.0);
#endif
}

std::string fixedString(const char* data, std::size_t max_length)
{
    std::size_t length = 0;
    while (length < max_length && data[length] != '\0') {
        ++length;
    }
    return std::string(data, data + length);
}

std::string paramIdString(const char param_id[16])
{
    return fixedString(param_id, 16);
}

void requestParamList(onboard::autopilot::MavlinkTransport& transport, const ProbeState& state)
{
    mavlink_message_t message {};
    mavlink_msg_param_request_list_pack(
        191,
        191,
        &message,
        state.target_system,
        state.target_component);
    transport.sendMessage(message);
}

bool processMessage(
    onboard::autopilot::MavlinkTransport& transport,
    ProbeState& state,
    const mavlink_message_t& message)
{
    state.message_counts[message.msgid]++;

    if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t heartbeat {};
        mavlink_msg_heartbeat_decode(&message, &heartbeat);
        if (!isAutopilotHeartbeat(heartbeat)) {
            return false;
        }
        transport.pinPeerFromLastMessage();
        state.heartbeat_seen = true;
        state.target_system = message.sysid;
        state.target_component =
            message.compid == 0 ? static_cast<std::uint8_t>(MAV_COMP_ID_AUTOPILOT1) : message.compid;
        state.custom_mode = heartbeat.custom_mode;
        state.base_mode = heartbeat.base_mode;
        state.system_status = heartbeat.system_status;
        state.armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        state.mode_name = copterModeName(heartbeat.custom_mode);
        state.last_heartbeat_time = Clock::now();
        return true;
    }

    if (message.sysid != state.target_system) {
        return false;
    }

    switch (message.msgid) {
    case MAVLINK_MSG_ID_SYS_STATUS: {
        mavlink_sys_status_t status {};
        mavlink_msg_sys_status_decode(&message, &status);
        if (status.voltage_battery != UINT16_MAX && status.voltage_battery > 0) {
            state.battery_voltage_v = status.voltage_battery / 1000.0;
        }
        if (status.current_battery != -1) {
            state.battery_current_a = status.current_battery / 100.0;
        }
        if (status.battery_remaining >= 0) {
            state.battery_remaining_pct = status.battery_remaining;
        }
        break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_gps_raw_int_t gps {};
        mavlink_msg_gps_raw_int_decode(&message, &gps);
        state.gps_fix_type = gps.fix_type;
        state.gps_satellites = gps.satellites_visible;
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t position {};
        mavlink_msg_global_position_int_decode(&message, &position);
        state.relative_altitude_m = position.relative_alt / 1000.0;
        break;
    }
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
        mavlink_local_position_ned_t position {};
        mavlink_msg_local_position_ned_decode(&message, &position);
        if (std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z)) {
            state.local_x_m = position.x;
            state.local_y_m = position.y;
            state.local_z_m = position.z;
            state.local_vx_mps = position.vx;
            state.local_vy_mps = position.vy;
            state.local_vz_mps = position.vz;
        }
        break;
    }
    case MAVLINK_MSG_ID_DISTANCE_SENSOR: {
        mavlink_distance_sensor_t distance {};
        mavlink_msg_distance_sensor_decode(&message, &distance);
        if (distance.current_distance > 0) {
            state.distance_sensor_m = distance.current_distance / 100.0;
        }
        break;
    }
#ifdef MAVLINK_MSG_ID_RANGEFINDER
    case MAVLINK_MSG_ID_RANGEFINDER: {
        mavlink_rangefinder_t range {};
        mavlink_msg_rangefinder_decode(&message, &range);
        if (std::isfinite(range.distance) && range.distance > 0.0f) {
            state.rangefinder_m = range.distance;
        }
        break;
    }
#endif
#ifdef MAVLINK_MSG_ID_OPTICAL_FLOW
    case MAVLINK_MSG_ID_OPTICAL_FLOW: {
        mavlink_optical_flow_t flow {};
        mavlink_msg_optical_flow_decode(&message, &flow);
        state.optical_flow_quality = flow.quality;
        const float ground_distance = flow.ground_distance;
        if (std::isfinite(ground_distance) && ground_distance > 0.0f) {
            state.optical_flow_ground_distance_m = ground_distance;
        }
        break;
    }
#endif
#ifdef MAVLINK_MSG_ID_OPTICAL_FLOW_RAD
    case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD: {
        mavlink_optical_flow_rad_t flow {};
        mavlink_msg_optical_flow_rad_decode(&message, &flow);
        state.optical_flow_quality = flow.quality;
        if (std::isfinite(flow.distance) && flow.distance > 0.0f) {
            state.optical_flow_ground_distance_m = flow.distance;
        }
        break;
    }
#endif
#ifdef MAVLINK_MSG_ID_EKF_STATUS_REPORT
    case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
        mavlink_ekf_status_report_t ekf {};
        mavlink_msg_ekf_status_report_decode(&message, &ekf);
        const std::uint16_t flags = ekf.flags;
        state.ekf_flags = flags;
        break;
    }
#endif
#ifdef MAVLINK_MSG_ID_RC_CHANNELS
    case MAVLINK_MSG_ID_RC_CHANNELS: {
        mavlink_rc_channels_t channels {};
        mavlink_msg_rc_channels_decode(&message, &channels);
        state.rc_channel_count = channels.chancount;
        state.rc_rssi = channels.rssi;
        break;
    }
#endif
    case MAVLINK_MSG_ID_STATUSTEXT: {
        mavlink_statustext_t text {};
        mavlink_msg_statustext_decode(&message, &text);
        const std::string line = fixedString(text.text, sizeof(text.text));
        if (!line.empty() &&
            std::find(state.statustext.begin(), state.statustext.end(), line) == state.statustext.end()) {
            state.statustext.push_back(line);
        }
        break;
    }
    case MAVLINK_MSG_ID_PARAM_VALUE: {
        mavlink_param_value_t param {};
        mavlink_msg_param_value_decode(&message, &param);
        state.params[paramIdString(param.param_id)] =
            ParamValue { param.param_value, param.param_type, param.param_index, param.param_count };
        break;
    }
#ifdef MAVLINK_MSG_ID_AUTOPILOT_VERSION
    case MAVLINK_MSG_ID_AUTOPILOT_VERSION: {
        mavlink_autopilot_version_t version {};
        mavlink_msg_autopilot_version_decode(&message, &version);
        state.autopilot_capabilities = version.capabilities;
        break;
    }
#endif
    default:
        break;
    }
    return false;
}

void pollFor(
    onboard::autopilot::MavlinkTransport& transport,
    ProbeState& state,
    std::chrono::milliseconds duration)
{
    const auto deadline = Clock::now() + duration;
    while (Clock::now() < deadline) {
        mavlink_message_t message {};
        if (transport.recvMessage(message, 100)) {
            processMessage(transport, state, message);
            for (int drained = 0; drained < 128; ++drained) {
                mavlink_message_t pending {};
                if (!transport.recvMessage(pending, 0)) {
                    break;
                }
                processMessage(transport, state, pending);
            }
        }
    }
}

void waitHeartbeat(
    onboard::autopilot::MavlinkTransport& transport,
    ProbeState& state,
    std::chrono::milliseconds timeout)
{
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        mavlink_message_t message {};
        if (transport.recvMessage(message, 1000)) {
            processMessage(transport, state, message);
            if (state.heartbeat_seen) {
                return;
            }
        }
    }
    throw std::runtime_error("timed out waiting for MAVLink heartbeat");
}

std::string optionalDouble(const std::optional<double>& value, int precision = 3)
{
    if (!value) {
        return "n/a";
    }
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(precision);
    out << *value;
    return out.str();
}

std::string optionalInt(const std::optional<int>& value)
{
    return value ? std::to_string(*value) : "n/a";
}

std::string optionalHex16(const std::optional<std::uint16_t>& value)
{
    if (!value) {
        return "n/a";
    }
    std::ostringstream out;
    out << "0x" << std::hex << *value;
    return out.str();
}

void printMessageCounts(const ProbeState& state)
{
    std::cout << "message_counts:";
    for (const auto& [id, count] : state.message_counts) {
        std::cout << ' ' << id << ':' << count;
    }
    std::cout << "\n";
}

void printSummary(const ProbeState& state)
{
    std::cout << "heartbeat: system=" << static_cast<int>(state.target_system)
              << " component=" << static_cast<int>(state.target_component)
              << " mode=" << state.mode_name << '(' << state.custom_mode << ')'
              << " armed=" << (state.armed ? "true" : "false")
              << " base_mode=0x" << std::hex << static_cast<int>(state.base_mode)
              << std::dec << " system_status=" << static_cast<int>(state.system_status) << "\n";
    std::cout << "battery: voltage_v=" << optionalDouble(state.battery_voltage_v, 2)
              << " current_a=" << optionalDouble(state.battery_current_a, 2)
              << " remaining_pct=" << optionalInt(state.battery_remaining_pct) << "\n";
    std::cout << "gps: fix_type=" << optionalInt(state.gps_fix_type)
              << " satellites=" << optionalInt(state.gps_satellites) << "\n";
    std::cout << "global: relative_alt_m=" << optionalDouble(state.relative_altitude_m) << "\n";
    std::cout << "local_position_ned: x=" << optionalDouble(state.local_x_m)
              << " y=" << optionalDouble(state.local_y_m)
              << " z=" << optionalDouble(state.local_z_m)
              << " vx=" << optionalDouble(state.local_vx_mps)
              << " vy=" << optionalDouble(state.local_vy_mps)
              << " vz=" << optionalDouble(state.local_vz_mps) << "\n";
    std::cout << "range: distance_sensor_m=" << optionalDouble(state.distance_sensor_m)
              << " rangefinder_m=" << optionalDouble(state.rangefinder_m) << "\n";
    std::cout << "optical_flow: quality=" << optionalInt(state.optical_flow_quality)
              << " ground_distance_m=" << optionalDouble(state.optical_flow_ground_distance_m) << "\n";
    std::cout << "rc: channels=" << optionalInt(state.rc_channel_count)
              << " rssi=" << optionalInt(state.rc_rssi) << "\n";
    std::cout << "ekf: flags=" << optionalHex16(state.ekf_flags) << "\n";
    if (!state.statustext.empty()) {
        std::cout << "statustext:\n";
        for (const auto& text : state.statustext) {
            std::cout << "  - " << text << "\n";
        }
    }
    printMessageCounts(state);
}

std::string trim(std::string value)
{
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::vector<ParamSetRequest> readParamFile(const std::string& path)
{
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("failed to open param file: " + path);
    }

    std::vector<ParamSetRequest> params;
    std::string line;
    int line_number = 0;
    while (std::getline(file, line)) {
        ++line_number;
        const auto comment = line.find('#');
        if (comment != std::string::npos) {
            line = line.substr(0, comment);
        }
        line = trim(line);
        if (line.empty()) {
            continue;
        }

        std::replace(line.begin(), line.end(), ',', ' ');
        const auto equals = line.find('=');
        if (equals != std::string::npos) {
            line[equals] = ' ';
        }

        std::istringstream input(line);
        std::string name;
        float value = 0.0f;
        if (!(input >> name >> value)) {
            throw std::runtime_error(
                "invalid param entry at " + path + ":" + std::to_string(line_number));
        }
        if (name.size() > 16) {
            throw std::runtime_error("MAVLink param name is longer than 16 chars: " + name);
        }
        params.push_back(ParamSetRequest { name, value });
    }
    return params;
}

void printParamComparison(
    const std::vector<ParamSetRequest>& wanted,
    const ProbeState& state)
{
    if (wanted.empty()) {
        return;
    }
    std::cout << "param_compare:\n";
    for (const auto& param : wanted) {
        const auto found = state.params.find(param.name);
        if (found == state.params.end()) {
            std::cout << "  " << param.name << " target=" << param.value << " current=missing\n";
            continue;
        }
        const float current = found->second.value;
        const bool matches = std::fabs(current - param.value) < 0.0001f;
        std::cout << "  " << param.name
                  << " target=" << param.value
                  << " current=" << current
                  << " status=" << (matches ? "ok" : "diff") << "\n";
    }
}

void printAllParams(const ProbeState& state)
{
    std::cout << "params_found=" << state.params.size() << "\n";
    for (const auto& [name, value] : state.params) {
        std::cout << name << '=' << value.value
                  << " type=" << static_cast<int>(value.type)
                  << " index=" << value.index
                  << " count=" << value.count << "\n";
    }
}

void sendParamSet(
    onboard::autopilot::MavlinkTransport& transport,
    const ProbeState& state,
    const ParamSetRequest& param)
{
    char param_id[16] {};
    std::copy(param.name.begin(), param.name.end(), param_id);

    mavlink_message_t message {};
    mavlink_msg_param_set_pack(
        191,
        191,
        &message,
        state.target_system,
        state.target_component,
        param_id,
        param.value,
        MAV_PARAM_TYPE_REAL32);
    transport.sendMessage(message);
}

bool waitParamEcho(
    onboard::autopilot::MavlinkTransport& transport,
    ProbeState& state,
    const ParamSetRequest& param,
    std::chrono::milliseconds timeout)
{
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        mavlink_message_t message {};
        if (!transport.recvMessage(message, 100)) {
            continue;
        }
        processMessage(transport, state, message);
        if (message.msgid != MAVLINK_MSG_ID_PARAM_VALUE) {
            continue;
        }
        mavlink_param_value_t value {};
        mavlink_msg_param_value_decode(&message, &value);
        if (paramIdString(value.param_id) == param.name &&
            std::fabs(value.param_value - param.value) < 0.0001f) {
            return true;
        }
    }
    return false;
}

void applyParams(
    onboard::autopilot::MavlinkTransport& transport,
    ProbeState& state,
    const std::vector<ParamSetRequest>& wanted)
{
    std::cout << "applying_params count=" << wanted.size() << "\n";
    for (const auto& param : wanted) {
        if (state.params.find(param.name) == state.params.end()) {
            std::cout << "[param] skip missing " << param.name << "\n";
            continue;
        }
        if (std::fabs(state.params[param.name].value - param.value) < 0.0001f) {
            std::cout << "[param] already ok " << param.name << '=' << param.value << "\n";
            continue;
        }
        std::cout << "[param] set " << param.name
                  << " from " << state.params[param.name].value
                  << " to " << param.value << "\n";
        sendParamSet(transport, state, param);
        if (!waitParamEcho(transport, state, param, std::chrono::seconds(3))) {
            std::cout << "[param] warning: no matching echo for " << param.name << "\n";
        }
    }
}

bool strictChecksPass(const Options& options, const ProbeState& state)
{
    bool ok = true;
    if (options.strict_local_estimate) {
        const bool have_local = state.local_x_m && state.local_y_m && state.local_z_m;
        const bool have_range = state.distance_sensor_m || state.rangefinder_m ||
            state.optical_flow_ground_distance_m;
        const bool have_flow = state.optical_flow_quality.has_value();
        if (!have_local || !have_range || !have_flow) {
            std::cerr << "[strict] local estimate gate failed:"
                      << " local=" << (have_local ? "yes" : "no")
                      << " range=" << (have_range ? "yes" : "no")
                      << " flow=" << (have_flow ? "yes" : "no") << "\n";
            ok = false;
        }
    }
    if (options.strict_rc) {
        if (!state.rc_channel_count || *state.rc_channel_count <= 0) {
            std::cerr << "[strict] RC_CHANNELS gate failed\n";
            ok = false;
        }
    }
    if (options.strict_battery) {
        if (!state.battery_voltage_v || *state.battery_voltage_v <= 0.0) {
            std::cerr << "[strict] battery telemetry gate failed\n";
            ok = false;
        }
    }
    return ok;
}

} // namespace

int main(int argc, char** argv)
{
    try {
        const Options options = parseOptions(argc, argv);
        if (options.apply_params && !options.allow_param_write) {
            throw std::runtime_error(
                "--apply-params requires --i-understand-this-writes-autopilot-params");
        }

        const EndpointConfig endpoint = loadEndpointConfig(options);
        std::cout << "[mavlink_probe] target="
                  << (options.target.empty() ? endpoint.label : options.target)
                  << " transport=" << (endpoint.kind == TransportKind::Serial ? "serial" : "udp");
        if (endpoint.kind == TransportKind::Serial) {
            std::cout << " device=" << endpoint.serial_device
                      << " baudrate=" << endpoint.serial_baudrate;
        } else {
            std::cout << " listen_port=" << endpoint.listen_port;
        }
        std::cout << "\n";

        auto transport = createTransport(endpoint);
        ProbeState state;
        waitHeartbeat(*transport, state, std::chrono::milliseconds(options.heartbeat_timeout_ms));
        sendGcsHeartbeat(*transport, 191, 191);
        requestDefaultStreams(*transport, state);

        std::vector<ParamSetRequest> param_file_entries;
        if (!options.param_file.empty()) {
            param_file_entries = readParamFile(options.param_file);
        }
        if (options.dump_params || !param_file_entries.empty()) {
            requestParamList(*transport, state);
        }

        pollFor(*transport, state, std::chrono::milliseconds(options.duration_ms));

        printSummary(state);
        printParamComparison(param_file_entries, state);
        if (options.dump_params) {
            printAllParams(state);
        }
        if (options.apply_params) {
            applyParams(*transport, state, param_file_entries);
            requestParamList(*transport, state);
            pollFor(*transport, state, std::chrono::milliseconds(5000));
            printParamComparison(param_file_entries, state);
        }

        return strictChecksPass(options, state) ? 0 : 3;
    } catch (const std::exception& error) {
        std::cerr << "[error] " << error.what() << "\n";
        return 1;
    }
}
