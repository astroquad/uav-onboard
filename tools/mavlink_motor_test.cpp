#include "autopilot/SerialMavlinkTransport.hpp"

#include <toml++/toml.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

using Clock = std::chrono::steady_clock;

struct Options {
    std::string config_dir = "config";
    std::string target = "ardupilot_serial";
    std::string device = "/dev/ttyACM0";
    int baudrate = 115200;
    int motor = 1;
    int motor_count = 1;
    float percent = 5.0f;
    float seconds = 1.0f;
    bool props_removed = false;
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
        << "Usage: mavlink_motor_test [options]\n\n"
        << "This tool sends MAV_CMD_DO_MOTOR_TEST. It does not arm or take off.\n"
        << "Run only with propellers removed and the vehicle physically clear.\n\n"
        << "Options:\n"
        << "  --config <dir>              Config directory\n"
        << "  --target <ardupilot_serial> Runtime target profile, default ardupilot_serial\n"
        << "  --device <path>             Serial device override\n"
        << "  --baudrate <n>              Serial baudrate, default 115200\n"
        << "  --motor <n>                 Motor instance, 1-based, default 1\n"
        << "  --motor-count <n>           Motor count for sequence, default 1\n"
        << "  --percent <n>               Throttle percent, 0-15 allowed here, default 5\n"
        << "  --seconds <n>               Motor test timeout seconds, 0.1-5, default 1\n"
        << "  --props-removed             Required safety acknowledgement\n"
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

float parseFloat(const std::string& value, float fallback)
{
    try {
        return std::stof(value);
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
        } else if (arg == "--device" && i + 1 < argc) {
            options.device = argv[++i];
        } else if (arg == "--baudrate" && i + 1 < argc) {
            options.baudrate = parseInt(argv[++i], options.baudrate);
        } else if (arg == "--motor" && i + 1 < argc) {
            options.motor = parseInt(argv[++i], options.motor);
        } else if (arg == "--motor-count" && i + 1 < argc) {
            options.motor_count = parseInt(argv[++i], options.motor_count);
        } else if (arg == "--percent" && i + 1 < argc) {
            options.percent = parseFloat(argv[++i], options.percent);
        } else if (arg == "--seconds" && i + 1 < argc) {
            options.seconds = parseFloat(argv[++i], options.seconds);
        } else if (arg == "--props-removed") {
            options.props_removed = true;
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

void loadRuntimeSerial(Options& options)
{
    try {
        const auto table = toml::parse_file(
            joinConfigPath(options.config_dir, "runtime." + options.target + ".toml"));
        if (const auto serial = table["serial"]) {
            options.device = serial["device"].value_or(options.device);
            options.baudrate = serial["baudrate"].value_or(options.baudrate);
        }
    } catch (const toml::parse_error&) {
    }
}

bool isAutopilotHeartbeat(const mavlink_heartbeat_t& heartbeat)
{
    return heartbeat.type != MAV_TYPE_GCS &&
           heartbeat.autopilot != MAV_AUTOPILOT_INVALID;
}

void sendGcsHeartbeat(onboard::autopilot::MavlinkTransport& transport)
{
    mavlink_message_t message {};
    mavlink_msg_heartbeat_pack(
        191,
        191,
        &message,
        MAV_TYPE_GCS,
        MAV_AUTOPILOT_INVALID,
        0,
        0,
        MAV_STATE_ACTIVE);
    transport.sendMessage(message);
}

struct Target {
    std::uint8_t system = 1;
    std::uint8_t component = 1;
    std::string mode = "unknown";
    bool armed = false;
};

Target waitHeartbeat(onboard::autopilot::MavlinkTransport& transport)
{
    const auto deadline = Clock::now() + std::chrono::seconds(30);
    Target target;
    while (Clock::now() < deadline) {
        sendGcsHeartbeat(transport);
        mavlink_message_t message {};
        if (!transport.recvMessage(message, 1000)) {
            continue;
        }
        if (message.msgid != MAVLINK_MSG_ID_HEARTBEAT) {
            continue;
        }
        mavlink_heartbeat_t heartbeat {};
        mavlink_msg_heartbeat_decode(&message, &heartbeat);
        if (!isAutopilotHeartbeat(heartbeat)) {
            continue;
        }
        target.system = message.sysid;
        target.component =
            message.compid == 0 ? static_cast<std::uint8_t>(MAV_COMP_ID_AUTOPILOT1) : message.compid;
        target.armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        target.mode = std::to_string(heartbeat.custom_mode);
        return target;
    }
    throw std::runtime_error("timed out waiting for MAVLink heartbeat");
}

void sendMotorTest(
    onboard::autopilot::MavlinkTransport& transport,
    const Target& target,
    const Options& options)
{
    mavlink_message_t message {};
    mavlink_msg_command_long_pack(
        191,
        191,
        &message,
        target.system,
        target.component,
        MAV_CMD_DO_MOTOR_TEST,
        0,
        static_cast<float>(options.motor),
        0.0f, // MOTOR_TEST_THROTTLE_PERCENT
        options.percent,
        options.seconds,
        static_cast<float>(options.motor_count),
        0.0f, // MOTOR_TEST_ORDER_DEFAULT
        0.0f);
    transport.sendMessage(message);
}

int waitCommandAck(onboard::autopilot::MavlinkTransport& transport)
{
    const auto deadline = Clock::now() + std::chrono::seconds(5);
    while (Clock::now() < deadline) {
        mavlink_message_t message {};
        if (!transport.recvMessage(message, 1000)) {
            continue;
        }
        if (message.msgid != MAVLINK_MSG_ID_COMMAND_ACK) {
            continue;
        }
        mavlink_command_ack_t ack {};
        mavlink_msg_command_ack_decode(&message, &ack);
        if (ack.command == MAV_CMD_DO_MOTOR_TEST) {
            return ack.result;
        }
    }
    return -1;
}

void validateOptions(const Options& options)
{
    if (!options.props_removed) {
        throw std::runtime_error("--props-removed is required");
    }
    if (options.motor < 1 || options.motor > 16) {
        throw std::runtime_error("--motor must be between 1 and 16");
    }
    if (options.motor_count < 1 || options.motor_count > 16) {
        throw std::runtime_error("--motor-count must be between 1 and 16");
    }
    if (options.percent < 0.0f || options.percent > 15.0f) {
        throw std::runtime_error("--percent must be between 0 and 15 for this safety tool");
    }
    if (options.seconds < 0.1f || options.seconds > 5.0f) {
        throw std::runtime_error("--seconds must be between 0.1 and 5");
    }
}

} // namespace

int main(int argc, char** argv)
{
    try {
        Options options = parseOptions(argc, argv);
        loadRuntimeSerial(options);
        validateOptions(options);

        std::cout << "[mavlink_motor_test] device=" << options.device
                  << " baudrate=" << options.baudrate
                  << " motor=" << options.motor
                  << " percent=" << options.percent
                  << " seconds=" << options.seconds << "\n";

        onboard::autopilot::SerialMavlinkTransport transport(
            options.device,
            options.baudrate,
            MAVLINK_COMM_0,
            "motor-test");
        const Target target = waitHeartbeat(transport);
        std::cout << "[mavlink] heartbeat system=" << static_cast<int>(target.system)
                  << " component=" << static_cast<int>(target.component)
                  << " armed=" << (target.armed ? "true" : "false") << "\n";

        sendMotorTest(transport, target, options);
        const int result = waitCommandAck(transport);
        if (result < 0) {
            std::cerr << "[motor-test] no COMMAND_ACK received\n";
            return 3;
        }
        std::cout << "[motor-test] COMMAND_ACK result=" << result << "\n";
        return result == MAV_RESULT_ACCEPTED ? 0 : 4;
    } catch (const std::exception& error) {
        std::cerr << "[error] " << error.what() << "\n";
        return 1;
    }
}
