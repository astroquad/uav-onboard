#include "autopilot/AutopilotMavlinkAdapter.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace onboard::autopilot {
namespace {

using Clock = std::chrono::steady_clock;
constexpr int kMaxPollDrainMessages = 128;

bool heartbeatArmed(const mavlink_heartbeat_t& heartbeat)
{
    return (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
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

bool isConfiguredTargetSystem(const mavlink_message_t& message, const MavlinkIds& ids)
{
    return ids.target_system == 0 || message.sysid == ids.target_system;
}

bool isAutopilotHeartbeat(const mavlink_heartbeat_t& heartbeat)
{
    return heartbeat.type != MAV_TYPE_GCS &&
           heartbeat.autopilot != MAV_AUTOPILOT_INVALID;
}

} // namespace

AutopilotMavlinkAdapter::AutopilotMavlinkAdapter(
    std::unique_ptr<MavlinkTransport> transport,
    MavlinkIds ids)
    : transport_(std::move(transport))
    , ids_(ids)
{
    state_.target_system = ids_.target_system;
    state_.target_component = ids_.target_component;
}

void AutopilotMavlinkAdapter::waitHeartbeat(std::chrono::seconds timeout)
{
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        if (poll(1000) && state_.heartbeat_seen) {
            return;
        }
    }
    throw std::runtime_error("timed out waiting for MAVLink heartbeat");
}

void AutopilotMavlinkAdapter::requestDefaultStreams()
{
    requestMessageInterval(MAVLINK_MSG_ID_DISTANCE_SENSOR, 10.0);
    requestMessageInterval(MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10.0);
    requestMessageInterval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5.0);
}

void AutopilotMavlinkAdapter::setGuidedMode(std::chrono::seconds timeout)
{
    setMode(static_cast<std::uint32_t>(COPTER_MODE_GUIDED), "GUIDED", timeout);
}

void AutopilotMavlinkAdapter::setLandMode(std::chrono::seconds timeout)
{
    setMode(static_cast<std::uint32_t>(COPTER_MODE_LAND), "LAND", timeout);
}

void AutopilotMavlinkAdapter::arm(std::chrono::seconds timeout)
{
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        sendCommandLong(MAV_CMD_COMPONENT_ARM_DISARM, 1.0f);
        const auto attempt_deadline = Clock::now() + std::chrono::seconds(3);
        while (Clock::now() < attempt_deadline) {
            poll(200);
            if (state_.armed) {
                return;
            }
        }
    }
    throw std::runtime_error("timed out waiting for armed state");
}

void AutopilotMavlinkAdapter::disarm(std::chrono::seconds timeout)
{
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        sendCommandLong(MAV_CMD_COMPONENT_ARM_DISARM, 0.0f);
        poll(200);
        if (!state_.armed) {
            return;
        }
    }
    throw std::runtime_error("timed out waiting for disarmed state");
}

void AutopilotMavlinkAdapter::takeoff(double target_altitude_m)
{
    sendCommandLong(
        MAV_CMD_NAV_TAKEOFF,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        static_cast<float>(target_altitude_m));
}

void AutopilotMavlinkAdapter::sendBodyVelocity(const BodyVelocityCommand& command)
{
    constexpr std::uint16_t type_mask =
        POSITION_TARGET_TYPEMASK_X_IGNORE |
        POSITION_TARGET_TYPEMASK_Y_IGNORE |
        POSITION_TARGET_TYPEMASK_Z_IGNORE |
        POSITION_TARGET_TYPEMASK_AX_IGNORE |
        POSITION_TARGET_TYPEMASK_AY_IGNORE |
        POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        POSITION_TARGET_TYPEMASK_YAW_IGNORE;

    const auto time_boot_ms = static_cast<std::uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            Clock::now().time_since_epoch())
            .count());

    mavlink_message_t message {};
    mavlink_msg_set_position_target_local_ned_pack(
        ids_.system_id,
        ids_.component_id,
        &message,
        time_boot_ms,
        state_.target_system,
        state_.target_component,
        MAV_FRAME_BODY_NED,
        type_mask,
        0.0f,
        0.0f,
        0.0f,
        command.vx_forward_mps,
        command.vy_right_mps,
        command.vz_down_mps,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        command.yaw_rate_rad_s);
    transport_->sendMessage(message);
}

bool AutopilotMavlinkAdapter::waitAltitudeReached(
    double target_altitude_m,
    double ratio,
    std::chrono::seconds timeout)
{
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        poll(500);
        const auto altitude = bestAltitudeM();
        if (altitude && *altitude >= target_altitude_m * ratio) {
            return true;
        }
    }
    return false;
}

bool AutopilotMavlinkAdapter::waitDisarmed(std::chrono::seconds timeout)
{
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        poll(500);
        if (state_.heartbeat_seen && !state_.armed) {
            return true;
        }
    }
    return false;
}

bool AutopilotMavlinkAdapter::poll(int timeout_ms)
{
    mavlink_message_t message {};
    if (!transport_->recvMessage(message, timeout_ms)) {
        return false;
    }
    processMessage(message);

    for (int drained = 0; drained < kMaxPollDrainMessages; ++drained) {
        mavlink_message_t pending {};
        if (!transport_->recvMessage(pending, 0)) {
            break;
        }
        processMessage(pending);
    }

    return true;
}

std::optional<double> AutopilotMavlinkAdapter::bestAltitudeM() const
{
    if (state_.local_altitude_m) {
        return state_.local_altitude_m;
    }
    if (state_.relative_altitude_m) {
        return state_.relative_altitude_m;
    }
    return state_.distance_sensor_m;
}

void AutopilotMavlinkAdapter::sendCommandLong(
    std::uint16_t command,
    float p1,
    float p2,
    float p3,
    float p4,
    float p5,
    float p6,
    float p7)
{
    mavlink_message_t message {};
    mavlink_msg_command_long_pack(
        ids_.system_id,
        ids_.component_id,
        &message,
        state_.target_system,
        state_.target_component,
        command,
        0,
        p1,
        p2,
        p3,
        p4,
        p5,
        p6,
        p7);
    transport_->sendMessage(message);
}

void AutopilotMavlinkAdapter::requestMessageInterval(std::uint32_t message_id, double rate_hz)
{
    if (rate_hz <= 0.0) {
        return;
    }
    sendCommandLong(
        MAV_CMD_SET_MESSAGE_INTERVAL,
        static_cast<float>(message_id),
        static_cast<float>(1000000.0 / rate_hz));
}

void AutopilotMavlinkAdapter::setMode(
    std::uint32_t mode,
    const std::string& name,
    std::chrono::seconds timeout)
{
    mavlink_message_t message {};
    mavlink_msg_set_mode_pack(
        ids_.system_id,
        ids_.component_id,
        &message,
        state_.target_system,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode);
    transport_->sendMessage(message);

    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        poll(500);
        if (state_.custom_mode == mode) {
            return;
        }
    }
    throw std::runtime_error("timed out waiting for mode " + name);
}

void AutopilotMavlinkAdapter::processMessage(const mavlink_message_t& message)
{
    if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t heartbeat {};
        mavlink_msg_heartbeat_decode(&message, &heartbeat);
        if (!isConfiguredTargetSystem(message, ids_) || !isAutopilotHeartbeat(heartbeat)) {
            return;
        }
        transport_->pinPeerFromLastMessage();
        state_.heartbeat_seen = true;
        state_.target_system = message.sysid;
        state_.target_component =
            message.compid == 0 ? static_cast<std::uint8_t>(MAV_COMP_ID_AUTOPILOT1) : message.compid;
        state_.custom_mode = heartbeat.custom_mode;
        state_.mode_name = copterModeName(heartbeat.custom_mode);
        state_.armed = heartbeatArmed(heartbeat);
        state_.last_heartbeat_time = Clock::now();
        return;
    }

    if (!isConfiguredTargetSystem(message, ids_)) {
        return;
    }

    if (message.msgid == MAVLINK_MSG_ID_DISTANCE_SENSOR) {
        mavlink_distance_sensor_t distance {};
        mavlink_msg_distance_sensor_decode(&message, &distance);
        if (distance.current_distance > 0) {
            state_.distance_sensor_m = distance.current_distance / 100.0;
        }
    } else if (message.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
        mavlink_local_position_ned_t position {};
        mavlink_msg_local_position_ned_decode(&message, &position);
        if (std::isfinite(position.z)) {
            state_.local_x_m = position.x;
            state_.local_y_m = position.y;
            state_.local_altitude_m = -position.z;
            state_.local_vx_mps = position.vx;
            state_.local_vy_mps = position.vy;
            state_.local_vz_mps = position.vz;
        }
    } else if (message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t position {};
        mavlink_msg_global_position_int_decode(&message, &position);
        state_.relative_altitude_m = position.relative_alt / 1000.0;
    }
}

} // namespace onboard::autopilot
