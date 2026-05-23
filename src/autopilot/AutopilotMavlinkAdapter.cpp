#include "autopilot/AutopilotMavlinkAdapter.hpp"

#include <algorithm>
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
    requestMessageInterval(MAVLINK_MSG_ID_ATTITUDE, 20.0);
#ifdef MAVLINK_MSG_ID_RANGEFINDER
    requestMessageInterval(MAVLINK_MSG_ID_RANGEFINDER, 10.0);
#endif
#ifdef MAVLINK_MSG_ID_OPTICAL_FLOW
    requestMessageInterval(MAVLINK_MSG_ID_OPTICAL_FLOW, 10.0);
#endif
#ifdef MAVLINK_MSG_ID_OPTICAL_FLOW_RAD
    requestMessageInterval(MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, 10.0);
#endif
#ifdef MAVLINK_MSG_ID_EKF_STATUS_REPORT
    requestMessageInterval(MAVLINK_MSG_ID_EKF_STATUS_REPORT, 5.0);
#endif
#ifdef MAVLINK_MSG_ID_RC_CHANNELS
    requestMessageInterval(MAVLINK_MSG_ID_RC_CHANNELS, 5.0);
#endif
}

void AutopilotMavlinkAdapter::setGuidedMode(std::chrono::seconds timeout)
{
    setMode(static_cast<std::uint32_t>(COPTER_MODE_GUIDED), "GUIDED", timeout);
}

void AutopilotMavlinkAdapter::setAltHoldMode(std::chrono::seconds timeout)
{
    setMode(static_cast<std::uint32_t>(COPTER_MODE_ALT_HOLD), "ALT_HOLD", timeout);
}

void AutopilotMavlinkAdapter::setLandMode(std::chrono::seconds timeout)
{
    setMode(static_cast<std::uint32_t>(COPTER_MODE_LAND), "LAND", timeout);
}

void AutopilotMavlinkAdapter::arm(std::chrono::seconds timeout)
{
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        requestArm();
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

void AutopilotMavlinkAdapter::requestArm()
{
    sendCommandLong(MAV_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void AutopilotMavlinkAdapter::requestDisarm()
{
    sendCommandLong(MAV_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

void AutopilotMavlinkAdapter::disarm(std::chrono::seconds timeout)
{
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        requestDisarm();
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

void AutopilotMavlinkAdapter::sendLocalNedPositionTarget(
    const LocalNedPositionTargetCommand& command)
{
    // ArduCopter Copter::Mode::Guided treats `pos_ignore` / `vel_ignore` as
    // true when ANY bit in that group is set. Mixing (e.g. X,Y position +
    // VZ velocity with VX/VY ignored) trips its "unsupported combination"
    // branch and silently invokes hold_position(), which exits the active
    // takeoff/posvel sub-mode. We must therefore emit one of the supported
    // groupings: position-only, velocity-only, or position+velocity.
    const bool use_pos = command.z_m.has_value();
    const bool use_vel = command.vz_down_mps.has_value();

    std::uint16_t type_mask =
        POSITION_TARGET_TYPEMASK_AX_IGNORE |
        POSITION_TARGET_TYPEMASK_AY_IGNORE |
        POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        POSITION_TARGET_TYPEMASK_YAW_IGNORE;
    if (!use_pos) {
        type_mask |=
            POSITION_TARGET_TYPEMASK_X_IGNORE |
            POSITION_TARGET_TYPEMASK_Y_IGNORE |
            POSITION_TARGET_TYPEMASK_Z_IGNORE;
    }
    if (!use_vel) {
        type_mask |=
            POSITION_TARGET_TYPEMASK_VX_IGNORE |
            POSITION_TARGET_TYPEMASK_VY_IGNORE |
            POSITION_TARGET_TYPEMASK_VZ_IGNORE;
    }

    const auto time_boot_ms = static_cast<std::uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            Clock::now().time_since_epoch())
            .count());

    const float pos_x = use_pos ? command.x_m : 0.0f;
    const float pos_y = use_pos ? command.y_m : 0.0f;
    const float pos_z = use_pos ? command.z_m.value() : 0.0f;
    const float vel_z = use_vel ? command.vz_down_mps.value() : 0.0f;

    mavlink_message_t message {};
    mavlink_msg_set_position_target_local_ned_pack(
        ids_.system_id,
        ids_.component_id,
        &message,
        time_boot_ms,
        state_.target_system,
        state_.target_component,
        MAV_FRAME_LOCAL_NED,
        type_mask,
        pos_x,
        pos_y,
        pos_z,
        0.0f,
        0.0f,
        vel_z,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        command.yaw_rate_rad_s);
    transport_->sendMessage(message);
}

void AutopilotMavlinkAdapter::sendRcChannelsOverride(
    const std::array<std::uint16_t, 18>& channels_pwm)
{
    mavlink_message_t message {};
    mavlink_msg_rc_channels_override_pack(
        ids_.system_id,
        ids_.component_id,
        &message,
        state_.target_system,
        state_.target_component,
        channels_pwm[0],
        channels_pwm[1],
        channels_pwm[2],
        channels_pwm[3],
        channels_pwm[4],
        channels_pwm[5],
        channels_pwm[6],
        channels_pwm[7],
        channels_pwm[8],
        channels_pwm[9],
        channels_pwm[10],
        channels_pwm[11],
        channels_pwm[12],
        channels_pwm[13],
        channels_pwm[14],
        channels_pwm[15],
        channels_pwm[16],
        channels_pwm[17]);
    transport_->sendMessage(message);
}

void AutopilotMavlinkAdapter::releaseRcChannelsOverride()
{
    std::array<std::uint16_t, 18> channels_pwm {};
    channels_pwm.fill(UINT16_MAX);
    std::fill_n(channels_pwm.begin(), 8, 0);
    sendRcChannelsOverride(channels_pwm);
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
    if (state_.distance_sensor_m) {
        return state_.distance_sensor_m;
    }
    if (state_.local_altitude_m) {
        return state_.local_altitude_m;
    }
    if (state_.relative_altitude_m) {
        return state_.relative_altitude_m;
    }
    return std::nullopt;
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
#ifdef MAVLINK_MSG_ID_RANGEFINDER
    } else if (message.msgid == MAVLINK_MSG_ID_RANGEFINDER) {
        mavlink_rangefinder_t rangefinder {};
        mavlink_msg_rangefinder_decode(&message, &rangefinder);
        if (std::isfinite(rangefinder.distance) && rangefinder.distance > 0.0f) {
            state_.distance_sensor_m = rangefinder.distance;
        }
#endif
    } else if (message.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
        mavlink_local_position_ned_t position {};
        mavlink_msg_local_position_ned_decode(&message, &position);
        if (std::isfinite(position.x) &&
            std::isfinite(position.y) &&
            std::isfinite(position.z)) {
            state_.local_x_m = position.x;
            state_.local_y_m = position.y;
            state_.local_z_m = position.z;
            state_.local_altitude_m = -position.z;
            state_.local_vx_mps = position.vx;
            state_.local_vy_mps = position.vy;
            state_.local_vz_mps = position.vz;
        }
    } else if (message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t position {};
        mavlink_msg_global_position_int_decode(&message, &position);
        state_.relative_altitude_m = position.relative_alt / 1000.0;
    } else if (message.msgid == MAVLINK_MSG_ID_SERVO_OUTPUT_RAW) {
        mavlink_servo_output_raw_t servo {};
        mavlink_msg_servo_output_raw_decode(&message, &servo);
        state_.servo_outputs_pwm = {
            servo.servo1_raw,
            servo.servo2_raw,
            servo.servo3_raw,
            servo.servo4_raw,
            servo.servo5_raw,
            servo.servo6_raw,
            servo.servo7_raw,
            servo.servo8_raw,
        };
        state_.servo_outputs_seen = true;
    } else if (message.msgid == MAVLINK_MSG_ID_ATTITUDE) {
        mavlink_attitude_t attitude {};
        mavlink_msg_attitude_decode(&message, &attitude);
        if (std::isfinite(attitude.roll) && std::isfinite(attitude.pitch) && std::isfinite(attitude.yaw)) {
            state_.attitude_roll_rad = attitude.roll;
            state_.attitude_pitch_rad = attitude.pitch;
            state_.attitude_yaw_rad = attitude.yaw;
            state_.attitude_yawspeed_rad_s = attitude.yawspeed;
            state_.last_attitude_time = Clock::now();
        }
#ifdef MAVLINK_MSG_ID_OPTICAL_FLOW
    } else if (message.msgid == MAVLINK_MSG_ID_OPTICAL_FLOW) {
        mavlink_optical_flow_t flow {};
        mavlink_msg_optical_flow_decode(&message, &flow);
        state_.optical_flow_quality = flow.quality;
        const float ground_distance = flow.ground_distance;
        if (std::isfinite(ground_distance) && ground_distance > 0.0f) {
            state_.optical_flow_ground_distance_m = ground_distance;
        }
        state_.last_optical_flow_time = Clock::now();
#endif
#ifdef MAVLINK_MSG_ID_OPTICAL_FLOW_RAD
    } else if (message.msgid == MAVLINK_MSG_ID_OPTICAL_FLOW_RAD) {
        mavlink_optical_flow_rad_t flow {};
        mavlink_msg_optical_flow_rad_decode(&message, &flow);
        state_.optical_flow_quality = flow.quality;
        const float distance = flow.distance;
        if (std::isfinite(distance) && distance > 0.0f) {
            state_.optical_flow_ground_distance_m = distance;
        }
        state_.last_optical_flow_time = Clock::now();
#endif
#ifdef MAVLINK_MSG_ID_EKF_STATUS_REPORT
    } else if (message.msgid == MAVLINK_MSG_ID_EKF_STATUS_REPORT) {
        mavlink_ekf_status_report_t ekf {};
        mavlink_msg_ekf_status_report_decode(&message, &ekf);
        const std::uint16_t flags = ekf.flags;
        state_.ekf_flags = flags;
        state_.last_ekf_status_time = Clock::now();
#endif
#ifdef MAVLINK_MSG_ID_RC_CHANNELS
    } else if (message.msgid == MAVLINK_MSG_ID_RC_CHANNELS) {
        mavlink_rc_channels_t channels {};
        mavlink_msg_rc_channels_decode(&message, &channels);
        state_.rc_channel_count = channels.chancount;
        state_.rc_rssi = channels.rssi;
        state_.rc_channels_pwm = {
            channels.chan1_raw,
            channels.chan2_raw,
            channels.chan3_raw,
            channels.chan4_raw,
            channels.chan5_raw,
            channels.chan6_raw,
            channels.chan7_raw,
            channels.chan8_raw,
            channels.chan9_raw,
            channels.chan10_raw,
            channels.chan11_raw,
            channels.chan12_raw,
            channels.chan13_raw,
            channels.chan14_raw,
            channels.chan15_raw,
            channels.chan16_raw,
            channels.chan17_raw,
            channels.chan18_raw,
        };
        state_.last_rc_channels_time = Clock::now();
#endif
    }
}

} // namespace onboard::autopilot
