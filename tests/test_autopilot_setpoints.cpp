// Tests are assert-based: keep assert() active even in Release
// builds (CMake adds -DNDEBUG there, which silently no-ops all checks).
#undef NDEBUG

#include "autopilot/AutopilotMavlinkAdapter.hpp"
#include "autopilot/MavlinkTransport.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <cstdint>
#include <vector>

namespace {

class CapturingTransport final : public onboard::autopilot::MavlinkTransport {
public:
    bool recvMessage(mavlink_message_t&, int) override { return false; }

    void sendMessage(const mavlink_message_t& message) override
    {
        sent_messages.push_back(message);
    }

    const std::string& name() const override { return name_; }

    std::vector<mavlink_message_t> sent_messages;

private:
    std::string name_ = "capture";
};

void requireNear(float actual, float expected)
{
    assert(std::fabs(actual - expected) < 0.0001f);
}

std::uint16_t commandId(const mavlink_message_t& message)
{
    mavlink_command_long_t command {};
    mavlink_msg_command_long_decode(&message, &command);
    return command.command;
}

} // namespace

int main()
{
    onboard::autopilot::MavlinkIds ids;
    ids.system_id = 191;
    ids.component_id = 192;
    ids.target_system = 1;
    ids.target_component = MAV_COMP_ID_AUTOPILOT1;

    auto transport = std::make_unique<CapturingTransport>();
    auto* capture = transport.get();
    onboard::autopilot::AutopilotMavlinkAdapter adapter(std::move(transport), ids);

    // Velocity-only mode: z absent + vz present. ArduCopter requires ALL
    // velocity bits valid for set_vel_accel_NED_m, so the adapter must ignore
    // every position bit and emit VX, VY, VZ.
    adapter.sendLocalNedPositionTarget(onboard::autopilot::LocalNedPositionTargetCommand {
        1.25f,
        -0.50f,
        std::nullopt,
        std::optional<float> {-0.20f},
        0.10f,
    });

    assert(capture->sent_messages.size() == 1);
    mavlink_set_position_target_local_ned_t vel_only {};
    mavlink_msg_set_position_target_local_ned_decode(
        &capture->sent_messages.back(),
        &vel_only);
    assert(vel_only.target_system == ids.target_system);
    assert(vel_only.target_component == ids.target_component);
    assert(vel_only.coordinate_frame == MAV_FRAME_LOCAL_NED);
    assert((vel_only.type_mask & POSITION_TARGET_TYPEMASK_X_IGNORE) != 0);
    assert((vel_only.type_mask & POSITION_TARGET_TYPEMASK_Y_IGNORE) != 0);
    assert((vel_only.type_mask & POSITION_TARGET_TYPEMASK_Z_IGNORE) != 0);
    assert((vel_only.type_mask & POSITION_TARGET_TYPEMASK_VX_IGNORE) == 0);
    assert((vel_only.type_mask & POSITION_TARGET_TYPEMASK_VY_IGNORE) == 0);
    assert((vel_only.type_mask & POSITION_TARGET_TYPEMASK_VZ_IGNORE) == 0);
    requireNear(vel_only.vx, 0.0f);
    requireNear(vel_only.vy, 0.0f);
    requireNear(vel_only.vz, -0.20f);
    requireNear(vel_only.yaw_rate, 0.10f);

    // Position-only mode: x, y, z set; vz absent. All velocity bits must be
    // ignored so ArduCopter dispatches to set_pos_NED_m.
    adapter.sendLocalNedPositionTarget(onboard::autopilot::LocalNedPositionTargetCommand {
        2.00f,
        3.00f,
        std::optional<float> {-1.50f},
        std::nullopt,
        0.0f,
    });

    assert(capture->sent_messages.size() == 2);
    mavlink_set_position_target_local_ned_t xyz_hold {};
    mavlink_msg_set_position_target_local_ned_decode(
        &capture->sent_messages.back(),
        &xyz_hold);
    assert((xyz_hold.type_mask & POSITION_TARGET_TYPEMASK_X_IGNORE) == 0);
    assert((xyz_hold.type_mask & POSITION_TARGET_TYPEMASK_Y_IGNORE) == 0);
    assert((xyz_hold.type_mask & POSITION_TARGET_TYPEMASK_Z_IGNORE) == 0);
    assert((xyz_hold.type_mask & POSITION_TARGET_TYPEMASK_VX_IGNORE) != 0);
    assert((xyz_hold.type_mask & POSITION_TARGET_TYPEMASK_VY_IGNORE) != 0);
    assert((xyz_hold.type_mask & POSITION_TARGET_TYPEMASK_VZ_IGNORE) != 0);
    requireNear(xyz_hold.x, 2.00f);
    requireNear(xyz_hold.y, 3.00f);
    requireNear(xyz_hold.z, -1.50f);

    adapter.requestDisarm();
    assert(capture->sent_messages.size() == 3);
    assert(capture->sent_messages.back().msgid == MAVLINK_MSG_ID_COMMAND_LONG);
    assert(commandId(capture->sent_messages.back()) == MAV_CMD_COMPONENT_ARM_DISARM);
    mavlink_command_long_t disarm {};
    mavlink_msg_command_long_decode(&capture->sent_messages.back(), &disarm);
    requireNear(disarm.param1, 0.0f);

    std::array<std::uint16_t, 18> rc_override {};
    rc_override.fill(UINT16_MAX);
    rc_override[0] = 1580;
    rc_override[1] = 1420;
    rc_override[2] = 1500;
    rc_override[3] = 1525;
    adapter.sendRcChannelsOverride(rc_override);
    assert(capture->sent_messages.size() == 4);
    assert(capture->sent_messages.back().msgid == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE);
    mavlink_rc_channels_override_t rc {};
    mavlink_msg_rc_channels_override_decode(&capture->sent_messages.back(), &rc);
    assert(rc.target_system == ids.target_system);
    assert(rc.target_component == ids.target_component);
    assert(rc.chan1_raw == 1580);
    assert(rc.chan2_raw == 1420);
    assert(rc.chan3_raw == 1500);
    assert(rc.chan4_raw == 1525);
    assert(rc.chan5_raw == UINT16_MAX);

    adapter.releaseRcChannelsOverride();
    assert(capture->sent_messages.size() == 5);
    mavlink_rc_channels_override_t release {};
    mavlink_msg_rc_channels_override_decode(&capture->sent_messages.back(), &release);
    assert(release.chan1_raw == 0);
    assert(release.chan2_raw == 0);
    assert(release.chan3_raw == 0);
    assert(release.chan4_raw == 0);
    assert(release.chan8_raw == 0);
    assert(release.chan9_raw == UINT16_MAX);

    return 0;
}
