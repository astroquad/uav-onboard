#include "autopilot/AutopilotMavlinkAdapter.hpp"
#include "autopilot/MavlinkTransport.hpp"

#include <cassert>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
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

    adapter.sendLocalNedPositionTarget(onboard::autopilot::LocalNedPositionTargetCommand {
        1.25f,
        -0.50f,
        std::nullopt,
        std::optional<float> {-0.20f},
        0.10f,
    });

    assert(capture->sent_messages.size() == 1);
    mavlink_set_position_target_local_ned_t xy_hold {};
    mavlink_msg_set_position_target_local_ned_decode(
        &capture->sent_messages.back(),
        &xy_hold);
    assert(xy_hold.target_system == ids.target_system);
    assert(xy_hold.target_component == ids.target_component);
    assert(xy_hold.coordinate_frame == MAV_FRAME_LOCAL_NED);
    assert((xy_hold.type_mask & POSITION_TARGET_TYPEMASK_X_IGNORE) == 0);
    assert((xy_hold.type_mask & POSITION_TARGET_TYPEMASK_Y_IGNORE) == 0);
    assert((xy_hold.type_mask & POSITION_TARGET_TYPEMASK_Z_IGNORE) != 0);
    assert((xy_hold.type_mask & POSITION_TARGET_TYPEMASK_VX_IGNORE) != 0);
    assert((xy_hold.type_mask & POSITION_TARGET_TYPEMASK_VY_IGNORE) != 0);
    assert((xy_hold.type_mask & POSITION_TARGET_TYPEMASK_VZ_IGNORE) == 0);
    requireNear(xy_hold.x, 1.25f);
    requireNear(xy_hold.y, -0.50f);
    requireNear(xy_hold.vz, -0.20f);
    requireNear(xy_hold.yaw_rate, 0.10f);

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
    assert((xyz_hold.type_mask & POSITION_TARGET_TYPEMASK_VZ_IGNORE) != 0);
    requireNear(xyz_hold.x, 2.00f);
    requireNear(xyz_hold.y, 3.00f);
    requireNear(xyz_hold.z, -1.50f);

    return 0;
}
