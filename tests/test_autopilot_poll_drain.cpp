#include "autopilot/AutopilotMavlinkAdapter.hpp"
#include "autopilot/MavlinkTransport.hpp"

#include <cassert>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

class FakeTransport final : public onboard::autopilot::MavlinkTransport {
public:
    explicit FakeTransport(std::vector<mavlink_message_t> messages)
        : messages_(std::move(messages))
    {
    }

    bool recvMessage(mavlink_message_t& message, int) override
    {
        if (next_ >= messages_.size()) {
            return false;
        }
        message = messages_[next_++];
        return true;
    }

    void sendMessage(const mavlink_message_t&) override {}
    const std::string& name() const override { return name_; }

private:
    std::vector<mavlink_message_t> messages_;
    std::size_t next_ = 0;
    std::string name_ = "fake";
};

mavlink_message_t packLocalPosition(float z_down_m)
{
    mavlink_message_t message {};
    mavlink_msg_local_position_ned_pack(
        1,
        MAV_COMP_ID_AUTOPILOT1,
        &message,
        100,
        0.0f,
        0.0f,
        z_down_m,
        0.0f,
        0.0f,
        0.0f);
    return message;
}

mavlink_message_t packHeartbeat(std::uint32_t custom_mode, bool armed)
{
    mavlink_message_t message {};
    mavlink_msg_heartbeat_pack(
        1,
        MAV_COMP_ID_AUTOPILOT1,
        &message,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0,
        custom_mode,
        MAV_STATE_ACTIVE);
    return message;
}

mavlink_message_t packRcChannels()
{
    mavlink_rc_channels_t channels {};
    channels.time_boot_ms = 100;
    channels.chancount = 8;
    channels.chan1_raw = 1500;
    channels.chan2_raw = 1501;
    channels.chan3_raw = 1000;
    channels.chan4_raw = 1502;
    channels.chan5_raw = 1800;
    channels.rssi = 200;

    mavlink_message_t message {};
    mavlink_msg_rc_channels_encode(
        1,
        MAV_COMP_ID_AUTOPILOT1,
        &message,
        &channels);
    return message;
}

} // namespace

int main()
{
    std::vector<mavlink_message_t> messages;
    messages.push_back(packLocalPosition(-1.25f));
    messages.push_back(packRcChannels());
    messages.push_back(packHeartbeat(COPTER_MODE_GUIDED, true));

    auto transport = std::make_unique<FakeTransport>(std::move(messages));
    onboard::autopilot::AutopilotMavlinkAdapter adapter(
        std::move(transport),
        onboard::autopilot::MavlinkIds {});

    assert(adapter.poll(10));
    assert(adapter.state().heartbeat_seen);
    assert(adapter.state().armed);
    assert(adapter.state().custom_mode == COPTER_MODE_GUIDED);
    assert(adapter.state().local_altitude_m.has_value());
    assert(*adapter.state().local_altitude_m > 1.24);
    assert(adapter.state().last_heartbeat_time.time_since_epoch().count() > 0);
    assert(adapter.state().rc_channel_count.has_value());
    assert(*adapter.state().rc_channel_count == 8);
    assert(adapter.state().rc_rssi.has_value());
    assert(*adapter.state().rc_rssi == 200);
    assert(adapter.state().rc_channels_pwm[0] == 1500);
    assert(adapter.state().last_rc_channels_time.time_since_epoch().count() > 0);

    return 0;
}
