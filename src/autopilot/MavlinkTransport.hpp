#pragma once

#include <string>

extern "C" {
#include <mavlink/v2.0/ardupilotmega/mavlink.h>
}

namespace onboard::autopilot {

class MavlinkTransport {
public:
    virtual ~MavlinkTransport() = default;

    virtual bool recvMessage(mavlink_message_t& message, int timeout_ms) = 0;
    virtual void sendMessage(const mavlink_message_t& message) = 0;
    virtual const std::string& name() const = 0;
    virtual void pinPeerFromLastMessage() {}
};

} // namespace onboard::autopilot
