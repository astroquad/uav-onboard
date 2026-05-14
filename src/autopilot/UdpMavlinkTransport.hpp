#pragma once

#include "autopilot/MavlinkTransport.hpp"

#include <cstdint>
#include <string>

#include <netinet/in.h>

namespace onboard::autopilot {

class UdpMavlinkTransport final : public MavlinkTransport {
public:
    UdpMavlinkTransport(std::uint16_t listen_port, std::uint8_t parse_channel, std::string name);
    ~UdpMavlinkTransport() override;

    UdpMavlinkTransport(const UdpMavlinkTransport&) = delete;
    UdpMavlinkTransport& operator=(const UdpMavlinkTransport&) = delete;

    bool recvMessage(mavlink_message_t& message, int timeout_ms) override;
    void sendMessage(const mavlink_message_t& message) override;
    const std::string& name() const override { return name_; }

private:
    int socket_fd_ = -1;
    bool have_peer_ = false;
    sockaddr_in peer_addr_ {};
    std::uint8_t parse_channel_ = 0;
    std::string name_;
};

} // namespace onboard::autopilot
