#pragma once

#include "autopilot/MavlinkTransport.hpp"

#include <cstdint>
#include <deque>
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
    void pinPeerFromLastMessage() override;

private:
    struct PendingMessage {
        mavlink_message_t message {};
        sockaddr_in source {};
    };

    int socket_fd_ = -1;
    bool have_peer_ = false;
    sockaddr_in peer_addr_ {};
    bool have_last_message_source_ = false;
    sockaddr_in last_message_source_ {};
    std::uint8_t parse_channel_ = 0;
    mavlink_status_t parse_status_ {};
    std::deque<PendingMessage> pending_messages_;
    std::string name_;
};

} // namespace onboard::autopilot
