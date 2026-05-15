#pragma once

#include "autopilot/MavlinkTransport.hpp"

#include <cstdint>
#include <deque>
#include <string>

#include <termios.h>

namespace onboard::autopilot {

class SerialMavlinkTransport final : public MavlinkTransport {
public:
    SerialMavlinkTransport(
        std::string device,
        int baudrate,
        std::uint8_t parse_channel,
        std::string name);
    ~SerialMavlinkTransport() override;

    SerialMavlinkTransport(const SerialMavlinkTransport&) = delete;
    SerialMavlinkTransport& operator=(const SerialMavlinkTransport&) = delete;

    bool recvMessage(mavlink_message_t& message, int timeout_ms) override;
    void sendMessage(const mavlink_message_t& message) override;
    const std::string& name() const override { return name_; }

private:
    void configurePort(int baudrate);

    int fd_ = -1;
    bool have_original_termios_ = false;
    termios original_termios_ {};
    std::uint8_t parse_channel_ = 0;
    mavlink_status_t parse_status_ {};
    std::deque<mavlink_message_t> pending_messages_;
    std::string device_;
    std::string name_;
};

} // namespace onboard::autopilot
