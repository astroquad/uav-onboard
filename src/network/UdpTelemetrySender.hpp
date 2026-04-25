#pragma once

#include <cstdint>
#include <string>

namespace onboard::network {

class UdpTelemetrySender {
public:
    UdpTelemetrySender();
    ~UdpTelemetrySender();

    UdpTelemetrySender(const UdpTelemetrySender&) = delete;
    UdpTelemetrySender& operator=(const UdpTelemetrySender&) = delete;

    bool open(const std::string& ip, std::uint16_t port);
    bool send(const std::string& payload) const;
    std::string lastError() const;

private:
    std::uintptr_t socket_ = 0;
    bool socket_open_ = false;
    std::string ip_;
    std::uint16_t port_ = 0;
    mutable std::string last_error_;
};

} // namespace onboard::network
