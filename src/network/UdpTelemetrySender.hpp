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
    // Sends one telemetry JSON message. Payloads above the MTU-safe chunk
    // size are transparently split into AQT1 datagrams (see TelemetryChunker).
    bool send(const std::string& payload);
    std::string lastError() const;

private:
    bool sendDatagram(const std::string& datagram);

    std::uintptr_t socket_ = 0;
    bool socket_open_ = false;
    std::string ip_;
    std::uint16_t port_ = 0;
    std::uint32_t message_counter_ = 0;
    mutable std::string last_error_;
};

} // namespace onboard::network
