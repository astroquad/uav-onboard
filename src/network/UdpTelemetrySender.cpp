#include "network/UdpTelemetrySender.hpp"

#include <cerrno>
#include <cstring>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace onboard::network {
namespace {

#ifdef _WIN32
std::string socketErrorString()
{
    return "winsock error " + std::to_string(WSAGetLastError());
}

void closeSocket(std::uintptr_t socket)
{
    closesocket(static_cast<SOCKET>(socket));
}
#else
std::string socketErrorString()
{
    return std::strerror(errno);
}

void closeSocket(std::uintptr_t socket)
{
    close(static_cast<int>(socket));
}
#endif

} // namespace

UdpTelemetrySender::UdpTelemetrySender()
{
#ifdef _WIN32
    WSADATA data {};
    WSAStartup(MAKEWORD(2, 2), &data);
#endif
}

UdpTelemetrySender::~UdpTelemetrySender()
{
    if (socket_open_) {
        closeSocket(socket_);
    }
#ifdef _WIN32
    WSACleanup();
#endif
}

bool UdpTelemetrySender::open(const std::string& ip, std::uint16_t port)
{
    if (ip.empty()) {
        last_error_ = "no telemetry destination IP configured";
        return false;
    }

    ip_ = ip;
    port_ = port;
    const auto raw_socket = socket(AF_INET, SOCK_DGRAM, 0);
#ifdef _WIN32
    if (raw_socket == INVALID_SOCKET) {
#else
    if (raw_socket < 0) {
#endif
        last_error_ = socketErrorString();
        return false;
    }
    socket_ = static_cast<std::uintptr_t>(raw_socket);

    const int enable_broadcast = 1;
    if (setsockopt(
#ifdef _WIN32
            static_cast<SOCKET>(socket_),
            SOL_SOCKET,
            SO_BROADCAST,
            reinterpret_cast<const char*>(&enable_broadcast),
#else
            static_cast<int>(socket_),
            SOL_SOCKET,
            SO_BROADCAST,
            &enable_broadcast,
#endif
            sizeof(enable_broadcast)) < 0) {
        last_error_ = socketErrorString();
        closeSocket(socket_);
        socket_ = 0;
        return false;
    }

    socket_open_ = true;
    return true;
}

bool UdpTelemetrySender::send(const std::string& payload) const
{
    if (!socket_open_) {
        last_error_ = "socket is not open";
        return false;
    }

    sockaddr_in address {};
    address.sin_family = AF_INET;
    address.sin_port = htons(port_);
    if (inet_pton(AF_INET, ip_.c_str(), &address.sin_addr) != 1) {
        last_error_ = "invalid destination IP: " + ip_;
        return false;
    }

    const auto sent = sendto(
#ifdef _WIN32
        static_cast<SOCKET>(socket_),
#else
        static_cast<int>(socket_),
#endif
        payload.data(),
        static_cast<int>(payload.size()),
        0,
        reinterpret_cast<sockaddr*>(&address),
        sizeof(address));
    if (sent < 0 || static_cast<std::size_t>(sent) != payload.size()) {
        last_error_ = socketErrorString();
        return false;
    }
    last_error_.clear();
    return true;
}

std::string UdpTelemetrySender::lastError() const
{
    return last_error_;
}

} // namespace onboard::network
