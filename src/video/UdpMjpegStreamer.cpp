#include "video/UdpMjpegStreamer.hpp"

#include "video/VideoPacket.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <vector>

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

namespace onboard::video {
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

UdpMjpegStreamer::UdpMjpegStreamer()
{
#ifdef _WIN32
    WSADATA data {};
    WSAStartup(MAKEWORD(2, 2), &data);
#endif
}

UdpMjpegStreamer::~UdpMjpegStreamer()
{
    close();
#ifdef _WIN32
    WSACleanup();
#endif
}

bool UdpMjpegStreamer::open(const std::string& ip, std::uint16_t port)
{
    if (ip.empty()) {
        last_error_ = "no video destination IP configured";
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

bool UdpMjpegStreamer::sendFrame(const camera::CameraFrame& frame)
{
    if (!socket_open_) {
        last_error_ = "socket is not open";
        return false;
    }
    if (frame.jpeg_data.empty()) {
        last_error_ = "cannot send an empty JPEG frame";
        return false;
    }

    sockaddr_in address {};
    address.sin_family = AF_INET;
    address.sin_port = htons(port_);
    if (inet_pton(AF_INET, ip_.c_str(), &address.sin_addr) != 1) {
        last_error_ = "invalid destination IP: " + ip_;
        return false;
    }

    const auto chunk_count = static_cast<std::uint16_t>(
        (frame.jpeg_data.size() + kVideoMaxPayloadSize - 1) / kVideoMaxPayloadSize);
    if (chunk_count == 0) {
        last_error_ = "invalid chunk count";
        return false;
    }

    std::vector<std::uint8_t> packet(kVideoHeaderSize + kVideoMaxPayloadSize);
    for (std::uint16_t chunk_index = 0; chunk_index < chunk_count; ++chunk_index) {
        const std::size_t offset = static_cast<std::size_t>(chunk_index) * kVideoMaxPayloadSize;
        const std::size_t payload_size =
            std::min(kVideoMaxPayloadSize, frame.jpeg_data.size() - offset);

        VideoPacketHeader header;
        header.frame_id = frame.frame_id;
        header.chunk_index = chunk_index;
        header.chunk_count = chunk_count;
        header.payload_size = static_cast<std::uint32_t>(payload_size);
        header.timestamp_ms = static_cast<std::uint64_t>(frame.timestamp_ms);

        const auto serialized_header = serializeHeader(header);
        std::copy(serialized_header.begin(), serialized_header.end(), packet.begin());
        std::copy(
            frame.jpeg_data.begin() + static_cast<std::ptrdiff_t>(offset),
            frame.jpeg_data.begin() + static_cast<std::ptrdiff_t>(offset + payload_size),
            packet.begin() + static_cast<std::ptrdiff_t>(kVideoHeaderSize));

        const auto packet_size = static_cast<int>(kVideoHeaderSize + payload_size);
        const auto sent = sendto(
#ifdef _WIN32
            static_cast<SOCKET>(socket_),
#else
            static_cast<int>(socket_),
#endif
            reinterpret_cast<const char*>(packet.data()),
            packet_size,
            0,
            reinterpret_cast<sockaddr*>(&address),
            sizeof(address));
        if (sent < 0 || sent != packet_size) {
            last_error_ = socketErrorString();
            return false;
        }
    }

    last_error_.clear();
    return true;
}

void UdpMjpegStreamer::close()
{
    if (socket_open_) {
        closeSocket(socket_);
        socket_open_ = false;
    }
}

std::string UdpMjpegStreamer::lastError() const
{
    return last_error_;
}

} // namespace onboard::video
