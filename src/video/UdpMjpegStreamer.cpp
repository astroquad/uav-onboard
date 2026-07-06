#include "video/UdpMjpegStreamer.hpp"

#include "video/VideoPacket.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
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

    // Debug video is best-effort: when the link backs up (e.g. a stalled
    // VPN tunnel fills the send queue) a blocking sendto would wedge the
    // sender worker indefinitely and the GCS would stop receiving even
    // after the path recovers. Drop chunks instead of blocking.
#ifdef _WIN32
    u_long non_blocking = 1;
    if (ioctlsocket(static_cast<SOCKET>(socket_), FIONBIO, &non_blocking) != 0) {
        last_error_ = socketErrorString();
        closeSocket(socket_);
        socket_ = 0;
        return false;
    }
#else
    const int flags = fcntl(static_cast<int>(socket_), F_GETFL, 0);
    if (flags < 0 ||
        fcntl(static_cast<int>(socket_), F_SETFL, flags | O_NONBLOCK) < 0) {
        last_error_ = socketErrorString();
        closeSocket(socket_);
        socket_ = 0;
        return false;
    }
#endif

    socket_open_ = true;
    return true;
}

void UdpMjpegStreamer::setChunkPacingUs(int pacing_us)
{
    chunk_pacing_us_ = std::max(0, pacing_us);
}

void UdpMjpegStreamer::setFrameSpreadUs(int spread_us)
{
    frame_spread_us_ = std::max(0, spread_us);
}

void UdpMjpegStreamer::setFecGroupSize(int group_size)
{
    // The group size travels in the flags high byte; clamp accordingly.
    fec_group_size_ = std::clamp(group_size, 0, 255);
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
    last_chunk_count_ = static_cast<int>(chunk_count);

    const int parity_count = fec_group_size_ > 0
        ? (static_cast<int>(chunk_count) + fec_group_size_ - 1) / fec_group_size_
        : 0;
    const int total_packets = static_cast<int>(chunk_count) + parity_count;

    const auto send_packet = [&](const std::uint8_t* data, int size) -> bool {
        const auto sent = sendto(
#ifdef _WIN32
            static_cast<SOCKET>(socket_),
#else
            static_cast<int>(socket_),
#endif
            reinterpret_cast<const char*>(data),
            size,
            0,
            reinterpret_cast<sockaddr*>(&address),
            sizeof(address));
        if (sent < 0 || sent != size) {
#ifdef _WIN32
            if (WSAGetLastError() == WSAEWOULDBLOCK) {
                last_error_ = "video frame dropped (send buffer full)";
                return false;
            }
#else
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                last_error_ = "video frame dropped (send buffer full)";
                return false;
            }
#endif
            last_error_ = socketErrorString();
            return false;
        }
        return true;
    };
    const auto pace = [&]() {
        const int spread_gap_us = frame_spread_us_ > 0
            ? frame_spread_us_ / std::max(1, total_packets)
            : 0;
        const int gap_us = std::max(chunk_pacing_us_, spread_gap_us);
        if (gap_us > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(gap_us));
        }
    };

    std::vector<std::uint8_t> packet(kVideoHeaderSize + kVideoFecPayloadSize);
    std::vector<std::uint8_t> parity(kVideoMaxPayloadSize, 0);
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

        if (!send_packet(packet.data(), static_cast<int>(kVideoHeaderSize + payload_size))) {
            return false;
        }

        if (fec_group_size_ > 0) {
            if (chunk_index % fec_group_size_ == 0) {
                std::fill(parity.begin(), parity.end(), 0);
            }
            for (std::size_t byte = 0; byte < payload_size; ++byte) {
                parity[byte] ^= frame.jpeg_data[offset + byte];
            }
            const bool group_end =
                (chunk_index % fec_group_size_) == fec_group_size_ - 1 ||
                chunk_index + 1 == chunk_count;
            if (group_end) {
                pace();
                VideoPacketHeader parity_header;
                parity_header.flags = static_cast<std::uint16_t>(
                    kVideoFlagFecParity |
                    (static_cast<std::uint16_t>(fec_group_size_) << 8));
                parity_header.frame_id = frame.frame_id;
                parity_header.chunk_index =
                    static_cast<std::uint16_t>(chunk_index / fec_group_size_);
                parity_header.chunk_count = chunk_count;
                parity_header.payload_size =
                    static_cast<std::uint32_t>(kVideoFecPayloadSize);
                parity_header.timestamp_ms =
                    static_cast<std::uint64_t>(frame.timestamp_ms);
                const auto parity_serialized = serializeHeader(parity_header);
                std::copy(parity_serialized.begin(), parity_serialized.end(),
                          packet.begin());
                const auto total_bytes =
                    static_cast<std::uint32_t>(frame.jpeg_data.size());
                packet[kVideoHeaderSize + 0] =
                    static_cast<std::uint8_t>((total_bytes >> 24) & 0xff);
                packet[kVideoHeaderSize + 1] =
                    static_cast<std::uint8_t>((total_bytes >> 16) & 0xff);
                packet[kVideoHeaderSize + 2] =
                    static_cast<std::uint8_t>((total_bytes >> 8) & 0xff);
                packet[kVideoHeaderSize + 3] =
                    static_cast<std::uint8_t>(total_bytes & 0xff);
                std::copy(parity.begin(), parity.end(),
                          packet.begin() +
                              static_cast<std::ptrdiff_t>(kVideoHeaderSize + 4));
                if (!send_packet(packet.data(),
                                 static_cast<int>(kVideoHeaderSize +
                                                  kVideoFecPayloadSize))) {
                    return false;
                }
            }
        }

        if (chunk_index + 1 < chunk_count) {
            pace();
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

int UdpMjpegStreamer::lastChunkCount() const
{
    return last_chunk_count_;
}

} // namespace onboard::video
