#include "autopilot/UdpMavlinkTransport.hpp"

#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace onboard::autopilot {

UdpMavlinkTransport::UdpMavlinkTransport(
    std::uint16_t listen_port,
    std::uint8_t parse_channel,
    std::string name)
    : parse_channel_(parse_channel)
    , name_(std::move(name))
{
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        throw std::runtime_error("socket() failed");
    }

    int reuse = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in local {};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(listen_port);

    if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
        const std::string error = std::strerror(errno);
        close(socket_fd_);
        socket_fd_ = -1;
        throw std::runtime_error("bind() failed on UDP port " +
            std::to_string(listen_port) + ": " + error);
    }
}

UdpMavlinkTransport::~UdpMavlinkTransport()
{
    if (socket_fd_ >= 0) {
        close(socket_fd_);
    }
}

bool UdpMavlinkTransport::recvMessage(mavlink_message_t& message, int timeout_ms)
{
    if (!pending_messages_.empty()) {
        const auto pending = pending_messages_.front();
        pending_messages_.pop_front();
        message = pending.message;
        last_message_source_ = pending.source;
        have_last_message_source_ = true;
        return true;
    }

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(socket_fd_, &readfds);

    timeval timeout {};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    const int ready = select(socket_fd_ + 1, &readfds, nullptr, nullptr, &timeout);
    if (ready <= 0) {
        return false;
    }

    std::uint8_t buffer[2048];
    sockaddr_in source {};
    socklen_t source_len = sizeof(source);
    const ssize_t received = recvfrom(
        socket_fd_,
        buffer,
        sizeof(buffer),
        0,
        reinterpret_cast<sockaddr*>(&source),
        &source_len);
    if (received <= 0) {
        return false;
    }

    for (ssize_t i = 0; i < received; ++i) {
        mavlink_message_t parsed {};
        if (mavlink_parse_char(parse_channel_, buffer[i], &parsed, &parse_status_)) {
            pending_messages_.push_back(PendingMessage { parsed, source });
        }
    }

    if (pending_messages_.empty()) {
        return false;
    }
    const auto pending = pending_messages_.front();
    pending_messages_.pop_front();
    message = pending.message;
    last_message_source_ = pending.source;
    have_last_message_source_ = true;
    return true;
}

void UdpMavlinkTransport::sendMessage(const mavlink_message_t& message)
{
    if (!have_peer_) {
        throw std::runtime_error("no MAVLink autopilot peer pinned yet; wait for heartbeat first");
    }

    std::uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const std::uint16_t length = mavlink_msg_to_send_buffer(buffer, &message);
    const ssize_t sent = sendto(
        socket_fd_,
        buffer,
        length,
        0,
        reinterpret_cast<const sockaddr*>(&peer_addr_),
        sizeof(peer_addr_));
    if (sent != static_cast<ssize_t>(length)) {
        throw std::runtime_error("sendto() failed");
    }
}

void UdpMavlinkTransport::pinPeerFromLastMessage()
{
    if (!have_last_message_source_) {
        return;
    }
    peer_addr_ = last_message_source_;
    have_peer_ = true;
}

} // namespace onboard::autopilot
