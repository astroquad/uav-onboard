// Tests are assert-based: keep assert() active even in Release
// builds (CMake adds -DNDEBUG there, which silently no-ops all checks).
#undef NDEBUG

#include "autopilot/UdpMavlinkTransport.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cassert>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace {

std::uint16_t findAvailableUdpPort()
{
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        throw std::runtime_error("socket() failed while finding test UDP port");
    }

    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(fd);
        throw std::runtime_error("bind() failed while finding test UDP port");
    }

    socklen_t len = sizeof(addr);
    if (getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) < 0) {
        close(fd);
        throw std::runtime_error("getsockname() failed while finding test UDP port");
    }
    const auto port = static_cast<std::uint16_t>(ntohs(addr.sin_port));
    close(fd);
    return port;
}

mavlink_message_t packLocalPosition()
{
    mavlink_message_t message {};
    mavlink_msg_local_position_ned_pack(
        1,
        MAV_COMP_ID_AUTOPILOT1,
        &message,
        100,
        0.0f,
        0.0f,
        -1.0f,
        0.0f,
        0.0f,
        0.0f);
    return message;
}

mavlink_message_t packHeartbeat()
{
    mavlink_message_t message {};
    mavlink_msg_heartbeat_pack(
        1,
        MAV_COMP_ID_AUTOPILOT1,
        &message,
        MAV_TYPE_QUADROTOR,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        MAV_MODE_FLAG_SAFETY_ARMED,
        COPTER_MODE_GUIDED,
        MAV_STATE_ACTIVE);
    return message;
}

mavlink_message_t packGcsHeartbeat()
{
    mavlink_message_t message {};
    mavlink_msg_heartbeat_pack(
        255,
        MAV_COMP_ID_MISSIONPLANNER,
        &message,
        MAV_TYPE_GCS,
        MAV_AUTOPILOT_INVALID,
        0,
        0,
        MAV_STATE_ACTIVE);
    return message;
}

void appendPacket(std::vector<std::uint8_t>& bytes, const mavlink_message_t& message)
{
    std::uint8_t packet[MAVLINK_MAX_PACKET_LEN];
    const auto length = mavlink_msg_to_send_buffer(packet, &message);
    bytes.insert(bytes.end(), packet, packet + length);
}

void sendDatagram(std::uint16_t port, const std::vector<std::uint8_t>& bytes)
{
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        throw std::runtime_error("sender socket() failed");
    }

    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);
    const ssize_t sent = sendto(
        fd,
        bytes.data(),
        bytes.size(),
        0,
        reinterpret_cast<sockaddr*>(&addr),
        sizeof(addr));
    close(fd);
    if (sent != static_cast<ssize_t>(bytes.size())) {
        throw std::runtime_error("sendto() failed");
    }
}

int openLoopbackUdpSocket()
{
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        throw std::runtime_error("socket() failed");
    }

    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(fd);
        throw std::runtime_error("bind() failed");
    }
    return fd;
}

void sendDatagramFrom(int fd, std::uint16_t port, const std::vector<std::uint8_t>& bytes)
{
    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);
    const ssize_t sent = sendto(
        fd,
        bytes.data(),
        bytes.size(),
        0,
        reinterpret_cast<sockaddr*>(&addr),
        sizeof(addr));
    if (sent != static_cast<ssize_t>(bytes.size())) {
        throw std::runtime_error("sendto() failed");
    }
}

bool receiveAnyDatagram(int fd, int timeout_ms)
{
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);

    timeval timeout {};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    const int ready = select(fd + 1, &readfds, nullptr, nullptr, &timeout);
    if (ready <= 0) {
        return false;
    }

    std::uint8_t buffer[2048];
    return recvfrom(fd, buffer, sizeof(buffer), 0, nullptr, nullptr) > 0;
}

void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

} // namespace

int main()
{
    const auto port = findAvailableUdpPort();
    onboard::autopilot::UdpMavlinkTransport transport(port, MAVLINK_COMM_1, "udp-test");

    std::vector<std::uint8_t> datagram;
    appendPacket(datagram, packLocalPosition());
    appendPacket(datagram, packHeartbeat());
    sendDatagram(port, datagram);

    mavlink_message_t first {};
    assert(transport.recvMessage(first, 1000));
    assert(first.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED);

    mavlink_message_t second {};
    assert(transport.recvMessage(second, 0));
    assert(second.msgid == MAVLINK_MSG_ID_HEARTBEAT);

    mavlink_message_t none {};
    assert(!transport.recvMessage(none, 0));

    const auto peer_port = findAvailableUdpPort();
    onboard::autopilot::UdpMavlinkTransport peer_transport(
        peer_port,
        MAVLINK_COMM_1,
        "udp-peer-test");
    const int autopilot_fd = openLoopbackUdpSocket();
    const int gcs_fd = openLoopbackUdpSocket();

    std::vector<std::uint8_t> autopilot_datagram;
    appendPacket(autopilot_datagram, packHeartbeat());
    sendDatagramFrom(autopilot_fd, peer_port, autopilot_datagram);

    mavlink_message_t autopilot_heartbeat {};
    require(peer_transport.recvMessage(autopilot_heartbeat, 1000), "missing autopilot heartbeat");
    require(autopilot_heartbeat.msgid == MAVLINK_MSG_ID_HEARTBEAT, "expected autopilot heartbeat");
    peer_transport.pinPeerFromLastMessage();

    std::vector<std::uint8_t> gcs_datagram;
    appendPacket(gcs_datagram, packGcsHeartbeat());
    sendDatagramFrom(gcs_fd, peer_port, gcs_datagram);

    mavlink_message_t gcs_heartbeat {};
    require(peer_transport.recvMessage(gcs_heartbeat, 1000), "missing gcs heartbeat");
    require(gcs_heartbeat.msgid == MAVLINK_MSG_ID_HEARTBEAT, "expected gcs heartbeat");

    peer_transport.sendMessage(packLocalPosition());
    require(receiveAnyDatagram(autopilot_fd, 1000), "autopilot peer did not receive command");
    require(!receiveAnyDatagram(gcs_fd, 100), "gcs peer hijacked command");

    close(autopilot_fd);
    close(gcs_fd);

    return 0;
}
