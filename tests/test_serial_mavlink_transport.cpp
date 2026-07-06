// Tests are assert-based: keep assert() active even in Release
// builds (CMake adds -DNDEBUG there, which silently no-ops all checks).
#undef NDEBUG

#include "autopilot/SerialMavlinkTransport.hpp"

#include <fcntl.h>
#include <sys/select.h>
#include <unistd.h>

#include <cassert>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

int openPty(std::string& slave_name)
{
    const int master_fd = posix_openpt(O_RDWR | O_NOCTTY);
    if (master_fd < 0) {
        throw std::runtime_error("posix_openpt() failed");
    }
    if (grantpt(master_fd) < 0 || unlockpt(master_fd) < 0) {
        close(master_fd);
        throw std::runtime_error("grantpt/unlockpt failed");
    }
    char* name = ptsname(master_fd);
    if (!name) {
        close(master_fd);
        throw std::runtime_error("ptsname() failed");
    }
    slave_name = name;
    return master_fd;
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

mavlink_message_t packLocalPosition()
{
    mavlink_message_t message {};
    mavlink_msg_local_position_ned_pack(
        191,
        191,
        &message,
        100,
        1.0f,
        2.0f,
        -3.0f,
        0.1f,
        0.2f,
        0.3f);
    return message;
}

std::vector<std::uint8_t> toBytes(const mavlink_message_t& message)
{
    std::uint8_t packet[MAVLINK_MAX_PACKET_LEN];
    const auto length = mavlink_msg_to_send_buffer(packet, &message);
    return std::vector<std::uint8_t>(packet, packet + length);
}

void writeAll(int fd, const std::vector<std::uint8_t>& bytes, std::size_t first_chunk)
{
    const ssize_t first = write(fd, bytes.data(), first_chunk);
    if (first != static_cast<ssize_t>(first_chunk)) {
        throw std::runtime_error("first write failed");
    }
    const ssize_t second = write(fd, bytes.data() + first_chunk, bytes.size() - first_chunk);
    if (second != static_cast<ssize_t>(bytes.size() - first_chunk)) {
        throw std::runtime_error("second write failed");
    }
}

bool readAndParseOne(int fd, mavlink_message_t& message)
{
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);

    timeval timeout {};
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    const int ready = select(fd + 1, &readfds, nullptr, nullptr, &timeout);
    if (ready <= 0) {
        return false;
    }

    std::uint8_t buffer[256];
    const ssize_t received = read(fd, buffer, sizeof(buffer));
    if (received <= 0) {
        return false;
    }

    mavlink_status_t status {};
    for (ssize_t i = 0; i < received; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_2, buffer[i], &message, &status)) {
            return true;
        }
    }
    return false;
}

} // namespace

int main()
{
    std::string slave_name;
    const int master_fd = openPty(slave_name);

    onboard::autopilot::SerialMavlinkTransport transport(
        slave_name,
        115200,
        MAVLINK_COMM_1,
        "serial-test");

    const auto heartbeat_bytes = toBytes(packHeartbeat());
    writeAll(master_fd, heartbeat_bytes, 3);

    mavlink_message_t received {};
    assert(transport.recvMessage(received, 1000));
    assert(received.msgid == MAVLINK_MSG_ID_HEARTBEAT);

    transport.sendMessage(packLocalPosition());
    mavlink_message_t outgoing {};
    assert(readAndParseOne(master_fd, outgoing));
    assert(outgoing.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED);

    close(master_fd);
    return 0;
}
