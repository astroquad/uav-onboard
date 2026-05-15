#include "autopilot/SerialMavlinkTransport.hpp"

#include <fcntl.h>
#include <sys/select.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace onboard::autopilot {
namespace {

speed_t baudrateToSpeed(int baudrate)
{
    switch (baudrate) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 921600:
        return B921600;
    default:
        throw std::runtime_error("unsupported serial baudrate: " + std::to_string(baudrate));
    }
}

std::string errnoMessage(const std::string& prefix)
{
    return prefix + ": " + std::strerror(errno);
}

} // namespace

SerialMavlinkTransport::SerialMavlinkTransport(
    std::string device,
    int baudrate,
    std::uint8_t parse_channel,
    std::string name)
    : parse_channel_(parse_channel)
    , device_(std::move(device))
    , name_(std::move(name))
{
    fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        throw std::runtime_error(errnoMessage("open(" + device_ + ") failed"));
    }

    try {
        configurePort(baudrate);
    } catch (...) {
        close(fd_);
        fd_ = -1;
        throw;
    }
}

SerialMavlinkTransport::~SerialMavlinkTransport()
{
    if (fd_ >= 0) {
        if (have_original_termios_) {
            tcsetattr(fd_, TCSANOW, &original_termios_);
        }
        close(fd_);
    }
}

void SerialMavlinkTransport::configurePort(int baudrate)
{
    if (tcgetattr(fd_, &original_termios_) == 0) {
        have_original_termios_ = true;
    }

    termios options {};
    if (tcgetattr(fd_, &options) < 0) {
        throw std::runtime_error(errnoMessage("tcgetattr(" + device_ + ") failed"));
    }

    cfmakeraw(&options);
    const speed_t speed = baudrateToSpeed(baudrate);
    if (cfsetispeed(&options, speed) < 0 || cfsetospeed(&options, speed) < 0) {
        throw std::runtime_error(errnoMessage("cfsetspeed(" + device_ + ") failed"));
    }

    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &options) < 0) {
        throw std::runtime_error(errnoMessage("tcsetattr(" + device_ + ") failed"));
    }
    tcflush(fd_, TCIOFLUSH);
}

bool SerialMavlinkTransport::recvMessage(mavlink_message_t& message, int timeout_ms)
{
    if (!pending_messages_.empty()) {
        message = pending_messages_.front();
        pending_messages_.pop_front();
        return true;
    }

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);

    timeval timeout {};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    const int ready = select(fd_ + 1, &readfds, nullptr, nullptr, &timeout);
    if (ready < 0) {
        if (errno == EINTR) {
            return false;
        }
        throw std::runtime_error(errnoMessage("select(" + device_ + ") failed"));
    }
    if (ready == 0) {
        return false;
    }

    std::uint8_t buffer[512];
    const ssize_t received = read(fd_, buffer, sizeof(buffer));
    if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            return false;
        }
        throw std::runtime_error(errnoMessage("read(" + device_ + ") failed"));
    }
    if (received == 0) {
        return false;
    }

    for (ssize_t i = 0; i < received; ++i) {
        mavlink_message_t parsed {};
        if (mavlink_parse_char(parse_channel_, buffer[i], &parsed, &parse_status_)) {
            pending_messages_.push_back(parsed);
        }
    }

    if (pending_messages_.empty()) {
        return false;
    }

    message = pending_messages_.front();
    pending_messages_.pop_front();
    return true;
}

void SerialMavlinkTransport::sendMessage(const mavlink_message_t& message)
{
    std::uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const std::uint16_t length = mavlink_msg_to_send_buffer(buffer, &message);

    std::uint16_t written = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (written < length) {
        const ssize_t sent = write(fd_, buffer + written, length - written);
        if (sent > 0) {
            written = static_cast<std::uint16_t>(written + sent);
            continue;
        }
        if (sent < 0 && (errno == EINTR)) {
            continue;
        }
        if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            if (std::chrono::steady_clock::now() >= deadline) {
                throw std::runtime_error("timed out writing MAVLink message to " + device_);
            }
            fd_set writefds;
            FD_ZERO(&writefds);
            FD_SET(fd_, &writefds);
            timeval timeout {};
            timeout.tv_sec = 0;
            timeout.tv_usec = 100000;
            const int ready = select(fd_ + 1, nullptr, &writefds, nullptr, &timeout);
            if (ready < 0 && errno != EINTR) {
                throw std::runtime_error(errnoMessage("select-write(" + device_ + ") failed"));
            }
            continue;
        }
        throw std::runtime_error(errnoMessage("write(" + device_ + ") failed"));
    }
}

} // namespace onboard::autopilot
