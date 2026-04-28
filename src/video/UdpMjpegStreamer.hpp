#pragma once

#include "camera/CameraFrame.hpp"

#include <cstdint>
#include <string>

namespace onboard::video {

class UdpMjpegStreamer {
public:
    UdpMjpegStreamer();
    ~UdpMjpegStreamer();

    UdpMjpegStreamer(const UdpMjpegStreamer&) = delete;
    UdpMjpegStreamer& operator=(const UdpMjpegStreamer&) = delete;

    bool open(const std::string& ip, std::uint16_t port);
    void setChunkPacingUs(int pacing_us);
    bool sendFrame(const camera::CameraFrame& frame);
    void close();
    std::string lastError() const;
    int lastChunkCount() const;

private:
    std::uintptr_t socket_ = 0;
    bool socket_open_ = false;
    std::string ip_;
    std::uint16_t port_ = 0;
    int chunk_pacing_us_ = 0;
    int last_chunk_count_ = 0;
    mutable std::string last_error_;
};

} // namespace onboard::video
