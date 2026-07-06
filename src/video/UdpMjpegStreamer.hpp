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
    // Spreads each frame's chunks across this window instead of one
    // line-rate burst. Shallow/bufferbloated paths (LTE uplinks, DERP
    // relays) tail-drop ~20-packet bursts; pacing them over most of the
    // frame period removes the burst without lowering the frame rate.
    // 0 keeps pure chunk_pacing behavior.
    void setFrameSpreadUs(int spread_us);
    // Sends one XOR parity packet per this many data chunks so the receiver
    // can reconstruct a single lost chunk per group (lossy LTE/relay paths).
    // 0 disables FEC. Bandwidth overhead is ~1/group_size.
    void setFecGroupSize(int group_size);
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
    int frame_spread_us_ = 0;
    int fec_group_size_ = 0;
    int last_chunk_count_ = 0;
    mutable std::string last_error_;
};

} // namespace onboard::video
