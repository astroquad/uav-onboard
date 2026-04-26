#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace onboard::video {

constexpr std::array<char, 4> kVideoMagic = {'A', 'Q', 'V', '1'};
constexpr std::size_t kVideoHeaderSize = 28;
constexpr std::size_t kVideoMaxPayloadSize = 1200;

struct VideoPacketHeader {
    std::uint16_t flags = 0;
    std::uint32_t frame_id = 0;
    std::uint16_t chunk_index = 0;
    std::uint16_t chunk_count = 0;
    std::uint32_t payload_size = 0;
    std::uint64_t timestamp_ms = 0;
};

std::array<std::uint8_t, kVideoHeaderSize> serializeHeader(const VideoPacketHeader& header);
bool parseHeader(const std::uint8_t* data, std::size_t size, VideoPacketHeader& header);

} // namespace onboard::video
