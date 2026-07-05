#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace onboard::video {

constexpr std::array<char, 4> kVideoMagic = {'A', 'Q', 'V', '1'};
// Telemetry JSON above kTelemetryMaxPayloadSize is chunked with the same
// header layout under this magic ("frame_id" carries the message id).
constexpr std::array<char, 4> kTelemetryMagic = {'A', 'Q', 'T', '1'};
constexpr std::size_t kVideoHeaderSize = 28;
// Keeps every datagram (header + payload) within a 1280-byte tunnel MTU
// (e.g. Tailscale/WireGuard) so UDP never relies on IP fragmentation.
constexpr std::size_t kVideoMaxPayloadSize = 1200;
constexpr std::size_t kTelemetryMaxPayloadSize = 1200;

struct VideoPacketHeader {
    std::uint16_t flags = 0;
    std::uint32_t frame_id = 0;
    std::uint16_t chunk_index = 0;
    std::uint16_t chunk_count = 0;
    std::uint32_t payload_size = 0;
    std::uint64_t timestamp_ms = 0;
};

std::array<std::uint8_t, kVideoHeaderSize> serializeHeader(
    const VideoPacketHeader& header,
    const std::array<char, 4>& magic);
bool parseHeader(
    const std::uint8_t* data,
    std::size_t size,
    VideoPacketHeader& header,
    const std::array<char, 4>& magic);

std::array<std::uint8_t, kVideoHeaderSize> serializeHeader(const VideoPacketHeader& header);
bool parseHeader(const std::uint8_t* data, std::size_t size, VideoPacketHeader& header);

} // namespace onboard::video
