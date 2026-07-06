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

// XOR forward-error-correction for video chunks (protocol v1.13). A parity
// packet follows each group of data chunks; the receiver reconstructs one
// missing data chunk per group. Parity packets set kVideoFlagFecParity in
// header.flags, carry the group size in the flags high byte, use chunk_index
// as the group index, and their payload is a 4-byte big-endian total frame
// byte count followed by the XOR of the group's zero-padded data payloads.
constexpr std::uint16_t kVideoFlagFecParity = 0x0001;
constexpr std::size_t kVideoFecPayloadSize = kVideoMaxPayloadSize + 4;

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
