#include "video/VideoPacket.hpp"

#include <algorithm>

namespace onboard::video {
namespace {

void writeU16(std::array<std::uint8_t, kVideoHeaderSize>& output, std::size_t offset, std::uint16_t value)
{
    output[offset] = static_cast<std::uint8_t>((value >> 8) & 0xff);
    output[offset + 1] = static_cast<std::uint8_t>(value & 0xff);
}

void writeU32(std::array<std::uint8_t, kVideoHeaderSize>& output, std::size_t offset, std::uint32_t value)
{
    output[offset] = static_cast<std::uint8_t>((value >> 24) & 0xff);
    output[offset + 1] = static_cast<std::uint8_t>((value >> 16) & 0xff);
    output[offset + 2] = static_cast<std::uint8_t>((value >> 8) & 0xff);
    output[offset + 3] = static_cast<std::uint8_t>(value & 0xff);
}

void writeU64(std::array<std::uint8_t, kVideoHeaderSize>& output, std::size_t offset, std::uint64_t value)
{
    for (int i = 7; i >= 0; --i) {
        output[offset + static_cast<std::size_t>(7 - i)] =
            static_cast<std::uint8_t>((value >> (i * 8)) & 0xff);
    }
}

std::uint16_t readU16(const std::uint8_t* data, std::size_t offset)
{
    return static_cast<std::uint16_t>((static_cast<std::uint16_t>(data[offset]) << 8) |
                                      static_cast<std::uint16_t>(data[offset + 1]));
}

std::uint32_t readU32(const std::uint8_t* data, std::size_t offset)
{
    return (static_cast<std::uint32_t>(data[offset]) << 24) |
           (static_cast<std::uint32_t>(data[offset + 1]) << 16) |
           (static_cast<std::uint32_t>(data[offset + 2]) << 8) |
           static_cast<std::uint32_t>(data[offset + 3]);
}

std::uint64_t readU64(const std::uint8_t* data, std::size_t offset)
{
    std::uint64_t value = 0;
    for (std::size_t i = 0; i < 8; ++i) {
        value = (value << 8) | static_cast<std::uint64_t>(data[offset + i]);
    }
    return value;
}

} // namespace

std::array<std::uint8_t, kVideoHeaderSize> serializeHeader(const VideoPacketHeader& header)
{
    std::array<std::uint8_t, kVideoHeaderSize> output {};
    std::copy(kVideoMagic.begin(), kVideoMagic.end(), output.begin());
    writeU16(output, 4, static_cast<std::uint16_t>(kVideoHeaderSize));
    writeU16(output, 6, header.flags);
    writeU32(output, 8, header.frame_id);
    writeU16(output, 12, header.chunk_index);
    writeU16(output, 14, header.chunk_count);
    writeU32(output, 16, header.payload_size);
    writeU64(output, 20, header.timestamp_ms);
    return output;
}

bool parseHeader(const std::uint8_t* data, std::size_t size, VideoPacketHeader& header)
{
    if (size < kVideoHeaderSize) {
        return false;
    }
    if (!std::equal(kVideoMagic.begin(), kVideoMagic.end(), reinterpret_cast<const char*>(data))) {
        return false;
    }
    if (readU16(data, 4) != kVideoHeaderSize) {
        return false;
    }

    header.flags = readU16(data, 6);
    header.frame_id = readU32(data, 8);
    header.chunk_index = readU16(data, 12);
    header.chunk_count = readU16(data, 14);
    header.payload_size = readU32(data, 16);
    header.timestamp_ms = readU64(data, 20);
    return header.payload_size <= size - kVideoHeaderSize;
}

} // namespace onboard::video
