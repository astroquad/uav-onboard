#include "network/TelemetryChunker.hpp"

#include "video/VideoPacket.hpp"

#include <algorithm>

namespace onboard::network {

std::vector<std::string> buildTelemetryDatagrams(
    const std::string& payload,
    std::uint32_t message_id,
    std::uint64_t timestamp_ms)
{
    std::vector<std::string> datagrams;
    if (payload.size() <= video::kTelemetryMaxPayloadSize) {
        datagrams.push_back(payload);
        return datagrams;
    }

    const std::size_t chunk_count =
        (payload.size() + video::kTelemetryMaxPayloadSize - 1) /
        video::kTelemetryMaxPayloadSize;
    datagrams.reserve(chunk_count);
    for (std::size_t index = 0; index < chunk_count; ++index) {
        const std::size_t offset = index * video::kTelemetryMaxPayloadSize;
        const std::size_t chunk_size =
            std::min(video::kTelemetryMaxPayloadSize, payload.size() - offset);

        video::VideoPacketHeader header;
        header.frame_id = message_id;
        header.chunk_index = static_cast<std::uint16_t>(index);
        header.chunk_count = static_cast<std::uint16_t>(chunk_count);
        header.payload_size = static_cast<std::uint32_t>(chunk_size);
        header.timestamp_ms = timestamp_ms;
        const auto header_bytes =
            video::serializeHeader(header, video::kTelemetryMagic);

        std::string datagram;
        datagram.reserve(video::kVideoHeaderSize + chunk_size);
        datagram.append(reinterpret_cast<const char*>(header_bytes.data()),
                        header_bytes.size());
        datagram.append(payload, offset, chunk_size);
        datagrams.push_back(std::move(datagram));
    }
    return datagrams;
}

} // namespace onboard::network
