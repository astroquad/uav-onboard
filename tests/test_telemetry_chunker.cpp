// Tests are assert-based: keep assert() active even in Release
// builds (CMake adds -DNDEBUG there, which silently no-ops all checks).
#undef NDEBUG

#include "network/TelemetryChunker.hpp"
#include "video/VideoPacket.hpp"

#include <cassert>
#include <cstdint>
#include <string>
#include <vector>

namespace {

std::string makePayload(std::size_t size)
{
    std::string payload = "{\"seq\":1,";
    payload.append(size - payload.size() - 1, 'x');
    payload.push_back('}');
    assert(payload.size() == size);
    return payload;
}

} // namespace

int main()
{
    using onboard::network::buildTelemetryDatagrams;
    using onboard::video::kTelemetryMagic;
    using onboard::video::kTelemetryMaxPayloadSize;
    using onboard::video::kVideoHeaderSize;
    using onboard::video::kVideoMagic;
    using onboard::video::VideoPacketHeader;

    // Small payload passes through unchanged as bare JSON.
    {
        const std::string payload = makePayload(500);
        const auto datagrams = buildTelemetryDatagrams(payload, 1, 1000);
        assert(datagrams.size() == 1);
        assert(datagrams[0] == payload);
        assert(datagrams[0].front() == '{');
    }

    // Boundary: exactly the max payload stays unchunked; one byte more chunks.
    {
        const auto at_limit =
            buildTelemetryDatagrams(makePayload(kTelemetryMaxPayloadSize), 2, 1000);
        assert(at_limit.size() == 1);
        assert(at_limit[0].front() == '{');

        const auto over_limit =
            buildTelemetryDatagrams(makePayload(kTelemetryMaxPayloadSize + 1), 3, 1000);
        assert(over_limit.size() == 2);
        assert(over_limit[0].front() != '{');
    }

    // 3500 bytes -> 3 chunks, headers consistent, reassembled slices identical.
    {
        const std::string payload = makePayload(3500);
        const auto datagrams = buildTelemetryDatagrams(payload, 77, 123456789);
        assert(datagrams.size() == 3);

        std::string reassembled;
        for (std::size_t i = 0; i < datagrams.size(); ++i) {
            const auto& datagram = datagrams[i];
            assert(datagram.size() <= kVideoHeaderSize + kTelemetryMaxPayloadSize);

            VideoPacketHeader header;
            const auto* bytes = reinterpret_cast<const std::uint8_t*>(datagram.data());
            assert(onboard::video::parseHeader(
                bytes, datagram.size(), header, kTelemetryMagic));
            // A telemetry datagram must not parse as a video packet.
            VideoPacketHeader video_header;
            assert(!onboard::video::parseHeader(
                bytes, datagram.size(), video_header, kVideoMagic));

            assert(header.frame_id == 77);
            assert(header.chunk_index == i);
            assert(header.chunk_count == 3);
            assert(header.timestamp_ms == 123456789);
            assert(header.payload_size == datagram.size() - kVideoHeaderSize);
            reassembled.append(datagram, kVideoHeaderSize, std::string::npos);
        }
        assert(reassembled == payload);
    }

    return 0;
}
