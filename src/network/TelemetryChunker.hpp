#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace onboard::network {

// Splits one telemetry JSON payload into MTU-safe UDP datagrams.
//
// Payloads up to kTelemetryMaxPayloadSize are returned unchanged as a single
// bare-JSON datagram (receivers detect the leading '{'), so small messages
// stay wire-compatible with older GCS builds. Larger payloads are split into
// AQT1-framed chunks that reuse the 28-byte video header layout with the
// frame_id field carrying message_id. Each datagram stays within a 1280-byte
// tunnel MTU (e.g. Tailscale/WireGuard), avoiding IP fragmentation.
std::vector<std::string> buildTelemetryDatagrams(
    const std::string& payload,
    std::uint32_t message_id,
    std::uint64_t timestamp_ms);

} // namespace onboard::network
