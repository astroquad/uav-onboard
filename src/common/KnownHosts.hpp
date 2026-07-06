#pragma once

#include <string>

namespace onboard::common {

// Central name -> IP mapping for the effectively-static Tailscale hosts.
// Keep this table byte-identical with uav-gcs/src/common/KnownHosts.hpp
// (the repos share no code on purpose).
struct KnownHost {
    const char* name;
    const char* ip;
};

inline constexpr KnownHost kKnownHosts[] = {
    {"gcs-laptop", "100.85.239.73"},   // Lenovo GCS laptop (Tailscale)
    {"pi5",        "100.101.84.47"},   // Raspberry Pi 5 (Tailscale)
    {"broadcast",  "255.255.255.255"}, // LAN/WSL discovery flows
};

// Known name -> IP; anything else (literal IPs, unknown names) is returned
// unchanged so existing configs and CLI usage keep working.
inline std::string resolveKnownHost(const std::string& name_or_ip)
{
    for (const auto& host : kKnownHosts) {
        if (name_or_ip == host.name) {
            return host.ip;
        }
    }
    return name_or_ip;
}

} // namespace onboard::common
