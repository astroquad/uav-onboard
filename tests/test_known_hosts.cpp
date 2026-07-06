// Tests are assert-based: keep assert() active even in Release
// builds (CMake adds -DNDEBUG there, which silently no-ops all checks).
#undef NDEBUG

#include "common/KnownHosts.hpp"

#include <cassert>
#include <string>

int main()
{
    using onboard::common::resolveKnownHost;

    assert(resolveKnownHost("gcs-laptop") == "100.85.239.73");
    assert(resolveKnownHost("pi5") == "100.101.84.47");
    assert(resolveKnownHost("broadcast") == "255.255.255.255");

    // Literal IPs and unknown names pass through unchanged.
    assert(resolveKnownHost("192.168.0.10") == "192.168.0.10");
    assert(resolveKnownHost("255.255.255.255") == "255.255.255.255");
    assert(resolveKnownHost("some-unknown-host") == "some-unknown-host");
    assert(resolveKnownHost("") == "");

    return 0;
}
