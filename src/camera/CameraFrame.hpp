#pragma once

#include <cstdint>
#include <vector>

namespace onboard::camera {

struct CameraFrame {
    std::uint32_t frame_id = 0;
    std::int64_t timestamp_ms = 0;
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> jpeg_data;
};

} // namespace onboard::camera
