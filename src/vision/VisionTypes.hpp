#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace onboard::vision {

struct Point2f {
    float x = 0.0f;
    float y = 0.0f;
};

struct MarkerObservation {
    int id = -1;
    Point2f center_px;
    std::array<Point2f, 4> corners_px {};
    float orientation_deg = 0.0f;
};

struct VisionResult {
    std::uint32_t frame_seq = 0;
    std::int64_t timestamp_ms = 0;
    int width = 0;
    int height = 0;
    std::vector<MarkerObservation> markers;

    bool line_detected = false;
    bool intersection_detected = false;
};

} // namespace onboard::vision
