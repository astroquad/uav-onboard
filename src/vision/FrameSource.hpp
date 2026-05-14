#pragma once

#include "common/VisionConfig.hpp"

#include <opencv2/core.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace onboard::vision {

struct Frame {
    cv::Mat image_bgr;
    std::uint32_t frame_id = 0;
    std::int64_t timestamp_ms = 0;
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> jpeg_data;
};

struct FrameSourceOptions {
    common::VisionConfig vision;
};

class FrameSource {
public:
    virtual ~FrameSource() = default;

    virtual bool open(const FrameSourceOptions& options) = 0;
    virtual bool read(Frame& frame) = 0;
    virtual void close() = 0;
    virtual std::string lastError() const = 0;
};

} // namespace onboard::vision
