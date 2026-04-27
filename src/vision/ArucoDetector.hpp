#pragma once

#include "common/VisionConfig.hpp"
#include "vision/VisionTypes.hpp"

#include <opencv2/core.hpp>

#include <string>

namespace onboard::vision {

class ArucoDetector {
public:
    explicit ArucoDetector(const common::ArucoConfig& config);

    VisionResult detect(
        const cv::Mat& image,
        std::uint32_t frame_seq,
        std::int64_t timestamp_ms) const;

    std::string dictionaryName() const;

private:
    common::ArucoConfig config_;
};

} // namespace onboard::vision
