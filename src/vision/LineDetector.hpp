#pragma once

#include "common/VisionConfig.hpp"
#include "vision/VisionTypes.hpp"

#include <opencv2/core.hpp>

namespace onboard::vision {

class LineDetector {
public:
    explicit LineDetector(const common::LineConfig& config);

    LineDetection detect(const cv::Mat& image) const;

private:
    common::LineConfig config_;
};

} // namespace onboard::vision
