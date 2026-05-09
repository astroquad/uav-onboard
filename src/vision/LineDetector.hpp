#pragma once

#include "common/VisionConfig.hpp"
#include "vision/LineMaskBuilder.hpp"
#include "vision/VisionTypes.hpp"

#include <opencv2/core.hpp>

namespace onboard::vision {

class LineDetector {
public:
    explicit LineDetector(const common::LineConfig& config);

    LineDetection detect(const cv::Mat& image) const;
    LineDetection detect(const LineMaskFrame& masks) const;

private:
    common::LineConfig config_;
};

LineDetection applyMarkerCenterTracking(
    LineDetection line,
    const std::vector<MarkerObservation>& markers,
    int frame_width,
    int frame_height);

} // namespace onboard::vision
