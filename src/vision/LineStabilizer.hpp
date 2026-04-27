#pragma once

#include "common/VisionConfig.hpp"
#include "vision/VisionTypes.hpp"

namespace onboard::vision {

class LineStabilizer {
public:
    explicit LineStabilizer(const common::LineConfig& config);

    LineDetection update(const LineDetection& raw, int image_width);
    void reset();

private:
    common::LineConfig config_;
    bool has_filtered_ = false;
    LineDetection filtered_;
    int missing_frames_ = 0;
    int jump_frames_ = 0;
};

} // namespace onboard::vision
