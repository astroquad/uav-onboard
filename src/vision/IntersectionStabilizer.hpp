#pragma once

#include "common/VisionConfig.hpp"
#include "vision/VisionTypes.hpp"

namespace onboard::vision {

class IntersectionStabilizer {
public:
    explicit IntersectionStabilizer(const common::LineConfig& config);

    IntersectionDetection update(const IntersectionDetection& raw);
    void reset();

private:
    common::LineConfig config_;
    bool has_filtered_ = false;
    IntersectionDetection filtered_;
    IntersectionType candidate_type_ = IntersectionType::None;
    int candidate_frames_ = 0;
    int stable_frames_ = 0;
    int missing_frames_ = 0;
};

} // namespace onboard::vision
