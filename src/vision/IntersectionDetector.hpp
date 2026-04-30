#pragma once

#include "common/VisionConfig.hpp"
#include "vision/LineMaskBuilder.hpp"
#include "vision/VisionTypes.hpp"

namespace onboard::vision {

class IntersectionDetector {
public:
    explicit IntersectionDetector(const common::LineConfig& config);

    IntersectionDetection detect(
        const LineMaskFrame& masks,
        const LineDetection& raw_line) const;

private:
    common::LineConfig config_;
};

const char* intersectionTypeName(IntersectionType type);
const char* branchDirectionName(BranchDirection direction);

} // namespace onboard::vision
