#pragma once

#include "common/VisionConfig.hpp"
#include "vision/VisionTypes.hpp"

#include <vector>

namespace onboard::vision {

class MarkerStabilizer {
public:
    explicit MarkerStabilizer(const common::ArucoConfig& config);

    std::vector<MarkerObservation> update(const std::vector<MarkerObservation>& detected_markers);
    void reset();

    int ageFrames() const;

private:
    common::ArucoConfig config_;
    std::vector<MarkerObservation> held_markers_;
    int age_frames_ = 1000000;
};

} // namespace onboard::vision
