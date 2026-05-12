#include "vision/MarkerStabilizer.hpp"

#include <algorithm>
#include <limits>

namespace onboard::vision {

MarkerStabilizer::MarkerStabilizer(const common::ArucoConfig& config)
    : config_(config)
{
}

std::vector<MarkerObservation> MarkerStabilizer::update(
    const std::vector<MarkerObservation>& detected_markers)
{
    if (!detected_markers.empty()) {
        held_markers_ = detected_markers;
        age_frames_ = 0;
        return held_markers_;
    }

    if (age_frames_ < std::numeric_limits<int>::max()) {
        ++age_frames_;
    }

    const int hold_frames = std::max(0, config_.hold_frames);
    if (!held_markers_.empty() && age_frames_ <= hold_frames) {
        return held_markers_;
    }

    if (age_frames_ > hold_frames) {
        held_markers_.clear();
    }
    return {};
}

void MarkerStabilizer::reset()
{
    held_markers_.clear();
    age_frames_ = 1000000;
}

int MarkerStabilizer::ageFrames() const
{
    return age_frames_;
}

} // namespace onboard::vision
