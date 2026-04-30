#include "vision/IntersectionStabilizer.hpp"

#include <cassert>

namespace {

onboard::vision::IntersectionDetection makeDetection(
    onboard::vision::IntersectionType type,
    float score = 0.8f)
{
    onboard::vision::IntersectionDetection detection;
    detection.valid = true;
    detection.type = type;
    detection.raw_type = type;
    detection.intersection_detected =
        type == onboard::vision::IntersectionType::L ||
        type == onboard::vision::IntersectionType::T ||
        type == onboard::vision::IntersectionType::Cross;
    detection.score = score;
    detection.raw_score = score;
    detection.center_px = {100.0f, 120.0f};
    detection.raw_center_px = detection.center_px;
    return detection;
}

} // namespace

int main()
{
    onboard::common::LineConfig config;
    config.filter_enabled = true;
    config.filter_reacquire_frames = 2;
    config.filter_hold_frames = 2;
    config.filter_min_confidence = 0.25;

    onboard::vision::IntersectionStabilizer stabilizer(config);

    auto first = stabilizer.update(makeDetection(onboard::vision::IntersectionType::T));
    assert(first.type == onboard::vision::IntersectionType::T);
    assert(!first.stable);
    assert(first.raw_type == onboard::vision::IntersectionType::T);

    auto second = stabilizer.update(makeDetection(onboard::vision::IntersectionType::T));
    assert(second.type == onboard::vision::IntersectionType::T);
    assert(second.stable);
    assert(second.stable_frames >= 2);

    auto transient = stabilizer.update(makeDetection(onboard::vision::IntersectionType::L));
    assert(transient.type == onboard::vision::IntersectionType::T);
    assert(transient.raw_type == onboard::vision::IntersectionType::L);
    assert(!transient.held);

    auto switched = stabilizer.update(makeDetection(onboard::vision::IntersectionType::L));
    assert(switched.type == onboard::vision::IntersectionType::L);
    assert(switched.raw_type == onboard::vision::IntersectionType::L);

    auto held = stabilizer.update({});
    assert(held.type == onboard::vision::IntersectionType::L);
    assert(held.held);
    assert(held.valid);

    return 0;
}
