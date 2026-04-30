#include "vision/IntersectionStabilizer.hpp"

#include <algorithm>
#include <cmath>

namespace onboard::vision {
namespace {

bool isIntersectionType(IntersectionType type)
{
    return type == IntersectionType::Cross ||
           type == IntersectionType::T ||
           type == IntersectionType::L;
}

float lerp(float from, float to, double alpha)
{
    return static_cast<float>(from * (1.0 - alpha) + to * alpha);
}

Point2f lerpPoint(Point2f from, Point2f to, double alpha)
{
    return {
        lerp(from.x, to.x, alpha),
        lerp(from.y, to.y, alpha),
    };
}

IntersectionDetection heldIntersection(IntersectionDetection detection, int missing_frames)
{
    detection.valid = true;
    detection.stable = true;
    detection.held = true;
    detection.intersection_detected = isIntersectionType(detection.type);
    const double decay = std::pow(0.85, std::max(1, missing_frames));
    detection.score = static_cast<float>(std::clamp(detection.score * decay, 0.0, 1.0));
    return detection;
}

void preserveRawFields(IntersectionDetection& output, const IntersectionDetection& raw)
{
    output.raw_type = raw.type;
    output.raw_score = raw.score;
    output.raw_center_px = raw.center_px;
}

} // namespace

IntersectionStabilizer::IntersectionStabilizer(const common::LineConfig& config)
    : config_(config)
{
}

void IntersectionStabilizer::reset()
{
    has_filtered_ = false;
    filtered_ = {};
    candidate_type_ = IntersectionType::None;
    candidate_frames_ = 0;
    stable_frames_ = 0;
    missing_frames_ = 0;
}

IntersectionDetection IntersectionStabilizer::update(const IntersectionDetection& raw)
{
    if (!config_.filter_enabled) {
        IntersectionDetection output = raw;
        output.raw_type = raw.type;
        output.raw_score = raw.score;
        output.raw_center_px = raw.center_px;
        output.stable = output.valid;
        output.held = false;
        output.stable_frames = output.valid ? 1 : 0;
        output.intersection_detected = isIntersectionType(output.type);
        return output;
    }

    const double alpha = std::clamp(config_.filter_ema_alpha, 0.05, 1.0);
    const int hold_frames = std::max(0, config_.filter_hold_frames);
    const int required_type_frames = std::clamp(config_.filter_reacquire_frames, 2, 3);
    const double min_score = std::clamp(config_.filter_min_confidence, 0.05, 0.95);
    const bool raw_accepted =
        raw.valid &&
        raw.type != IntersectionType::None &&
        raw.score >= min_score;

    if (!raw_accepted) {
        ++missing_frames_;
        candidate_type_ = IntersectionType::None;
        candidate_frames_ = 0;
        if (has_filtered_ && missing_frames_ <= hold_frames) {
            IntersectionDetection held = heldIntersection(filtered_, missing_frames_);
            preserveRawFields(held, raw);
            return held;
        }
        reset();
        IntersectionDetection output = raw;
        output.raw_type = raw.type;
        output.raw_score = raw.score;
        output.raw_center_px = raw.center_px;
        return output;
    }

    missing_frames_ = 0;
    IntersectionDetection accepted = raw;
    accepted.raw_type = raw.type;
    accepted.raw_score = raw.score;
    accepted.raw_center_px = raw.center_px;
    accepted.held = false;

    if (!has_filtered_) {
        has_filtered_ = true;
        stable_frames_ = 1;
        candidate_type_ = IntersectionType::None;
        candidate_frames_ = 0;
        accepted.stable = false;
        accepted.stable_frames = stable_frames_;
        accepted.intersection_detected = isIntersectionType(accepted.type);
        filtered_ = accepted;
        return filtered_;
    }

    IntersectionType output_type = filtered_.type;
    if (raw.type == filtered_.type) {
        ++stable_frames_;
        candidate_type_ = IntersectionType::None;
        candidate_frames_ = 0;
        output_type = filtered_.type;
    } else {
        if (candidate_type_ == raw.type) {
            ++candidate_frames_;
        } else {
            candidate_type_ = raw.type;
            candidate_frames_ = 1;
        }

        if (candidate_frames_ >= required_type_frames) {
            output_type = raw.type;
            stable_frames_ = candidate_frames_;
            candidate_type_ = IntersectionType::None;
            candidate_frames_ = 0;
        } else {
            output_type = filtered_.type;
        }
    }

    accepted.type = output_type;
    accepted.center_px = lerpPoint(filtered_.center_px, raw.center_px, alpha);
    accepted.score = static_cast<float>(std::clamp(
        filtered_.score * (1.0 - alpha) + raw.score * alpha,
        0.0,
        1.0));
    accepted.stable_frames = stable_frames_;
    accepted.stable = stable_frames_ >= required_type_frames;
    accepted.valid = true;
    accepted.intersection_detected = isIntersectionType(accepted.type);

    filtered_ = accepted;
    return filtered_;
}

} // namespace onboard::vision
