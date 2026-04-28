#include "vision/LineStabilizer.hpp"

#include <algorithm>
#include <cmath>

namespace onboard::vision {
namespace {

double clampRatio(double value, double fallback)
{
    if (!std::isfinite(value)) {
        return fallback;
    }
    return std::clamp(value, 0.0, 1.0);
}

double angleDeltaDeg(double a, double b)
{
    double delta = std::fmod(a - b + 180.0, 360.0);
    if (delta < 0.0) {
        delta += 360.0;
    }
    return delta - 180.0;
}

float lerp(float from, float to, double alpha)
{
    return static_cast<float>(from * (1.0 - alpha) + to * alpha);
}

LineDetection heldLine(LineDetection line, int missing_frames)
{
    line.detected = true;
    line.filtered = true;
    line.held = true;
    line.rejected_jump = false;
    const double decay = std::pow(0.85, std::max(1, missing_frames));
    line.confidence = static_cast<float>(std::clamp(line.confidence * decay, 0.0, 1.0));
    return line;
}

} // namespace

LineStabilizer::LineStabilizer(const common::LineConfig& config)
    : config_(config)
{
}

void LineStabilizer::reset()
{
    has_filtered_ = false;
    filtered_ = {};
    missing_frames_ = 0;
    jump_frames_ = 0;
}

LineDetection LineStabilizer::update(const LineDetection& raw, int image_width)
{
    if (!config_.filter_enabled) {
        return raw;
    }

    const double base_alpha = std::clamp(config_.filter_ema_alpha, 0.05, 1.0);
    const int hold_frames = std::max(0, config_.filter_hold_frames);
    const int reacquire_frames = std::max(1, config_.filter_reacquire_frames);
    const double max_offset_jump =
        std::max(8.0, image_width * clampRatio(config_.filter_max_offset_jump_ratio, 0.16));
    const double max_offset_velocity =
        std::max(4.0, image_width * clampRatio(config_.filter_max_offset_velocity_ratio, 0.08));
    const double max_angle_jump = std::max(1.0, config_.filter_max_angle_jump_deg);

    if (!raw.detected || raw.confidence < config_.filter_min_confidence) {
        ++missing_frames_;
        jump_frames_ = 0;
        if (has_filtered_ && missing_frames_ <= hold_frames) {
            LineDetection held = heldLine(filtered_, missing_frames_);
            held.raw_detected = false;
            held.raw_tracking_point_px = {};
            held.raw_center_offset_px = 0.0f;
            held.raw_angle_deg = 0.0f;
            held.mask_count = raw.mask_count;
            held.contours_found = raw.contours_found;
            held.candidates_evaluated = raw.candidates_evaluated;
            held.roi_pixels = raw.roi_pixels;
            held.selected_contour_points = static_cast<int>(held.contour_px.size());
            return held;
        }
        reset();
        return {};
    }

    LineDetection accepted = raw;
    accepted.raw_detected = true;
    accepted.raw_tracking_point_px = raw.tracking_point_px;
    accepted.raw_center_offset_px = raw.center_offset_px;
    accepted.raw_angle_deg = raw.angle_deg;

    if (!has_filtered_) {
        missing_frames_ = 0;
        jump_frames_ = 0;
        accepted.filtered = true;
        filtered_ = accepted;
        has_filtered_ = true;
        return filtered_;
    }

    const double offset_jump =
        std::abs(raw.center_offset_px - filtered_.center_offset_px);
    const double angle_jump =
        std::abs(angleDeltaDeg(raw.angle_deg, filtered_.angle_deg));
    const bool is_jump = offset_jump > max_offset_jump || angle_jump > max_angle_jump;

    if (is_jump && jump_frames_ + 1 < reacquire_frames) {
        ++jump_frames_;
        missing_frames_ = 0;
        LineDetection held = heldLine(filtered_, jump_frames_);
        held.raw_detected = true;
        held.raw_tracking_point_px = raw.tracking_point_px;
        held.raw_center_offset_px = raw.center_offset_px;
        held.raw_angle_deg = raw.angle_deg;
        held.rejected_jump = true;
        held.mask_count = raw.mask_count;
        held.contours_found = raw.contours_found;
        held.candidates_evaluated = raw.candidates_evaluated;
        held.roi_pixels = raw.roi_pixels;
        held.selected_contour_points = static_cast<int>(held.contour_px.size());
        return held;
    }

    missing_frames_ = 0;
    jump_frames_ = 0;
    accepted.filtered = true;
    accepted.held = false;
    accepted.rejected_jump = false;
    const double confidence_scale = std::clamp(
        (raw.confidence - config_.filter_min_confidence) /
            std::max(0.01, 1.0 - config_.filter_min_confidence),
        std::clamp(config_.filter_confidence_alpha_min, 0.05, 1.0),
        1.0);
    const double alpha = base_alpha * confidence_scale;
    const float velocity_limited_x = static_cast<float>(std::clamp(
        static_cast<double>(raw.tracking_point_px.x),
        filtered_.tracking_point_px.x - max_offset_velocity,
        filtered_.tracking_point_px.x + max_offset_velocity));
    accepted.tracking_point_px.x = lerp(
        filtered_.tracking_point_px.x,
        velocity_limited_x,
        alpha);
    accepted.tracking_point_px.y = lerp(
        filtered_.tracking_point_px.y,
        raw.tracking_point_px.y,
        alpha);
    accepted.center_offset_px =
        accepted.tracking_point_px.x - static_cast<float>(image_width) / 2.0f;
    accepted.angle_deg = lerp(
        filtered_.angle_deg,
        raw.angle_deg,
        alpha);
    accepted.selected_contour_points = static_cast<int>(accepted.contour_px.size());

    filtered_ = accepted;
    has_filtered_ = true;
    return filtered_;
}

} // namespace onboard::vision
