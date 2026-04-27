#include "common/VisionConfig.hpp"
#include "vision/LineStabilizer.hpp"
#include "vision/VisionTypes.hpp"

#include <cassert>

namespace {

onboard::vision::LineDetection makeLine(float tracking_x, float confidence = 0.8f)
{
    onboard::vision::LineDetection line;
    line.detected = true;
    line.raw_detected = true;
    line.tracking_point_px = {tracking_x, 240.0f};
    line.raw_tracking_point_px = line.tracking_point_px;
    line.center_offset_px = tracking_x - 320.0f;
    line.raw_center_offset_px = line.center_offset_px;
    line.angle_deg = 0.0f;
    line.raw_angle_deg = 0.0f;
    line.confidence = confidence;
    return line;
}

} // namespace

int main()
{
    onboard::common::LineConfig config;
    config.filter_enabled = true;
    config.filter_ema_alpha = 0.35;
    config.filter_min_confidence = 0.25;
    config.filter_max_offset_jump_ratio = 0.10;
    config.filter_max_angle_jump_deg = 90.0;
    config.filter_hold_frames = 2;
    config.filter_reacquire_frames = 3;

    onboard::vision::LineStabilizer stabilizer(config);

    const auto initial = stabilizer.update(makeLine(320.0f), 640);
    assert(initial.detected);
    assert(initial.filtered);
    assert(!initial.held);
    assert(initial.center_offset_px == 0.0f);

    const auto jump1 = stabilizer.update(makeLine(520.0f), 640);
    assert(jump1.detected);
    assert(jump1.held);
    assert(jump1.rejected_jump);
    assert(jump1.raw_detected);
    assert(jump1.raw_center_offset_px == 200.0f);
    assert(jump1.center_offset_px == 0.0f);

    const auto jump2 = stabilizer.update(makeLine(520.0f), 640);
    assert(jump2.detected);
    assert(jump2.held);
    assert(jump2.rejected_jump);
    assert(jump2.center_offset_px == 0.0f);

    const auto reacquired = stabilizer.update(makeLine(520.0f), 640);
    assert(reacquired.detected);
    assert(!reacquired.held);
    assert(!reacquired.rejected_jump);
    assert(reacquired.center_offset_px > 0.0f);
    assert(reacquired.center_offset_px < 200.0f);

    const onboard::vision::LineDetection missing;
    const auto held_missing = stabilizer.update(missing, 640);
    assert(held_missing.detected);
    assert(held_missing.held);
    assert(!held_missing.raw_detected);

    return 0;
}
