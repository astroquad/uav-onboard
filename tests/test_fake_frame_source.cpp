// Tests are assert-based: keep assert() active even in Release
// builds (CMake adds -DNDEBUG there, which silently no-ops all checks).
#undef NDEBUG

#include "vision/FakeFrameSource.hpp"
#include "vision/VisionProcessor.hpp"

#include <cassert>
#include <cmath>

int main()
{
    onboard::common::VisionConfig config;
    config.camera.width = 320;
    config.camera.height = 240;
    config.line.mode = "light_on_dark";
    config.line.mask_strategy = "threshold";
    config.line.threshold = 128;
    config.line.process_width = 320;
    config.line.min_area_px = 20;
    config.line.morph_open_kernel = 1;
    config.line.morph_close_kernel = 3;
    config.line.morph_dilate_kernel = 1;
    config.line.confidence_min = 0.20;
    config.line.filter_enabled = false;
    config.source.fake_line_offset_px = 24.0;

    onboard::vision::FakeFrameSource source;
    assert(source.open(onboard::vision::FrameSourceOptions {config}));

    onboard::vision::Frame frame;
    assert(source.read(frame));
    assert(frame.frame_id == 1);
    assert(frame.width == 320);
    assert(frame.height == 240);
    assert(!frame.image_bgr.empty());

    onboard::vision::VisionProcessor processor(onboard::vision::VisionProcessorOptions {
        config,
        false,
        true,
    });
    const auto output = processor.process(
        frame.image_bgr,
        onboard::vision::VisionFrameMetadata {
            frame.frame_id,
            frame.timestamp_ms,
            frame.width,
            frame.height,
        });
    assert(output.result.line.detected);
    assert(output.result.line.center_offset_px > 8.0f);

    return 0;
}
