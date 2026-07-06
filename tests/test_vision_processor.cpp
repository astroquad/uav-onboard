// Tests are assert-based: keep assert() active even in Release
// builds (CMake adds -DNDEBUG there, which silently no-ops all checks).
#undef NDEBUG

#include "vision/VisionProcessor.hpp"

#include <opencv2/imgproc.hpp>

#include <cassert>
#include <cmath>

namespace {

onboard::common::VisionConfig makeConfig()
{
    onboard::common::VisionConfig config;
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
    config.line.intersection_threshold = 0.45;
    return config;
}

} // namespace

int main()
{
    cv::Mat image(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::line(image, {160, 20}, {160, 220}, cv::Scalar(255, 255, 255), 34, cv::LINE_8);

    onboard::vision::VisionProcessor processor(onboard::vision::VisionProcessorOptions {
        makeConfig(),
        false,
        true,
    });

    const auto output = processor.process(
        image,
        onboard::vision::VisionFrameMetadata {
            42,
            1234,
            image.cols,
            image.rows,
        });

    assert(output.result.frame_seq == 42);
    assert(output.result.timestamp_ms == 1234);
    assert(output.result.width == 320);
    assert(output.result.height == 240);
    assert(output.result.markers.empty());
    assert(output.result.line.detected);
    assert(output.result.line_detected);
    assert(std::abs(output.result.line.tracking_point_px.x - 160.0f) < 18.0f);
    assert(std::abs(output.result.line.center_offset_px) < 18.0f);
    assert(output.metrics.line_latency_ms >= 0.0);

    return 0;
}
