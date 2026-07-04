// Production mask-strategy coverage on MONO (1-channel) frames:
//  1. white_fill degenerates to a fixed gray threshold on mono input and
//     still detects a bright line on textured ground under even lighting.
//  2. Under an illumination gradient the absolute white_fill threshold loses
//     the dim-side line while local_contrast (blur-subtract) keeps it —
//     the reason the real-field strategy is local_contrast.
//  3. Marker occlusion scaling: with a detected marker passed in, the bright
//     pad ring AROUND the marker polygon is erased from the light mask
//     (marker_occlusion_scale), while the grid line stays intact.

#include "vision/LineDetector.hpp"
#include "vision/LineMaskBuilder.hpp"

#include <opencv2/imgproc.hpp>

#include <cassert>
#include <cmath>

namespace {

using onboard::common::LineConfig;
using onboard::vision::LineDetector;
using onboard::vision::LineMaskBuilder;
using onboard::vision::MarkerObservation;

constexpr int kWidth = 320;
constexpr int kHeight = 240;

LineConfig makeConfig(const std::string& strategy)
{
    LineConfig config;
    config.mode = "light_on_dark";
    config.mask_strategy = strategy;
    config.white_v_min = 145;
    config.local_contrast_blur = 31;
    config.local_contrast_threshold = 10;
    // Work resolution == source resolution and no top crop, so mask pixels
    // can be asserted in source coordinates directly.
    config.process_width = kWidth;
    config.roi_top_ratio = 0.0;
    config.min_area_px = 20;
    config.min_line_width_px = 6;
    config.morph_open_kernel = 1;
    config.morph_close_kernel = 3;
    config.morph_dilate_kernel = 1;
    config.confidence_min = 0.20;
    config.filter_enabled = false;
    return config;
}

// Textured "grass" background (deterministic checker, max value 120) with a
// vertical bright line of the given intensity centered at line_x.
cv::Mat makeMonoGrassFrame(int line_x, int line_value)
{
    cv::Mat image(kHeight, kWidth, CV_8UC1);
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            image.at<std::uint8_t>(y, x) =
                static_cast<std::uint8_t>(90 + 30 * ((x / 3 + y / 3) % 2));
        }
    }
    cv::line(image, {line_x, 0}, {line_x, kHeight - 1}, cv::Scalar(line_value), 20, cv::LINE_8);
    return image;
}

void applyHorizontalGradient(cv::Mat& image)
{
    for (int x = 0; x < kWidth; ++x) {
        const double factor = 0.50 + 0.50 * (static_cast<double>(x) / kWidth);
        for (int y = 0; y < kHeight; ++y) {
            image.at<std::uint8_t>(y, x) = static_cast<std::uint8_t>(
                std::lround(image.at<std::uint8_t>(y, x) * factor));
        }
    }
}

int maskCoverageOnLine(const cv::Mat& mask, int line_x)
{
    int hits = 0;
    for (int y = 0; y < mask.rows; ++y) {
        if (mask.at<std::uint8_t>(y, std::min(line_x, mask.cols - 1)) > 0) {
            ++hits;
        }
    }
    return hits;
}

MarkerObservation makeMarker(float cx, float cy, float side)
{
    MarkerObservation marker;
    marker.id = 3;
    const float h = side / 2.0f;
    marker.corners_px = {{{cx - h, cy - h}, {cx + h, cy - h}, {cx + h, cy + h}, {cx - h, cy + h}}};
    marker.center_px = {cx, cy};
    return marker;
}

void testWhiteFillMono()
{
    const cv::Mat image = makeMonoGrassFrame(160, 230);
    LineConfig config = makeConfig("white_fill");
    LineMaskBuilder builder(config);
    const auto frame = builder.build(image);
    assert(!frame.masks.empty());
    assert(maskCoverageOnLine(frame.masks.front().mask, 160) > kHeight / 2);

    LineDetector detector(config);
    const auto line = detector.detect(frame);
    assert(line.detected);
    assert(std::abs(line.tracking_point_px.x - 160.0f) < 18.0f);
}

void testLocalContrastBeatsWhiteFillUnderGradient()
{
    // Line at x=80 sits in the dim half: 230 * (0.50 + 0.50*80/320) = ~143,
    // below the absolute white_v_min=145 threshold.
    cv::Mat image = makeMonoGrassFrame(80, 230);
    applyHorizontalGradient(image);

    LineConfig white_config = makeConfig("white_fill");
    const auto white_frame = LineMaskBuilder(white_config).build(image);
    assert(!white_frame.masks.empty());
    const int white_hits = maskCoverageOnLine(white_frame.masks.front().mask, 80);

    LineConfig contrast_config = makeConfig("local_contrast");
    const auto contrast_frame = LineMaskBuilder(contrast_config).build(image);
    assert(!contrast_frame.masks.empty());
    const int contrast_hits = maskCoverageOnLine(contrast_frame.masks.front().mask, 80);

    assert(white_hits < kHeight / 4);
    assert(contrast_hits > kHeight / 2);

    LineDetector detector(contrast_config);
    const auto line = detector.detect(contrast_frame);
    assert(line.detected);
    assert(std::abs(line.tracking_point_px.x - 80.0f) < 18.0f);
}

void testMarkerPadOcclusionScaling()
{
    // Bright 64px pad (quiet zone) with a 40px marker polygon on top of it,
    // away from the line. Without occlusion scaling the pad ring outside the
    // marker polygon stays in the light mask as a false contour source.
    cv::Mat image = makeMonoGrassFrame(160, 230);
    cv::rectangle(image, {208, 108}, {272, 172}, cv::Scalar(235), cv::FILLED);

    LineConfig config = makeConfig("white_fill");
    config.marker_mask_enabled = true;
    config.marker_occlusion_scale = 1.7;
    const std::vector<MarkerObservation> markers {makeMarker(240.0f, 140.0f, 40.0f)};

    const auto frame = LineMaskBuilder(config).build(image, markers, false);
    assert(!frame.masks.empty());
    assert(frame.marker_occlusion_count >= 1);

    const cv::Mat& mask = frame.masks.front().mask;
    // Pad ring points: inside the 64px pad but outside the 40px marker body.
    assert(mask.at<std::uint8_t>(140, 212) == 0);
    assert(mask.at<std::uint8_t>(140, 268) == 0);
    assert(mask.at<std::uint8_t>(112, 240) == 0);
    assert(mask.at<std::uint8_t>(168, 240) == 0);
    // The line far from the pad must survive.
    assert(maskCoverageOnLine(mask, 160) > kHeight / 2);
}

} // namespace

int main()
{
    testWhiteFillMono();
    testLocalContrastBeatsWhiteFillUnderGradient();
    testMarkerPadOcclusionScaling();
    return 0;
}
