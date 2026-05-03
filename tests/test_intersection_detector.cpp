#include "vision/IntersectionDetector.hpp"
#include "vision/LineDetector.hpp"
#include "vision/LineMaskBuilder.hpp"

#include <opencv2/imgproc.hpp>

#include <cassert>
#include <cmath>
#include <string>
#include <utility>

namespace {

onboard::common::LineConfig makeConfig(std::string mode)
{
    onboard::common::LineConfig config;
    config.mode = std::move(mode);
    config.mask_strategy = "threshold";
    config.threshold = 128;
    config.process_width = 320;
    config.min_area_px = 20;
    config.morph_open_kernel = 1;
    config.morph_close_kernel = 3;
    config.morph_dilate_kernel = 1;
    config.intersection_threshold = 0.45;
    config.filter_min_confidence = 0.2;
    return config;
}

cv::Mat makeCanvas(bool dark_line)
{
    const cv::Scalar background = dark_line ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 0);
    return cv::Mat(240, 320, CV_8UC3, background);
}

void drawSegment(cv::Mat& image, cv::Point start, cv::Point end, bool dark_line)
{
    const cv::Scalar color = dark_line ? cv::Scalar(0, 0, 0) : cv::Scalar(255, 255, 255);
    cv::line(image, start, end, color, 28, cv::LINE_8);
}

onboard::vision::IntersectionDetection detect(cv::Mat image, const onboard::common::LineConfig& config)
{
    onboard::vision::LineMaskBuilder builder(config);
    onboard::vision::IntersectionDetector detector(config);
    const auto masks = builder.build(image);
    return detector.detect(masks, {});
}

void assertType(
    cv::Mat image,
    const onboard::common::LineConfig& config,
    onboard::vision::IntersectionType expected)
{
    const auto detection = detect(std::move(image), config);
    assert(detection.valid);
    assert(detection.type == expected);
}

void assertDarkLineWorksWithWhiteFillStrategy()
{
    auto config = makeConfig("dark_on_light");
    config.mask_strategy = "white_fill";
    config.dark_v_max = 100;
    config.dark_fill_close_kernel = 13;
    config.dark_fill_dilate_kernel = 3;
    config.min_line_width_px = 12;
    config.dark_max_line_width_ratio = 0.34;
    config.confidence_min = 0.20;

    cv::Mat image(240, 320, CV_8UC3, cv::Scalar(230, 230, 230));
    cv::rectangle(image, {0, 0}, {319, 239}, cv::Scalar(255, 255, 255), 6);
    cv::line(image, {160, 20}, {160, 220}, cv::Scalar(0, 0, 0), 72, cv::LINE_8);

    onboard::vision::LineMaskBuilder builder(config);
    onboard::vision::LineDetector detector(config);
    const auto masks = builder.build(image);
    const auto line = detector.detect(masks);
    assert(line.detected);
    assert(std::abs(line.tracking_point_px.x - 160.0f) < 18.0f);
    assert(std::abs(line.center_offset_px) < 18.0f);
    assert(line.contour_px.size() >= 2);
}

void assertLightLineWhiteFillStillWorks()
{
    auto config = makeConfig("light_on_dark");
    config.mask_strategy = "white_fill";
    config.white_v_min = 145;
    config.white_s_max = 90;
    config.fill_close_kernel = 11;
    config.fill_dilate_kernel = 3;
    config.min_line_width_px = 12;
    config.max_line_width_ratio = 0.22;
    config.confidence_min = 0.20;

    cv::Mat image(240, 320, CV_8UC3, cv::Scalar(20, 20, 20));
    cv::line(image, {160, 20}, {160, 220}, cv::Scalar(245, 245, 245), 44, cv::LINE_8);

    onboard::vision::LineMaskBuilder builder(config);
    onboard::vision::LineDetector detector(config);
    const auto masks = builder.build(image);
    const auto line = detector.detect(masks);
    assert(line.detected);
    assert(std::abs(line.tracking_point_px.x - 160.0f) < 18.0f);
    assert(std::abs(line.center_offset_px) < 18.0f);
}

} // namespace

int main()
{
    const auto light_config = makeConfig("light_on_dark");

    {
        auto image = makeCanvas(false);
        drawSegment(image, {160, 20}, {160, 220}, false);
        drawSegment(image, {40, 120}, {280, 120}, false);
        assertType(std::move(image), light_config, onboard::vision::IntersectionType::Cross);
    }

    {
        auto image = makeCanvas(false);
        drawSegment(image, {160, 20}, {160, 120}, false);
        drawSegment(image, {40, 120}, {280, 120}, false);
        assertType(std::move(image), light_config, onboard::vision::IntersectionType::T);
    }

    {
        auto image = makeCanvas(false);
        drawSegment(image, {160, 120}, {280, 120}, false);
        drawSegment(image, {160, 120}, {160, 220}, false);
        assertType(std::move(image), light_config, onboard::vision::IntersectionType::L);
    }

    {
        auto image = makeCanvas(false);
        drawSegment(image, {40, 120}, {280, 120}, false);
        assertType(std::move(image), light_config, onboard::vision::IntersectionType::Straight);
    }

    {
        const auto dark_config = makeConfig("dark_on_light");
        auto image = makeCanvas(true);
        drawSegment(image, {160, 20}, {160, 220}, true);
        drawSegment(image, {40, 120}, {280, 120}, true);
        assertType(std::move(image), dark_config, onboard::vision::IntersectionType::Cross);
    }

    assertDarkLineWorksWithWhiteFillStrategy();
    assertLightLineWhiteFillStillWorks();

    return 0;
}
