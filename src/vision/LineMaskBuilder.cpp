#include "vision/LineMaskBuilder.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace onboard::vision {
namespace {

double clampRatio(double value, double fallback)
{
    if (!std::isfinite(value)) {
        return fallback;
    }
    return std::clamp(value, 0.0, 1.0);
}

int oddKernelSize(int value)
{
    if (value <= 1) {
        return 1;
    }
    return value % 2 == 0 ? value + 1 : value;
}

int boundedOddKernelSize(int value, const cv::Size& size)
{
    const int max_size = std::max(1, std::min(size.width, size.height));
    int kernel_size = std::min(oddKernelSize(value), max_size);
    if (kernel_size % 2 == 0) {
        --kernel_size;
    }
    return std::max(1, kernel_size);
}

cv::Mat thresholdMask(const cv::Mat& gray, bool light_line, int configured_threshold)
{
    cv::Mat mask;
    const int base_type = light_line ? cv::THRESH_BINARY : cv::THRESH_BINARY_INV;
    if (configured_threshold <= 0) {
        cv::threshold(gray, mask, 0, 255, base_type | cv::THRESH_OTSU);
    } else {
        cv::threshold(
            gray,
            mask,
            std::clamp(configured_threshold, 0, 255),
            255,
            base_type);
    }
    return mask;
}

cv::Mat localContrastMask(
    const cv::Mat& gray,
    bool light_line,
    const common::LineConfig& config)
{
    const int blur_kernel = boundedOddKernelSize(config.local_contrast_blur, gray.size());
    if (blur_kernel <= 1) {
        return thresholdMask(gray, light_line, config.threshold);
    }

    cv::Mat background;
    cv::GaussianBlur(
        gray,
        background,
        cv::Size(blur_kernel, blur_kernel),
        0.0,
        0.0,
        cv::BORDER_REPLICATE);

    cv::Mat contrast;
    if (light_line) {
        cv::subtract(gray, background, contrast);
    } else {
        cv::subtract(background, gray, contrast);
    }

    cv::Mat mask;
    if (config.local_contrast_threshold <= 0) {
        cv::threshold(contrast, mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    } else {
        cv::threshold(
            contrast,
            mask,
            std::clamp(config.local_contrast_threshold, 0, 255),
            255,
            cv::THRESH_BINARY);
    }
    return mask;
}

cv::Mat absoluteWhiteMask(
    const cv::Mat& work_image,
    const cv::Mat& gray,
    const common::LineConfig& config)
{
    if (work_image.channels() < 3) {
        cv::Mat mask;
        cv::threshold(
            gray,
            mask,
            std::clamp(config.white_v_min, 0, 255),
            255,
            cv::THRESH_BINARY);
        return mask;
    }

    cv::Mat hsv;
    cv::cvtColor(work_image, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(
        hsv,
        cv::Scalar(0, 0, std::clamp(config.white_v_min, 0, 255)),
        cv::Scalar(180, std::clamp(config.white_s_max, 0, 255), 255),
        mask);
    return mask;
}

cv::Mat absoluteDarkMask(const cv::Mat& work_image, const cv::Mat& gray, const common::LineConfig& config)
{
    if (work_image.channels() < 3) {
        cv::Mat mask;
        cv::threshold(
            gray,
            mask,
            std::clamp(config.dark_v_max, 0, 255),
            255,
            cv::THRESH_BINARY_INV);
        return mask;
    }

    cv::Mat hsv;
    cv::cvtColor(work_image, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(
        hsv,
        cv::Scalar(0, 0, 0),
        cv::Scalar(180, 255, std::clamp(config.dark_v_max, 0, 255)),
        mask);
    return mask;
}

void applyMorphology(cv::Mat& mask, int morph_type, int morph_kernel)
{
    const int kernel_size = oddKernelSize(morph_kernel);
    if (kernel_size <= 1) {
        return;
    }

    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(kernel_size, kernel_size));
    if (morph_type == cv::MORPH_DILATE) {
        cv::dilate(mask, mask, kernel);
    } else {
        cv::morphologyEx(mask, mask, morph_type, kernel);
    }
}

void fillMask(cv::Mat& mask, const common::LineConfig& config, const LineMaskGeometry& geometry)
{
    applyMorphology(mask, cv::MORPH_CLOSE, scaledLineKernelSize(config.fill_close_kernel, geometry));
    applyMorphology(mask, cv::MORPH_DILATE, scaledLineKernelSize(config.fill_dilate_kernel, geometry));
}

void fillDarkMask(cv::Mat& mask, const common::LineConfig& config, const LineMaskGeometry& geometry)
{
    applyMorphology(mask, cv::MORPH_CLOSE, scaledLineKernelSize(config.dark_fill_close_kernel, geometry));
    applyMorphology(mask, cv::MORPH_DILATE, scaledLineKernelSize(config.dark_fill_dilate_kernel, geometry));
}

void clearThinBorderArtifacts(cv::Mat& mask)
{
    if (mask.empty()) {
        return;
    }
    const int border = std::max(2, std::min(mask.cols, mask.rows) / 90);
    const int bottom_top = std::max(0, mask.rows - border);
    mask(cv::Rect(0, bottom_top, mask.cols, mask.rows - bottom_top)).setTo(0);
    mask(cv::Rect(0, 0, std::min(border, mask.cols), mask.rows)).setTo(0);
    const int right_left = std::max(0, mask.cols - border);
    mask(cv::Rect(right_left, 0, mask.cols - right_left, mask.rows)).setTo(0);
}

int configuredMorphOpenKernel(const common::LineConfig& config)
{
    return config.morph_open_kernel > 0
        ? config.morph_open_kernel
        : config.morph_kernel;
}

int configuredMorphCloseKernel(const common::LineConfig& config)
{
    return config.morph_close_kernel > 0
        ? config.morph_close_kernel
        : config.morph_kernel;
}

void cleanMask(cv::Mat& mask, const common::LineConfig& config, const LineMaskGeometry& geometry)
{
    applyMorphology(mask, cv::MORPH_OPEN, scaledLineKernelSize(configuredMorphOpenKernel(config), geometry));
    applyMorphology(mask, cv::MORPH_CLOSE, scaledLineKernelSize(configuredMorphCloseKernel(config), geometry));
    applyMorphology(mask, cv::MORPH_DILATE, scaledLineKernelSize(config.morph_dilate_kernel, geometry));
}

bool usesLocalContrast(const std::string& strategy)
{
    return strategy == "local_contrast" ||
           strategy == "white_local_contrast" ||
           strategy == "light_on_dark_v2" ||
           strategy == "local_contrast_fill";
}

bool usesWhiteFill(const std::string& strategy)
{
    return strategy == "white_fill" ||
           strategy == "local_contrast_fill";
}

bool usesDarkFill(const std::string& strategy)
{
    return strategy == "white_fill" ||
           strategy == "dark_fill" ||
           strategy == "local_contrast_fill";
}

} // namespace

LineMaskBuilder::LineMaskBuilder(const common::LineConfig& config)
    : config_(config)
{
}

LineMaskFrame LineMaskBuilder::build(const cv::Mat& image) const
{
    LineMaskFrame output;
    if (!config_.enabled || image.empty() || image.cols <= 0 || image.rows <= 0) {
        return output;
    }

    const int width = image.cols;
    const int height = image.rows;
    const int roi_top = std::clamp(
        static_cast<int>(std::lround(height * clampRatio(config_.roi_top_ratio, 0.08))),
        0,
        std::max(0, height - 1));
    const cv::Rect source_roi(0, roi_top, width, height - roi_top);
    if (source_roi.width <= 0 || source_roi.height <= 0) {
        return output;
    }

    auto& geometry = output.geometry;
    geometry.source_width = width;
    geometry.source_height = height;
    geometry.roi_top = roi_top;
    geometry.roi_height = source_roi.height;

    const int requested_width = config_.process_width > 0 ? config_.process_width : width;
    geometry.work_width = std::clamp(requested_width, 1, width);
    if (geometry.work_width == width) {
        geometry.work_height = source_roi.height;
    } else {
        const double scale = geometry.work_width / static_cast<double>(width);
        geometry.work_height =
            std::max(1, static_cast<int>(std::lround(source_roi.height * scale)));
    }
    geometry.scale_x = width / static_cast<double>(geometry.work_width);
    geometry.scale_y = source_roi.height / static_cast<double>(geometry.work_height);

    cv::Mat roi_image = image(source_roi);
    cv::Mat work_image;
    if (geometry.work_width == source_roi.width && geometry.work_height == source_roi.height) {
        work_image = roi_image;
    } else {
        cv::resize(
            roi_image,
            work_image,
            cv::Size(geometry.work_width, geometry.work_height),
            0.0,
            0.0,
            cv::INTER_AREA);
    }

    cv::Mat gray;
    if (work_image.channels() == 1) {
        gray = work_image;
    } else {
        cv::cvtColor(work_image, gray, cv::COLOR_BGR2GRAY);
    }

    const bool use_local_contrast = usesLocalContrast(config_.mask_strategy);
    const bool use_white_fill = usesWhiteFill(config_.mask_strategy);
    const bool use_dark_fill = usesDarkFill(config_.mask_strategy);
    const auto make_mask = [&](bool light_line) {
        if (use_white_fill && light_line) {
            cv::Mat mask = absoluteWhiteMask(work_image, gray, config_);
            if (use_local_contrast && light_line) {
                cv::Mat contrast = localContrastMask(gray, true, config_);
                cv::bitwise_or(mask, contrast, mask);
            }
            fillMask(mask, config_, geometry);
            clearThinBorderArtifacts(mask);
            return mask;
        }
        if (use_dark_fill && !light_line) {
            cv::Mat mask = absoluteDarkMask(work_image, gray, config_);
            if (use_local_contrast) {
                cv::Mat contrast = localContrastMask(gray, false, config_);
                cv::bitwise_or(mask, contrast, mask);
            }
            fillDarkMask(mask, config_, geometry);
            clearThinBorderArtifacts(mask);
            return mask;
        }
        return use_local_contrast
            ? localContrastMask(gray, light_line, config_)
            : thresholdMask(gray, light_line, config_.threshold);
    };

    const auto add_mask = [&](bool light_line) {
        LineMaskCandidate candidate;
        candidate.mask = make_mask(light_line);
        candidate.polarity = light_line ? LinePolarity::LightOnDark : LinePolarity::DarkOnLight;
        cleanMask(candidate.mask, config_, geometry);
        output.masks.push_back(std::move(candidate));
    };

    if (config_.mode == "auto") {
        add_mask(true);
        add_mask(false);
    } else if (config_.mode == "dark_on_light") {
        add_mask(false);
    } else {
        add_mask(true);
    }

    return output;
}

cv::Point toSourcePoint(const cv::Point& point, const LineMaskGeometry& geometry)
{
    return {
        static_cast<int>(std::lround(point.x * geometry.scale_x)),
        geometry.roi_top + static_cast<int>(std::lround(point.y * geometry.scale_y)),
    };
}

cv::Point toWorkPoint(float source_x, float source_y, const LineMaskGeometry& geometry)
{
    return {
        std::clamp(
            static_cast<int>(std::lround(source_x / geometry.scale_x)),
            0,
            std::max(0, geometry.work_width - 1)),
        std::clamp(
            static_cast<int>(std::lround((source_y - geometry.roi_top) / geometry.scale_y)),
            0,
            std::max(0, geometry.work_height - 1)),
    };
}

int scaledLineKernelSize(int configured_kernel, const LineMaskGeometry& geometry)
{
    const double scale = std::max(1.0, std::max(geometry.scale_x, geometry.scale_y));
    return oddKernelSize(static_cast<int>(std::lround(configured_kernel / scale)));
}

} // namespace onboard::vision
