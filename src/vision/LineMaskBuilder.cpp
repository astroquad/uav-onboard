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
    const cv::Mat& hsv,
    const common::LineConfig& config)
{
    if (work_image.channels() < 3 || hsv.empty()) {
        cv::Mat mask;
        cv::threshold(
            gray,
            mask,
            std::clamp(config.white_v_min, 0, 255),
            255,
            cv::THRESH_BINARY);
        return mask;
    }

    cv::Mat mask;
    cv::inRange(
        hsv,
        cv::Scalar(0, 0, std::clamp(config.white_v_min, 0, 255)),
        cv::Scalar(180, std::clamp(config.white_s_max, 0, 255), 255),
        mask);
    return mask;
}

cv::Mat absoluteDarkMask(
    const cv::Mat& work_image,
    const cv::Mat& gray,
    const cv::Mat& hsv,
    const common::LineConfig& config)
{
    if (work_image.channels() < 3 || hsv.empty()) {
        cv::Mat mask;
        cv::threshold(
            gray,
            mask,
            std::clamp(config.dark_v_max, 0, 255),
            255,
            cv::THRESH_BINARY_INV);
        return mask;
    }

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

cv::Point markerCornerToWorkPoint(const Point2f& point, const LineMaskGeometry& geometry)
{
    return {
        std::clamp(
            static_cast<int>(std::lround(point.x / geometry.scale_x)),
            0,
            std::max(0, geometry.work_width - 1)),
        std::clamp(
            static_cast<int>(std::lround((point.y - geometry.roi_top) / geometry.scale_y)),
            0,
            std::max(0, geometry.work_height - 1)),
    };
}

void addMarkerPolygonOccluders(
    cv::Mat& occlusion_mask,
    const std::vector<MarkerObservation>& markers,
    const LineMaskGeometry& geometry,
    double occlusion_scale)
{
    if (markers.empty() || occlusion_mask.empty()) {
        return;
    }

    const double scale = std::max(1.0, occlusion_scale);
    for (const auto& marker : markers) {
        if (marker.id < 0) {
            continue;
        }
        std::vector<cv::Point> polygon;
        polygon.reserve(marker.corners_px.size());
        for (const auto& corner : marker.corners_px) {
            polygon.push_back(markerCornerToWorkPoint(corner, geometry));
        }
        // Expand the polygon about its centroid: the marker sits on a white
        // pad / quiet zone larger than the marker itself, and the un-erased
        // pad ring otherwise survives as false line/intersection contours.
        if (scale > 1.0 && !polygon.empty()) {
            double cx = 0.0;
            double cy = 0.0;
            for (const auto& p : polygon) {
                cx += p.x;
                cy += p.y;
            }
            cx /= static_cast<double>(polygon.size());
            cy /= static_cast<double>(polygon.size());
            for (auto& p : polygon) {
                p.x = static_cast<int>(std::lround(cx + (p.x - cx) * scale));
                p.y = static_cast<int>(std::lround(cy + (p.y - cy) * scale));
            }
        }
        const std::vector<std::vector<cv::Point>> polygons {polygon};
        cv::fillPoly(occlusion_mask, polygons, cv::Scalar(255), cv::LINE_8);
    }
}

void addMarkerLikeSquareOccluders(
    cv::Mat& occlusion_mask,
    const cv::Mat& work_image,
    const cv::Mat& gray,
    const cv::Mat& hsv)
{
    if (occlusion_mask.empty() || work_image.empty() || gray.empty()) {
        return;
    }

    cv::Mat dark;
    if (work_image.channels() >= 3 && !hsv.empty()) {
        cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 95), dark);
    } else {
        cv::threshold(gray, dark, 95, 255, cv::THRESH_BINARY_INV);
    }

    const cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(dark, dark, cv::MORPH_CLOSE, close_kernel);

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int components = cv::connectedComponentsWithStats(dark, labels, stats, centroids, 8);
    const double frame_area = static_cast<double>(std::max(1, work_image.cols * work_image.rows));
    for (int label = 1; label < components; ++label) {
        const int x = stats.at<int>(label, cv::CC_STAT_LEFT);
        const int y = stats.at<int>(label, cv::CC_STAT_TOP);
        const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
        const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (width <= 0 || height <= 0) {
            continue;
        }

        const double rect_ratio = width / static_cast<double>(height);
        const double area_ratio = area / frame_area;
        if (rect_ratio < 0.65 || rect_ratio > 1.55 ||
            area_ratio < 0.004 || area_ratio > 0.18) {
            continue;
        }

        const cv::Rect rect = cv::Rect(x, y, width, height) &
            cv::Rect(0, 0, work_image.cols, work_image.rows);
        if (rect.empty()) {
            continue;
        }

        const cv::Mat rect_gray = gray(rect);
        cv::Mat bright;
        cv::threshold(rect_gray, bright, 180, 255, cv::THRESH_BINARY);
        const double bright_ratio = cv::countNonZero(bright) /
            static_cast<double>(std::max(1, rect.area()));
        if (bright_ratio < 0.05 || bright_ratio > 0.65) {
            continue;
        }

        const int pad = std::max(2, std::min(width, height) / 8);
        const cv::Rect expanded = cv::Rect(
            x - pad,
            y - pad,
            width + pad * 2,
            height + pad * 2) &
            cv::Rect(0, 0, work_image.cols, work_image.rows);
        if (!expanded.empty()) {
            occlusion_mask(expanded).setTo(255);
        }
    }
}

cv::Mat buildMarkerOcclusionMask(
    const cv::Mat& work_image,
    const cv::Mat& gray,
    const cv::Mat& hsv,
    const LineMaskGeometry& geometry,
    const std::vector<MarkerObservation>& markers,
    bool detect_marker_like_occluders,
    double occlusion_scale)
{
    cv::Mat occlusion_mask(
        geometry.work_height,
        geometry.work_width,
        CV_8UC1,
        cv::Scalar(0));

    addMarkerPolygonOccluders(occlusion_mask, markers, geometry, occlusion_scale);
    if (detect_marker_like_occluders) {
        addMarkerLikeSquareOccluders(occlusion_mask, work_image, gray, hsv);
    }

    if (cv::countNonZero(occlusion_mask) == 0) {
        return {};
    }

    const int dilate_kernel = std::clamp(
        scaledLineKernelSize(16, geometry),
        3,
        15);
    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(oddKernelSize(dilate_kernel), oddKernelSize(dilate_kernel)));
    cv::dilate(occlusion_mask, occlusion_mask, kernel);
    return occlusion_mask;
}

void eraseMarkerOcclusion(cv::Mat& mask, const cv::Mat& occlusion_mask)
{
    if (!mask.empty() && !occlusion_mask.empty() && mask.size() == occlusion_mask.size()) {
        mask.setTo(0, occlusion_mask);
    }
}

} // namespace

LineMaskBuilder::LineMaskBuilder(const common::LineConfig& config)
    : config_(config)
{
}

LineMaskFrame LineMaskBuilder::build(const cv::Mat& image) const
{
    return build(image, {}, false);
}

LineMaskFrame LineMaskBuilder::build(
    const cv::Mat& image,
    const std::vector<MarkerObservation>& markers,
    bool detect_marker_like_occluders) const
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
    // Polygon occlusion from actually-detected markers applies in every
    // polarity mode (the dark marker body pollutes dark masks exactly like
    // the white pad pollutes light masks). Only the heuristic marker-like
    // candidate detector stays disabled in dark_on_light, where dark line
    // junctions could be misread as marker squares.
    const bool marker_candidates_active =
        detect_marker_like_occluders &&
        config_.marker_mask_detect_candidates &&
        config_.mode != "dark_on_light";
    const bool marker_mask_active =
        config_.marker_mask_enabled &&
        (!markers.empty() || marker_candidates_active);

    // Lazily compute HSV once and share between absolute-color masks and the
    // marker-like dark detector. In auto mode plus marker masking this saved
    // up to two redundant BGR->HSV conversions per frame.
    cv::Mat hsv;
    bool hsv_built = false;
    const auto ensure_hsv = [&]() -> const cv::Mat& {
        if (!hsv_built && work_image.channels() >= 3) {
            cv::cvtColor(work_image, hsv, cv::COLOR_BGR2HSV);
            hsv_built = true;
        }
        return hsv;
    };

    if (marker_mask_active) {
        output.marker_occlusion_mask = buildMarkerOcclusionMask(
            work_image,
            gray,
            ensure_hsv(),
            geometry,
            markers,
            marker_candidates_active,
            config_.marker_occlusion_scale);
        if (!output.marker_occlusion_mask.empty()) {
            cv::Mat labels;
            output.marker_occlusion_count = cv::connectedComponents(
                output.marker_occlusion_mask,
                labels,
                8) - 1;
        }
    }
    const auto make_mask = [&](bool light_line) {
        if (use_white_fill && light_line) {
            cv::Mat mask = absoluteWhiteMask(work_image, gray, ensure_hsv(), config_);
            if (use_local_contrast && light_line) {
                cv::Mat contrast = localContrastMask(gray, true, config_);
                cv::bitwise_or(mask, contrast, mask);
            }
            eraseMarkerOcclusion(mask, output.marker_occlusion_mask);
            fillMask(mask, config_, geometry);
            clearThinBorderArtifacts(mask);
            return mask;
        }
        if (use_dark_fill && !light_line) {
            cv::Mat mask = absoluteDarkMask(work_image, gray, ensure_hsv(), config_);
            if (use_local_contrast) {
                cv::Mat contrast = localContrastMask(gray, false, config_);
                cv::bitwise_or(mask, contrast, mask);
            }
            eraseMarkerOcclusion(mask, output.marker_occlusion_mask);
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
        // Both polarities: the dark marker body is dark-mask content just as
        // the white pad is light-mask content.
        eraseMarkerOcclusion(candidate.mask, output.marker_occlusion_mask);
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
