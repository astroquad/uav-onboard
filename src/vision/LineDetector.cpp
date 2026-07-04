#include "vision/LineDetector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace onboard::vision {
namespace {

struct Candidate {
    LineDetection line;
    double score = 0.0;
};

struct WorkGeometry {
    int source_width = 0;
    int source_height = 0;
    int roi_top = 0;
    int roi_height = 0;
    int work_width = 0;
    int work_height = 0;
    double scale_x = 1.0;
    double scale_y = 1.0;
};

struct ProjectionRun {
    int start_x = 0;
    int end_x = 0;
    int weight = 0;
};

double clampRatio(double value, double fallback)
{
    if (!std::isfinite(value)) {
        return fallback;
    }
    return std::clamp(value, 0.0, 1.0);
}


cv::Point toSourcePoint(const cv::Point& point, const WorkGeometry& geometry)
{
    return {
        static_cast<int>(std::lround(point.x * geometry.scale_x)),
        geometry.roi_top + static_cast<int>(std::lround(point.y * geometry.scale_y)),
    };
}

Point2f toTelemetryPoint(const cv::Point& point)
{
    return {
        static_cast<float>(point.x),
        static_cast<float>(point.y),
    };
}

std::vector<cv::Point> toSourceContour(
    const std::vector<cv::Point>& contour,
    const WorkGeometry& geometry)
{
    std::vector<cv::Point> output;
    output.reserve(contour.size());
    for (const auto& point : contour) {
        output.push_back(toSourcePoint(point, geometry));
    }
    return output;
}

std::vector<Point2f> simplifyContour(
    const std::vector<cv::Point>& contour,
    int max_points)
{
    std::vector<cv::Point> simplified;
    cv::approxPolyDP(contour, simplified, 2.0, true);
    if (simplified.empty()) {
        simplified = contour;
    }

    if (max_points > 0 && static_cast<int>(simplified.size()) > max_points) {
        std::vector<cv::Point> sampled;
        sampled.reserve(static_cast<std::size_t>(max_points));
        for (int index = 0; index < max_points; ++index) {
            const std::size_t source_index =
                static_cast<std::size_t>(index) * simplified.size() /
                static_cast<std::size_t>(max_points);
            sampled.push_back(simplified[source_index]);
        }
        simplified = std::move(sampled);
    }

    std::vector<Point2f> output;
    output.reserve(simplified.size());
    for (const auto& point : simplified) {
        output.push_back(toTelemetryPoint(point));
    }
    return output;
}

float lineAngleDeg(const std::vector<cv::Point>& contour)
{
    if (contour.size() < 2) {
        return 0.0f;
    }

    cv::Vec4f line;
    cv::fitLine(contour, line, cv::DIST_L2, 0.0, 0.01, 0.01);
    return static_cast<float>(std::atan2(line[1], line[0]) * 180.0 / CV_PI);
}

float markerApproxSidePx(const MarkerObservation& marker)
{
    float sum = 0.0f;
    for (std::size_t index = 0; index < marker.corners_px.size(); ++index) {
        const auto& lhs = marker.corners_px[index];
        const auto& rhs = marker.corners_px[(index + 1) % marker.corners_px.size()];
        const float dx = lhs.x - rhs.x;
        const float dy = lhs.y - rhs.y;
        sum += std::sqrt(dx * dx + dy * dy);
    }
    return sum / static_cast<float>(marker.corners_px.size());
}


int scaledMinArea(const common::LineConfig& config, const WorkGeometry& geometry)
{
    const double scaled_area =
        config.min_area_px / std::max(1.0, geometry.scale_x * geometry.scale_y);
    return std::max(20, static_cast<int>(std::lround(scaled_area)));
}

int scaledMinLineWidth(const common::LineConfig& config, const WorkGeometry& geometry)
{
    const double scaled_width = config.min_line_width_px / std::max(1.0, geometry.scale_x);
    return std::max(2, static_cast<int>(std::lround(scaled_width)));
}

int scaledRunMergeGap(const common::LineConfig& config, const WorkGeometry& geometry)
{
    const double scaled_gap = config.line_run_merge_gap_px / std::max(1.0, geometry.scale_x);
    return std::max(0, static_cast<int>(std::lround(scaled_gap)));
}

std::vector<std::size_t> sortedCandidateIndexes(
    const std::vector<std::vector<cv::Point>>& contours,
    int min_area,
    int max_candidates)
{
    std::vector<std::pair<double, std::size_t>> ranked;
    ranked.reserve(contours.size());
    for (std::size_t index = 0; index < contours.size(); ++index) {
        const double area = cv::contourArea(contours[index]);
        if (area >= min_area) {
            ranked.push_back({area, index});
        }
    }

    std::sort(
        ranked.begin(),
        ranked.end(),
        [](const auto& lhs, const auto& rhs) {
            return lhs.first > rhs.first;
        });

    if (max_candidates > 0 && static_cast<int>(ranked.size()) > max_candidates) {
        ranked.resize(static_cast<std::size_t>(max_candidates));
    }

    std::vector<std::size_t> indexes;
    indexes.reserve(ranked.size());
    for (const auto& entry : ranked) {
        indexes.push_back(entry.second);
    }
    return indexes;
}

std::vector<ProjectionRun> projectionRuns(
    const cv::Mat& mask,
    const cv::Rect& bounds,
    int y0,
    int y1)
{
    const int band_height = std::max(1, y1 - y0);
    const int min_count = std::max(1, band_height / 4);
    std::vector<int> projection(static_cast<std::size_t>(bounds.width), 0);

    for (int y = y0; y < y1; ++y) {
        const auto* row = mask.ptr<std::uint8_t>(y);
        for (int x = bounds.x; x < bounds.x + bounds.width; ++x) {
            if (row[x] != 0) {
                ++projection[static_cast<std::size_t>(x - bounds.x)];
            }
        }
    }

    std::vector<ProjectionRun> runs;
    int start = -1;
    int weight = 0;
    for (int i = 0; i < bounds.width; ++i) {
        const int count = projection[static_cast<std::size_t>(i)];
        if (count >= min_count) {
            if (start < 0) {
                start = i;
                weight = 0;
            }
            weight += count;
        } else if (start >= 0) {
            runs.push_back({bounds.x + start, bounds.x + i - 1, weight});
            start = -1;
            weight = 0;
        }
    }
    if (start >= 0) {
        runs.push_back({bounds.x + start, bounds.x + bounds.width - 1, weight});
    }

    return runs;
}

std::optional<ProjectionRun> bestProjectionRun(
    const std::vector<ProjectionRun>& runs,
    int band_height,
    int min_line_width,
    int max_line_width,
    int max_merge_gap,
    int work_width)
{
    if (runs.empty()) {
        return std::nullopt;
    }

    std::optional<ProjectionRun> best;
    double best_score = -1.0;
    const double half_width = std::max(1.0, work_width / 2.0);

    for (std::size_t start_index = 0; start_index < runs.size(); ++start_index) {
        int weight = 0;
        int active_columns = 0;
        int previous_end_x = runs[start_index].start_x - 1;
        for (std::size_t end_index = start_index; end_index < runs.size(); ++end_index) {
            const int gap = runs[end_index].start_x - previous_end_x - 1;
            if (end_index > start_index && gap > max_merge_gap) {
                break;
            }

            const int start_x = runs[start_index].start_x;
            const int end_x = runs[end_index].end_x;
            const int width = end_x - start_x + 1;
            if (width > max_line_width) {
                break;
            }

            weight += runs[end_index].weight;
            active_columns += runs[end_index].end_x - runs[end_index].start_x + 1;
            previous_end_x = runs[end_index].end_x;
            if (width < min_line_width) {
                continue;
            }

            const double center = (start_x + end_x) / 2.0;
            const double center_score = 1.0 - std::clamp(
                std::abs(center - work_width / 2.0) / half_width,
                0.0,
                1.0);
            const double width_score = 1.0 - std::clamp(
                std::abs(width - (min_line_width + max_line_width) / 3.0) /
                    std::max(1.0, max_line_width - static_cast<double>(min_line_width)),
                0.0,
                1.0);
            const double density_score = std::clamp(
                weight / std::max(1.0, width * static_cast<double>(band_height)),
                0.0,
                1.0);
            const double coverage_score = std::clamp(
                active_columns / static_cast<double>(std::max(1, width)),
                0.0,
                1.0);
            const double score =
                0.35 * center_score +
                0.30 * width_score +
                0.20 * density_score +
                0.15 * coverage_score;

            if (score > best_score) {
                best = ProjectionRun {start_x, end_x, weight};
                best_score = score;
            }
        }
    }

    return best;
}

std::optional<Candidate> evaluateContour(
    const std::vector<cv::Point>& work_contour,
    const cv::Mat& mask,
    LinePolarity polarity,
    const common::LineConfig& config,
    const WorkGeometry& geometry,
    int work_lookahead_y)
{
    const double work_area = cv::contourArea(work_contour);
    if (work_area < scaledMinArea(config, geometry)) {
        return std::nullopt;
    }

    const cv::Rect bounds = cv::boundingRect(work_contour);
    const double band_ratio = clampRatio(config.lookahead_band_ratio, 0.06);
    const int band_height = std::max(
        3,
        static_cast<int>(std::lround(geometry.work_height * band_ratio)));
    const int band_y0 = std::clamp(
        work_lookahead_y - band_height / 2,
        0,
        std::max(0, geometry.work_height - 1));
    const int band_y1 = std::clamp(
        band_y0 + band_height,
        band_y0 + 1,
        geometry.work_height);
    const int scan_y0 = std::max(band_y0, bounds.y);
    const int scan_y1 = std::min(band_y1, bounds.y + bounds.height);

    const int min_line_width = scaledMinLineWidth(config, geometry);
    const double max_line_width_ratio = polarity == LinePolarity::DarkOnLight
        ? clampRatio(config.dark_max_line_width_ratio, 0.34)
        : clampRatio(config.max_line_width_ratio, 0.22);
    const int max_line_width = std::max(
        min_line_width + 1,
        static_cast<int>(std::lround(
            geometry.work_width * max_line_width_ratio)));
    std::optional<ProjectionRun> lookahead_run;
    int lookahead_scan_height = 0;
    if (scan_y0 < scan_y1) {
        const auto runs = projectionRuns(mask, bounds, scan_y0, scan_y1);
        lookahead_scan_height = scan_y1 - scan_y0;
        lookahead_run = bestProjectionRun(
            runs,
            lookahead_scan_height,
            min_line_width,
            max_line_width,
            scaledRunMergeGap(config, geometry),
            geometry.work_width);
    }

    const int anchor_y = std::clamp(
        static_cast<int>(std::lround(geometry.work_height * 0.74)),
        0,
        geometry.work_height - 1);
    const int anchor_y0 = std::clamp(
        anchor_y - band_height / 2,
        0,
        std::max(0, geometry.work_height - 1));
    const int anchor_y1 = std::clamp(
        anchor_y0 + band_height,
        anchor_y0 + 1,
        geometry.work_height);
    const int anchor_scan_y0 = std::max(anchor_y0, bounds.y);
    const int anchor_scan_y1 = std::min(anchor_y1, bounds.y + bounds.height);
    std::optional<ProjectionRun> anchor_run;
    int anchor_scan_height = 0;
    if (anchor_scan_y0 < anchor_scan_y1) {
        const auto anchor_runs = projectionRuns(mask, bounds, anchor_scan_y0, anchor_scan_y1);
        anchor_scan_height = anchor_scan_y1 - anchor_scan_y0;
        anchor_run = bestProjectionRun(
            anchor_runs,
            anchor_scan_height,
            min_line_width,
            max_line_width,
            scaledRunMergeGap(config, geometry),
            geometry.work_width);
    }
    const auto selected_run = anchor_run ? anchor_run : lookahead_run;
    const int selected_scan_height = anchor_run ? anchor_scan_height : lookahead_scan_height;
    if (!selected_run) {
        return std::nullopt;
    }

    const int work_line_width = selected_run->end_x - selected_run->start_x + 1;
    if (work_line_width < min_line_width) {
        return std::nullopt;
    }
    const double work_tracking_x =
        (selected_run->start_x + selected_run->end_x) / 2.0;
    const double source_tracking_x = work_tracking_x * geometry.scale_x;
    const double source_lookahead_y =
        geometry.roi_top + work_lookahead_y * geometry.scale_y;

    const auto source_contour = toSourceContour(work_contour, geometry);
    const cv::Rect source_bounds = cv::boundingRect(source_contour);
    const int long_span = std::max(source_bounds.width, source_bounds.height);
    const double span_score = std::clamp(
        long_span / static_cast<double>(std::max(1, geometry.roi_height)),
        0.0,
        1.0);

    const double source_area = work_area * geometry.scale_x * geometry.scale_y;
    const double roi_area = static_cast<double>(geometry.source_width) * geometry.roi_height;
    const double area_score = roi_area > 0.0
        ? std::clamp(source_area / (roi_area * 0.08), 0.0, 1.0)
        : 0.0;

    const double width_score = 1.0 - std::clamp(
        (work_line_width - static_cast<double>(min_line_width)) /
            std::max(1.0, max_line_width - static_cast<double>(min_line_width)),
        0.0,
        1.0);
    const double band_density_score = std::clamp(
        selected_run->weight /
            std::max(1.0, work_line_width * static_cast<double>(std::max(1, selected_scan_height))),
        0.0,
        1.0);

    const double center_score = 1.0 - std::clamp(
        std::abs(source_tracking_x - geometry.source_width / 2.0) /
            std::max(1.0, geometry.source_width / 2.0),
        0.0,
        1.0);

    const bool touches_side =
        source_bounds.x <= 2 ||
        (source_bounds.x + source_bounds.width) >= geometry.source_width - 2;
    const double edge_penalty = touches_side ? 0.20 : 0.0;
    const double anchor_score = anchor_run ? 1.0 : 0.0;

    const double score =
        0.12 * span_score +
        0.08 * area_score +
        0.28 * width_score +
        0.30 * center_score +
        0.12 * band_density_score +
        0.10 * anchor_score -
        edge_penalty;
    if (score < config.confidence_min) {
        return std::nullopt;
    }

    LineDetection line;
    line.detected = true;
    line.raw_detected = true;
    line.tracking_point_px.x = static_cast<float>(source_tracking_x);
    line.tracking_point_px.y = static_cast<float>(source_lookahead_y);
    line.raw_tracking_point_px = line.tracking_point_px;
    line.center_offset_px =
        line.tracking_point_px.x - static_cast<float>(geometry.source_width) / 2.0f;
    line.raw_center_offset_px = line.center_offset_px;
    line.angle_deg = lineAngleDeg(source_contour);
    line.raw_angle_deg = line.angle_deg;
    line.confidence = static_cast<float>(std::clamp(score, 0.0, 1.0));
    line.contour_px = simplifyContour(source_contour, config.max_contour_points);
    line.selected_contour_points = static_cast<int>(line.contour_px.size());

    cv::Moments moments = cv::moments(source_contour);
    if (std::abs(moments.m00) > std::numeric_limits<double>::epsilon()) {
        line.centroid_px.x = static_cast<float>(moments.m10 / moments.m00);
        line.centroid_px.y = static_cast<float>(moments.m01 / moments.m00);
    } else {
        line.centroid_px.x = static_cast<float>(source_bounds.x + source_bounds.width / 2.0);
        line.centroid_px.y = static_cast<float>(source_bounds.y + source_bounds.height / 2.0);
    }

    Candidate candidate;
    candidate.line = std::move(line);
    candidate.score = score;
    return candidate;
}

} // namespace

LineDetector::LineDetector(const common::LineConfig& config)
    : config_(config)
{
}

LineDetection LineDetector::detect(const LineMaskFrame& masks) const
{
    const auto& mask_geometry = masks.geometry;
    if (!config_.enabled ||
        mask_geometry.source_width <= 0 ||
        mask_geometry.source_height <= 0 ||
        mask_geometry.work_width <= 0 ||
        mask_geometry.work_height <= 0 ||
        masks.masks.empty()) {
        return {};
    }

    WorkGeometry geometry;
    geometry.source_width = mask_geometry.source_width;
    geometry.source_height = mask_geometry.source_height;
    geometry.roi_top = mask_geometry.roi_top;
    geometry.roi_height = mask_geometry.roi_height;
    geometry.work_width = mask_geometry.work_width;
    geometry.work_height = mask_geometry.work_height;
    geometry.scale_x = mask_geometry.scale_x;
    geometry.scale_y = mask_geometry.scale_y;

    const int source_lookahead_y = std::clamp(
        static_cast<int>(std::lround(
            geometry.source_height * clampRatio(config_.lookahead_y_ratio, 0.55))),
        geometry.roi_top,
        geometry.source_height - 1);
    const int work_lookahead_y = std::clamp(
        static_cast<int>(std::lround((source_lookahead_y - geometry.roi_top) / geometry.scale_y)),
        0,
        geometry.work_height - 1);

    int total_contours = 0;
    int candidates_evaluated = 0;
    std::optional<Candidate> best;
    for (const auto& candidate_mask : masks.masks) {
        if (candidate_mask.mask.empty()) {
            continue;
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(candidate_mask.mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        total_contours += static_cast<int>(contours.size());

        const auto candidate_indexes = sortedCandidateIndexes(
            contours,
            scaledMinArea(config_, geometry),
            config_.max_candidates);
        for (const auto index : candidate_indexes) {
            ++candidates_evaluated;
            auto candidate = evaluateContour(
                contours[index],
                candidate_mask.mask,
                candidate_mask.polarity,
                config_,
                geometry,
                work_lookahead_y);
            if (candidate && (!best || candidate->score > best->score)) {
                best = std::move(candidate);
            }
        }
    }

    LineDetection output = best ? best->line : LineDetection {};
    output.mask_count = static_cast<int>(masks.masks.size());
    output.contours_found = total_contours;
    output.candidates_evaluated = candidates_evaluated;
    output.roi_pixels = geometry.work_width * geometry.work_height;
    output.selected_contour_points = static_cast<int>(output.contour_px.size());
    return output;
}

LineDetection LineDetector::detect(const cv::Mat& image) const
{
    if (!config_.enabled || image.empty() || image.cols <= 0 || image.rows <= 0) {
        return {};
    }

    LineMaskBuilder mask_builder(config_);
    return detect(mask_builder.build(image));
}

LineDetection applyMarkerCenterTracking(
    LineDetection line,
    const std::vector<MarkerObservation>& markers,
    int frame_width,
    int frame_height)
{
    if (markers.empty() || frame_width <= 0 || frame_height <= 0) {
        return line;
    }

    const MarkerObservation* best_marker = nullptr;
    double best_score = std::numeric_limits<double>::infinity();
    const double center_x = frame_width / 2.0;
    const double center_y = frame_height / 2.0;
    const double frame_diag = std::max(1.0, std::hypot(center_x, center_y));
    for (const auto& marker : markers) {
        if (marker.id < 0 ||
            marker.center_px.x < 0.0f ||
            marker.center_px.x >= frame_width ||
            marker.center_px.y < 0.0f ||
            marker.center_px.y >= frame_height) {
            continue;
        }
        const double distance_score =
            std::hypot(marker.center_px.x - center_x, marker.center_px.y - center_y) / frame_diag;
        const double size_score =
            markerApproxSidePx(marker) /
            std::max(1.0, static_cast<double>(std::min(frame_width, frame_height)));
        const double score = distance_score - 0.20 * size_score;
        if (score < best_score) {
            best_score = score;
            best_marker = &marker;
        }
    }
    if (best_marker == nullptr) {
        return line;
    }

    const float tracking_x = best_marker->center_px.x;
    const float tracking_y = static_cast<float>(frame_height) / 2.0f;
    line.detected = true;
    line.raw_detected = true;
    line.tracking_point_px = {tracking_x, tracking_y};
    line.raw_tracking_point_px = line.tracking_point_px;
    line.centroid_px = line.tracking_point_px;
    line.center_offset_px = tracking_x - static_cast<float>(frame_width) / 2.0f;
    line.raw_center_offset_px = line.center_offset_px;
    line.held = false;
    line.rejected_jump = false;
    line.confidence = std::max(line.confidence, 0.75f);
    return line;
}

} // namespace onboard::vision
