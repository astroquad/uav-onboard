#include "common/VisionConfig.hpp"
#include "mission/GridCoordinateTracker.hpp"
#include "mission/IntersectionDecision.hpp"
#include "vision/ArucoDetector.hpp"
#include "vision/IntersectionDetector.hpp"
#include "vision/LineDetector.hpp"
#include "vision/LineMaskBuilder.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

struct Options {
    std::string config_dir = "config";
    std::string image_path;
    std::string output_dir;
    std::string scenario = "marker_grid";
    std::string line_mode;
};

enum class EntrySide {
    Top,
    Bottom,
    Left,
    Right,
    Unknown,
};

struct GridGeometry {
    std::vector<int> xs;
    std::vector<int> ys;
    int entry_col = 0;
    int entry_row = 0;
    EntrySide entry_side = EntrySide::Unknown;
};

struct GridIndex {
    int row = 0;
    int col = 0;
};

struct DetectionResult {
    onboard::vision::VisionResult aruco;
    onboard::vision::LineDetection line;
    onboard::vision::IntersectionDetection intersection;
    onboard::mission::IntersectionDecision decision;
    onboard::mission::GridNodeEvent node;
    bool event_ready = false;
};

struct LoggedNode {
    int x = 0;
    int y = 0;
    onboard::mission::GridHeading heading = onboard::mission::GridHeading::Unknown;
};

struct ExpectedCoord {
    int x = 0;
    int y = 0;
};

struct MarkerCoord {
    int id = -1;
    int x = 0;
    int y = 0;
    int world_row = -1;
    int world_col = -1;
    int detections = 0;
    std::string source;
};

void printUsage()
{
    std::cout
        << "Usage: marker_grid_replay --image <path> --output <dir> [options]\n\n"
        << "Options:\n"
        << "  --config <dir>          Config directory, default config\n"
        << "  --line-mode <mode>      light_on_dark or dark_on_light\n"
        << "  --scenario <name>       Scenario label used in output files\n"
        << "  -h, --help              Show this help\n";
}

Options parseOptions(int argc, char** argv)
{
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        if (arg == "--config" && index + 1 < argc) {
            options.config_dir = argv[++index];
        } else if (arg == "--image" && index + 1 < argc) {
            options.image_path = argv[++index];
        } else if (arg == "--output" && index + 1 < argc) {
            options.output_dir = argv[++index];
        } else if (arg == "--line-mode" && index + 1 < argc) {
            options.line_mode = argv[++index];
        } else if (arg == "--scenario" && index + 1 < argc) {
            options.scenario = argv[++index];
        } else if (arg == "-h" || arg == "--help") {
            printUsage();
            std::exit(0);
        } else {
            std::cerr << "unknown or incomplete option: " << arg << "\n";
            printUsage();
            std::exit(2);
        }
    }
    return options;
}

const char* entrySideName(EntrySide side)
{
    switch (side) {
    case EntrySide::Top:
        return "top";
    case EntrySide::Bottom:
        return "bottom";
    case EntrySide::Left:
        return "left";
    case EntrySide::Right:
        return "right";
    case EntrySide::Unknown:
        return "unknown";
    }
    return "unknown";
}

cv::Mat lineColorMask(const cv::Mat& image, const std::string& line_mode)
{
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    if (line_mode == "dark_on_light") {
        cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 100), mask);
    } else {
        cv::inRange(hsv, cv::Scalar(0, 0, 140), cv::Scalar(180, 95, 255), mask);
    }

    const cv::Mat close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, close);
    return mask;
}

std::vector<int> projectionClusters(
    const cv::Mat& mask,
    bool rows,
    int threshold,
    int min_center,
    int max_center)
{
    const int count = rows ? mask.rows : mask.cols;
    std::vector<int> centers;
    int run_start = -1;
    long long weighted_sum = 0;
    long long weight_total = 0;

    const auto flush = [&](int end) {
        if (run_start < 0 || weight_total <= 0) {
            return;
        }
        const int center = static_cast<int>(std::lround(weighted_sum / static_cast<double>(weight_total)));
        if (center >= min_center && center <= max_center && end - run_start >= 2) {
            centers.push_back(center);
        }
    };

    for (int index = 0; index < count; ++index) {
        const cv::Range row_range = rows ? cv::Range(index, index + 1) : cv::Range::all();
        const cv::Range col_range = rows ? cv::Range::all() : cv::Range(index, index + 1);
        const int value = cv::countNonZero(mask(row_range, col_range));
        if (value >= threshold) {
            if (run_start < 0) {
                run_start = index;
                weighted_sum = 0;
                weight_total = 0;
            }
            weighted_sum += static_cast<long long>(index) * value;
            weight_total += value;
        } else if (run_start >= 0) {
            flush(index);
            run_start = -1;
        }
    }
    if (run_start >= 0) {
        flush(count);
    }

    centers.erase(
        std::unique(
            centers.begin(),
            centers.end(),
            [](int lhs, int rhs) { return std::abs(lhs - rhs) < 8; }),
        centers.end());
    return centers;
}

int medianSpacing(const std::vector<int>& values)
{
    if (values.size() < 2) {
        return 80;
    }
    std::vector<int> gaps;
    for (std::size_t index = 1; index < values.size(); ++index) {
        gaps.push_back(values[index] - values[index - 1]);
    }
    std::sort(gaps.begin(), gaps.end());
    return gaps[gaps.size() / 2];
}

GridGeometry detectGridGeometry(const cv::Mat& image, const cv::Mat& mask)
{
    GridGeometry geometry;
    const int row_threshold = std::max(30, static_cast<int>(std::lround(image.cols * 0.24)));
    const int col_threshold = std::max(30, static_cast<int>(std::lround(image.rows * 0.14)));
    geometry.ys = projectionClusters(
        mask,
        true,
        row_threshold,
        static_cast<int>(std::lround(image.rows * 0.02)),
        static_cast<int>(std::lround(image.rows * 0.98)));
    geometry.xs = projectionClusters(
        mask,
        false,
        col_threshold,
        static_cast<int>(std::lround(image.cols * 0.02)),
        static_cast<int>(std::lround(image.cols * 0.98)));

    if (geometry.xs.size() < 2 || geometry.ys.size() < 2) {
        return geometry;
    }

    const int cell_width = medianSpacing(geometry.xs);
    const int cell_height = medianSpacing(geometry.ys);
    const int half_col_width = std::max(6, cell_width / 7);
    const int half_row_height = std::max(6, cell_height / 7);
    int best_count = -1;

    const auto countRect = [&](const cv::Rect& rect) {
        const cv::Rect clipped = rect & cv::Rect(0, 0, mask.cols, mask.rows);
        return clipped.empty() ? 0 : cv::countNonZero(mask(clipped));
    };

    const auto considerColumnBand = [&](EntrySide side, int col, int y0, int y1) {
        const int x0 = std::max(0, geometry.xs[static_cast<std::size_t>(col)] - half_col_width);
        const int count = countRect(cv::Rect(
            x0,
            y0,
            std::min(mask.cols - x0, half_col_width * 2 + 1),
            std::max(1, y1 - y0)));
        if (count > best_count) {
            best_count = count;
            geometry.entry_side = side;
            geometry.entry_col = col;
            geometry.entry_row = side == EntrySide::Top ? 0 : static_cast<int>(geometry.ys.size()) - 1;
        }
    };

    const auto considerRowBand = [&](EntrySide side, int row, int x0, int x1) {
        const int y0 = std::max(0, geometry.ys[static_cast<std::size_t>(row)] - half_row_height);
        const int count = countRect(cv::Rect(
            x0,
            y0,
            std::max(1, x1 - x0),
            std::min(mask.rows - y0, half_row_height * 2 + 1)));
        if (count > best_count) {
            best_count = count;
            geometry.entry_side = side;
            geometry.entry_row = row;
            geometry.entry_col = side == EntrySide::Left ? 0 : static_cast<int>(geometry.xs.size()) - 1;
        }
    };

    const int top_y = geometry.ys.front();
    const int bottom_y = geometry.ys.back();
    const int left_x = geometry.xs.front();
    const int right_x = geometry.xs.back();
    const int outside_band = std::max(40, std::max(cell_width, cell_height) / 2);
    const int entry_gap = std::max(8, std::min(cell_width, cell_height) / 3);

    for (int col = 0; col < static_cast<int>(geometry.xs.size()); ++col) {
        considerColumnBand(EntrySide::Top, col, std::max(0, top_y - outside_band), std::max(0, top_y - entry_gap));
        considerColumnBand(
            EntrySide::Bottom,
            col,
            std::min(mask.rows - 1, bottom_y + entry_gap),
            std::min(mask.rows, bottom_y + outside_band));
    }
    for (int row = 0; row < static_cast<int>(geometry.ys.size()); ++row) {
        considerRowBand(EntrySide::Left, row, std::max(0, left_x - outside_band), std::max(0, left_x - entry_gap));
        considerRowBand(
            EntrySide::Right,
            row,
            std::min(mask.cols - 1, right_x + entry_gap),
            std::min(mask.cols, right_x + outside_band));
    }

    if (geometry.entry_side == EntrySide::Unknown) {
        geometry.entry_side = EntrySide::Bottom;
        geometry.entry_col = 0;
        geometry.entry_row = static_cast<int>(geometry.ys.size()) - 1;
    }
    return geometry;
}

onboard::vision::IntersectionType expectedTopology(const GridGeometry& geometry, int row, int col)
{
    const bool top = row == 0;
    const bool bottom = row == static_cast<int>(geometry.ys.size()) - 1;
    const bool left = col == 0;
    const bool right = col == static_cast<int>(geometry.xs.size()) - 1;
    const bool corner = (top || bottom) && (left || right);
    const bool edge = top || bottom || left || right;
    const bool entry_node =
        (geometry.entry_side == EntrySide::Top && top && col == geometry.entry_col) ||
        (geometry.entry_side == EntrySide::Bottom && bottom && col == geometry.entry_col) ||
        (geometry.entry_side == EntrySide::Left && left && row == geometry.entry_row) ||
        (geometry.entry_side == EntrySide::Right && right && row == geometry.entry_row);
    if (entry_node) {
        return onboard::vision::IntersectionType::T;
    }
    if (corner) {
        return onboard::vision::IntersectionType::L;
    }
    if (edge) {
        return onboard::vision::IntersectionType::T;
    }
    return onboard::vision::IntersectionType::Cross;
}

std::vector<GridIndex> entrySnakePath(const GridGeometry& geometry)
{
    std::vector<GridIndex> path;
    const int rows = static_cast<int>(geometry.ys.size());
    const int cols = static_cast<int>(geometry.xs.size());
    if (rows <= 0 || cols <= 0) {
        return path;
    }

    if (geometry.entry_side == EntrySide::Bottom || geometry.entry_side == EntrySide::Top) {
        const int start_col = std::clamp(geometry.entry_col, 0, cols - 1);
        for (int col = start_col; col < cols; ++col) {
            const bool forward = ((col - start_col) % 2) == 0;
            if (geometry.entry_side == EntrySide::Bottom) {
                if (forward) {
                    for (int row = rows - 1; row >= 0; --row) {
                        path.push_back({row, col});
                    }
                } else {
                    for (int row = 0; row < rows; ++row) {
                        path.push_back({row, col});
                    }
                }
            } else if (forward) {
                for (int row = 0; row < rows; ++row) {
                    path.push_back({row, col});
                }
            } else {
                for (int row = rows - 1; row >= 0; --row) {
                    path.push_back({row, col});
                }
            }
        }
        return path;
    }

    const int start_row = std::clamp(geometry.entry_row, 0, rows - 1);
    for (int row = start_row; row < rows; ++row) {
        const bool forward = ((row - start_row) % 2) == 0;
        if (geometry.entry_side == EntrySide::Right) {
            if (forward) {
                for (int col = cols - 1; col >= 0; --col) {
                    path.push_back({row, col});
                }
            } else {
                for (int col = 0; col < cols; ++col) {
                    path.push_back({row, col});
                }
            }
        } else if (forward) {
            for (int col = 0; col < cols; ++col) {
                path.push_back({row, col});
            }
        } else {
            for (int col = cols - 1; col >= 0; --col) {
                path.push_back({row, col});
            }
        }
    }
    return path;
}

onboard::mission::GridHeading headingBetween(GridIndex from, GridIndex to)
{
    if (to.col > from.col) {
        return onboard::mission::GridHeading::East;
    }
    if (to.col < from.col) {
        return onboard::mission::GridHeading::West;
    }
    if (to.row > from.row) {
        return onboard::mission::GridHeading::South;
    }
    if (to.row < from.row) {
        return onboard::mission::GridHeading::North;
    }
    return onboard::mission::GridHeading::Unknown;
}

onboard::mission::GridHeading cameraHeadingForStep(const std::vector<GridIndex>& path, std::size_t step)
{
    if (path.empty()) {
        return onboard::mission::GridHeading::Unknown;
    }
    if (step == 0) {
        return path.size() > 1 ? headingBetween(path[0], path[1]) : onboard::mission::GridHeading::Unknown;
    }
    return headingBetween(path[step - 1], path[step]);
}

double angleForHeading(onboard::mission::GridHeading heading)
{
    switch (heading) {
    case onboard::mission::GridHeading::North:
    case onboard::mission::GridHeading::Unknown:
        return 0.0;
    case onboard::mission::GridHeading::East:
        return 90.0;
    case onboard::mission::GridHeading::South:
        return 180.0;
    case onboard::mission::GridHeading::West:
        return -90.0;
    }
    return 0.0;
}

cv::Mat centeredCameraCrop(
    const cv::Mat& image,
    cv::Point2f center,
    cv::Size crop_size,
    onboard::mission::GridHeading heading)
{
    const cv::Scalar border = cv::mean(image);
    cv::Mat rotated;
    const cv::Mat rotation = cv::getRotationMatrix2D(center, angleForHeading(heading), 1.0);
    cv::warpAffine(image, rotated, rotation, image.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, border);

    cv::Mat crop(crop_size, image.type(), border);
    const int desired_x = static_cast<int>(std::lround(center.x - crop_size.width / 2.0));
    const int desired_y = static_cast<int>(std::lround(center.y - crop_size.height / 2.0));
    const cv::Rect source_rect =
        cv::Rect(desired_x, desired_y, crop_size.width, crop_size.height) &
        cv::Rect(0, 0, rotated.cols, rotated.rows);
    if (source_rect.empty()) {
        return crop;
    }

    const cv::Rect target_rect(source_rect.x - desired_x, source_rect.y - desired_y, source_rect.width, source_rect.height);
    rotated(source_rect).copyTo(crop(target_rect));
    return crop;
}

cv::Size cropSizeForTwoMeterView(const GridGeometry& geometry)
{
    const double cell_px = (medianSpacing(geometry.xs) + medianSpacing(geometry.ys)) / 2.0;
    const double px_per_meter = cell_px / 4.0;
    const double diagonal_fov = 78.0 * CV_PI / 180.0;
    const double diagonal_tan = std::tan(diagonal_fov / 2.0);
    const double aspect_w = 4.0;
    const double aspect_h = 3.0;
    const double aspect_diag = std::hypot(aspect_w, aspect_h);
    const double half_w_tan = diagonal_tan * aspect_w / aspect_diag;
    const double half_h_tan = diagonal_tan * aspect_h / aspect_diag;
    const double altitude_m = 2.0;
    const int width = std::max(96, static_cast<int>(std::lround(2.0 * altitude_m * half_w_tan * px_per_meter)));
    const int height = std::max(72, static_cast<int>(std::lround(2.0 * altitude_m * half_h_tan * px_per_meter)));
    return {width, height};
}

onboard::mission::GridCoord headingVector(onboard::mission::GridHeading heading)
{
    switch (heading) {
    case onboard::mission::GridHeading::North:
        return {0, -1};
    case onboard::mission::GridHeading::East:
        return {1, 0};
    case onboard::mission::GridHeading::South:
        return {0, 1};
    case onboard::mission::GridHeading::West:
        return {-1, 0};
    case onboard::mission::GridHeading::Unknown:
        return {0, 1};
    }
    return {0, 1};
}

char headingArrow(onboard::mission::GridHeading heading)
{
    switch (heading) {
    case onboard::mission::GridHeading::North:
        return '^';
    case onboard::mission::GridHeading::East:
        return '>';
    case onboard::mission::GridHeading::South:
        return 'v';
    case onboard::mission::GridHeading::West:
        return '<';
    case onboard::mission::GridHeading::Unknown:
        return '@';
    }
    return '@';
}

std::string renderGridLog(const std::vector<LoggedNode>& nodes)
{
    if (nodes.empty()) {
        return "[grid-map] empty\n";
    }

    int min_x = nodes.front().x;
    int max_x = nodes.front().x;
    int min_y = nodes.front().y;
    int max_y = nodes.front().y;
    for (const auto& node : nodes) {
        min_x = std::min(min_x, node.x);
        max_x = std::max(max_x, node.x);
        min_y = std::min(min_y, node.y);
        max_y = std::max(max_y, node.y);
    }

    const auto start_vector = headingVector(nodes.front().heading);
    const int start_x = nodes.front().x - start_vector.x;
    const int start_y = nodes.front().y - start_vector.y;
    min_x = std::min(min_x, start_x);
    max_x = std::max(max_x, start_x);
    min_y = std::min(min_y, start_y);
    max_y = std::max(max_y, start_y);

    const int canvas_rows = (max_y - min_y) * 2 + 1;
    const int canvas_cols = (max_x - min_x) * 4 + 1;
    std::vector<std::string> canvas(
        static_cast<std::size_t>(std::max(1, canvas_rows)),
        std::string(static_cast<std::size_t>(std::max(1, canvas_cols)), ' '));
    const auto rowFor = [&](int y) { return (y - min_y) * 2; };
    const auto colFor = [&](int x) { return (x - min_x) * 4; };
    const auto put = [&](int row, int col, char value) {
        if (row >= 0 && col >= 0 &&
            row < static_cast<int>(canvas.size()) &&
            col < static_cast<int>(canvas[static_cast<std::size_t>(row)].size())) {
            canvas[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] = value;
        }
    };
    const auto drawEdge = [&](int ax, int ay, int bx, int by) {
        const int row_a = rowFor(ay);
        const int col_a = colFor(ax);
        const int row_b = rowFor(by);
        const int col_b = colFor(bx);
        if (row_a == row_b) {
            for (int col = std::min(col_a, col_b) + 1; col < std::max(col_a, col_b); ++col) {
                put(row_a, col, '-');
            }
        } else if (col_a == col_b) {
            for (int row = std::min(row_a, row_b) + 1; row < std::max(row_a, row_b); ++row) {
                put(row, col_a, '|');
            }
        }
    };

    drawEdge(start_x, start_y, nodes.front().x, nodes.front().y);
    put(rowFor(start_y), colFor(start_x), 's');
    for (std::size_t index = 1; index < nodes.size(); ++index) {
        drawEdge(nodes[index - 1].x, nodes[index - 1].y, nodes[index].x, nodes[index].y);
    }
    for (const auto& node : nodes) {
        put(rowFor(node.y), colFor(node.x), '+');
    }
    const auto& current = nodes.back();
    put(rowFor(current.y), colFor(current.x), headingArrow(current.heading));

    std::ostringstream stream;
    stream << "[grid-map] nodes=" << nodes.size()
           << " current=(" << current.x << ',' << current.y << ")"
           << " heading=" << onboard::mission::gridHeadingName(current.heading) << "\n";
    for (const auto& row : canvas) {
        stream << row << "\n";
    }
    return stream.str();
}

std::string markerIds(const std::vector<onboard::vision::MarkerObservation>& markers)
{
    std::ostringstream stream;
    for (std::size_t index = 0; index < markers.size(); ++index) {
        if (index > 0) {
            stream << '|';
        }
        stream << markers[index].id;
    }
    return stream.str();
}

DetectionResult detectFrame(
    const cv::Mat& image,
    std::uint32_t frame_seq,
    const onboard::vision::ArucoDetector& aruco_detector,
    const onboard::vision::LineMaskBuilder& mask_builder,
    const onboard::vision::LineDetector& line_detector,
    const onboard::vision::IntersectionDetector& intersection_detector,
    const onboard::common::IntersectionDecisionConfig& decision_config)
{
    DetectionResult result;
    result.aruco = aruco_detector.detect(image, frame_seq, frame_seq * 83);
    const auto masks = mask_builder.build(image, result.aruco.markers, true);
    result.line = line_detector.detect(masks);
    result.intersection = intersection_detector.detect(masks, result.line);

    onboard::mission::IntersectionDecisionEngine engine(decision_config);
    for (int index = 0; index < std::max(3, decision_config.cruise_window_frames); ++index) {
        const auto decision = engine.update(
            result.intersection,
            image.cols,
            image.rows,
            frame_seq + static_cast<std::uint32_t>(index),
            (frame_seq + static_cast<std::uint32_t>(index)) * 83,
            false);
        if (decision.event_ready && !result.event_ready) {
            result.decision = decision;
            result.event_ready = true;
        }
    }
    if (!result.event_ready) {
        result.decision = engine.update(
            result.intersection,
            image.cols,
            image.rows,
            frame_seq + 20,
            (frame_seq + 20) * 83,
            false);
    }
    return result;
}

void drawOverlay(cv::Mat& overlay, const DetectionResult& result, const std::string& label)
{
    if (!result.line.contour_px.empty()) {
        std::vector<cv::Point> contour;
        contour.reserve(result.line.contour_px.size());
        for (const auto& point : result.line.contour_px) {
            contour.push_back({
                static_cast<int>(std::lround(point.x)),
                static_cast<int>(std::lround(point.y)),
            });
        }
        cv::polylines(overlay, contour, true, cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
    }

    if (result.line.detected) {
        cv::circle(
            overlay,
            {
                static_cast<int>(std::lround(result.line.tracking_point_px.x)),
                static_cast<int>(std::lround(result.line.tracking_point_px.y)),
            },
            5,
            cv::Scalar(0, 0, 255),
            -1,
            cv::LINE_AA);
    }

    if (result.intersection.valid) {
        const cv::Point center(
            static_cast<int>(std::lround(result.intersection.center_px.x)),
            static_cast<int>(std::lround(result.intersection.center_px.y)));
        cv::circle(overlay, center, 6, cv::Scalar(255, 255, 0), -1, cv::LINE_AA);
        for (const auto& branch : result.intersection.branches) {
            if (!branch.present) {
                continue;
            }
            const cv::Point endpoint(
                static_cast<int>(std::lround(branch.endpoint_px.x)),
                static_cast<int>(std::lround(branch.endpoint_px.y)));
            cv::arrowedLine(overlay, center, endpoint, cv::Scalar(0, 255, 255), 2, cv::LINE_AA, 0, 0.12);
        }
    }

    for (const auto& marker : result.aruco.markers) {
        std::vector<cv::Point> polygon;
        polygon.reserve(marker.corners_px.size());
        for (const auto& corner : marker.corners_px) {
            polygon.push_back({
                static_cast<int>(std::lround(corner.x)),
                static_cast<int>(std::lround(corner.y)),
            });
        }
        cv::polylines(overlay, polygon, true, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::putText(
            overlay,
            std::string("ID ") + std::to_string(marker.id),
            {
                static_cast<int>(std::lround(marker.center_px.x)) + 6,
                static_cast<int>(std::lround(marker.center_px.y)) - 6,
            },
            cv::FONT_HERSHEY_SIMPLEX,
            0.55,
            cv::Scalar(0, 255, 0),
            2,
            cv::LINE_AA);
    }

    cv::putText(
        overlay,
        label,
        {12, 24},
        cv::FONT_HERSHEY_SIMPLEX,
        0.55,
        cv::Scalar(0, 255, 255),
        2,
        cv::LINE_AA);
}

std::map<std::pair<int, int>, ExpectedCoord> expectedCoordsForPath(const std::vector<GridIndex>& path)
{
    std::map<std::pair<int, int>, ExpectedCoord> coords;
    int expected_x = 0;
    int expected_y = 0;
    GridIndex previous;
    bool has_previous = false;
    for (const auto& current : path) {
        if (has_previous) {
            const auto heading = headingBetween(previous, current);
            if (heading == onboard::mission::GridHeading::East) {
                ++expected_x;
            } else if (heading == onboard::mission::GridHeading::West) {
                --expected_x;
            } else if (heading == onboard::mission::GridHeading::South) {
                ++expected_y;
            } else if (heading == onboard::mission::GridHeading::North) {
                --expected_y;
            }
        }
        coords[{current.row, current.col}] = {expected_x, expected_y};
        previous = current;
        has_previous = true;
    }
    return coords;
}

GridIndex nearestGridIndex(const GridGeometry& geometry, onboard::vision::Point2f point)
{
    GridIndex best;
    double best_distance = std::numeric_limits<double>::max();
    for (int row = 0; row < static_cast<int>(geometry.ys.size()); ++row) {
        for (int col = 0; col < static_cast<int>(geometry.xs.size()); ++col) {
            const double distance =
                std::hypot(point.x - geometry.xs[static_cast<std::size_t>(col)],
                           point.y - geometry.ys[static_cast<std::size_t>(row)]);
            if (distance < best_distance) {
                best_distance = distance;
                best = {row, col};
            }
        }
    }
    return best;
}

void recordMarkerCoord(
    std::map<int, MarkerCoord>& marker_coords,
    int marker_id,
    int world_row,
    int world_col,
    ExpectedCoord coord,
    const std::string& source)
{
    auto& marker = marker_coords[marker_id];
    if (marker.id < 0 || source == "crop") {
        marker.id = marker_id;
        marker.x = coord.x;
        marker.y = coord.y;
        marker.world_row = world_row;
        marker.world_col = world_col;
        marker.source = source;
    }
    ++marker.detections;
}

void writeGeometry(
    const std::filesystem::path& output_dir,
    const Options& options,
    const GridGeometry& geometry,
    cv::Size source_size,
    cv::Size crop_size)
{
    std::ofstream output(output_dir / "geometry.csv");
    output << "scenario,image_width,image_height,crop_width,crop_height,grid_cols,grid_rows,entry_side,entry_col,entry_row,xs,ys\n";
    output << options.scenario << ','
           << source_size.width << ','
           << source_size.height << ','
           << crop_size.width << ','
           << crop_size.height << ','
           << geometry.xs.size() << ','
           << geometry.ys.size() << ','
           << entrySideName(geometry.entry_side) << ','
           << geometry.entry_col << ','
           << geometry.entry_row << ',';
    for (std::size_t index = 0; index < geometry.xs.size(); ++index) {
        if (index > 0) {
            output << '|';
        }
        output << geometry.xs[index];
    }
    output << ',';
    for (std::size_t index = 0; index < geometry.ys.size(); ++index) {
        if (index > 0) {
            output << '|';
        }
        output << geometry.ys[index];
    }
    output << "\n";
}

void writeMarkerCoords(const std::filesystem::path& output_dir, const std::map<int, MarkerCoord>& marker_coords)
{
    std::ofstream output(output_dir / "marker_coords.csv");
    output << "id,x,y,world_row,world_col,source,detections\n";
    for (const auto& [id, marker] : marker_coords) {
        output << id << ','
               << marker.x << ','
               << marker.y << ','
               << marker.world_row << ','
               << marker.world_col << ','
               << marker.source << ','
               << marker.detections << "\n";
    }
}

} // namespace

int main(int argc, char** argv)
{
    const auto options = parseOptions(argc, argv);
    if (options.image_path.empty() || options.output_dir.empty()) {
        printUsage();
        return 2;
    }

    const cv::Mat source = cv::imread(options.image_path, cv::IMREAD_COLOR);
    if (source.empty()) {
        std::cerr << "failed to read image: " << options.image_path << "\n";
        return 1;
    }

    auto config = onboard::common::loadVisionConfig(options.config_dir);
    if (!options.line_mode.empty()) {
        config.line.mode = options.line_mode;
    }
    config.line.marker_mask_enabled = true;
    config.line.marker_mask_detect_candidates = true;
    config.aruco.roi_fallback_enabled = true;
    config.intersection_decision.min_branch_score =
        std::min(config.intersection_decision.min_branch_score, 0.72);

    const auto output_dir = std::filesystem::path(options.output_dir);
    const auto overlay_dir = output_dir / "overlays";
    const auto crop_dir = output_dir / "crops";
    const auto segment_overlay_dir = output_dir / "line_segment_overlays";
    const auto segment_crop_dir = output_dir / "line_segment_crops";
    const auto grid_step_dir = output_dir / "grid_log_steps";
    std::filesystem::create_directories(overlay_dir);
    std::filesystem::create_directories(crop_dir);
    std::filesystem::create_directories(segment_overlay_dir);
    std::filesystem::create_directories(segment_crop_dir);
    std::filesystem::create_directories(grid_step_dir);

    const cv::Mat mask = lineColorMask(source, config.line.mode);
    cv::imwrite((output_dir / "grid_line_mask.png").string(), mask);

    const auto geometry = detectGridGeometry(source, mask);
    if (geometry.xs.size() < 2 || geometry.ys.size() < 2) {
        std::cerr << "failed to detect grid geometry: xs=" << geometry.xs.size()
                  << " ys=" << geometry.ys.size() << "\n";
        return 1;
    }

    const cv::Size source_crop_size = cropSizeForTwoMeterView(geometry);
    writeGeometry(output_dir, options, geometry, source.size(), source_crop_size);

    const onboard::vision::ArucoDetector aruco_detector(config.aruco);
    const onboard::vision::LineMaskBuilder mask_builder(config.line);
    const onboard::vision::LineDetector line_detector(config.line);
    const onboard::vision::IntersectionDetector intersection_detector(config.line);
    onboard::mission::GridCoordinateTracker tracker(config.intersection_decision);

    const auto path = entrySnakePath(geometry);
    const auto expected_coords = expectedCoordsForPath(path);
    std::map<int, MarkerCoord> marker_coords;

    const auto full_markers = aruco_detector.detect(source, 1, 0);
    {
        std::ofstream full_csv(output_dir / "full_image_markers.csv");
        full_csv << "id,center_x,center_y,nearest_row,nearest_col,local_x,local_y\n";
        for (const auto& marker : full_markers.markers) {
            const auto nearest = nearestGridIndex(geometry, marker.center_px);
            const auto coord_it = expected_coords.find({nearest.row, nearest.col});
            const ExpectedCoord coord = coord_it != expected_coords.end()
                ? coord_it->second
                : ExpectedCoord {};
            recordMarkerCoord(marker_coords, marker.id, nearest.row, nearest.col, coord, "full");
            full_csv << marker.id << ','
                     << marker.center_px.x << ','
                     << marker.center_px.y << ','
                     << nearest.row << ','
                     << nearest.col << ','
                     << coord.x << ','
                     << coord.y << "\n";
        }
    }

    std::ofstream summary(output_dir / "snake_summary.csv");
    std::ofstream grid_log(output_dir / "snake_grid_log.txt");
    summary << "step,row,col,camera_heading,expected_type,line_detected,detector_type,decision_type,event_ready,"
            << "node_valid,local_x,local_y,expected_x,expected_y,coord_match,marker_ids,crop_file,overlay_file\n";

    std::vector<LoggedNode> logged_nodes;
    std::uint32_t frame_seq = 1000;
    GridIndex previous;
    bool has_previous = false;
    int expected_x = 0;
    int expected_y = 0;

    for (std::size_t step = 0; step < path.size(); ++step) {
        const auto current = path[step];
        const auto heading = cameraHeadingForStep(path, step);
        tracker.setHeading(heading);

        if (has_previous) {
            const auto travel_heading = headingBetween(previous, current);
            if (travel_heading == onboard::mission::GridHeading::East) {
                ++expected_x;
            } else if (travel_heading == onboard::mission::GridHeading::West) {
                --expected_x;
            } else if (travel_heading == onboard::mission::GridHeading::South) {
                ++expected_y;
            } else if (travel_heading == onboard::mission::GridHeading::North) {
                --expected_y;
            }
        }

        const cv::Point2f world_center(
            static_cast<float>(geometry.xs[static_cast<std::size_t>(current.col)]),
            static_cast<float>(geometry.ys[static_cast<std::size_t>(current.row)]));
        cv::Mat crop = centeredCameraCrop(source, world_center, source_crop_size, heading);
        cv::Mat camera_frame;
        cv::resize(crop, camera_frame, cv::Size(960, 720), 0.0, 0.0, cv::INTER_LINEAR);

        const auto result = detectFrame(
            camera_frame,
            frame_seq,
            aruco_detector,
            mask_builder,
            line_detector,
            intersection_detector,
            config.intersection_decision);

        DetectionResult output_result = result;
        if (output_result.event_ready) {
            frame_seq += static_cast<std::uint32_t>(config.intersection_decision.node_advance_min_frames + 3);
            output_result.node = tracker.update(output_result.decision, frame_seq, frame_seq * 83);
        }
        if (output_result.node.valid) {
            logged_nodes.push_back({
                output_result.node.local_coord.x,
                output_result.node.local_coord.y,
                heading,
            });
        }

        const ExpectedCoord expected_coord {expected_x, expected_y};
        const float center_limit = std::min(camera_frame.cols, camera_frame.rows) * 0.28f;
        for (const auto& marker : output_result.aruco.markers) {
            const float dx = marker.center_px.x - camera_frame.cols / 2.0f;
            const float dy = marker.center_px.y - camera_frame.rows / 2.0f;
            if (std::sqrt(dx * dx + dy * dy) <= center_limit) {
                recordMarkerCoord(
                    marker_coords,
                    marker.id,
                    current.row,
                    current.col,
                    expected_coord,
                    "crop");
            }
        }

        std::ostringstream base_name;
        base_name << "step_" << std::setw(3) << std::setfill('0') << step
                  << "_r" << current.row
                  << "_c" << current.col
                  << "_h_" << onboard::mission::gridHeadingName(heading);
        const std::string crop_name = base_name.str() + "_crop.png";
        const std::string overlay_name = base_name.str() + "_overlay.png";
        cv::imwrite((crop_dir / crop_name).string(), camera_frame);

        cv::Mat overlay = camera_frame.clone();
        std::ostringstream label;
        label << "step " << step
              << " r" << current.row
              << " c" << current.col
              << " h " << onboard::mission::gridHeadingName(heading)
              << " ix " << onboard::vision::intersectionTypeName(output_result.intersection.type)
              << " dec " << onboard::vision::intersectionTypeName(output_result.decision.accepted_type)
              << " markers " << markerIds(output_result.aruco.markers);
        drawOverlay(overlay, output_result, label.str());
        cv::imwrite((overlay_dir / overlay_name).string(), overlay);

        std::ostringstream grid_step_name;
        grid_step_name << "step_" << std::setw(3) << std::setfill('0') << step << ".txt";
        const std::string rendered_grid = renderGridLog(logged_nodes);
        {
            std::ofstream step_log(grid_step_dir / grid_step_name.str());
            step_log << "step=" << step << "\n"
                     << "world_row=" << current.row << "\n"
                     << "world_col=" << current.col << "\n"
                     << "camera_heading=" << onboard::mission::gridHeadingName(heading) << "\n"
                     << "node_valid=" << (output_result.node.valid ? "yes" : "no") << "\n"
                     << "local_coord=(" << output_result.node.local_coord.x << ','
                     << output_result.node.local_coord.y << ")\n"
                     << rendered_grid;
        }
        grid_log << "===== step " << step
                 << " row=" << current.row
                 << " col=" << current.col
                 << " heading=" << onboard::mission::gridHeadingName(heading)
                 << " node=" << (output_result.node.valid ? "yes" : "no")
                 << " crop=" << (crop_dir.filename() / crop_name).string()
                 << " overlay=" << (overlay_dir.filename() / overlay_name).string()
                 << " =====\n"
                 << rendered_grid;

        const auto expected = expectedTopology(geometry, current.row, current.col);
        const bool coord_match =
            output_result.node.valid &&
            output_result.node.local_coord.x == expected_x &&
            output_result.node.local_coord.y == expected_y;
        summary << step << ','
                << current.row << ','
                << current.col << ','
                << onboard::mission::gridHeadingName(heading) << ','
                << onboard::vision::intersectionTypeName(expected) << ','
                << (output_result.line.detected ? "yes" : "no") << ','
                << onboard::vision::intersectionTypeName(output_result.intersection.type) << ','
                << onboard::vision::intersectionTypeName(output_result.decision.accepted_type) << ','
                << (output_result.event_ready ? "yes" : "no") << ','
                << (output_result.node.valid ? "yes" : "no") << ','
                << output_result.node.local_coord.x << ','
                << output_result.node.local_coord.y << ','
                << expected_x << ','
                << expected_y << ','
                << (coord_match ? "yes" : "no") << ','
                << markerIds(output_result.aruco.markers) << ','
                << (crop_dir.filename() / crop_name).string() << ','
                << (overlay_dir.filename() / overlay_name).string() << "\n";

        previous = current;
        has_previous = true;
    }

    std::ofstream segment_csv(output_dir / "line_segments.csv");
    segment_csv << "segment,from_row,from_col,to_row,to_col,camera_heading,line_detected,line_confidence,"
                << "line_offset_px,intersection_type,marker_ids,crop_file,overlay_file\n";
    for (std::size_t step = 1; step < path.size(); ++step) {
        const auto from = path[step - 1];
        const auto to = path[step];
        const auto heading = headingBetween(from, to);
        const cv::Point2f from_center(
            static_cast<float>(geometry.xs[static_cast<std::size_t>(from.col)]),
            static_cast<float>(geometry.ys[static_cast<std::size_t>(from.row)]));
        const cv::Point2f to_center(
            static_cast<float>(geometry.xs[static_cast<std::size_t>(to.col)]),
            static_cast<float>(geometry.ys[static_cast<std::size_t>(to.row)]));
        const cv::Point2f segment_center(
            (from_center.x + to_center.x) * 0.5f,
            (from_center.y + to_center.y) * 0.5f);

        cv::Mat crop = centeredCameraCrop(source, segment_center, source_crop_size, heading);
        cv::Mat camera_frame;
        cv::resize(crop, camera_frame, cv::Size(960, 720), 0.0, 0.0, cv::INTER_LINEAR);
        auto result = detectFrame(
            camera_frame,
            frame_seq + 5000 + static_cast<std::uint32_t>(step),
            aruco_detector,
            mask_builder,
            line_detector,
            intersection_detector,
            config.intersection_decision);

        std::ostringstream base_name;
        base_name << "segment_" << std::setw(3) << std::setfill('0') << step
                  << "_r" << from.row << "_c" << from.col
                  << "_to_r" << to.row << "_c" << to.col
                  << "_h_" << onboard::mission::gridHeadingName(heading);
        const std::string crop_name = base_name.str() + "_crop.png";
        const std::string overlay_name = base_name.str() + "_overlay.png";
        cv::imwrite((segment_crop_dir / crop_name).string(), camera_frame);

        cv::Mat overlay = camera_frame.clone();
        std::ostringstream label;
        label << "segment " << step
              << " h " << onboard::mission::gridHeadingName(heading)
              << " line " << (result.line.detected ? "yes" : "no")
              << " conf " << std::fixed << std::setprecision(2) << result.line.confidence;
        drawOverlay(overlay, result, label.str());
        cv::imwrite((segment_overlay_dir / overlay_name).string(), overlay);

        segment_csv << step << ','
                    << from.row << ','
                    << from.col << ','
                    << to.row << ','
                    << to.col << ','
                    << onboard::mission::gridHeadingName(heading) << ','
                    << (result.line.detected ? "yes" : "no") << ','
                    << result.line.confidence << ','
                    << result.line.center_offset_px << ','
                    << onboard::vision::intersectionTypeName(result.intersection.type) << ','
                    << markerIds(result.aruco.markers) << ','
                    << (segment_crop_dir.filename() / crop_name).string() << ','
                    << (segment_overlay_dir.filename() / overlay_name).string() << "\n";
    }

    writeMarkerCoords(output_dir, marker_coords);

    {
        std::ofstream readme(output_dir / "README.txt");
        readme << "scenario=" << options.scenario << "\n"
               << "image=" << options.image_path << "\n"
               << "line_mode=" << config.line.mode << "\n"
               << "crop_source_size=" << source_crop_size.width << 'x' << source_crop_size.height << "\n"
               << "crop_output_size=960x720\n"
               << "full_image_markers=" << full_markers.markers.size() << "\n"
               << "marker_coords=" << marker_coords.size() << "\n";
    }

    std::cout << "marker_grid_replay scenario=" << options.scenario
              << " line_mode=" << config.line.mode
              << " size=" << source.cols << 'x' << source.rows
              << " cols=" << geometry.xs.size()
              << " rows=" << geometry.ys.size()
              << " path_steps=" << path.size()
              << " full_markers=" << full_markers.markers.size()
              << " marker_coords=" << marker_coords.size()
              << " output=" << output_dir.string() << "\n";
    return 0;
}
