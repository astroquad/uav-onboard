#include "common/VisionConfig.hpp"
#include "mission/GridCoordinateTracker.hpp"
#include "mission/IntersectionDecision.hpp"
#include "vision/IntersectionDetector.hpp"
#include "vision/LineMaskBuilder.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
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
    std::string scenario = "grid";
};

struct GridGeometry {
    std::vector<int> xs;
    std::vector<int> ys;
    int entry_col = 0;
};

struct GridIndex {
    int row = 0;
    int col = 0;
};

struct DetectionResult {
    onboard::vision::IntersectionDetection detection;
    onboard::mission::IntersectionDecision decision;
    onboard::mission::GridNodeEvent node;
    bool event_ready = false;
};

void printUsage()
{
    std::cout
        << "Usage: grid_image_smoke --image <path> --output <dir> [options]\n\n"
        << "Options:\n"
        << "  --config <dir>        Config directory, default config\n"
        << "  --scenario <name>     Scenario label used in output CSV\n"
        << "  -h, --help            Show this help\n";
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

cv::Mat whiteMask(const cv::Mat& image)
{
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(0, 0, 145), cv::Scalar(180, 85, 255), mask);

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
        static_cast<int>(std::lround(image.rows * 0.12)),
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
    const int top_y = geometry.ys.front();
    const int band_top = std::max(0, top_y - std::max(40, cell_width / 2));
    const int band_height = std::max(1, top_y - band_top - 2);
    int best_count = -1;
    int best_col = 0;
    for (std::size_t index = 0; index < geometry.xs.size(); ++index) {
        const int half_width = std::max(6, cell_width / 7);
        const cv::Rect roi(
            std::max(0, geometry.xs[index] - half_width),
            band_top,
            std::min(mask.cols - std::max(0, geometry.xs[index] - half_width), half_width * 2 + 1),
            band_height);
        const int count = roi.empty() ? 0 : cv::countNonZero(mask(roi));
        if (count > best_count) {
            best_count = count;
            best_col = static_cast<int>(index);
        }
    }
    geometry.entry_col = best_col;
    return geometry;
}

cv::Rect cropRectForNode(const cv::Mat& image, const GridGeometry& geometry, int row, int col)
{
    const int cell_width = medianSpacing(geometry.xs);
    const int cell_height = medianSpacing(geometry.ys);
    const int half_width = std::max(45, static_cast<int>(std::lround(cell_width * 0.72)));
    const int half_height = std::max(45, static_cast<int>(std::lround(cell_height * 0.72)));
    const int cx = geometry.xs[static_cast<std::size_t>(col)];
    const int cy = geometry.ys[static_cast<std::size_t>(row)];
    return cv::Rect(
        std::max(0, cx - half_width),
        std::max(0, cy - half_height),
        std::min(image.cols - std::max(0, cx - half_width), half_width * 2),
        std::min(image.rows - std::max(0, cy - half_height), half_height * 2));
}

cv::Mat centeredCropForNode(const cv::Mat& image, const GridGeometry& geometry, int row, int col)
{
    const int cell_width = medianSpacing(geometry.xs);
    const int cell_height = medianSpacing(geometry.ys);
    const int half_width = std::max(45, static_cast<int>(std::lround(cell_width * 0.72)));
    const int half_height = std::max(45, static_cast<int>(std::lround(cell_height * 0.72)));
    const int width = half_width * 2;
    const int height = half_height * 2;
    const int cx = geometry.xs[static_cast<std::size_t>(col)];
    const int cy = geometry.ys[static_cast<std::size_t>(row)];
    const int desired_x = cx - half_width;
    const int desired_y = cy - half_height;

    cv::Mat crop(height, width, image.type(), cv::Scalar(55, 75, 115));
    const int src_x0 = std::max(0, desired_x);
    const int src_y0 = std::max(0, desired_y);
    const int src_x1 = std::min(image.cols, desired_x + width);
    const int src_y1 = std::min(image.rows, desired_y + height);
    if (src_x1 <= src_x0 || src_y1 <= src_y0) {
        return crop;
    }

    const cv::Rect src(src_x0, src_y0, src_x1 - src_x0, src_y1 - src_y0);
    const cv::Rect dst(src_x0 - desired_x, src_y0 - desired_y, src.width, src.height);
    image(src).copyTo(crop(dst));
    return crop;
}

onboard::vision::IntersectionType expectedTopology(
    const GridGeometry& geometry,
    int row,
    int col)
{
    const bool top = row == 0;
    const bool bottom = row == static_cast<int>(geometry.ys.size()) - 1;
    const bool left = col == 0;
    const bool right = col == static_cast<int>(geometry.xs.size()) - 1;
    const bool corner = (top || bottom) && (left || right);
    const bool edge = top || bottom || left || right;
    if (top && col == geometry.entry_col) {
        return corner ? onboard::vision::IntersectionType::T : onboard::vision::IntersectionType::T;
    }
    if (corner) {
        return onboard::vision::IntersectionType::L;
    }
    if (edge) {
        return onboard::vision::IntersectionType::T;
    }
    return onboard::vision::IntersectionType::Cross;
}

void annotateCrop(
    cv::Mat& crop,
    const DetectionResult& result,
    const std::string& label)
{
    cv::putText(
        crop,
        label,
        {12, 24},
        cv::FONT_HERSHEY_SIMPLEX,
        0.55,
        cv::Scalar(0, 255, 255),
        1,
        cv::LINE_AA);
    if (result.detection.valid) {
        cv::circle(
            crop,
            {
                static_cast<int>(std::lround(result.detection.center_px.x)),
                static_cast<int>(std::lround(result.detection.center_px.y)),
            },
            7,
            cv::Scalar(255, 255, 0),
            -1,
            cv::LINE_AA);
    }
}

DetectionResult detectNode(
    const cv::Mat& crop,
    onboard::vision::LineMaskBuilder& mask_builder,
    onboard::vision::IntersectionDetector& detector,
    const onboard::common::IntersectionDecisionConfig& config)
{
    DetectionResult result;
    const auto masks = mask_builder.build(crop);
    result.detection = detector.detect(masks, {});

    onboard::mission::IntersectionDecisionEngine engine(config);
    std::uint32_t frame_seq = 1;
    for (int index = 0; index < std::max(3, config.cruise_window_frames); ++index) {
        auto decision = engine.update(
            result.detection,
            crop.cols,
            crop.rows,
            frame_seq,
            frame_seq * 83,
            false);
        if (decision.event_ready && !result.event_ready) {
            result.decision = decision;
            result.event_ready = true;
        }
        ++frame_seq;
    }
    if (!result.event_ready) {
        result.decision = engine.update(
            result.detection,
            crop.cols,
            crop.rows,
            frame_seq,
            frame_seq * 83,
            false);
    }
    return result;
}

std::vector<GridIndex> fullSnakePath(const GridGeometry& geometry)
{
    std::vector<GridIndex> path;
    for (int row = 0; row < static_cast<int>(geometry.ys.size()); ++row) {
        if (row % 2 == 0) {
            for (int col = 0; col < static_cast<int>(geometry.xs.size()); ++col) {
                path.push_back({row, col});
            }
        } else {
            for (int col = static_cast<int>(geometry.xs.size()) - 1; col >= 0; --col) {
                path.push_back({row, col});
            }
        }
    }
    return path;
}

std::vector<GridIndex> entrySnakePath(const GridGeometry& geometry)
{
    std::vector<GridIndex> path;
    for (int col = geometry.entry_col; col < static_cast<int>(geometry.xs.size()); ++col) {
        path.push_back({0, col});
    }
    for (int row = 1; row < static_cast<int>(geometry.ys.size()); ++row) {
        if (row % 2 == 1) {
            for (int col = static_cast<int>(geometry.xs.size()) - 1; col >= 0; --col) {
                path.push_back({row, col});
            }
        } else {
            for (int col = 0; col < static_cast<int>(geometry.xs.size()); ++col) {
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

onboard::mission::GridHeading cameraHeadingForStep(
    const std::vector<GridIndex>& path,
    std::size_t step)
{
    if (path.empty()) {
        return onboard::mission::GridHeading::Unknown;
    }
    if (step == 0) {
        return path.size() > 1
            ? headingBetween(path[0], path[1])
            : onboard::mission::GridHeading::Unknown;
    }
    return headingBetween(path[step - 1], path[step]);
}

cv::Mat rotateWorldCropToCameraFront(
    const cv::Mat& world_crop,
    onboard::mission::GridHeading heading)
{
    cv::Mat rotated;
    switch (heading) {
    case onboard::mission::GridHeading::North:
    case onboard::mission::GridHeading::Unknown:
        return world_crop.clone();
    case onboard::mission::GridHeading::East:
        cv::rotate(world_crop, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
        return rotated;
    case onboard::mission::GridHeading::South:
        cv::rotate(world_crop, rotated, cv::ROTATE_180);
        return rotated;
    case onboard::mission::GridHeading::West:
        cv::rotate(world_crop, rotated, cv::ROTATE_90_CLOCKWISE);
        return rotated;
    }
    return world_crop.clone();
}

void writeGeometry(
    const std::filesystem::path& output_dir,
    const Options& options,
    const GridGeometry& geometry)
{
    std::ofstream output(output_dir / "geometry.csv");
    output << "scenario,image_width,image_height,grid_cols,grid_rows,entry_col,xs,ys\n";
    output << options.scenario << ",,,"
           << geometry.xs.size() << ','
           << geometry.ys.size() << ','
           << geometry.entry_col << ',';
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

void runSectionCrops(
    const cv::Mat& image,
    const GridGeometry& geometry,
    const std::filesystem::path& output_dir,
    onboard::vision::LineMaskBuilder& mask_builder,
    onboard::vision::IntersectionDetector& detector,
    const onboard::common::IntersectionDecisionConfig& decision_config)
{
    const auto section_dir = output_dir / "sections";
    std::filesystem::create_directories(section_dir);
    std::ofstream csv(output_dir / "sections.csv");
    csv << "row,col,x,y,expected_type,detector_type,decision_type,event_ready,score,branch_mask,"
        << "front_frames,right_frames,back_frames,left_frames,"
        << "front_max,right_max,back_max,left_max,crop_file\n";

    for (int row = 0; row < static_cast<int>(geometry.ys.size()); ++row) {
        for (int col = 0; col < static_cast<int>(geometry.xs.size()); ++col) {
            const auto rect = cropRectForNode(image, geometry, row, col);
            cv::Mat crop = image(rect).clone();
            auto result = detectNode(crop, mask_builder, detector, decision_config);
            const auto expected = expectedTopology(geometry, row, col);
            std::ostringstream name;
            name << "section_r" << row << "_c" << col
                 << "_det_" << onboard::vision::intersectionTypeName(result.detection.type)
                 << "_dec_" << onboard::vision::intersectionTypeName(result.decision.accepted_type)
                 << ".png";
            annotateCrop(
                crop,
                result,
                std::string("r") + std::to_string(row) +
                    " c" + std::to_string(col) +
                    " exp " + onboard::vision::intersectionTypeName(expected) +
                    " det " + onboard::vision::intersectionTypeName(result.detection.type) +
                    " dec " + onboard::vision::intersectionTypeName(result.decision.accepted_type));
            cv::imwrite((section_dir / name.str()).string(), crop);

            csv << row << ','
                << col << ','
                << geometry.xs[static_cast<std::size_t>(col)] << ','
                << geometry.ys[static_cast<std::size_t>(row)] << ','
                << onboard::vision::intersectionTypeName(expected) << ','
                << onboard::vision::intersectionTypeName(result.detection.type) << ','
                << onboard::vision::intersectionTypeName(result.decision.accepted_type) << ','
                << (result.event_ready ? "yes" : "no") << ','
                << result.detection.score << ','
                << static_cast<int>(result.decision.accepted_branch_mask) << ','
                << result.decision.branch_evidence[0].present_frames << ','
                << result.decision.branch_evidence[1].present_frames << ','
                << result.decision.branch_evidence[2].present_frames << ','
                << result.decision.branch_evidence[3].present_frames << ','
                << result.decision.branch_evidence[0].max_score << ','
                << result.decision.branch_evidence[1].max_score << ','
                << result.decision.branch_evidence[2].max_score << ','
                << result.decision.branch_evidence[3].max_score << ','
                << (section_dir.filename() / name.str()).string() << "\n";
        }
    }
}

void runSnake(
    const cv::Mat& image,
    const GridGeometry& geometry,
    const std::filesystem::path& output_dir,
    const std::string& name,
    const std::vector<GridIndex>& path,
    onboard::vision::LineMaskBuilder& mask_builder,
    onboard::vision::IntersectionDetector& detector,
    const onboard::common::IntersectionDecisionConfig& decision_config)
{
    const auto snake_dir = output_dir / (name + "_zoom");
    std::filesystem::create_directories(snake_dir);
    std::ofstream csv(output_dir / (name + ".csv"));
    csv << "step,row,col,camera_heading,rotated_to_camera_front,detector_type,decision_type,node_valid,"
        << "local_x,local_y,expected_x,expected_y,coord_match,crop_file\n";

    onboard::mission::GridCoordinateTracker tracker(decision_config);
    std::uint32_t frame_seq = 1000;
    int expected_x = 0;
    int expected_y = 0;
    GridIndex previous;
    bool has_previous = false;
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

        cv::Mat world_crop = centeredCropForNode(image, geometry, current.row, current.col);
        cv::Mat crop = rotateWorldCropToCameraFront(world_crop, heading);
        auto result = detectNode(crop, mask_builder, detector, decision_config);
        if (result.event_ready) {
            frame_seq += static_cast<std::uint32_t>(decision_config.node_advance_min_frames + 2);
            result.node = tracker.update(result.decision, frame_seq, frame_seq * 83);
        }

        cv::Mat zoom;
        cv::resize(crop, zoom, cv::Size(), 2.0, 2.0, cv::INTER_LINEAR);
        std::ostringstream label;
        label << "step " << step
              << " r" << current.row
              << " c" << current.col
              << " h " << onboard::mission::gridHeadingName(heading)
              << " local(" << result.node.local_coord.x << ',' << result.node.local_coord.y << ')';
        annotateCrop(zoom, result, label.str());
        std::ostringstream file_name;
        file_name << name << "_step_" << step
                  << "_r" << current.row
                  << "_c" << current.col << ".png";
        cv::imwrite((snake_dir / file_name.str()).string(), zoom);

        const bool coord_match =
            result.node.valid &&
            result.node.local_coord.x == expected_x &&
            result.node.local_coord.y == expected_y;
        csv << step << ','
            << current.row << ','
            << current.col << ','
            << onboard::mission::gridHeadingName(heading) << ','
            << (heading == onboard::mission::GridHeading::Unknown ? "no" : "yes") << ','
            << onboard::vision::intersectionTypeName(result.detection.type) << ','
            << onboard::vision::intersectionTypeName(result.decision.accepted_type) << ','
            << (result.node.valid ? "yes" : "no") << ','
            << result.node.local_coord.x << ','
            << result.node.local_coord.y << ','
            << expected_x << ','
            << expected_y << ','
            << (coord_match ? "yes" : "no") << ','
            << (snake_dir.filename() / file_name.str()).string() << "\n";

        previous = current;
        has_previous = true;
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

    const cv::Mat image = cv::imread(options.image_path, cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "failed to read image: " << options.image_path << "\n";
        return 1;
    }

    const auto output_dir = std::filesystem::path(options.output_dir);
    std::filesystem::create_directories(output_dir);

    auto config = onboard::common::loadVisionConfig(options.config_dir);
    config.intersection_decision.min_branch_score =
        std::min(config.intersection_decision.min_branch_score, 0.72);

    onboard::vision::LineMaskBuilder mask_builder(config.line);
    onboard::vision::IntersectionDetector detector(config.line);

    const cv::Mat mask = whiteMask(image);
    cv::imwrite((output_dir / "white_mask.png").string(), mask);

    const auto geometry = detectGridGeometry(image, mask);
    if (geometry.xs.size() < 2 || geometry.ys.size() < 2) {
        std::cerr << "failed to detect grid geometry: xs=" << geometry.xs.size()
                  << " ys=" << geometry.ys.size() << "\n";
        return 1;
    }
    writeGeometry(output_dir, options, geometry);
    runSectionCrops(image, geometry, output_dir, mask_builder, detector, config.intersection_decision);
    runSnake(
        image,
        geometry,
        output_dir,
        "snake_full_field",
        fullSnakePath(geometry),
        mask_builder,
        detector,
        config.intersection_decision);
    runSnake(
        image,
        geometry,
        output_dir,
        "snake_from_entry",
        entrySnakePath(geometry),
        mask_builder,
        detector,
        config.intersection_decision);

    std::cout << "grid_image_smoke scenario=" << options.scenario
              << " size=" << image.cols << 'x' << image.rows
              << " cols=" << geometry.xs.size()
              << " rows=" << geometry.ys.size()
              << " entry_col=" << geometry.entry_col
              << " output=" << output_dir.string() << "\n";
    return 0;
}
