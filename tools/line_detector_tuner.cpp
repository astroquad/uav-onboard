#include "common/VisionConfig.hpp"
#include "vision/LineDetector.hpp"
#include "vision/LineMaskBuilder.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {

struct Options {
    std::string config_dir = "config";
    std::string image_path;
    std::string output_dir;
    std::string mode;
    std::string mask_strategy;
    int threshold = -1;
    int local_contrast_threshold = -1;
    int white_v_min = -1;
    int white_s_max = -1;
    int dark_v_max = -1;
    int process_width = -1;
    int morph_open_kernel = -1;
    int morph_close_kernel = -1;
    int fill_close_kernel = -1;
    int fill_dilate_kernel = -1;
    int dark_fill_close_kernel = -1;
    int dark_fill_dilate_kernel = -1;
    int line_run_merge_gap_px = -1;
    double dark_max_line_width_ratio = -1.0;
    double roi_top_ratio = -1.0;
    double lookahead_y_ratio = -1.0;
    double lookahead_band_ratio = -1.0;
};

void printUsage()
{
    std::cout
        << "Usage: line_detector_tuner --image <path> [options]\n\n"
        << "Options:\n"
        << "  --config <dir>       Config directory, default config\n"
        << "  --image <path>       Input image path\n"
        << "  --output <dir>       Optional directory for mask, overlay, and summary files\n"
        << "  --mode <mode>        auto, light_on_dark, or dark_on_light\n"
        << "  --mask <strategy>    global, local_contrast, white_fill, dark_fill\n"
        << "  --threshold <n>      Override line threshold 0..255\n"
        << "  --local-threshold <n> Override local contrast threshold 0..255\n"
        << "  --white-v-min <n>    Override white-fill value minimum 0..255\n"
        << "  --white-s-max <n>    Override white-fill saturation maximum 0..255\n"
        << "  --dark-v-max <n>     Override dark-fill value maximum 0..255\n"
        << "  --process-width <n>  Override resized processing width\n"
        << "  --morph-open <n>     Override morphology open kernel\n"
        << "  --morph-close <n>    Override morphology close kernel\n"
        << "  --fill-close <n>     Override white-fill morphology close kernel\n"
        << "  --fill-dilate <n>    Override white-fill morphology dilate kernel\n"
        << "  --dark-fill-close <n> Override dark-fill morphology close kernel\n"
        << "  --dark-fill-dilate <n> Override dark-fill morphology dilate kernel\n"
        << "  --merge-gap <n>      Override projection run merge gap in source px\n"
        << "  --dark-max-width <r> Override dark-line max width ratio\n"
        << "  --roi-top <ratio>    Override ROI top ratio 0.0..1.0\n"
        << "  --lookahead <ratio>  Override tracking point Y ratio 0.0..1.0\n"
        << "  --band <ratio>       Override lookahead band height ratio 0.0..1.0\n"
        << "  -h, --help           Show this help\n";
}

int parseInt(const std::string& value, int fallback)
{
    try {
        return std::stoi(value);
    } catch (...) {
        return fallback;
    }
}

double parseDouble(const std::string& value, double fallback)
{
    try {
        return std::stod(value);
    } catch (...) {
        return fallback;
    }
}

Options parseOptions(int argc, char** argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            options.config_dir = argv[++i];
        } else if (arg == "--image" && i + 1 < argc) {
            options.image_path = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            options.output_dir = argv[++i];
        } else if (arg == "--mode" && i + 1 < argc) {
            options.mode = argv[++i];
        } else if (arg == "--mask" && i + 1 < argc) {
            options.mask_strategy = argv[++i];
        } else if (arg == "--threshold" && i + 1 < argc) {
            options.threshold = parseInt(argv[++i], options.threshold);
        } else if (arg == "--local-threshold" && i + 1 < argc) {
            options.local_contrast_threshold = parseInt(argv[++i], options.local_contrast_threshold);
        } else if (arg == "--white-v-min" && i + 1 < argc) {
            options.white_v_min = parseInt(argv[++i], options.white_v_min);
        } else if (arg == "--white-s-max" && i + 1 < argc) {
            options.white_s_max = parseInt(argv[++i], options.white_s_max);
        } else if (arg == "--dark-v-max" && i + 1 < argc) {
            options.dark_v_max = parseInt(argv[++i], options.dark_v_max);
        } else if (arg == "--process-width" && i + 1 < argc) {
            options.process_width = parseInt(argv[++i], options.process_width);
        } else if (arg == "--morph-open" && i + 1 < argc) {
            options.morph_open_kernel = parseInt(argv[++i], options.morph_open_kernel);
        } else if (arg == "--morph-close" && i + 1 < argc) {
            options.morph_close_kernel = parseInt(argv[++i], options.morph_close_kernel);
        } else if (arg == "--fill-close" && i + 1 < argc) {
            options.fill_close_kernel = parseInt(argv[++i], options.fill_close_kernel);
        } else if (arg == "--fill-dilate" && i + 1 < argc) {
            options.fill_dilate_kernel = parseInt(argv[++i], options.fill_dilate_kernel);
        } else if (arg == "--dark-fill-close" && i + 1 < argc) {
            options.dark_fill_close_kernel = parseInt(argv[++i], options.dark_fill_close_kernel);
        } else if (arg == "--dark-fill-dilate" && i + 1 < argc) {
            options.dark_fill_dilate_kernel = parseInt(argv[++i], options.dark_fill_dilate_kernel);
        } else if (arg == "--merge-gap" && i + 1 < argc) {
            options.line_run_merge_gap_px = parseInt(argv[++i], options.line_run_merge_gap_px);
        } else if (arg == "--dark-max-width" && i + 1 < argc) {
            options.dark_max_line_width_ratio = parseDouble(argv[++i], options.dark_max_line_width_ratio);
        } else if (arg == "--roi-top" && i + 1 < argc) {
            options.roi_top_ratio = parseDouble(argv[++i], options.roi_top_ratio);
        } else if (arg == "--lookahead" && i + 1 < argc) {
            options.lookahead_y_ratio = parseDouble(argv[++i], options.lookahead_y_ratio);
        } else if (arg == "--band" && i + 1 < argc) {
            options.lookahead_band_ratio = parseDouble(argv[++i], options.lookahead_band_ratio);
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

std::vector<cv::Point> contourPoints(const onboard::vision::LineDetection& line)
{
    std::vector<cv::Point> points;
    points.reserve(line.contour_px.size());
    for (const auto& point : line.contour_px) {
        points.push_back({
            static_cast<int>(std::lround(point.x)),
            static_cast<int>(std::lround(point.y)),
        });
    }
    return points;
}

void writeOutputs(
    const Options& options,
    const cv::Mat& image,
    const onboard::vision::LineMaskFrame& masks,
    const onboard::vision::LineDetection& line)
{
    if (options.output_dir.empty()) {
        return;
    }

    const auto output_dir = std::filesystem::path(options.output_dir);
    std::filesystem::create_directories(output_dir);

    for (std::size_t index = 0; index < masks.masks.size(); ++index) {
        if (!masks.masks[index].mask.empty()) {
            std::ostringstream name;
            name << "mask_" << index << ".png";
            cv::imwrite((output_dir / name.str()).string(), masks.masks[index].mask);
        }
    }

    cv::Mat overlay = image.clone();
    const auto contour = contourPoints(line);
    if (contour.size() >= 2) {
        const std::vector<std::vector<cv::Point>> contours {contour};
        cv::polylines(overlay, contours, true, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
    }
    if (line.detected) {
        const cv::Point frame_center(image.cols / 2, image.rows / 2);
        const cv::Point line_center(
            static_cast<int>(std::lround(line.tracking_point_px.x)),
            image.rows / 2);
        cv::line(overlay, frame_center, line_center, cv::Scalar(0, 255, 0), 4, cv::LINE_AA);
        cv::circle(overlay, line_center, 14, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        cv::circle(overlay, frame_center, 5, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
    }
    cv::imwrite((output_dir / "line_overlay.png").string(), overlay);

    const auto bounds = contour.empty() ? cv::Rect {} : cv::boundingRect(contour);
    const double contour_area = contour.size() >= 3 ? std::abs(cv::contourArea(contour)) : 0.0;
    std::ofstream summary(output_dir / "summary.txt");
    summary << std::fixed << std::setprecision(2)
            << "image=" << options.image_path << "\n"
            << "detected=" << (line.detected ? "true" : "false") << "\n"
            << "tracking_point=(" << line.tracking_point_px.x << ','
            << line.tracking_point_px.y << ")\n"
            << "center_overlay_y=" << image.rows / 2 << "\n"
            << "offset=" << line.center_offset_px << "px\n"
            << "angle=" << line.angle_deg << "deg\n"
            << "confidence=" << line.confidence << "\n"
            << "contour_points=" << line.contour_px.size() << "\n"
            << "contour_area=" << contour_area << "\n"
            << "contour_bounds=(" << bounds.x << ',' << bounds.y << ','
            << bounds.width << ',' << bounds.height << ")\n"
            << "mask_count=" << line.mask_count << "\n"
            << "contours_found=" << line.contours_found << "\n"
            << "candidates_evaluated=" << line.candidates_evaluated << "\n"
            << "roi_pixels=" << line.roi_pixels << "\n";
}

} // namespace

int main(int argc, char** argv)
{
    const Options options = parseOptions(argc, argv);
    if (options.image_path.empty()) {
        std::cerr << "--image is required\n";
        printUsage();
        return 2;
    }

    auto config = onboard::common::loadVisionConfig(options.config_dir);
    if (!options.mode.empty()) {
        config.line.mode = options.mode;
    }
    if (!options.mask_strategy.empty()) {
        config.line.mask_strategy = options.mask_strategy;
    }
    if (options.threshold >= 0) {
        config.line.threshold = options.threshold;
    }
    if (options.local_contrast_threshold >= 0) {
        config.line.local_contrast_threshold = options.local_contrast_threshold;
    }
    if (options.white_v_min >= 0) {
        config.line.white_v_min = options.white_v_min;
    }
    if (options.white_s_max >= 0) {
        config.line.white_s_max = options.white_s_max;
    }
    if (options.dark_v_max >= 0) {
        config.line.dark_v_max = options.dark_v_max;
    }
    if (options.process_width > 0) {
        config.line.process_width = options.process_width;
    }
    if (options.morph_open_kernel > 0) {
        config.line.morph_open_kernel = options.morph_open_kernel;
    }
    if (options.morph_close_kernel > 0) {
        config.line.morph_close_kernel = options.morph_close_kernel;
    }
    if (options.fill_close_kernel > 0) {
        config.line.fill_close_kernel = options.fill_close_kernel;
    }
    if (options.fill_dilate_kernel > 0) {
        config.line.fill_dilate_kernel = options.fill_dilate_kernel;
    }
    if (options.dark_fill_close_kernel > 0) {
        config.line.dark_fill_close_kernel = options.dark_fill_close_kernel;
    }
    if (options.dark_fill_dilate_kernel > 0) {
        config.line.dark_fill_dilate_kernel = options.dark_fill_dilate_kernel;
    }
    if (options.line_run_merge_gap_px >= 0) {
        config.line.line_run_merge_gap_px = options.line_run_merge_gap_px;
    }
    if (options.dark_max_line_width_ratio > 0.0) {
        config.line.dark_max_line_width_ratio = options.dark_max_line_width_ratio;
    }
    if (options.roi_top_ratio >= 0.0) {
        config.line.roi_top_ratio = options.roi_top_ratio;
    }
    if (options.lookahead_y_ratio >= 0.0) {
        config.line.lookahead_y_ratio = options.lookahead_y_ratio;
    }
    if (options.lookahead_band_ratio >= 0.0) {
        config.line.lookahead_band_ratio = options.lookahead_band_ratio;
    }

    const cv::Mat image = cv::imread(options.image_path, cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "failed to read image: " << options.image_path << "\n";
        return 1;
    }

    const onboard::vision::LineMaskBuilder mask_builder(config.line);
    const auto masks = mask_builder.build(image);
    const onboard::vision::LineDetector detector(config.line);
    const auto line = detector.detect(masks);
    writeOutputs(options, image, masks, line);

    std::cout << std::fixed << std::setprecision(2)
              << "detected=" << (line.detected ? "true" : "false") << "\n"
              << "tracking_point=(" << line.tracking_point_px.x << ','
              << line.tracking_point_px.y << ")\n"
              << "centroid=(" << line.centroid_px.x << ','
              << line.centroid_px.y << ")\n"
              << "offset=" << line.center_offset_px << "px\n"
              << "angle=" << line.angle_deg << "deg\n"
              << "confidence=" << line.confidence << "\n"
              << "contour_points=" << line.contour_px.size() << "\n"
              << "mask_count=" << line.mask_count << "\n"
              << "contours_found=" << line.contours_found << "\n"
              << "candidates_evaluated=" << line.candidates_evaluated << "\n"
              << "roi_pixels=" << line.roi_pixels << "\n";

    return line.detected ? 0 : 3;
}
