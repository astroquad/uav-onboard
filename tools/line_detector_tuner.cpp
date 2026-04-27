#include "common/VisionConfig.hpp"
#include "vision/LineDetector.hpp"

#include <opencv2/imgcodecs.hpp>

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

struct Options {
    std::string config_dir = "config";
    std::string image_path;
    std::string mode;
    int threshold = -1;
    double roi_top_ratio = -1.0;
    double lookahead_y_ratio = -1.0;
};

void printUsage()
{
    std::cout
        << "Usage: line_detector_tuner --image <path> [options]\n\n"
        << "Options:\n"
        << "  --config <dir>       Config directory, default config\n"
        << "  --image <path>       Input image path\n"
        << "  --mode <mode>        auto, light_on_dark, or dark_on_light\n"
        << "  --threshold <n>      Override line threshold 0..255\n"
        << "  --roi-top <ratio>    Override ROI top ratio 0.0..1.0\n"
        << "  --lookahead <ratio>  Override tracking point Y ratio 0.0..1.0\n"
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
        } else if (arg == "--mode" && i + 1 < argc) {
            options.mode = argv[++i];
        } else if (arg == "--threshold" && i + 1 < argc) {
            options.threshold = parseInt(argv[++i], options.threshold);
        } else if (arg == "--roi-top" && i + 1 < argc) {
            options.roi_top_ratio = parseDouble(argv[++i], options.roi_top_ratio);
        } else if (arg == "--lookahead" && i + 1 < argc) {
            options.lookahead_y_ratio = parseDouble(argv[++i], options.lookahead_y_ratio);
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
    if (options.threshold >= 0) {
        config.line.threshold = options.threshold;
    }
    if (options.roi_top_ratio >= 0.0) {
        config.line.roi_top_ratio = options.roi_top_ratio;
    }
    if (options.lookahead_y_ratio >= 0.0) {
        config.line.lookahead_y_ratio = options.lookahead_y_ratio;
    }

    const cv::Mat image = cv::imread(options.image_path, cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "failed to read image: " << options.image_path << "\n";
        return 1;
    }

    const onboard::vision::LineDetector detector(config.line);
    const auto line = detector.detect(image);

    std::cout << std::fixed << std::setprecision(2)
              << "detected=" << (line.detected ? "true" : "false") << "\n"
              << "tracking_point=(" << line.tracking_point_px.x << ','
              << line.tracking_point_px.y << ")\n"
              << "centroid=(" << line.centroid_px.x << ','
              << line.centroid_px.y << ")\n"
              << "offset=" << line.center_offset_px << "px\n"
              << "angle=" << line.angle_deg << "deg\n"
              << "confidence=" << line.confidence << "\n"
              << "contour_points=" << line.contour_px.size() << "\n";

    return line.detected ? 0 : 3;
}
