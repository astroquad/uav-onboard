#include "common/VisionConfig.hpp"
#include "vision/ArucoDetector.hpp"

#include <opencv2/imgcodecs.hpp>

#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

struct Options {
    std::string config_dir = "config";
    std::string image_path;
    std::string dictionary_override;
};

void printUsage()
{
    std::cout
        << "Usage: aruco_detector_tester [options]\n\n"
        << "Options:\n"
        << "  --config <dir>       Config directory, default config\n"
        << "  --image <path>       Input image path\n"
        << "  --dictionary <name>  Override ArUco dictionary\n"
        << "  -h, --help           Show this help\n";
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
        } else if (arg == "--dictionary" && i + 1 < argc) {
            options.dictionary_override = argv[++i];
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
        std::cerr << "--image <path> is required\n";
        printUsage();
        return 2;
    }

    auto config = onboard::common::loadVisionConfig(options.config_dir);
    if (!options.dictionary_override.empty()) {
        config.aruco.dictionary = options.dictionary_override;
    }

    const cv::Mat image = cv::imread(options.image_path, cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "failed to read image: " << options.image_path << "\n";
        return 1;
    }

    const onboard::vision::ArucoDetector detector(config.aruco);
    const auto result = detector.detect(image, 1, 0);

    std::cout << "frame=" << options.image_path
              << " dictionary=" << detector.dictionaryName()
              << " markers=" << result.markers.size() << "\n";
    for (const auto& marker : result.markers) {
        std::cout << "id=" << marker.id
                  << " center=(" << marker.center_px.x << "," << marker.center_px.y << ")"
                  << " orientation=" << marker.orientation_deg << "deg"
                  << " corners=[";
        for (std::size_t index = 0; index < marker.corners_px.size(); ++index) {
            const auto& corner = marker.corners_px[index];
            std::cout << "(" << corner.x << "," << corner.y << ")";
            if (index + 1 < marker.corners_px.size()) {
                std::cout << ",";
            }
        }
        std::cout << "]\n";
    }

    return 0;
}
