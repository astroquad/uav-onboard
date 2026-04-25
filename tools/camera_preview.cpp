#include "vision/OpenCvCameraSource.hpp"

#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

struct Options {
    int device = 0;
    int width = 640;
    int height = 480;
    int fps = 30;
    int frames = 30;
    std::string save_path = "camera_smoke.jpg";
};

void printUsage()
{
    std::cout
        << "Usage: camera_preview [options]\n"
        << "\n"
        << "Options:\n"
        << "  --device <n>    Camera device index, default 0\n"
        << "  --width <n>     Requested frame width, default 640\n"
        << "  --height <n>    Requested frame height, default 480\n"
        << "  --fps <n>       Requested FPS, default 30\n"
        << "  --frames <n>    Number of frames to read, default 30\n"
        << "  --save <path>   Save the last captured frame, default camera_smoke.jpg\n"
        << "  --no-save       Do not save a frame\n"
        << "  -h, --help      Show this help\n";
}

int parseInt(const std::string& value, int fallback)
{
    try {
        return std::stoi(value);
    } catch (...) {
        return fallback;
    }
}

Options parseOptions(int argc, char** argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--device" && i + 1 < argc) {
            options.device = parseInt(argv[++i], options.device);
        } else if (arg == "--width" && i + 1 < argc) {
            options.width = parseInt(argv[++i], options.width);
        } else if (arg == "--height" && i + 1 < argc) {
            options.height = parseInt(argv[++i], options.height);
        } else if (arg == "--fps" && i + 1 < argc) {
            options.fps = parseInt(argv[++i], options.fps);
        } else if (arg == "--frames" && i + 1 < argc) {
            options.frames = parseInt(argv[++i], options.frames);
        } else if (arg == "--save" && i + 1 < argc) {
            options.save_path = argv[++i];
        } else if (arg == "--no-save") {
            options.save_path.clear();
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

    onboard::vision::OpenCvCameraSource camera;
    onboard::vision::CameraOpenOptions camera_options;
    camera_options.device_index = options.device;
    camera_options.width = options.width;
    camera_options.height = options.height;
    camera_options.fps = options.fps;

    std::cout << "Opening camera device " << options.device
              << " requested=" << options.width << "x" << options.height
              << "@" << options.fps << "fps\n";

    if (!camera.open(camera_options)) {
        std::cerr << "camera open failed: " << camera.lastError() << "\n";
        std::cerr << "Check rpicam-hello/libcamera-hello first, then verify /dev/video* access.\n";
        return 1;
    }

    cv::Mat frame;
    int read_count = 0;
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < options.frames; ++i) {
        if (!camera.read(frame)) {
            std::cerr << "frame read failed at index " << i << ": " << camera.lastError() << "\n";
            return 1;
        }
        ++read_count;
    }
    const auto end = std::chrono::steady_clock::now();
    const std::chrono::duration<double> elapsed = end - start;
    const double measured_fps = elapsed.count() > 0.0 ? read_count / elapsed.count() : 0.0;

    std::cout << "Captured " << read_count << " frame(s)\n";
    std::cout << "Last frame: " << frame.cols << "x" << frame.rows
              << " channels=" << frame.channels() << "\n";
    std::cout << "Measured FPS: " << measured_fps << "\n";

    if (!options.save_path.empty()) {
        if (!cv::imwrite(options.save_path, frame)) {
            std::cerr << "failed to save frame to " << options.save_path << "\n";
            return 1;
        }
        std::cout << "Saved last frame to " << options.save_path << "\n";
    }

    return 0;
}
