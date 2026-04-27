#include "app/VisionDebugPipeline.hpp"

#include "camera/RpicamMjpegSource.hpp"
#include "network/UdpTelemetrySender.hpp"
#include "protocol/TelemetryMessage.hpp"
#include "video/UdpMjpegStreamer.hpp"
#include "vision/ArucoDetector.hpp"
#include "vision/LineDetector.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

namespace onboard::app {
namespace {

protocol::Point2f toProtocolPoint(const vision::Point2f& point)
{
    return {point.x, point.y};
}

protocol::MarkerTelemetry toProtocolMarker(const vision::MarkerObservation& marker)
{
    protocol::MarkerTelemetry output;
    output.id = marker.id;
    output.center_px = toProtocolPoint(marker.center_px);
    for (std::size_t index = 0; index < output.corners_px.size(); ++index) {
        output.corners_px[index] = toProtocolPoint(marker.corners_px[index]);
    }
    output.orientation_deg = marker.orientation_deg;
    return output;
}

protocol::LineTelemetry toProtocolLine(const vision::LineDetection& line)
{
    protocol::LineTelemetry output;
    output.detected = line.detected;
    output.tracking_point_px = toProtocolPoint(line.tracking_point_px);
    output.centroid_px = toProtocolPoint(line.centroid_px);
    output.center_offset_px = line.center_offset_px;
    output.angle_deg = line.angle_deg;
    output.confidence = line.confidence;
    output.contour_px.reserve(line.contour_px.size());
    for (const auto& point : line.contour_px) {
        output.contour_px.push_back(toProtocolPoint(point));
    }
    return output;
}

class LatestVideoSender {
public:
    bool start(const std::string& ip, std::uint16_t port)
    {
        if (!streamer_.open(ip, port)) {
            last_error_ = streamer_.lastError();
            return false;
        }

        running_ = true;
        worker_ = std::thread([this]() { workerLoop(); });
        return true;
    }

    void submit(camera::CameraFrame frame)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_) {
            ++dropped_frames_;
        }
        latest_ = std::move(frame);
        condition_.notify_one();
    }

    void stop()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            running_ = false;
            latest_.reset();
        }
        condition_.notify_one();
        if (worker_.joinable()) {
            worker_.join();
        }
        streamer_.close();
    }

    std::string takeLastError()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string output = std::move(last_error_);
        last_error_.clear();
        return output;
    }

    std::uint64_t sentFrames() const
    {
        return sent_frames_.load();
    }

    std::uint64_t droppedFrames() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_frames_;
    }

private:
    void workerLoop()
    {
        while (true) {
            std::optional<camera::CameraFrame> frame;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [this]() { return !running_ || latest_.has_value(); });
                if (!running_ && !latest_) {
                    return;
                }
                frame = std::move(latest_);
                latest_.reset();
            }

            if (frame && !streamer_.sendFrame(*frame)) {
                std::lock_guard<std::mutex> lock(mutex_);
                last_error_ = streamer_.lastError();
            } else if (frame) {
                ++sent_frames_;
            }
        }
    }

    video::UdpMjpegStreamer streamer_;
    std::atomic<bool> running_ {false};
    std::thread worker_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::optional<camera::CameraFrame> latest_;
    std::string last_error_;
    std::uint64_t dropped_frames_ = 0;
    std::atomic<std::uint64_t> sent_frames_ {0};
};

} // namespace

int VisionDebugPipeline::run(const VisionDebugPipelineOptions& options)
{
    LatestVideoSender video_sender;
    if (options.send_video &&
        !video_sender.start(options.network.gcs_ip, options.network.video_port)) {
        std::cerr << "failed to open UDP video streamer: "
                  << video_sender.takeLastError() << "\n";
        return 1;
    }

    network::UdpTelemetrySender telemetry_sender;
    if (options.send_telemetry &&
        !telemetry_sender.open(options.network.gcs_ip, options.network.telemetry_port)) {
        std::cerr << "failed to open UDP telemetry sender: "
                  << telemetry_sender.lastError() << "\n";
        video_sender.stop();
        return 1;
    }

    camera::RpicamMjpegSource camera;
    camera::RpicamOptions camera_options;
    camera_options.width = options.vision.video.width;
    camera_options.height = options.vision.video.height;
    camera_options.fps = options.vision.video.fps;
    camera_options.jpeg_quality = options.vision.video.jpeg_quality;
    if (!camera.open(camera_options)) {
        std::cerr << "failed to open rpicam source: " << camera.lastError() << "\n";
        video_sender.stop();
        return 1;
    }

    const vision::ArucoDetector aruco_detector(options.vision.aruco);
    const vision::LineDetector line_detector(options.vision.line);

    std::cout << "vision_debug_node\n"
              << "  destination: " << options.network.gcs_ip << "\n"
              << "  telemetry UDP port: " << options.network.telemetry_port << "\n"
              << "  video UDP port: " << options.network.video_port << "\n"
              << "  size: " << options.vision.video.width << 'x' << options.vision.video.height << "\n"
              << "  fps: " << options.vision.video.fps << "\n"
              << "  aruco_dictionary: " << aruco_detector.dictionaryName() << "\n"
              << "  aruco: " << (options.enable_aruco ? "on" : "off") << "\n"
              << "  line: " << (options.enable_line && options.vision.line.enabled ? "on" : "off") << "\n"
              << "  video: " << (options.send_video ? "on best-effort latest-frame" : "off") << "\n"
              << "  telemetry: " << (options.send_telemetry ? "on" : "off") << "\n"
              << "  count: " << (options.count == 0 ? std::string("forever") : std::to_string(options.count))
              << "\n";

    std::uint32_t telemetry_seq = 1;
    int processed_count = 0;
    while (options.count == 0 || processed_count < options.count) {
        camera::CameraFrame frame;
        if (!camera.readFrame(frame)) {
            std::cerr << "failed to read rpicam frame: " << camera.lastError() << "\n";
            video_sender.stop();
            return 1;
        }

        const auto processing_started = std::chrono::steady_clock::now();
        const cv::Mat encoded(
            1,
            static_cast<int>(frame.jpeg_data.size()),
            CV_8UC1,
            frame.jpeg_data.data());
        const cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);

        vision::VisionResult result;
        result.frame_seq = frame.frame_id;
        result.timestamp_ms = frame.timestamp_ms;
        result.width = frame.width;
        result.height = frame.height;

        double aruco_latency_ms = 0.0;
        double line_latency_ms = 0.0;

        if (!image.empty()) {
            if (options.enable_aruco) {
                const auto aruco_started = std::chrono::steady_clock::now();
                const auto aruco_result = aruco_detector.detect(image, frame.frame_id, frame.timestamp_ms);
                const auto aruco_finished = std::chrono::steady_clock::now();
                result.markers = aruco_result.markers;
                aruco_latency_ms = std::chrono::duration<double, std::milli>(
                    aruco_finished - aruco_started).count();
            }

            if (options.enable_line && options.vision.line.enabled) {
                const auto line_started = std::chrono::steady_clock::now();
                result.line = line_detector.detect(image);
                const auto line_finished = std::chrono::steady_clock::now();
                result.line_detected = result.line.detected;
                line_latency_ms = std::chrono::duration<double, std::milli>(
                    line_finished - line_started).count();
            }
        }
        const auto processing_finished = std::chrono::steady_clock::now();

        if (options.send_telemetry) {
            protocol::BringupTelemetry telemetry;
            telemetry.seq = telemetry_seq++;
            telemetry.timestamp_ms = frame.timestamp_ms;
            telemetry.camera.status = image.empty() ? "decode_failed" : "streaming";
            telemetry.camera.width = frame.width;
            telemetry.camera.height = frame.height;
            telemetry.camera.fps = options.vision.video.fps;
            telemetry.camera.frame_seq = frame.frame_id;
            telemetry.vision.marker_detected = !result.markers.empty();
            telemetry.vision.marker_id = result.markers.empty() ? -1 : result.markers.front().id;
            for (const auto& marker : result.markers) {
                telemetry.vision.markers.push_back(toProtocolMarker(marker));
            }
            telemetry.vision.line_detected = result.line.detected;
            telemetry.vision.line_offset = result.line.center_offset_px;
            telemetry.vision.line_angle = result.line.angle_deg;
            telemetry.vision.line = toProtocolLine(result.line);
            telemetry.debug.aruco_latency_ms = aruco_latency_ms;
            telemetry.debug.line_latency_ms = line_latency_ms;
            telemetry.debug.processing_latency_ms = std::chrono::duration<double, std::milli>(
                processing_finished - processing_started).count();
            telemetry.note = "vision_debug_node";

            const std::string payload = protocol::buildTelemetryJson(telemetry);
            if (!telemetry_sender.send(payload)) {
                std::cerr << "telemetry send warning: "
                          << telemetry_sender.lastError() << "\n";
            }
        }

        if (options.send_video) {
            video_sender.submit(frame);
            const std::string video_error = video_sender.takeLastError();
            if (!video_error.empty()) {
                std::cerr << "video send warning: " << video_error << "\n";
            }
        }

        ++processed_count;
        std::cout << "frame=" << frame.frame_id
                  << " markers=" << result.markers.size()
                  << " line=" << (result.line.detected ? "yes" : "no")
                  << " line_conf=" << result.line.confidence
                  << " jpeg_bytes=" << frame.jpeg_data.size()
                  << " video_sent=" << video_sender.sentFrames()
                  << " video_dropped=" << video_sender.droppedFrames()
                  << "\n";
    }

    video_sender.stop();
    return 0;
}

} // namespace onboard::app
