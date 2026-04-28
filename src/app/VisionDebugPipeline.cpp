#include "app/VisionDebugPipeline.hpp"

#include "camera/RpicamMjpegSource.hpp"
#include "network/UdpTelemetrySender.hpp"
#include "protocol/TelemetryMessage.hpp"
#include "video/UdpMjpegStreamer.hpp"
#include "vision/ArucoDetector.hpp"
#include "vision/LineDetector.hpp"
#include "vision/LineStabilizer.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <fstream>
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
    output.raw_detected = line.raw_detected;
    output.filtered = line.filtered;
    output.held = line.held;
    output.rejected_jump = line.rejected_jump;
    output.tracking_point_px = toProtocolPoint(line.tracking_point_px);
    output.raw_tracking_point_px = toProtocolPoint(line.raw_tracking_point_px);
    output.centroid_px = toProtocolPoint(line.centroid_px);
    output.center_offset_px = line.center_offset_px;
    output.raw_center_offset_px = line.raw_center_offset_px;
    output.angle_deg = line.angle_deg;
    output.raw_angle_deg = line.raw_angle_deg;
    output.confidence = line.confidence;
    output.contour_px.reserve(line.contour_px.size());
    for (const auto& point : line.contour_px) {
        output.contour_px.push_back(toProtocolPoint(point));
    }
    return output;
}

double readCpuTempC()
{
#ifdef _WIN32
    return 0.0;
#else
    std::ifstream input("/sys/class/thermal/thermal_zone0/temp");
    double milli_celsius = 0.0;
    if (!(input >> milli_celsius)) {
        return 0.0;
    }
    return milli_celsius / 1000.0;
#endif
}

class LatestVideoSender {
public:
    bool start(const std::string& ip, std::uint16_t port, int chunk_pacing_us)
    {
        if (!streamer_.open(ip, port)) {
            last_error_ = streamer_.lastError();
            return false;
        }
        streamer_.setChunkPacingUs(chunk_pacing_us);

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

    void noteSkippedFrame()
    {
        skipped_frames_.fetch_add(1);
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

    std::uint64_t skippedFrames() const
    {
        return skipped_frames_.load();
    }

    std::uint64_t chunksSent() const
    {
        return chunks_sent_.load();
    }

    int lastChunkCount() const
    {
        return last_chunk_count_.load();
    }

    double lastSendMs() const
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        return last_send_ms_;
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

            if (frame) {
                const auto send_started = std::chrono::steady_clock::now();
                const bool sent = streamer_.sendFrame(*frame);
                const auto send_finished = std::chrono::steady_clock::now();
                const double send_ms = std::chrono::duration<double, std::milli>(
                    send_finished - send_started).count();
                {
                    std::lock_guard<std::mutex> lock(stats_mutex_);
                    last_send_ms_ = send_ms;
                }
                last_chunk_count_.store(streamer_.lastChunkCount());
                if (sent) {
                    ++sent_frames_;
                    chunks_sent_.fetch_add(static_cast<std::uint64_t>(
                        std::max(0, streamer_.lastChunkCount())));
                    continue;
                }

                std::lock_guard<std::mutex> lock(mutex_);
                last_error_ = streamer_.lastError();
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
    std::atomic<std::uint64_t> skipped_frames_ {0};
    std::atomic<std::uint64_t> chunks_sent_ {0};
    std::atomic<int> last_chunk_count_ {0};
    mutable std::mutex stats_mutex_;
    double last_send_ms_ = 0.0;
};

} // namespace

int VisionDebugPipeline::run(const VisionDebugPipelineOptions& options)
{
    LatestVideoSender video_sender;
    if (options.send_video &&
        !video_sender.start(
            options.network.gcs_ip,
            options.network.video_port,
            options.vision.video.chunk_pacing_us)) {
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
    vision::LineStabilizer line_stabilizer(options.vision.line);

    std::cout << "vision_debug_node\n"
              << "  destination: " << options.network.gcs_ip << "\n"
              << "  telemetry UDP port: " << options.network.telemetry_port << "\n"
              << "  video UDP port: " << options.network.video_port << "\n"
              << "  size: " << options.vision.video.width << 'x' << options.vision.video.height << "\n"
              << "  fps: " << options.vision.video.fps << "\n"
              << "  video_send_fps: " << options.vision.video.send_fps << "\n"
              << "  video_chunk_pacing_us: " << options.vision.video.chunk_pacing_us << "\n"
              << "  aruco_dictionary: " << aruco_detector.dictionaryName() << "\n"
              << "  aruco: " << (options.enable_aruco ? "on" : "off") << "\n"
              << "  line: " << (options.enable_line && options.vision.line.enabled ? "on" : "off") << "\n"
              << "  line_mode: " << options.vision.line.mode << "\n"
              << "  line_process_width: " << options.vision.line.process_width << "\n"
              << "  line_filter: " << (options.vision.line.filter_enabled ? "on" : "off") << "\n"
              << "  video: " << (options.send_video ? "on best-effort latest-frame" : "off") << "\n"
              << "  telemetry: " << (options.send_telemetry ? "on" : "off") << "\n"
              << "  count: " << (options.count == 0 ? std::string("forever") : std::to_string(options.count))
              << "\n";

    std::uint32_t telemetry_seq = 1;
    int processed_count = 0;
    double last_telemetry_build_ms = 0.0;
    double last_telemetry_send_ms = 0.0;
    double last_video_submit_ms = 0.0;
    std::uint64_t last_telemetry_bytes = 0;
    double last_cpu_temp_c = readCpuTempC();
    auto last_video_sent_time = std::chrono::steady_clock::time_point {};
    while (options.count == 0 || processed_count < options.count) {
        camera::CameraFrame frame;
        const auto read_started = std::chrono::steady_clock::now();
        if (!camera.readFrame(frame)) {
            std::cerr << "failed to read rpicam frame: " << camera.lastError() << "\n";
            video_sender.stop();
            return 1;
        }
        const auto read_finished = std::chrono::steady_clock::now();
        const double read_frame_ms = std::chrono::duration<double, std::milli>(
            read_finished - read_started).count();
        const std::uint32_t frame_id = frame.frame_id;
        const std::size_t jpeg_bytes = frame.jpeg_data.size();

        const auto processing_started = std::chrono::steady_clock::now();
        const auto decode_started = std::chrono::steady_clock::now();
        const cv::Mat encoded(
            1,
            static_cast<int>(frame.jpeg_data.size()),
            CV_8UC1,
            frame.jpeg_data.data());
        const cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
        const auto decode_finished = std::chrono::steady_clock::now();
        const double jpeg_decode_ms = std::chrono::duration<double, std::milli>(
            decode_finished - decode_started).count();

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
                const auto raw_line = line_detector.detect(image);
                const auto line_finished = std::chrono::steady_clock::now();
                result.line = line_stabilizer.update(raw_line, frame.width);
                result.line_detected = result.line.detected;
                line_latency_ms = std::chrono::duration<double, std::milli>(
                    line_finished - line_started).count();
            }
        }
        const auto processing_finished = std::chrono::steady_clock::now();

        double telemetry_build_ms = last_telemetry_build_ms;
        double telemetry_send_ms = last_telemetry_send_ms;
        if (options.send_telemetry) {
            if (processed_count % 30 == 0) {
                last_cpu_temp_c = readCpuTempC();
            }
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
            telemetry.debug.read_frame_ms = read_frame_ms;
            telemetry.debug.jpeg_decode_ms = jpeg_decode_ms;
            telemetry.debug.telemetry_build_ms = last_telemetry_build_ms;
            telemetry.debug.telemetry_send_ms = last_telemetry_send_ms;
            telemetry.debug.video_submit_ms = last_video_submit_ms;
            telemetry.debug.video_send_ms = video_sender.lastSendMs();
            telemetry.debug.cpu_temp_c = last_cpu_temp_c;
            telemetry.debug.telemetry_bytes = last_telemetry_bytes;
            telemetry.debug.video_jpeg_bytes = jpeg_bytes;
            telemetry.debug.video_sent_frames = video_sender.sentFrames();
            telemetry.debug.video_dropped_frames = video_sender.droppedFrames();
            telemetry.debug.video_skipped_frames = video_sender.skippedFrames();
            telemetry.debug.video_chunks_sent = video_sender.chunksSent();
            telemetry.debug.video_chunk_count = video_sender.lastChunkCount();
            telemetry.debug.line_mask_count = result.line.mask_count;
            telemetry.debug.line_contours_found = result.line.contours_found;
            telemetry.debug.line_candidates_evaluated = result.line.candidates_evaluated;
            telemetry.debug.line_roi_pixels = result.line.roi_pixels;
            telemetry.debug.line_selected_contour_points = result.line.selected_contour_points;
            telemetry.note = "vision_debug_node";

            const auto telemetry_build_started = std::chrono::steady_clock::now();
            const std::string payload = protocol::buildTelemetryJson(telemetry);
            const auto telemetry_build_finished = std::chrono::steady_clock::now();
            telemetry_build_ms = std::chrono::duration<double, std::milli>(
                telemetry_build_finished - telemetry_build_started).count();
            last_telemetry_build_ms = telemetry_build_ms;
            last_telemetry_bytes = payload.size();

            const auto telemetry_send_started = std::chrono::steady_clock::now();
            if (!telemetry_sender.send(payload)) {
                std::cerr << "telemetry send warning: "
                          << telemetry_sender.lastError() << "\n";
            }
            const auto telemetry_send_finished = std::chrono::steady_clock::now();
            telemetry_send_ms = std::chrono::duration<double, std::milli>(
                telemetry_send_finished - telemetry_send_started).count();
            last_telemetry_send_ms = telemetry_send_ms;
        }

        double video_submit_ms = last_video_submit_ms;
        if (options.send_video) {
            const int send_fps = std::clamp(options.vision.video.send_fps, 1, options.vision.video.fps);
            const auto now = std::chrono::steady_clock::now();
            const auto min_period = std::chrono::microseconds(1000000 / send_fps);
            const bool should_send =
                last_video_sent_time.time_since_epoch().count() == 0 ||
                now - last_video_sent_time >= min_period;
            if (should_send) {
                const auto video_submit_started = std::chrono::steady_clock::now();
                video_sender.submit(std::move(frame));
                const auto video_submit_finished = std::chrono::steady_clock::now();
                video_submit_ms = std::chrono::duration<double, std::milli>(
                    video_submit_finished - video_submit_started).count();
                last_video_submit_ms = video_submit_ms;
                last_video_sent_time = now;
                const std::string video_error = video_sender.takeLastError();
                if (!video_error.empty()) {
                    std::cerr << "video send warning: " << video_error << "\n";
                }
            } else {
                video_sender.noteSkippedFrame();
            }
        }

        ++processed_count;
        std::cout << "frame=" << frame_id
                  << " markers=" << result.markers.size()
                  << " line=" << (result.line.detected ? "yes" : "no")
                  << " raw_line=" << (result.line.raw_detected ? "yes" : "no")
                  << " held=" << (result.line.held ? "yes" : "no")
                  << " rejected_jump=" << (result.line.rejected_jump ? "yes" : "no")
                  << " line_conf=" << result.line.confidence
                  << " read_ms=" << read_frame_ms
                  << " decode_ms=" << jpeg_decode_ms
                  << " aruco_ms=" << aruco_latency_ms
                  << " line_ms=" << line_latency_ms
                  << " process_ms=" << std::chrono::duration<double, std::milli>(
                         processing_finished - processing_started).count()
                  << " json_ms=" << telemetry_build_ms
                  << " tsend_ms=" << telemetry_send_ms
                  << " vsubmit_ms=" << video_submit_ms
                  << " vsend_ms=" << video_sender.lastSendMs()
                  << " jpeg_bytes=" << jpeg_bytes
                  << " video_chunks=" << video_sender.lastChunkCount()
                  << " video_sent=" << video_sender.sentFrames()
                  << " video_dropped=" << video_sender.droppedFrames()
                  << " video_skipped=" << video_sender.skippedFrames()
                  << "\n";
    }

    video_sender.stop();
    return 0;
}

} // namespace onboard::app
