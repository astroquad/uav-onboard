#include "app/VisionDebugPipeline.hpp"

#include "camera/RpicamMjpegSource.hpp"
#include "mission/GridCoordinateTracker.hpp"
#include "mission/IntersectionDecision.hpp"
#include "network/UdpTelemetrySender.hpp"
#include "protocol/TelemetryMessage.hpp"
#include "video/UdpMjpegStreamer.hpp"
#include "vision/ArucoDetector.hpp"
#include "vision/IntersectionDetector.hpp"
#include "vision/IntersectionStabilizer.hpp"
#include "vision/LineDetector.hpp"
#include "vision/LineMaskBuilder.hpp"
#include "vision/LineStabilizer.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
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

protocol::IntersectionTelemetry toProtocolIntersection(const vision::IntersectionDetection& intersection)
{
    protocol::IntersectionTelemetry output;
    output.valid = intersection.valid;
    output.detected = intersection.intersection_detected;
    output.type = vision::intersectionTypeName(intersection.type);
    output.raw_type = vision::intersectionTypeName(intersection.raw_type);
    output.stable = intersection.stable;
    output.held = intersection.held;
    output.center_px = toProtocolPoint(intersection.center_px);
    output.raw_center_px = toProtocolPoint(intersection.raw_center_px);
    output.score = intersection.score;
    output.raw_score = intersection.raw_score;
    output.branch_mask = intersection.branch_mask;
    output.branch_count = intersection.branch_count;
    output.stable_frames = intersection.stable_frames;
    output.radius_px = intersection.radius_px;
    output.selected_mask_index = intersection.selected_mask_index;
    output.branches.reserve(intersection.branches.size());
    for (const auto& branch : intersection.branches) {
        protocol::BranchTelemetry branch_output;
        branch_output.direction = vision::branchDirectionName(branch.direction);
        branch_output.present = branch.present;
        branch_output.score = branch.score;
        branch_output.endpoint_px = toProtocolPoint(branch.endpoint_px);
        branch_output.angle_deg = branch.angle_deg;
        output.branches.push_back(branch_output);
    }
    return output;
}

protocol::GridNodeTelemetry toProtocolGridNode(const mission::GridNodeEvent& node)
{
    protocol::GridNodeTelemetry output;
    output.valid = node.valid;
    output.id = node.node_id;
    output.x = node.local_coord.x;
    output.y = node.local_coord.y;
    output.topology = vision::intersectionTypeName(node.topology);
    output.arrival_heading = mission::gridHeadingName(node.arrival_heading);
    output.camera_branch_mask = node.camera_branch_mask;
    output.grid_branch_mask = node.grid_branch_mask;
    output.first_node = node.first_node;
    output.origin_local_only = node.origin_local_only;
    return output;
}

protocol::IntersectionDecisionTelemetry toProtocolIntersectionDecision(
    const mission::IntersectionDecision& decision,
    const mission::GridNodeEvent& node)
{
    protocol::IntersectionDecisionTelemetry output;
    output.state = mission::decisionStateName(decision.state);
    output.action = mission::decisionActionName(decision.action);
    output.accepted_type = vision::intersectionTypeName(decision.accepted_type);
    output.best_observed_type = vision::intersectionTypeName(decision.best_observed_type);
    output.event_ready = decision.event_ready;
    output.turn_candidate = decision.turn_candidate;
    output.required_turn = decision.required_turn;
    output.front_available = decision.front_available;
    output.node_recorded = decision.node_recorded;
    output.cooldown_active = decision.cooldown_active;
    output.accepted_branch_mask = decision.accepted_branch_mask;
    output.window_frames = decision.window_frames;
    output.age_ms = decision.age_ms;
    output.confidence = decision.confidence;
    output.center_px = toProtocolPoint(decision.center_px);
    output.center_y_norm = decision.center_y_norm;
    output.approach_phase = decision.approach_phase;
    output.overshoot_risk = decision.overshoot_risk;
    output.too_late_to_turn = decision.too_late_to_turn;
    const char* labels[] = {"front", "right", "back", "left"};
    for (std::size_t index = 0; index < decision.branch_evidence.size(); ++index) {
        protocol::BranchEvidenceTelemetry branch;
        branch.direction = labels[index];
        branch.present_frames = decision.branch_evidence[index].present_frames;
        branch.max_score = decision.branch_evidence[index].max_score;
        branch.average_score = decision.branch_evidence[index].average_score;
        output.branches.push_back(branch);
    }
    output.node = toProtocolGridNode(node);
    return output;
}

std::string branchScoreSummary(const vision::IntersectionDetection& intersection)
{
    std::ostringstream stream;
    for (std::size_t index = 0; index < intersection.branches.size(); ++index) {
        if (index > 0) {
            stream << ',';
        }
        const auto& branch = intersection.branches[index];
        const char* label = "?";
        switch (branch.direction) {
        case vision::BranchDirection::Front:
            label = "F";
            break;
        case vision::BranchDirection::Right:
            label = "R";
            break;
        case vision::BranchDirection::Back:
            label = "B";
            break;
        case vision::BranchDirection::Left:
            label = "L";
            break;
        }
        stream << label << ':' << branch.score;
    }
    return stream.str();
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

std::string trimText(std::string value)
{
    value.erase(
        std::remove(value.begin(), value.end(), '\0'),
        value.end());
    while (!value.empty() &&
           (value.back() == '\n' || value.back() == '\r' || value.back() == ' ' || value.back() == '\t')) {
        value.pop_back();
    }
    std::size_t first = 0;
    while (first < value.size() && (value[first] == ' ' || value[first] == '\t')) {
        ++first;
    }
    if (first > 0) {
        value.erase(0, first);
    }
    return value;
}

std::string readFirstLine(const std::string& path)
{
    std::ifstream input(path);
    std::string line;
    if (!std::getline(input, line)) {
        return {};
    }
    return trimText(line);
}

std::string readPrettyOsRelease()
{
#ifdef _WIN32
    return {};
#else
    std::ifstream input("/etc/os-release");
    std::string line;
    while (std::getline(input, line)) {
        constexpr const char* key = "PRETTY_NAME=";
        if (line.rfind(key, 0) != 0) {
            continue;
        }
        std::string value = line.substr(std::string(key).size());
        if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
            value = value.substr(1, value.size() - 2);
        }
        return trimText(value);
    }
    return {};
#endif
}

std::string runCommandFirstLine(const std::string& command)
{
#ifdef _WIN32
    (void)command;
    return {};
#else
    FILE* pipe = popen(command.c_str(), "r");
    if (pipe == nullptr) {
        return {};
    }
    char buffer[256] {};
    std::string line;
    if (std::fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        line = buffer;
    }
    pclose(pipe);
    return trimText(line);
#endif
}

double readUptimeS()
{
#ifdef _WIN32
    return 0.0;
#else
    std::ifstream input("/proc/uptime");
    double uptime = 0.0;
    input >> uptime;
    return uptime;
#endif
}

double readLoad1m()
{
#ifdef _WIN32
    return 0.0;
#else
    std::ifstream input("/proc/loadavg");
    double load = 0.0;
    input >> load;
    return load;
#endif
}

std::uint64_t readMemAvailableKb()
{
#ifdef _WIN32
    return 0;
#else
    std::ifstream input("/proc/meminfo");
    std::string key;
    std::uint64_t value = 0;
    std::string unit;
    while (input >> key >> value >> unit) {
        if (key == "MemAvailable:") {
            return value;
        }
    }
    return 0;
#endif
}

double readWifiSignalDbm()
{
#ifdef _WIN32
    return 0.0;
#else
    std::ifstream input("/proc/net/wireless");
    std::string line;
    while (std::getline(input, line)) {
        if (line.find(':') == std::string::npos) {
            continue;
        }
        std::istringstream stream(line);
        std::string iface;
        std::string status;
        double link = 0.0;
        double level = 0.0;
        stream >> iface >> status >> link >> level;
        if (stream && iface.rfind("wlan", 0) == 0) {
            return level;
        }
    }
    return 0.0;
#endif
}

double readWifiTxBitrateMbps()
{
#ifdef _WIN32
    return 0.0;
#else
    const std::string output = runCommandFirstLine("iw dev wlan0 link 2>/dev/null | grep 'tx bitrate' || true");
    const auto position = output.find("tx bitrate:");
    if (position == std::string::npos) {
        return 0.0;
    }
    std::istringstream stream(output.substr(position + std::string("tx bitrate:").size()));
    double bitrate = 0.0;
    stream >> bitrate;
    return bitrate;
#endif
}

protocol::SystemTelemetry readSystemTelemetry()
{
    protocol::SystemTelemetry telemetry;
#ifdef _WIN32
    return telemetry;
#else
    telemetry.board_model = readFirstLine("/proc/device-tree/model");
    if (telemetry.board_model.empty()) {
        telemetry.board_model = readFirstLine("/sys/firmware/devicetree/base/model");
    }
    telemetry.os_release = readPrettyOsRelease();
    telemetry.uptime_s = readUptimeS();
    telemetry.cpu_temp_c = readCpuTempC();
    telemetry.throttled_raw = runCommandFirstLine("vcgencmd get_throttled 2>/dev/null || true");
    telemetry.cpu_load_1m = readLoad1m();
    telemetry.mem_available_kb = readMemAvailableKb();
    telemetry.wifi_signal_dbm = readWifiSignalDbm();
    telemetry.wifi_tx_bitrate_mbps = readWifiTxBitrateMbps();
    return telemetry;
#endif
}

class RateMeter {
public:
    double note(std::chrono::steady_clock::time_point now)
    {
        if (window_start_.time_since_epoch().count() == 0) {
            window_start_ = now;
        }
        ++count_;
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - window_start_);
        if (elapsed.count() >= 1000) {
            rate_ = count_ * 1000.0 / std::max(1, static_cast<int>(elapsed.count()));
            count_ = 0;
            window_start_ = now;
        }
        return rate_;
    }

    double rate() const
    {
        return rate_;
    }

private:
    std::chrono::steady_clock::time_point window_start_ {};
    int count_ = 0;
    double rate_ = 0.0;
};

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

    std::uint64_t sendFailures() const
    {
        return send_failures_.load();
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

                send_failures_.fetch_add(1);
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
    std::atomic<std::uint64_t> send_failures_ {0};
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
            options.vision.debug_video.chunk_pacing_us)) {
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
    camera_options.camera_index = options.vision.camera.device;
    camera_options.width = options.vision.camera.width;
    camera_options.height = options.vision.camera.height;
    camera_options.fps = options.vision.camera.fps;
    camera_options.jpeg_quality = options.vision.camera.jpeg_quality;
    camera_options.codec = options.vision.camera.codec;
    camera_options.autofocus_mode = options.vision.camera.autofocus_mode;
    camera_options.autofocus_range = options.vision.camera.autofocus_range;
    camera_options.autofocus_speed = options.vision.camera.autofocus_speed;
    camera_options.autofocus_window = options.vision.camera.autofocus_window;
    camera_options.lens_position = options.vision.camera.lens_position;
    camera_options.exposure = options.vision.camera.exposure;
    camera_options.shutter_us = options.vision.camera.shutter_us;
    camera_options.gain = options.vision.camera.gain;
    camera_options.ev = options.vision.camera.ev;
    camera_options.awb = options.vision.camera.awb;
    camera_options.awbgains = options.vision.camera.awbgains;
    camera_options.metering = options.vision.camera.metering;
    camera_options.denoise = options.vision.camera.denoise;
    camera_options.sharpness = options.vision.camera.sharpness;
    camera_options.contrast = options.vision.camera.contrast;
    camera_options.brightness = options.vision.camera.brightness;
    camera_options.saturation = options.vision.camera.saturation;
    camera_options.roi = options.vision.camera.roi;
    camera_options.tuning_file = options.vision.camera.tuning_file;
    camera_options.hflip = options.vision.camera.hflip;
    camera_options.vflip = options.vision.camera.vflip;
    camera_options.rotation = options.vision.camera.rotation;
    if (!camera.open(camera_options)) {
        std::cerr << "failed to open rpicam source: " << camera.lastError() << "\n";
        video_sender.stop();
        return 1;
    }

    const vision::ArucoDetector aruco_detector(options.vision.aruco);
    const vision::LineMaskBuilder line_mask_builder(options.vision.line);
    const vision::LineDetector line_detector(options.vision.line);
    vision::LineStabilizer line_stabilizer(options.vision.line);
    const vision::IntersectionDetector intersection_detector(options.vision.line);
    vision::IntersectionStabilizer intersection_stabilizer(options.vision.line);
    mission::IntersectionDecisionEngine intersection_decision_engine(options.vision.intersection_decision);
    mission::GridCoordinateTracker grid_tracker(options.vision.intersection_decision);

    std::cout << "vision_debug_node\n"
              << "  destination: " << options.network.gcs_ip << "\n"
              << "  telemetry UDP port: " << options.network.telemetry_port << "\n"
              << "  video UDP port: " << options.network.video_port << "\n"
              << "  camera: " << options.vision.camera.sensor_model
              << " index=" << options.vision.camera.device << "\n"
              << "  size: " << options.vision.camera.width << 'x' << options.vision.camera.height << "\n"
              << "  fps: " << options.vision.camera.fps << "\n"
              << "  jpeg_quality: " << options.vision.camera.jpeg_quality << "\n"
              << "  autofocus_mode: " << options.vision.camera.autofocus_mode << "\n"
              << "  lens_position: " << options.vision.camera.lens_position << "\n"
              << "  exposure: " << options.vision.camera.exposure << "\n"
              << "  video_send_fps: " << options.vision.debug_video.send_fps << "\n"
              << "  video_chunk_pacing_us: " << options.vision.debug_video.chunk_pacing_us << "\n"
              << "  aruco_dictionary: " << aruco_detector.dictionaryName() << "\n"
              << "  aruco: " << (options.enable_aruco ? "on" : "off") << "\n"
              << "  line: " << (options.enable_line && options.vision.line.enabled ? "on" : "off") << "\n"
              << "  line_mode: " << options.vision.line.mode << "\n"
              << "  line_process_width: " << options.vision.line.process_width << "\n"
              << "  line_filter: " << (options.vision.line.filter_enabled ? "on" : "off") << "\n"
              << "  intersection: " << (options.enable_line && options.vision.line.enabled ? "on" : "off") << "\n"
              << "  intersection_threshold: " << options.vision.line.intersection_threshold << "\n"
              << "  intersection_decision: " << (options.vision.intersection_decision.enabled ? "on" : "off") << "\n"
              << "  intersection_decision_window: "
              << options.vision.intersection_decision.cruise_window_frames << " frames\n"
              << "  intersection_decision_min_branch_score: "
              << options.vision.intersection_decision.min_branch_score << "\n"
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
    protocol::SystemTelemetry last_system = readSystemTelemetry();
    RateMeter capture_rate;
    RateMeter processing_rate;
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
        const double capture_fps = capture_rate.note(read_finished);
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
        double intersection_latency_ms = 0.0;
        double intersection_decision_latency_ms = 0.0;
        mission::IntersectionDecision intersection_decision;
        mission::GridNodeEvent grid_node;

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
                const auto line_masks = line_mask_builder.build(image);
                const auto raw_line = line_detector.detect(line_masks);
                const auto line_finished = std::chrono::steady_clock::now();
                result.line = line_stabilizer.update(raw_line, frame.width);
                result.line_detected = result.line.detected;
                line_latency_ms = std::chrono::duration<double, std::milli>(
                    line_finished - line_started).count();

                const auto intersection_started = std::chrono::steady_clock::now();
                const auto raw_intersection = intersection_detector.detect(line_masks, raw_line);
                result.intersection = intersection_stabilizer.update(raw_intersection);
                result.intersection_detected = result.intersection.intersection_detected;
                const auto intersection_finished = std::chrono::steady_clock::now();
                intersection_latency_ms = std::chrono::duration<double, std::milli>(
                    intersection_finished - intersection_started).count();

                const auto decision_started = std::chrono::steady_clock::now();
                intersection_decision = intersection_decision_engine.update(
                    result.intersection,
                    frame.width,
                    frame.height,
                    frame.frame_id,
                    frame.timestamp_ms,
                    false);
                if (intersection_decision.event_ready) {
                    grid_node = grid_tracker.update(
                        intersection_decision,
                        frame.frame_id,
                        frame.timestamp_ms);
                }
                const auto decision_finished = std::chrono::steady_clock::now();
                intersection_decision_latency_ms = std::chrono::duration<double, std::milli>(
                    decision_finished - decision_started).count();
            }
        }
        const auto processing_finished = std::chrono::steady_clock::now();
        const double processing_fps = processing_rate.note(processing_finished);

        double telemetry_build_ms = last_telemetry_build_ms;
        double telemetry_send_ms = last_telemetry_send_ms;
        if (options.send_telemetry) {
            if (processed_count % 30 == 0) {
                last_system = readSystemTelemetry();
            }
            protocol::BringupTelemetry telemetry;
            telemetry.seq = telemetry_seq++;
            telemetry.timestamp_ms = frame.timestamp_ms;
            telemetry.system = last_system;
            telemetry.camera.status = image.empty() ? "decode_failed" : "streaming";
            telemetry.camera.sensor_model = options.vision.camera.sensor_model;
            telemetry.camera.camera_index = options.vision.camera.device;
            telemetry.camera.width = frame.width;
            telemetry.camera.height = frame.height;
            telemetry.camera.fps = options.vision.camera.fps;
            telemetry.camera.configured_fps = options.vision.camera.fps;
            telemetry.camera.measured_capture_fps = capture_fps;
            telemetry.camera.frame_seq = frame.frame_id;
            telemetry.camera.autofocus_mode = options.vision.camera.autofocus_mode;
            telemetry.camera.lens_position = options.vision.camera.lens_position;
            telemetry.camera.exposure_mode = options.vision.camera.exposure;
            telemetry.camera.shutter_us = options.vision.camera.shutter_us;
            telemetry.camera.gain = options.vision.camera.gain;
            telemetry.camera.awb = options.vision.camera.awb;
            telemetry.vision.marker_detected = !result.markers.empty();
            telemetry.vision.marker_id = result.markers.empty() ? -1 : result.markers.front().id;
            for (const auto& marker : result.markers) {
                telemetry.vision.markers.push_back(toProtocolMarker(marker));
            }
            telemetry.vision.line_detected = result.line.detected;
            telemetry.vision.line_offset = result.line.center_offset_px;
            telemetry.vision.line_angle = result.line.angle_deg;
            telemetry.vision.line = toProtocolLine(result.line);
            telemetry.vision.intersection_detected = result.intersection.intersection_detected;
            telemetry.vision.intersection_score = result.intersection.score;
            telemetry.vision.intersection = toProtocolIntersection(result.intersection);
            telemetry.vision.intersection_decision =
                toProtocolIntersectionDecision(intersection_decision, grid_node);
            telemetry.vision.grid_node = toProtocolGridNode(grid_node);
            if (grid_node.valid) {
                telemetry.grid.col = grid_node.local_coord.x;
                telemetry.grid.row = grid_node.local_coord.y;
            }
            telemetry.debug.aruco_latency_ms = aruco_latency_ms;
            telemetry.debug.line_latency_ms = line_latency_ms;
            telemetry.debug.intersection_latency_ms = intersection_latency_ms;
            telemetry.debug.intersection_decision_latency_ms = intersection_decision_latency_ms;
            telemetry.debug.processing_latency_ms = std::chrono::duration<double, std::milli>(
                processing_finished - processing_started).count();
            telemetry.debug.read_frame_ms = read_frame_ms;
            telemetry.debug.jpeg_decode_ms = jpeg_decode_ms;
            telemetry.debug.telemetry_build_ms = last_telemetry_build_ms;
            telemetry.debug.telemetry_send_ms = last_telemetry_send_ms;
            telemetry.debug.video_submit_ms = last_video_submit_ms;
            telemetry.debug.video_send_ms = video_sender.lastSendMs();
            telemetry.debug.capture_fps = capture_fps;
            telemetry.debug.processing_fps = processing_fps;
            telemetry.debug.debug_video_send_fps = options.vision.debug_video.send_fps;
            telemetry.debug.video_chunk_pacing_us = options.vision.debug_video.chunk_pacing_us;
            telemetry.debug.cpu_temp_c = last_system.cpu_temp_c;
            telemetry.debug.telemetry_bytes = last_telemetry_bytes;
            telemetry.debug.video_jpeg_bytes = jpeg_bytes;
            telemetry.debug.video_sent_frames = video_sender.sentFrames();
            telemetry.debug.video_dropped_frames = video_sender.droppedFrames();
            telemetry.debug.video_skipped_frames = video_sender.skippedFrames();
            telemetry.debug.video_chunks_sent = video_sender.chunksSent();
            telemetry.debug.video_send_failures = video_sender.sendFailures();
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
            const int send_fps = std::clamp(
                options.vision.debug_video.send_fps,
                1,
                std::max(1, options.vision.camera.fps));
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
                  << " ix_type=" << vision::intersectionTypeName(result.intersection.type)
                  << " ix_raw=" << vision::intersectionTypeName(result.intersection.raw_type)
                  << " ix_valid=" << (result.intersection.valid ? "yes" : "no")
                  << " ix_stable=" << (result.intersection.stable ? "yes" : "no")
                  << " ix_held=" << (result.intersection.held ? "yes" : "no")
                  << " ix_detected=" << (result.intersection.intersection_detected ? "yes" : "no")
                  << " ix_score=" << result.intersection.score
                  << " ix_center=(" << result.intersection.center_px.x
                  << ',' << result.intersection.center_px.y << ')'
                  << " branches=" << branchScoreSummary(result.intersection)
                  << " dec_state=" << mission::decisionStateName(intersection_decision.state)
                  << " dec_action=" << mission::decisionActionName(intersection_decision.action)
                  << " dec_type=" << vision::intersectionTypeName(intersection_decision.accepted_type)
                  << " dec_best=" << vision::intersectionTypeName(intersection_decision.best_observed_type)
                  << " dec_window=" << intersection_decision.window_frames
                  << " dec_conf=" << intersection_decision.confidence
                  << " dec_mask=" << static_cast<int>(intersection_decision.accepted_branch_mask)
                  << " dec_event=" << (intersection_decision.event_ready ? "yes" : "no")
                  << " dec_front=" << (intersection_decision.front_available ? "yes" : "no")
                  << " dec_req_turn=" << (intersection_decision.required_turn ? "yes" : "no")
                  << " dec_y=" << intersection_decision.center_y_norm
                  << " dec_phase=" << intersection_decision.approach_phase
                  << " dec_overshoot=" << (intersection_decision.overshoot_risk ? "yes" : "no")
                  << " grid_valid=" << (grid_node.valid ? "yes" : "no")
                  << " grid=(" << grid_node.local_coord.x << ',' << grid_node.local_coord.y << ')'
                  << " grid_origin=" << (grid_node.origin_local_only ? "local" : "official")
                  << " read_ms=" << read_frame_ms
                  << " decode_ms=" << jpeg_decode_ms
                  << " aruco_ms=" << aruco_latency_ms
                  << " line_ms=" << line_latency_ms
                  << " ix_ms=" << intersection_latency_ms
                  << " dec_ms=" << intersection_decision_latency_ms
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
