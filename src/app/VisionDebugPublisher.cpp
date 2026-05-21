#include "app/VisionDebugPublisher.hpp"

#include "camera/CameraFrame.hpp"
#include "network/UdpTelemetrySender.hpp"
#include "protocol/TelemetryMessage.hpp"
#include "video/UdpMjpegStreamer.hpp"
#include "vision/IntersectionDetector.hpp"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

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

std::string trimText(std::string value)
{
    value.erase(std::remove(value.begin(), value.end(), '\0'), value.end());
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
}

std::string runCommandFirstLine(const std::string& command)
{
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
}

double readDoubleFromFile(const std::string& path, double scale)
{
    std::ifstream input(path);
    double value = 0.0;
    if (!(input >> value)) {
        return 0.0;
    }
    return value * scale;
}

protocol::SystemTelemetry readSystemTelemetry()
{
    protocol::SystemTelemetry telemetry;
    telemetry.board_model = readFirstLine("/proc/device-tree/model");
    if (telemetry.board_model.empty()) {
        telemetry.board_model = readFirstLine("/sys/firmware/devicetree/base/model");
    }
    telemetry.os_release = readPrettyOsRelease();
    telemetry.cpu_temp_c = readDoubleFromFile("/sys/class/thermal/thermal_zone0/temp", 0.001);
    telemetry.throttled_raw = runCommandFirstLine("vcgencmd get_throttled 2>/dev/null || true");

    std::ifstream uptime_input("/proc/uptime");
    uptime_input >> telemetry.uptime_s;

    std::ifstream load_input("/proc/loadavg");
    load_input >> telemetry.cpu_load_1m;

    std::ifstream mem_input("/proc/meminfo");
    std::string key;
    std::uint64_t value = 0;
    std::string unit;
    while (mem_input >> key >> value >> unit) {
        if (key == "MemAvailable:") {
            telemetry.mem_available_kb = value;
            break;
        }
    }

    std::ifstream wireless_input("/proc/net/wireless");
    std::string line;
    while (std::getline(wireless_input, line)) {
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
            telemetry.wifi_signal_dbm = level;
            break;
        }
    }

    const std::string bitrate =
        runCommandFirstLine("iw dev wlan0 link 2>/dev/null | grep 'tx bitrate' || true");
    const auto position = bitrate.find("tx bitrate:");
    if (position != std::string::npos) {
        std::istringstream stream(bitrate.substr(position + std::string("tx bitrate:").size()));
        stream >> telemetry.wifi_tx_bitrate_mbps;
    }
    return telemetry;
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

    std::uint64_t sentFrames() const { return sent_frames_.load(); }
    std::uint64_t skippedFrames() const { return skipped_frames_.load(); }
    std::uint64_t chunksSent() const { return chunks_sent_.load(); }
    std::uint64_t sendFailures() const { return send_failures_.load(); }
    int lastChunkCount() const { return last_chunk_count_.load(); }

    std::uint64_t droppedFrames() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_frames_;
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

            if (!frame) {
                continue;
            }

            const auto send_started = std::chrono::steady_clock::now();
            const bool sent = streamer_.sendFrame(*frame);
            const auto send_finished = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                last_send_ms_ = std::chrono::duration<double, std::milli>(
                    send_finished - send_started).count();
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

struct VisionDebugPublisher::Impl {
    VisionDebugPublisherOptions options;
    network::UdpTelemetrySender telemetry_sender;
    LatestVideoSender video_sender;
    std::uint32_t telemetry_seq = 1;
    int published_count = 0;
    std::string last_error;
    protocol::SystemTelemetry last_system;
    std::chrono::steady_clock::time_point last_video_sent_time {};
    double last_telemetry_build_ms = 0.0;
    double last_telemetry_send_ms = 0.0;
    double last_video_submit_ms = 0.0;
    std::uint64_t last_telemetry_bytes = 0;
};

VisionDebugPublisher::VisionDebugPublisher()
    : impl_(new Impl)
{
}

VisionDebugPublisher::~VisionDebugPublisher()
{
    close();
    delete impl_;
}

bool VisionDebugPublisher::open(const VisionDebugPublisherOptions& options)
{
    close();
    impl_->options = options;
    impl_->last_error.clear();
    impl_->telemetry_seq = 1;
    impl_->published_count = 0;
    impl_->last_system = readSystemTelemetry();
    impl_->last_video_sent_time = {};
    impl_->last_telemetry_build_ms = 0.0;
    impl_->last_telemetry_send_ms = 0.0;
    impl_->last_video_submit_ms = 0.0;
    impl_->last_telemetry_bytes = 0;

    if (options.send_video &&
        !impl_->video_sender.start(
            options.network.gcs_ip,
            options.network.video_port,
            options.vision.debug_video.chunk_pacing_us)) {
        impl_->last_error = impl_->video_sender.takeLastError();
        return false;
    }

    if (options.send_telemetry &&
        !impl_->telemetry_sender.open(options.network.gcs_ip, options.network.telemetry_port)) {
        impl_->last_error = impl_->telemetry_sender.lastError();
        impl_->video_sender.stop();
        return false;
    }
    return true;
}

VisionDebugPublishStats VisionDebugPublisher::publish(VisionDebugPublishInput input)
{
    VisionDebugPublishStats stats;
    const auto& result = input.vision_output.result;
    const std::size_t input_jpeg_bytes = input.frame.jpeg_data.size();

    double telemetry_build_ms = impl_->last_telemetry_build_ms;
    double telemetry_send_ms = impl_->last_telemetry_send_ms;
    if (impl_->options.send_telemetry) {
        if (impl_->published_count % 30 == 0) {
            impl_->last_system = readSystemTelemetry();
        }

        protocol::BringupTelemetry telemetry;
        telemetry.seq = impl_->telemetry_seq++;
        telemetry.timestamp_ms = input.frame.timestamp_ms;
        telemetry.system = impl_->last_system;
        telemetry.camera.status = input.camera_status;
        telemetry.camera.sensor_model = impl_->options.camera_sensor_model.empty()
            ? impl_->options.vision.camera.sensor_model
            : impl_->options.camera_sensor_model;
        telemetry.camera.camera_index = impl_->options.camera_index;
        telemetry.camera.width = input.frame.width;
        telemetry.camera.height = input.frame.height;
        telemetry.camera.fps = impl_->options.vision.camera.fps;
        telemetry.camera.configured_fps = impl_->options.vision.camera.fps;
        telemetry.camera.measured_capture_fps = input.capture_fps;
        telemetry.camera.frame_seq = input.frame.frame_id;
        telemetry.camera.autofocus_mode = impl_->options.vision.camera.autofocus_mode;
        telemetry.camera.lens_position = impl_->options.vision.camera.lens_position;
        telemetry.camera.exposure_mode = impl_->options.vision.camera.exposure;
        telemetry.camera.shutter_us = impl_->options.vision.camera.shutter_us;
        telemetry.camera.gain = impl_->options.vision.camera.gain;
        telemetry.camera.awb = impl_->options.vision.camera.awb;
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
            toProtocolIntersectionDecision(input.intersection_decision, input.grid_node);
        telemetry.vision.grid_node = toProtocolGridNode(input.grid_node);
        telemetry.vision.drone_position.valid = input.drone_position_valid;
        telemetry.vision.drone_position.cell_progress = input.drone_cell_progress;
        telemetry.vision.drone_position.grid_offset_x = input.drone_grid_offset_x;
        telemetry.vision.drone_position.grid_offset_y = input.drone_grid_offset_y;
        // Cycle 23: forward MissionTelemetry (state + markers_found registry)
        // verbatim. The TelemetryMessage JSON serialiser only emits the
        // mission block when input.mission.present == true.
        telemetry.mission = input.mission;
        if (input.grid_node.valid) {
            telemetry.grid.col = input.grid_node.local_coord.x;
            telemetry.grid.row = input.grid_node.local_coord.y;
        }
        telemetry.debug.aruco_latency_ms = input.vision_output.metrics.aruco_latency_ms;
        telemetry.debug.line_latency_ms = input.vision_output.metrics.line_latency_ms;
        telemetry.debug.intersection_latency_ms = input.vision_output.metrics.intersection_latency_ms;
        telemetry.debug.intersection_decision_latency_ms = input.intersection_decision_latency_ms;
        telemetry.debug.processing_latency_ms = input.processing_latency_ms;
        telemetry.debug.read_frame_ms = input.read_frame_ms;
        telemetry.debug.jpeg_decode_ms = input.jpeg_decode_ms;
        telemetry.debug.telemetry_build_ms = impl_->last_telemetry_build_ms;
        telemetry.debug.telemetry_send_ms = impl_->last_telemetry_send_ms;
        telemetry.debug.video_submit_ms = impl_->last_video_submit_ms;
        telemetry.debug.video_send_ms = impl_->video_sender.lastSendMs();
        telemetry.debug.capture_fps = input.capture_fps;
        telemetry.debug.processing_fps = input.processing_fps;
        telemetry.debug.debug_video_send_fps = impl_->options.vision.debug_video.send_fps;
        telemetry.debug.video_chunk_pacing_us = impl_->options.vision.debug_video.chunk_pacing_us;
        telemetry.debug.cpu_temp_c = impl_->last_system.cpu_temp_c;
        telemetry.debug.telemetry_bytes = impl_->last_telemetry_bytes;
        telemetry.debug.video_jpeg_bytes = input_jpeg_bytes;
        telemetry.debug.video_sent_frames = impl_->video_sender.sentFrames();
        telemetry.debug.video_dropped_frames = impl_->video_sender.droppedFrames();
        telemetry.debug.video_skipped_frames = impl_->video_sender.skippedFrames();
        telemetry.debug.video_chunks_sent = impl_->video_sender.chunksSent();
        telemetry.debug.video_send_failures = impl_->video_sender.sendFailures();
        telemetry.debug.video_chunk_count = impl_->video_sender.lastChunkCount();
        telemetry.debug.line_mask_count = result.line.mask_count;
        telemetry.debug.line_contours_found = result.line.contours_found;
        telemetry.debug.line_candidates_evaluated = result.line.candidates_evaluated;
        telemetry.debug.line_roi_pixels = result.line.roi_pixels;
        telemetry.debug.line_selected_contour_points = result.line.selected_contour_points;
        telemetry.note = impl_->options.note;

        const auto telemetry_build_started = std::chrono::steady_clock::now();
        const std::string payload = protocol::buildTelemetryJson(telemetry);
        const auto telemetry_build_finished = std::chrono::steady_clock::now();
        telemetry_build_ms = std::chrono::duration<double, std::milli>(
            telemetry_build_finished - telemetry_build_started).count();
        impl_->last_telemetry_build_ms = telemetry_build_ms;
        impl_->last_telemetry_bytes = payload.size();

        const auto telemetry_send_started = std::chrono::steady_clock::now();
        if (!impl_->telemetry_sender.send(payload)) {
            impl_->last_error = impl_->telemetry_sender.lastError();
        }
        const auto telemetry_send_finished = std::chrono::steady_clock::now();
        telemetry_send_ms = std::chrono::duration<double, std::milli>(
            telemetry_send_finished - telemetry_send_started).count();
        impl_->last_telemetry_send_ms = telemetry_send_ms;
    }

    double video_submit_ms = impl_->last_video_submit_ms;
    std::uint64_t video_jpeg_bytes = input_jpeg_bytes;
    if (impl_->options.send_video) {
        camera::CameraFrame video_frame;
        video_frame.frame_id = input.frame.frame_id;
        video_frame.timestamp_ms = input.frame.timestamp_ms;
        video_frame.width = input.frame.width;
        video_frame.height = input.frame.height;
        video_frame.jpeg_data = std::move(input.frame.jpeg_data);

        if (video_frame.jpeg_data.empty() && !input.image_bgr.empty()) {
            std::vector<int> params {
                cv::IMWRITE_JPEG_QUALITY,
                std::clamp(impl_->options.vision.debug_video.jpeg_quality, 1, 100),
            };
            std::vector<unsigned char> encoded;
            if (cv::imencode(".jpg", input.image_bgr, encoded, params)) {
                video_frame.jpeg_data.assign(encoded.begin(), encoded.end());
            } else {
                impl_->last_error = "failed to JPEG-encode debug video frame";
            }
        }
        video_jpeg_bytes = video_frame.jpeg_data.size();

        if (!video_frame.jpeg_data.empty()) {
            const int send_fps = std::clamp(
                impl_->options.vision.debug_video.send_fps,
                1,
                std::max(1, impl_->options.vision.camera.fps));
            const auto now = std::chrono::steady_clock::now();
            const auto min_period = std::chrono::microseconds(1000000 / send_fps);
            const bool send_every_processed_frame =
                send_fps >= std::max(1, impl_->options.vision.camera.fps);
            const bool should_send =
                send_every_processed_frame ||
                impl_->last_video_sent_time.time_since_epoch().count() == 0 ||
                now - impl_->last_video_sent_time >= min_period;
            if (should_send) {
                const auto video_submit_started = std::chrono::steady_clock::now();
                impl_->video_sender.submit(std::move(video_frame));
                const auto video_submit_finished = std::chrono::steady_clock::now();
                video_submit_ms = std::chrono::duration<double, std::milli>(
                    video_submit_finished - video_submit_started).count();
                impl_->last_video_submit_ms = video_submit_ms;
                impl_->last_video_sent_time = now;
                const std::string video_error = impl_->video_sender.takeLastError();
                if (!video_error.empty()) {
                    impl_->last_error = video_error;
                }
            } else {
                impl_->video_sender.noteSkippedFrame();
            }
        }
    }

    ++impl_->published_count;
    stats.telemetry_build_ms = telemetry_build_ms;
    stats.telemetry_send_ms = telemetry_send_ms;
    stats.video_submit_ms = video_submit_ms;
    stats.video_send_ms = impl_->video_sender.lastSendMs();
    stats.telemetry_bytes = impl_->last_telemetry_bytes;
    stats.video_jpeg_bytes = video_jpeg_bytes;
    stats.video_sent_frames = impl_->video_sender.sentFrames();
    stats.video_dropped_frames = impl_->video_sender.droppedFrames();
    stats.video_skipped_frames = impl_->video_sender.skippedFrames();
    stats.video_chunks_sent = impl_->video_sender.chunksSent();
    stats.video_send_failures = impl_->video_sender.sendFailures();
    stats.video_chunk_count = impl_->video_sender.lastChunkCount();
    return stats;
}

void VisionDebugPublisher::close()
{
    if (impl_ == nullptr) {
        return;
    }
    impl_->video_sender.stop();
}

std::string VisionDebugPublisher::lastError() const
{
    return impl_->last_error;
}

} // namespace onboard::app
