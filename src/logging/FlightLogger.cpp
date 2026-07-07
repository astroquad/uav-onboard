#include "logging/FlightLogger.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <system_error>

namespace onboard::logging {
namespace fs = std::filesystem;

namespace {

// Parses the NNNN out of "run_NNNN_..."; returns -1 for non-run entries.
int runNumberOf(const std::string& name)
{
    constexpr const char* kPrefix = "run_";
    if (name.rfind(kPrefix, 0) != 0) {
        return -1;
    }
    int value = 0;
    bool any_digit = false;
    for (std::size_t i = 4; i < name.size() && name[i] != '_'; ++i) {
        if (name[i] < '0' || name[i] > '9') {
            return -1;
        }
        value = value * 10 + (name[i] - '0');
        any_digit = true;
    }
    return any_digit ? value : -1;
}

std::string localTimestampForDirName()
{
    const std::time_t now = std::time(nullptr);
    std::tm tm_buf {};
#if defined(_WIN32)
    localtime_s(&tm_buf, &now);
#else
    localtime_r(&now, &tm_buf);
#endif
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d_%02d-%02d-%02d",
                  tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                  tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec);
    return buf;
}

} // namespace

std::string jsonEscape(const std::string& text)
{
    std::string out;
    out.reserve(text.size() + 8);
    for (const char c : text) {
        switch (c) {
        case '"':  out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                out += buf;
            } else {
                out += c;
            }
        }
    }
    return out;
}

FlightLogger::~FlightLogger()
{
    close();
}

bool FlightLogger::open(const FlightLoggerOptions& options)
{
    close();
    options_ = options;
    enabled_ = false;
    last_error_.clear();
    last_frame_log_s_ = -1.0;
    if (!options.enabled) {
        return true;
    }

    std::error_code ec;
    fs::create_directories(options.base_dir, ec);
    if (ec) {
        last_error_ = "cannot create log base dir '" + options.base_dir +
                      "': " + ec.message();
        return false;
    }

    int max_run = 0;
    for (const auto& entry : fs::directory_iterator(options.base_dir, ec)) {
        if (ec) break;
        max_run = std::max(max_run, runNumberOf(entry.path().filename().string()));
    }

    char run_name[64];
    std::snprintf(run_name, sizeof(run_name), "run_%04d_%s",
                  max_run + 1, localTimestampForDirName().c_str());
    run_dir_ = (fs::path(options.base_dir) / run_name).string();
    fs::create_directories(run_dir_, ec);
    if (ec) {
        last_error_ = "cannot create run dir '" + run_dir_ + "': " + ec.message();
        return false;
    }

    meta_file_.open((fs::path(run_dir_) / "meta.json").string(), std::ios::trunc);
    frames_file_.open((fs::path(run_dir_) / "frames.csv").string(), std::ios::trunc);
    events_file_.open((fs::path(run_dir_) / "events.jsonl").string(), std::ios::trunc);
    if (!meta_file_ || !frames_file_ || !events_file_) {
        last_error_ = "cannot open log files in '" + run_dir_ + "'";
        meta_file_.close();
        frames_file_.close();
        events_file_.close();
        return false;
    }

    running_ = true;
    enabled_ = true;
    writer_ = std::thread([this]() { writerLoop(); });
    return true;
}

void FlightLogger::close()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
    }
    condition_.notify_one();
    if (writer_.joinable()) {
        writer_.join();
    }
    if (frames_file_.is_open()) frames_file_.flush();
    if (events_file_.is_open()) events_file_.flush();
    meta_file_.close();
    frames_file_.close();
    events_file_.close();
    enabled_ = false;
}

std::uint64_t FlightLogger::droppedLines() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_lines_;
}

void FlightLogger::writeMeta(const std::string& json_text)
{
    if (!enabled_) return;
    // meta is written once at startup before the loop runs; direct write is
    // fine and keeps it on disk even if the process dies early.
    meta_file_ << json_text << "\n";
    meta_file_.flush();
}

void FlightLogger::writeFrameHeader(const std::string& csv_header)
{
    if (!enabled_) return;
    enqueue(Channel::Frame, csv_header);
}

void FlightLogger::logFrame(double now_s, std::string csv_line)
{
    if (!enabled_) return;
    if (options_.frame_log_hz > 0.0) {
        const double min_period_s = 1.0 / options_.frame_log_hz;
        if (last_frame_log_s_ >= 0.0 && now_s - last_frame_log_s_ < min_period_s) {
            return;
        }
        last_frame_log_s_ = now_s;
    }
    enqueue(Channel::Frame, std::move(csv_line));
}

void FlightLogger::logEvent(std::string json_line)
{
    if (!enabled_) return;
    enqueue(Channel::Event, std::move(json_line));
}

void FlightLogger::enqueue(Channel channel, std::string line)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) return;
        // Drop the oldest FRAME line first; events are precious and small.
        if (queue_.size() >= options_.max_queue_lines) {
            auto victim = std::find_if(queue_.begin(), queue_.end(),
                [](const Entry& e) { return e.channel == Channel::Frame; });
            if (victim != queue_.end()) {
                queue_.erase(victim);
            } else {
                queue_.pop_front();
            }
            ++dropped_lines_;
        }
        queue_.push_back(Entry{channel, std::move(line)});
    }
    condition_.notify_one();
}

void FlightLogger::writerLoop()
{
    const auto flush_interval = std::chrono::duration<double>(
        std::max(0.1, options_.flush_interval_s));
    auto last_flush = std::chrono::steady_clock::now();
    bool pending_event_flush = false;

    for (;;) {
        std::deque<Entry> batch;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait_for(lock, std::chrono::milliseconds(200),
                [this]() { return !running_ || !queue_.empty(); });
            if (!running_ && queue_.empty()) {
                return;
            }
            batch.swap(queue_);
        }

        for (auto& entry : batch) {
            if (entry.channel == Channel::Frame) {
                frames_file_ << entry.line << '\n';
            } else {
                events_file_ << entry.line << '\n';
                pending_event_flush = true;
            }
        }

        const auto now = std::chrono::steady_clock::now();
        if (pending_event_flush || now - last_flush >= flush_interval) {
            frames_file_.flush();
            events_file_.flush();
            last_flush = now;
            pending_event_flush = false;
        }
    }
}

} // namespace onboard::logging
