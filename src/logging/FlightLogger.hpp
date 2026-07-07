#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

namespace onboard::logging {

struct FlightLoggerOptions {
    bool enabled = true;
    // Base directory for run folders. Relative paths resolve against the
    // process working directory (repo root in the documented runbooks).
    std::string base_dir = "logs/flights";
    // Per-frame CSV rate cap in Hz. 0 logs every mission tick (~20 Hz,
    // ~25 MB/h). Lower it on tight storage; events are never rate-capped.
    double frame_log_hz = 0.0;
    // Writer-queue bound. When the SD card stalls and the queue fills,
    // the oldest frame lines are dropped (counted) instead of blocking the
    // mission loop.
    std::size_t max_queue_lines = 8192;
    double flush_interval_s = 1.0;
};

// Per-run flight logger. open() allocates the next sequential run folder
// (`<base_dir>/run_0001_2026-07-07_18-30-12/`) and starts a writer thread.
// The mission loop only formats lines and enqueues them; all file I/O and
// flushing happens on the writer thread so logging can never stall the
// 20 Hz control loop.
//
// Files inside a run folder:
//   meta.json   — one-shot run description (args, key config, start time)
//   frames.csv  — one row per mission tick (vision/mission/control/MAVLink)
//   events.jsonl— state transitions, node/marker commits, safety events
class FlightLogger {
public:
    FlightLogger() = default;
    ~FlightLogger();

    FlightLogger(const FlightLogger&) = delete;
    FlightLogger& operator=(const FlightLogger&) = delete;

    // Returns false when the run directory could not be created (error via
    // lastError()); the logger then acts as disabled and every log call is a
    // cheap no-op, so callers never need to guard on the result.
    bool open(const FlightLoggerOptions& options);
    void close();

    bool enabled() const { return enabled_; }
    const std::string& runDir() const { return run_dir_; }
    std::string lastError() const { return last_error_; }
    std::uint64_t droppedLines() const;

    // meta.json — written & flushed immediately (called once, small).
    void writeMeta(const std::string& json_text);

    // frames.csv header — written once, bypasses the rate cap.
    void writeFrameHeader(const std::string& csv_header);

    // frames.csv row — rate-capped by frame_log_hz.
    void logFrame(double now_s, std::string csv_line);

    // events.jsonl line — never rate-capped, flushed on the next writer pass.
    void logEvent(std::string json_line);

private:
    enum class Channel { Frame, Event };
    struct Entry {
        Channel channel;
        std::string line;
    };

    void enqueue(Channel channel, std::string line);
    void writerLoop();

    FlightLoggerOptions options_;
    bool enabled_ = false;
    std::string run_dir_;
    std::string last_error_;
    double last_frame_log_s_ = -1.0;

    std::ofstream meta_file_;
    std::ofstream frames_file_;
    std::ofstream events_file_;

    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<Entry> queue_;
    std::uint64_t dropped_lines_ = 0;
    bool running_ = false;
    std::thread writer_;
};

// Escapes a string for embedding inside a JSON string literal.
std::string jsonEscape(const std::string& text);

} // namespace onboard::logging
