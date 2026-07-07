// FlightLogger behaviour: sequential run-dir allocation, file creation,
// frame rate capping, event pass-through, and disabled-mode no-ops.

#undef NDEBUG
#include "logging/FlightLogger.hpp"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using onboard::logging::FlightLogger;
using onboard::logging::FlightLoggerOptions;
using onboard::logging::jsonEscape;

namespace {

std::vector<std::string> readLines(const fs::path& path)
{
    std::ifstream in(path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) {
        lines.push_back(line);
    }
    return lines;
}

} // namespace

int main()
{
    const fs::path base = fs::temp_directory_path() / "astroquad_flight_log_test";
    fs::remove_all(base);

    // Run 1: files created, header + rows + events written on close.
    {
        FlightLoggerOptions options;
        options.base_dir = base.string();
        options.frame_log_hz = 0.0;   // every call
        FlightLogger logger;
        assert(logger.open(options));
        assert(logger.enabled());
        assert(logger.runDir().find("run_0001_") != std::string::npos);

        logger.writeMeta("{\"test\":true}");
        logger.writeFrameHeader("t_s,state");
        logger.logFrame(0.00, "0.00,ARM_TAKEOFF");
        logger.logFrame(0.05, "0.05,ARM_TAKEOFF");
        logger.logEvent("{\"t\":0.05,\"type\":\"state\"}");
        logger.close();

        const fs::path run_dir = logger.runDir();
        assert(fs::exists(run_dir / "meta.json"));
        const auto frames = readLines(run_dir / "frames.csv");
        assert(frames.size() == 3);   // header + 2 rows
        assert(frames[0] == "t_s,state");
        const auto events = readLines(run_dir / "events.jsonl");
        assert(events.size() == 1);
        assert(logger.droppedLines() == 0);
    }

    // Run 2: sequential numbering continues past run 1.
    {
        FlightLoggerOptions options;
        options.base_dir = base.string();
        FlightLogger logger;
        assert(logger.open(options));
        assert(logger.runDir().find("run_0002_") != std::string::npos);
        logger.close();
    }

    // Rate cap: 10 Hz drops same-instant duplicates but keeps spaced rows.
    {
        FlightLoggerOptions options;
        options.base_dir = base.string();
        options.frame_log_hz = 10.0;
        FlightLogger logger;
        assert(logger.open(options));
        logger.logFrame(0.00, "row0");
        logger.logFrame(0.05, "row-dropped");   // 50ms later: below 100ms period
        logger.logFrame(0.20, "row1");
        logger.close();
        const auto frames = readLines(fs::path(logger.runDir()) / "frames.csv");
        assert(frames.size() == 2);
        assert(frames[0] == "row0");
        assert(frames[1] == "row1");
    }

    // Disabled: open succeeds, no run dir, all calls are no-ops.
    {
        FlightLoggerOptions options;
        options.enabled = false;
        options.base_dir = base.string();
        FlightLogger logger;
        assert(logger.open(options));
        assert(!logger.enabled());
        logger.logFrame(0.0, "ignored");
        logger.logEvent("ignored");
        logger.close();
        int run_count = 0;
        for (const auto& entry : fs::directory_iterator(base)) {
            (void)entry;
            ++run_count;
        }
        assert(run_count == 3);
    }

    assert(jsonEscape("a\"b\\c\nd") == "a\\\"b\\\\c\\nd");

    fs::remove_all(base);
    return 0;
}
