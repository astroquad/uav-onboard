#pragma once

#include "mission/GridCoordinateTracker.hpp"

#include <cstdint>
#include <optional>
#include <unordered_set>
#include <vector>

namespace onboard::mission {

struct MarkerRecord {
    int aruco_id = -1;
    GridCoord grid_coord;
    bool grid_coord_valid = false;
    std::int64_t first_seen_ms = 0;
    std::int64_t last_seen_ms = 0;
    std::uint32_t first_frame_seq = 0;
    float orientation_deg = 0.0f;
};

class MarkerRegistry {
public:
    MarkerRegistry() = default;

    // Returns true iff this id was newly added (not seen before).
    // grid_coord_valid=false means "spotted but pre-grid (e.g. vertiport)".
    bool observe(int aruco_id,
                 GridCoord grid_coord,
                 bool grid_coord_valid,
                 float orientation_deg,
                 std::int64_t timestamp_ms,
                 std::uint32_t frame_seq);

    bool hasId(int aruco_id) const;
    std::size_t uniqueCount() const { return seen_ids_.size(); }
    std::size_t gridMarkerCount() const;

    // All markers that have been bound to a grid coord (i.e. excluding vertiport).
    const std::vector<MarkerRecord>& records() const { return records_; }

    // Cycle 10: GCS / log access to the committed id set.
    const std::unordered_set<int>& seenIds() const { return seen_ids_; }

    std::optional<MarkerRecord> findByGrid(GridCoord coord) const;
    std::optional<MarkerRecord> findById(int aruco_id) const;

    void reset();

private:
    std::unordered_set<int> seen_ids_;
    std::vector<MarkerRecord> records_;
};

} // namespace onboard::mission
