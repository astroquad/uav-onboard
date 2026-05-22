#include "mission/MarkerRegistry.hpp"

namespace onboard::mission {

bool MarkerRegistry::observe(int aruco_id,
                             GridCoord grid_coord,
                             bool grid_coord_valid,
                             float orientation_deg,
                             std::int64_t timestamp_ms,
                             std::uint32_t frame_seq)
{
    auto inserted = seen_ids_.insert(aruco_id);
    if (!inserted.second) {
        for (auto& record : records_) {
            if (record.aruco_id == aruco_id) {
                record.last_seen_ms = timestamp_ms;
                if (grid_coord_valid && !record.grid_coord_valid) {
                    record.grid_coord = grid_coord;
                    record.grid_coord_valid = true;
                }
            }
        }
        return false;
    }
    MarkerRecord record;
    record.aruco_id = aruco_id;
    record.grid_coord = grid_coord;
    record.grid_coord_valid = grid_coord_valid;
    record.orientation_deg = orientation_deg;
    record.first_seen_ms = timestamp_ms;
    record.last_seen_ms = timestamp_ms;
    record.first_frame_seq = frame_seq;
    records_.push_back(record);
    return true;
}

bool MarkerRegistry::hasId(int aruco_id) const
{
    return seen_ids_.find(aruco_id) != seen_ids_.end();
}

std::size_t MarkerRegistry::gridMarkerCount() const
{
    std::size_t count = 0;
    for (const auto& record : records_) {
        if (record.grid_coord_valid) {
            ++count;
        }
    }
    return count;
}

std::size_t MarkerRegistry::revisitedGridMarkerCount() const
{
    std::size_t count = 0;
    for (const auto& record : records_) {
        if (record.grid_coord_valid && record.revisited) {
            ++count;
        }
    }
    return count;
}

bool MarkerRegistry::markRevisited(int aruco_id, std::int64_t timestamp_ms)
{
    for (auto& record : records_) {
        if (record.aruco_id == aruco_id) {
            record.revisited = true;
            record.revisited_ms = timestamp_ms;
            return true;
        }
    }
    return false;
}

std::optional<MarkerRecord> MarkerRegistry::findByGrid(GridCoord coord) const
{
    for (const auto& record : records_) {
        if (record.grid_coord_valid &&
            record.grid_coord.x == coord.x &&
            record.grid_coord.y == coord.y) {
            return record;
        }
    }
    return std::nullopt;
}

std::optional<MarkerRecord> MarkerRegistry::findById(int aruco_id) const
{
    for (const auto& record : records_) {
        if (record.aruco_id == aruco_id) {
            return record;
        }
    }
    return std::nullopt;
}

void MarkerRegistry::reset()
{
    seen_ids_.clear();
    records_.clear();
}

} // namespace onboard::mission
