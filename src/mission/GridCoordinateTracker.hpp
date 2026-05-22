#pragma once

#include "common/VisionConfig.hpp"
#include "mission/IntersectionDecision.hpp"
#include "vision/VisionTypes.hpp"

#include <cstdint>
#include <map>

namespace onboard::mission {

enum class GridHeading {
    North,
    East,
    South,
    West,
    Unknown,
};

struct GridCoord {
    int x = 0;
    int y = 0;
};

struct GridCoordLess {
    bool operator()(const GridCoord& lhs, const GridCoord& rhs) const
    {
        if (lhs.y != rhs.y) {
            return lhs.y < rhs.y;
        }
        return lhs.x < rhs.x;
    }
};

struct GridNodeEvent {
    bool valid = false;
    std::uint32_t node_id = 0;
    GridCoord local_coord;
    vision::IntersectionType topology = vision::IntersectionType::Unknown;
    GridHeading arrival_heading = GridHeading::Unknown;
    std::uint8_t camera_branch_mask = 0;
    std::uint8_t grid_branch_mask = 0;
    bool first_node = false;
    bool origin_local_only = true;
    // False for map-only synthetic nodes that complete the known grid shape
    // without implying that the drone physically moved to that node.
    bool updates_current = true;
};

class GridCoordinateTracker {
public:
    explicit GridCoordinateTracker(const common::IntersectionDecisionConfig& config);

    // Peek-only: previews what the next node event would look like (its coord,
    // topology, branches) given the current heading. Does NOT advance internal
    // state. Cycle 9 split: GridMission decides via distance/lockout gates
    // whether to commit, then calls commitAdvance to actually move the tracker.
    GridNodeEvent update(
        const IntersectionDecision& decision,
        std::uint32_t frame_seq,
        std::int64_t timestamp_ms);

    // Commit a previously previewed event: advances current_coord_, updates
    // heading via chooseNextHeading, records the node in the map. Idempotent
    // if called twice for the same frame_seq.
    void commitAdvance(const GridNodeEvent& event);

    void notifyTurnCompleted(GridHeading new_heading);
    void setHeading(GridHeading heading);
    void setCurrentPose(GridCoord coord, GridHeading heading);
    void resetLocalOrigin();
    void forceOrigin(GridCoord coord, GridHeading heading);

    GridHeading currentHeading() const;
    GridCoord currentCoord() const;
    const std::map<GridCoord, GridNodeEvent, GridCoordLess>& nodes() const;

    // Returns the GridCoord one cell forward of `coord` along `heading`.
    // Cycle 23: exposed so GridMission can synthesise a boundary commit event
    // when the IntersectionDecision routes straight to TurnReady (skipping
    // NodeRecord) and tracker peek did not fire a valid event.
    GridCoord advance(GridCoord coord, GridHeading heading) const;

private:
    GridHeading chooseNextHeading(const IntersectionDecision& decision);

    common::IntersectionDecisionConfig config_;
    bool has_origin_ = false;
    GridHeading current_heading_ = GridHeading::Unknown;
    bool using_default_start_heading_ = false;
    bool pending_second_turn_ = false;
    bool pending_turn_right_ = true;
    GridCoord current_coord_;
    std::uint32_t next_node_id_ = 1;
    std::uint32_t last_node_frame_seq_ = 0;
    std::map<GridCoord, GridNodeEvent, GridCoordLess> nodes_;
};

const char* gridHeadingName(GridHeading heading);
std::uint8_t rotateCameraBranchMaskToGrid(std::uint8_t camera_mask, GridHeading heading);

} // namespace onboard::mission
