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
};

class GridCoordinateTracker {
public:
    explicit GridCoordinateTracker(const common::IntersectionDecisionConfig& config);

    GridNodeEvent update(
        const IntersectionDecision& decision,
        std::uint32_t frame_seq,
        std::int64_t timestamp_ms);

    void notifyTurnCompleted(GridHeading new_heading);
    void setHeading(GridHeading heading);
    void resetLocalOrigin();

    GridHeading currentHeading() const;
    GridCoord currentCoord() const;
    const std::map<GridCoord, GridNodeEvent, GridCoordLess>& nodes() const;

private:
    GridCoord advance(GridCoord coord, GridHeading heading) const;
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
