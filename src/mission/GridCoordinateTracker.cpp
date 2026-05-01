#include "mission/GridCoordinateTracker.hpp"

#include <algorithm>

namespace onboard::mission {
namespace {

constexpr std::uint8_t kNorthBit = 1 << 0;
constexpr std::uint8_t kEastBit = 1 << 1;
constexpr std::uint8_t kSouthBit = 1 << 2;
constexpr std::uint8_t kWestBit = 1 << 3;

std::uint8_t mapFrontBit(GridHeading heading)
{
    switch (heading) {
    case GridHeading::North:
        return kNorthBit;
    case GridHeading::East:
        return kEastBit;
    case GridHeading::South:
        return kSouthBit;
    case GridHeading::West:
        return kWestBit;
    case GridHeading::Unknown:
        return kNorthBit;
    }
    return kNorthBit;
}

GridHeading turnRight(GridHeading heading)
{
    switch (heading) {
    case GridHeading::North:
        return GridHeading::East;
    case GridHeading::East:
        return GridHeading::South;
    case GridHeading::South:
        return GridHeading::West;
    case GridHeading::West:
        return GridHeading::North;
    case GridHeading::Unknown:
        return GridHeading::Unknown;
    }
    return GridHeading::Unknown;
}

GridHeading turnBack(GridHeading heading)
{
    return turnRight(turnRight(heading));
}

GridHeading turnLeft(GridHeading heading)
{
    return turnRight(turnBack(heading));
}

std::uint8_t headingBit(GridHeading heading)
{
    switch (heading) {
    case GridHeading::North:
        return kNorthBit;
    case GridHeading::East:
        return kEastBit;
    case GridHeading::South:
        return kSouthBit;
    case GridHeading::West:
        return kWestBit;
    case GridHeading::Unknown:
        return 0;
    }
    return 0;
}

} // namespace

GridCoordinateTracker::GridCoordinateTracker(const common::IntersectionDecisionConfig& config)
    : config_(config)
{
}

GridNodeEvent GridCoordinateTracker::update(
    const IntersectionDecision& decision,
    std::uint32_t frame_seq,
    std::int64_t timestamp_ms)
{
    (void)timestamp_ms;
    GridNodeEvent event;
    if (!decision.event_ready) {
        return event;
    }
    if (last_node_frame_seq_ != 0 &&
        frame_seq > last_node_frame_seq_ &&
        frame_seq - last_node_frame_seq_ < static_cast<std::uint32_t>(std::max(1, config_.node_advance_min_frames))) {
        return event;
    }

    if (!has_origin_) {
        has_origin_ = true;
        current_coord_ = {};
        event.first_node = true;
    } else if (current_heading_ != GridHeading::Unknown) {
        current_coord_ = advance(current_coord_, current_heading_);
    }

    event.valid = true;
    event.node_id = next_node_id_++;
    event.local_coord = current_coord_;
    event.topology = decision.accepted_type;
    event.arrival_heading = current_heading_;
    event.camera_branch_mask = decision.accepted_branch_mask;
    event.grid_branch_mask = rotateCameraBranchMaskToGrid(
        decision.accepted_branch_mask,
        current_heading_);
    event.origin_local_only = true;

    nodes_[event.local_coord] = event;
    last_node_frame_seq_ = frame_seq;
    return event;
}

void GridCoordinateTracker::notifyTurnCompleted(GridHeading new_heading)
{
    current_heading_ = new_heading;
}

void GridCoordinateTracker::setHeading(GridHeading heading)
{
    current_heading_ = heading;
}

void GridCoordinateTracker::resetLocalOrigin()
{
    has_origin_ = false;
    current_coord_ = {};
    next_node_id_ = 1;
    last_node_frame_seq_ = 0;
    nodes_.clear();
}

GridHeading GridCoordinateTracker::currentHeading() const
{
    return current_heading_;
}

GridCoord GridCoordinateTracker::currentCoord() const
{
    return current_coord_;
}

const std::map<GridCoord, GridNodeEvent, GridCoordLess>& GridCoordinateTracker::nodes() const
{
    return nodes_;
}

GridCoord GridCoordinateTracker::advance(GridCoord coord, GridHeading heading) const
{
    switch (heading) {
    case GridHeading::North:
        --coord.y;
        break;
    case GridHeading::East:
        ++coord.x;
        break;
    case GridHeading::South:
        ++coord.y;
        break;
    case GridHeading::West:
        --coord.x;
        break;
    case GridHeading::Unknown:
        break;
    }
    return coord;
}

const char* gridHeadingName(GridHeading heading)
{
    switch (heading) {
    case GridHeading::North:
        return "north";
    case GridHeading::East:
        return "east";
    case GridHeading::South:
        return "south";
    case GridHeading::West:
        return "west";
    case GridHeading::Unknown:
        return "unknown";
    }
    return "unknown";
}

std::uint8_t rotateCameraBranchMaskToGrid(std::uint8_t camera_mask, GridHeading heading)
{
    if (heading == GridHeading::Unknown) {
        return camera_mask;
    }

    std::uint8_t grid_mask = 0;
    if ((camera_mask & (1 << 0)) != 0) {
        grid_mask = static_cast<std::uint8_t>(grid_mask | mapFrontBit(heading));
    }
    if ((camera_mask & (1 << 1)) != 0) {
        grid_mask = static_cast<std::uint8_t>(grid_mask | headingBit(turnRight(heading)));
    }
    if ((camera_mask & (1 << 2)) != 0) {
        grid_mask = static_cast<std::uint8_t>(grid_mask | headingBit(turnBack(heading)));
    }
    if ((camera_mask & (1 << 3)) != 0) {
        grid_mask = static_cast<std::uint8_t>(grid_mask | headingBit(turnLeft(heading)));
    }
    return grid_mask;
}

} // namespace onboard::mission
