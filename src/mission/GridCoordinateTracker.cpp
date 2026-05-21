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

    // Cycle 9: peek-only. Compute what the next node WOULD be — its coord
    // (one cell forward of current_coord_ in current_heading_), topology,
    // and branches — but do NOT mutate current_coord_/current_heading_/nodes_.
    // GridMission gates the decision and calls commitAdvance to finalize.
    GridHeading peek_heading = current_heading_;
    GridCoord peek_coord = current_coord_;
    bool peek_first_node = false;

    if (!has_origin_) {
        if (peek_heading == GridHeading::Unknown) {
            peek_heading = GridHeading::North;
        }
        peek_coord = {};
        peek_first_node = true;
    } else if (peek_heading != GridHeading::Unknown) {
        peek_coord = advance(peek_coord, peek_heading);
    }

    event.valid = true;
    event.node_id = next_node_id_;  // not incremented during peek
    event.local_coord = peek_coord;
    event.topology = decision.accepted_type;
    event.arrival_heading = peek_heading;
    event.camera_branch_mask = decision.accepted_branch_mask;
    event.grid_branch_mask = rotateCameraBranchMaskToGrid(
        decision.accepted_branch_mask,
        peek_heading);
    event.first_node = peek_first_node;
    event.origin_local_only = true;

    // Frame-level dedup: bump regardless of whether mission ends up committing.
    // Without this, every frame after lockout expiry would re-emit the same
    // peek event indefinitely.
    last_node_frame_seq_ = frame_seq;
    return event;
}

void GridCoordinateTracker::commitAdvance(const GridNodeEvent& event)
{
    if (!event.valid) {
        return;
    }
    // Idempotency: if this event was already committed (last_node_frame_seq_
    // matches and coord identical), skip.
    if (event.node_id != 0 && event.node_id < next_node_id_ &&
        nodes_.count(event.local_coord) != 0) {
        return;
    }
    if (!event.updates_current && !has_origin_) {
        return;
    }

    if (!has_origin_) {
        has_origin_ = true;
        if (current_heading_ == GridHeading::Unknown) {
            current_heading_ = event.arrival_heading != GridHeading::Unknown
                ? event.arrival_heading
                : GridHeading::North;
            using_default_start_heading_ = true;
        }
        current_coord_ = event.local_coord;
    } else if (event.updates_current && current_heading_ != GridHeading::Unknown) {
        current_coord_ = event.local_coord;
    }

    GridNodeEvent committed = event;
    committed.node_id = next_node_id_++;
    nodes_[committed.local_coord] = committed;

    if (event.updates_current) {
        // chooseNextHeading uses decision-derived branch info; reconstruct
        // minimal decision from event.camera_branch_mask for the heading choice.
        IntersectionDecision proxy;
        proxy.accepted_branch_mask = event.camera_branch_mask;
        current_heading_ = chooseNextHeading(proxy);
    }
}

void GridCoordinateTracker::notifyTurnCompleted(GridHeading new_heading)
{
    current_heading_ = new_heading;
    using_default_start_heading_ = false;
}

void GridCoordinateTracker::setHeading(GridHeading heading)
{
    current_heading_ = heading;
    using_default_start_heading_ = false;
}

void GridCoordinateTracker::resetLocalOrigin()
{
    has_origin_ = false;
    current_coord_ = {};
    current_heading_ = GridHeading::Unknown;
    using_default_start_heading_ = false;
    pending_second_turn_ = false;
    pending_turn_right_ = true;
    next_node_id_ = 1;
    last_node_frame_seq_ = 0;
    nodes_.clear();
}

void GridCoordinateTracker::forceOrigin(GridCoord coord, GridHeading heading)
{
    has_origin_ = true;
    current_coord_ = coord;
    current_heading_ = heading;
    using_default_start_heading_ = false;
    pending_second_turn_ = false;
    pending_turn_right_ = true;
    // The forced origin reserves node_id=1 (latchGridOrigin synthesises an
    // origin GridNodeEvent with id=1 and ships it to GCS directly without
    // going through commitAdvance). Without bumping next_node_id_ here, the
    // first SnakeForward arrival's peek event would also carry node_id=1 and
    // GCS's id-based dedup would silently drop it — making (0,-1) appear
    // skipped on the rendered grid even though onboard committed it.
    if (next_node_id_ < 2) {
        next_node_id_ = 2;
    }
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
    // Cycle 13: reverted to screen convention — north = -y, south = +y. The
    // first grid node is (0,0), then (0,-1), (0,-2)... as the drone proceeds
    // north up the column; vertiport sits at (0,+1). GCS canvasRow uses
    // (y - min_y) * 2 so smaller y still renders at the top of the canvas.
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

GridHeading GridCoordinateTracker::chooseNextHeading(const IntersectionDecision& decision)
{
    if (current_heading_ == GridHeading::Unknown) {
        return current_heading_;
    }

    if (pending_second_turn_) {
        pending_second_turn_ = false;
        return pending_turn_right_ ? turnRight(current_heading_) : turnLeft(current_heading_);
    }

    const bool front_available = (decision.accepted_branch_mask & (1 << 0)) != 0;
    const bool right_available = (decision.accepted_branch_mask & (1 << 1)) != 0;
    const bool left_available = (decision.accepted_branch_mask & (1 << 3)) != 0;

    // At the first grid node, prefer the visible side branch so the bottom-entry
    // hand-held smoke test starts expanding across the first row.
    if (using_default_start_heading_ && nodes_.size() == 1 && right_available) {
        using_default_start_heading_ = false;
        return turnRight(current_heading_);
    }
    if (using_default_start_heading_ && nodes_.size() == 1 && !right_available && left_available) {
        using_default_start_heading_ = false;
        return turnLeft(current_heading_);
    }

    if (front_available) {
        return current_heading_;
    }

    if (right_available) {
        pending_second_turn_ = true;
        pending_turn_right_ = true;
        return turnRight(current_heading_);
    }
    if (left_available) {
        pending_second_turn_ = true;
        pending_turn_right_ = false;
        return turnLeft(current_heading_);
    }
    if ((decision.accepted_branch_mask & (1 << 2)) != 0) {
        return turnBack(current_heading_);
    }
    return current_heading_;
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
